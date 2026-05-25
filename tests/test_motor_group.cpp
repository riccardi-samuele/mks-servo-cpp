// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for MotorGroup: dispatch order, wait_all_settled correctness,
// status propagation across N independent paired transports.

#include "doctest/doctest.h"

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#include "mks_servo/motor.hpp"
#include "mks_servo/motor_group.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::Motor;
using mks_servo::MotorGroup;
using mks_servo::MoveSpec;
using mks_servo::Mechanical;
using mks_servo::MotorStatusEx;
using mks_servo::RawDriver;
using mks_servo::Transport;

namespace {

// Mock side of socketpair: writes are fire-and-forget. Capturing the
// return value (vs a bare `(void)::write(...)`) satisfies glibc's
// warn_unused_result attribute on hardened toolchains.
inline void mock_write(int fd, const void* buf, std::size_t n) noexcept {
    const ssize_t r = ::write(fd, buf, n);
    (void)r;
}

constexpr std::size_t N_MOTORS = 3;

struct Rig {
    Transport  transports[N_MOTORS];
    RawDriver* raws[N_MOTORS]    = {};
    Motor*     motors[N_MOTORS]  = {};
    int        peer_fds[N_MOTORS] = {-1, -1, -1};

    Rig() {
        for (std::size_t i = 0; i < N_MOTORS; ++i) {
            int fds[2] = {-1, -1};
            REQUIRE(::socketpair(AF_UNIX, SOCK_STREAM, 0, fds) == 0);
            transports[i] = Transport(fds[0]);
            peer_fds[i]   = fds[1];
            raws[i]       = new RawDriver(transports[i], /*addr=*/1, /*timeout=*/100'000);
            motors[i]     = new Motor(*raws[i], Mechanical{1.0, 0});
        }
    }

    ~Rig() {
        for (std::size_t i = 0; i < N_MOTORS; ++i) {
            delete motors[i];
            delete raws[i];
            if (peer_fds[i] >= 0) ::close(peer_fds[i]);
        }
    }
};

std::vector<std::uint8_t> read_n(int fd, std::size_t n, int timeout_ms = 500) {
    std::vector<std::uint8_t> out(n);
    std::size_t got = 0;
    while (got < n) {
        struct pollfd pfd{};
        pfd.fd     = fd;
        pfd.events = POLLIN;
        if (::poll(&pfd, 1, timeout_ms) <= 0) break;
        const ssize_t r = ::read(fd, out.data() + got, n - got);
        if (r <= 0) break;
        got += static_cast<std::size_t>(r);
    }
    out.resize(got);
    return out;
}

std::array<std::uint8_t, 10> make_encoder_response(std::int64_t counts) {
    std::uint8_t buf[10];
    buf[0] = 0xFB;
    buf[1] = 0x01;
    buf[2] = 0x31;
    const std::uint64_t u = static_cast<std::uint64_t>(counts) & 0x0000FFFFFFFFFFFFull;
    for (int i = 0; i < 6; ++i) {
        buf[3 + i] = static_cast<std::uint8_t>((u >> ((5 - i) * 8)) & 0xFF);
    }
    buf[9] = mks_servo::checksum8(buf, 9);
    std::array<std::uint8_t, 10> out;
    std::memcpy(out.data(), buf, 10);
    return out;
}

}  // namespace

// ─── basic construction ───────────────────────────────────────────
TEST_CASE("MotorGroup: empty after default construction") {
    MotorGroup g;
    CHECK(g.size() == 0);
}

TEST_CASE("MotorGroup: add increments size and exposes references") {
    Rig rig;
    MotorGroup g;
    for (std::size_t i = 0; i < N_MOTORS; ++i) g.add(*rig.motors[i]);
    REQUIRE(g.size() == N_MOTORS);
    for (std::size_t i = 0; i < N_MOTORS; ++i) {
        CHECK(&g[static_cast<std::size_t>(i)] == rig.motors[i]);
    }
}

// ─── dispatch_all: writes a MOVE frame on every motor's bus ────────
TEST_CASE("MotorGroup::dispatch_all sends MOVE_ABS_AXIS and reads ack") {
    Rig rig;
    MotorGroup g;
    for (std::size_t i = 0; i < N_MOTORS; ++i) g.add(*rig.motors[i]);

    const MoveSpec specs[N_MOTORS] = {
        {  10.0, {500,  100}},
        {  90.0, {800,  150}},
        {-180.0, {1500, 255}},
    };

    // dispatch_all reads the "started" ack inline, so we need a worker per
    // motor that responds with the standard 5-byte 0x01 ack.
    std::array<std::thread, N_MOTORS> workers;
    for (std::size_t i = 0; i < N_MOTORS; ++i) {
        workers[i] = std::thread([&, i] {
            auto req = read_n(rig.peer_fds[i], 11);
            REQUIRE(req.size() == 11);
            CHECK(req[2] == 0xF5);
            std::uint8_t body[] = {0xFB, 0x01, 0xF5, 0x01};
            std::uint8_t frame[5];
            std::memcpy(frame, body, 4);
            frame[4] = mks_servo::checksum8(body, 4);
            mock_write(rig.peer_fds[i], frame, 5);
        });
    }

    const auto s = g.dispatch_all(specs);
    for (auto& w : workers) w.join();
    CHECK(s == Transport::Status::OK);
}

// ─── wait_all_settled: returns OK once every encoder is in tolerance ─
TEST_CASE("MotorGroup::wait_all_settled returns OK when every motor reports target") {
    Rig rig;
    MotorGroup g;
    for (std::size_t i = 0; i < N_MOTORS; ++i) g.add(*rig.motors[i]);

    const std::int32_t targets[N_MOTORS] = {1000, 2000, 3000};

    // For each motor, a worker thread responds to every encoder query with
    // the target value (so wait_all_settled sees consecutive in-window reads).
    std::array<std::thread, N_MOTORS> workers;
    std::array<std::atomic<bool>, N_MOTORS> stop;
    for (std::size_t i = 0; i < N_MOTORS; ++i) stop[i] = false;

    for (std::size_t i = 0; i < N_MOTORS; ++i) {
        workers[i] = std::thread([&, i] {
            while (!stop[i].load()) {
                auto req = read_n(rig.peer_fds[i], 4, /*timeout_ms=*/50);
                if (req.empty()) continue;
                auto resp = make_encoder_response(targets[i]);
                mock_write(rig.peer_fds[i], resp.data(), resp.size());
            }
        });
    }

    const auto s = g.wait_all_settled(targets,
                                      /*tolerance=*/16,
                                      /*timeout_us=*/2'000'000,
                                      /*consecutive=*/2,
                                      /*poll_interval_us=*/0);
    for (std::size_t i = 0; i < N_MOTORS; ++i) stop[i].store(true);
    for (auto& w : workers) w.join();
    CHECK(s == MotorStatusEx::OK);
}

// ─── wait_all_settled: times out if any motor never reaches ────────
TEST_CASE("MotorGroup::wait_all_settled times out if one motor is stuck off-target") {
    Rig rig;
    MotorGroup g;
    for (std::size_t i = 0; i < N_MOTORS; ++i) g.add(*rig.motors[i]);

    const std::int32_t targets[N_MOTORS] = {1000, 2000, 3000};
    // motor 0 and 1 respond at target; motor 2 always responds off-target.
    const std::int32_t actuals[N_MOTORS] = {1000, 2000, 9999};

    std::array<std::thread, N_MOTORS> workers;
    std::array<std::atomic<bool>, N_MOTORS> stop;
    for (std::size_t i = 0; i < N_MOTORS; ++i) stop[i] = false;
    for (std::size_t i = 0; i < N_MOTORS; ++i) {
        workers[i] = std::thread([&, i] {
            while (!stop[i].load()) {
                auto req = read_n(rig.peer_fds[i], 4, /*timeout_ms=*/50);
                if (req.empty()) continue;
                auto resp = make_encoder_response(actuals[i]);
                mock_write(rig.peer_fds[i], resp.data(), resp.size());
            }
        });
    }

    MotorStatusEx per[N_MOTORS];
    const auto s = g.wait_all_settled(targets,
                                      /*tolerance=*/16,
                                      /*timeout_us=*/200'000,  // 200 ms
                                      /*consecutive=*/2,
                                      /*poll_interval_us=*/0,
                                      per);
    for (std::size_t i = 0; i < N_MOTORS; ++i) stop[i].store(true);
    for (auto& w : workers) w.join();
    CHECK(s == MotorStatusEx::Timeout);
    CHECK(per[0] == MotorStatusEx::OK);
    CHECK(per[1] == MotorStatusEx::OK);
    CHECK(per[2] == MotorStatusEx::Timeout);
}

// ─── wait_all_settled: surfaces transport error on any motor ───────
TEST_CASE("MotorGroup::wait_all_settled surfaces transport errors") {
    Rig rig;
    MotorGroup g;
    for (std::size_t i = 0; i < N_MOTORS; ++i) g.add(*rig.motors[i]);

    const std::int32_t targets[N_MOTORS] = {1000, 2000, 3000};
    // motor 0 doesn't respond at all → ReadTimeout → TransportError.

    std::array<std::thread, N_MOTORS> workers;
    std::array<std::atomic<bool>, N_MOTORS> stop;
    for (std::size_t i = 0; i < N_MOTORS; ++i) stop[i] = false;
    // Only motors 1 and 2 reply; motor 0 stays silent.
    for (std::size_t i = 1; i < N_MOTORS; ++i) {
        workers[i] = std::thread([&, i] {
            while (!stop[i].load()) {
                auto req = read_n(rig.peer_fds[i], 4, /*timeout_ms=*/50);
                if (req.empty()) continue;
                auto resp = make_encoder_response(targets[i]);
                mock_write(rig.peer_fds[i], resp.data(), resp.size());
            }
        });
    }

    MotorStatusEx per[N_MOTORS];
    const auto s = g.wait_all_settled(targets,
                                      /*tolerance=*/16,
                                      /*timeout_us=*/200'000,
                                      /*consecutive=*/2,
                                      /*poll_interval_us=*/0,
                                      per);
    for (std::size_t i = 1; i < N_MOTORS; ++i) stop[i].store(true);
    for (std::size_t i = 1; i < N_MOTORS; ++i) workers[i].join();
    CHECK(s == MotorStatusEx::TransportError);
    CHECK(per[0] == MotorStatusEx::TransportError);
}
