// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for motor.hpp: math conversions, soft limits, set_origin.
//
// Motor talks to the bus through RawDriver, which we attach via socketpair
// just like in test_raw_driver_mock. The "motor side" of the socketpair
// just responds with canned encoder values when asked.

#include "doctest/doctest.h"

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#include "mks_servo/motor.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::Motor;
using mks_servo::Mechanical;
using mks_servo::MoveParams;
using mks_servo::MotorStatusEx;
using mks_servo::OnViolation;
using mks_servo::RawDriver;
using mks_servo::Transport;

namespace {

// Mock side of socketpair: writes are fire-and-forget. We capture the
// return value and discard it, which (unlike a bare `(void)::write(...)`)
// satisfies glibc's warn_unused_result attribute on hardened toolchains.
inline void mock_write(int fd, const void* buf, std::size_t n) noexcept {
    const ssize_t r = ::write(fd, buf, n);
    (void)r;
}

struct PairedTransport {
    Transport transport;
    int       peer_fd = -1;
};

PairedTransport make_paired_transport() {
    int fds[2] = {-1, -1};
    const int rc = ::socketpair(AF_UNIX, SOCK_STREAM, 0, fds);
    REQUIRE(rc == 0);
    return PairedTransport{Transport(fds[0]), fds[1]};
}

// poll-based read with a timeout, so tests that expect "no data" don't
// hang. timeout_ms=0 means "drain whatever is already available right now".
std::vector<std::uint8_t> read_n(int fd, std::size_t n, int timeout_ms = 1000) {
    std::vector<std::uint8_t> out(n);
    std::size_t got = 0;
    while (got < n) {
        struct pollfd pfd{};
        pfd.fd     = fd;
        pfd.events = POLLIN;
        const int pr = ::poll(&pfd, 1, timeout_ms);
        if (pr <= 0) break;  // timeout or error
        const ssize_t r = ::read(fd, out.data() + got, n - got);
        if (r <= 0) break;
        got += static_cast<std::size_t>(r);
    }
    out.resize(got);
    return out;
}

// Helper: build an encoder-addition response carrying `counts` as int48 BE.
std::vector<std::uint8_t> make_encoder_response(std::int64_t counts) {
    // FB 01 31 <6 BE bytes> CRC
    std::uint8_t buf[10];
    buf[0] = 0xFB;
    buf[1] = 0x01;
    buf[2] = 0x31;
    const std::uint64_t u = static_cast<std::uint64_t>(counts) & 0x0000FFFFFFFFFFFFull;
    for (int i = 0; i < 6; ++i) {
        buf[3 + i] = static_cast<std::uint8_t>((u >> ((5 - i) * 8)) & 0xFF);
    }
    buf[9] = mks_servo::checksum8(buf, 9);
    return std::vector<std::uint8_t>(buf, buf + 10);
}

}  // namespace

// ─── pure math (no transport needed) ───────────────────────────────
TEST_CASE("Motor: angle_to_counts and counts_to_angle round-trip at gear_ratio=1") {
    int fds[2]; ::socketpair(AF_UNIX, SOCK_STREAM, 0, fds);
    Transport t(fds[0]);
    RawDriver d(t);
    Motor m(d, {/*gear_ratio=*/1.0, /*offset=*/0});

    // 90° at gear=1, offset=0 → 16384/4 = 4096 counts.
    CHECK(m.angle_to_counts(90.0) == 4096);
    CHECK(m.angle_to_counts(180.0) == 8192);
    CHECK(m.angle_to_counts(-90.0) == -4096);

    // Round-trip identity (within rounding).
    CHECK(m.counts_to_angle(m.angle_to_counts(45.0)) == doctest::Approx(45.0).epsilon(0.01));
    CHECK(m.counts_to_angle(m.angle_to_counts(-30.0)) == doctest::Approx(-30.0).epsilon(0.01));

    ::close(fds[1]);
}

TEST_CASE("Motor: gear_ratio scales motor revs vs output revs") {
    int fds[2]; ::socketpair(AF_UNIX, SOCK_STREAM, 0, fds);
    Transport t(fds[0]);
    RawDriver d(t);
    Motor m(d, {/*gear_ratio=*/4.0, /*offset=*/0});

    // gear_ratio=4: 4 motor turns = 1 output turn. So 90° output = 360° motor
    // = 16384 counts.
    CHECK(m.angle_to_counts(90.0) == 16384);
    CHECK(m.counts_to_angle(16384) == doctest::Approx(90.0));

    ::close(fds[1]);
}

TEST_CASE("Motor: origin_offset shifts the zero point") {
    int fds[2]; ::socketpair(AF_UNIX, SOCK_STREAM, 0, fds);
    Transport t(fds[0]);
    RawDriver d(t);
    Motor m(d, {/*gear_ratio=*/1.0, /*offset=*/4096});  // origin at counts=4096

    // 0° angle should now translate to counts=0 (since the firmware will
    // store target = encoder_value − offset). And reading counts=4096 should
    // give 0° (the origin).
    CHECK(m.angle_to_counts(0.0) == -4096);
    CHECK(m.counts_to_angle(4096) == doctest::Approx(0.0));

    ::close(fds[1]);
}

// ─── limits ───────────────────────────────────────────────────────
TEST_CASE("Motor: position Reject rejects out-of-range without sending a frame") {
    auto p = make_paired_transport();
    RawDriver d(p.transport);
    Motor m(d);
    m.set_position_limits(-90.0, 90.0, OnViolation::Reject);

    auto r = m.write(120.0, {}, /*blocking=*/false);
    CHECK_FALSE(r.ok());
    CHECK(r.status == MotorStatusEx::LimitExceeded);

    // Nothing should have hit the wire.
    auto leftover = read_n(p.peer_fd, 1, /*timeout_ms=*/50);
    CHECK(leftover.empty());

    ::close(p.peer_fd);
}

TEST_CASE("Motor: position Clamp silently clamps and sends the clamped target") {
    auto p = make_paired_transport();
    RawDriver d(p.transport, 0x01, 100'000);
    Motor m(d, {/*gear=*/1.0, /*offset=*/0});
    m.set_position_limits(-90.0, 90.0, OnViolation::Clamp);

    std::thread t([&] {
        auto req = read_n(p.peer_fd, 11);
        REQUIRE(req.size() == 11);
        CHECK(req[2] == 0xF5);  // MOVE_ABS_AXIS
        // Frame: head[0] addr[1] code[2] rpm_hi[3] rpm_lo[4] acc[5]
        //        counts[6..9 BE signed] crc[10]
        const std::int32_t counts = static_cast<std::int32_t>(
            (std::uint32_t(req[6]) << 24) |
            (std::uint32_t(req[7]) << 16) |
            (std::uint32_t(req[8]) << 8)  |
             std::uint32_t(req[9]));
        CHECK(counts == 4096);  // clamped from 120° to 90° → 4096 counts

        // Send the "started" ack that move_absolute_axis reads.
        std::uint8_t body[] = {0xFB, 0x01, 0xF5, 0x01};
        std::uint8_t crc = mks_servo::checksum8(body, 4);
        std::uint8_t frame[5];
        std::memcpy(frame, body, 4);
        frame[4] = crc;
        mock_write(p.peer_fd, frame, 5);
    });

    auto r = m.write(120.0, {}, /*blocking=*/false);
    t.join();
    CHECK(r.ok());
    CHECK(r.status == MotorStatusEx::OK);

    ::close(p.peer_fd);
}

TEST_CASE("Motor: speed Reject blocks an over-RPM request") {
    auto p = make_paired_transport();
    RawDriver d(p.transport);
    Motor m(d);
    m.set_speed_limit_rpm(1000, OnViolation::Reject);

    auto r = m.write(45.0, MoveParams{/*rpm=*/2000, /*acc=*/255}, /*blocking=*/false);
    CHECK_FALSE(r.ok());
    CHECK(r.status == MotorStatusEx::LimitExceeded);
    CHECK(read_n(p.peer_fd, 1, /*timeout_ms=*/50).empty());

    ::close(p.peer_fd);
}

TEST_CASE("Motor: speed Clamp clamps RPM in the frame") {
    auto p = make_paired_transport();
    RawDriver d(p.transport, 0x01, 100'000);
    Motor m(d);
    m.set_speed_limit_rpm(1000, OnViolation::Clamp);

    std::thread t([&] {
        auto req = read_n(p.peer_fd, 11);
        REQUIRE(req.size() == 11);
        // rpm is bytes 3..4 BE.
        const std::uint16_t rpm = static_cast<std::uint16_t>(
            (std::uint16_t(req[3]) << 8) | std::uint16_t(req[4]));
        CHECK(rpm == 1000);  // clamped from 2500

        std::uint8_t body[] = {0xFB, 0x01, 0xF5, 0x01};
        std::uint8_t crc = mks_servo::checksum8(body, 4);
        std::uint8_t frame[5];
        std::memcpy(frame, body, 4);
        frame[4] = crc;
        mock_write(p.peer_fd, frame, 5);
    });

    auto r = m.write(45.0, MoveParams{/*rpm=*/2500, /*acc=*/200}, /*blocking=*/false);
    t.join();
    CHECK(r.ok());

    ::close(p.peer_fd);
}

// ─── read ─────────────────────────────────────────────────────────
TEST_CASE("Motor::read returns angle in degrees from encoder counts") {
    auto p = make_paired_transport();
    RawDriver d(p.transport, 0x01, 100'000);
    Motor m(d, {/*gear=*/1.0, /*offset=*/0});

    std::thread t([&] {
        auto req = read_n(p.peer_fd, 4);
        REQUIRE(req.size() == 4);
        CHECK(req[2] == 0x31);  // READ_ENCODER_ADDITION
        // Send back counts = 4096 → angle = 90°
        auto resp = make_encoder_response(4096);
        mock_write(p.peer_fd, resp.data(), resp.size());
    });

    auto r = m.read();
    t.join();
    REQUIRE(r.ok());
    CHECK(r.value == doctest::Approx(90.0).epsilon(0.01));

    ::close(p.peer_fd);
}

// ─── set_origin (hard) ─────────────────────────────────────────────
TEST_CASE("Motor::set_origin sends SET_ZERO_POINT and resets offset to 0") {
    auto p = make_paired_transport();
    RawDriver d(p.transport, 0x01, 100'000);
    Motor m(d, {/*gear=*/1.0, /*offset=*/9999});  // preload a nonzero offset

    std::thread t([&] {
        auto req = read_n(p.peer_fd, 4);
        REQUIRE(req.size() == 4);
        CHECK(req[2] == 0x92);  // SET_ZERO_POINT

        // Send success ack: FB 01 92 01 CRC
        std::uint8_t body[] = {0xFB, 0x01, 0x92, 0x01};
        std::uint8_t crc = mks_servo::checksum8(body, 4);
        std::uint8_t frame[5];
        std::memcpy(frame, body, 4);
        frame[4] = crc;
        mock_write(p.peer_fd, frame, 5);
    });

    auto origin = m.set_origin();
    t.join();
    REQUIRE(origin.ok());
    CHECK(m.mechanical().origin_offset_counts == 0);  // reset

    ::close(p.peer_fd);
}

// ─── set_origin_soft (software only) ───────────────────────────────
TEST_CASE("Motor::set_origin_soft captures encoder counts as new zero") {
    auto p = make_paired_transport();
    RawDriver d(p.transport, 0x01, 100'000);
    Motor m(d, {/*gear=*/1.0, /*offset=*/0});

    std::thread t([&] {
        // First call: set_origin_soft — read encoder.
        (void)read_n(p.peer_fd, 4);
        auto resp = make_encoder_response(12345);
        mock_write(p.peer_fd, resp.data(), resp.size());
        // Second call: read() — same counts, but now we should see angle=0.
        (void)read_n(p.peer_fd, 4);
        mock_write(p.peer_fd, resp.data(), resp.size());
    });

    auto origin = m.set_origin_soft();
    REQUIRE(origin.ok());

    auto angle = m.read();
    t.join();
    REQUIRE(angle.ok());
    CHECK(angle.value == doctest::Approx(0.0).epsilon(0.001));

    ::close(p.peer_fd);
}

// ─── auto_clear_protection ────────────────────────────────────────
//
// When the firmware refuses a MOVE (ack payload 0x00, typically because
// stall protection has latched), Motor::write should — if auto_clear is
// enabled — issue RELEASE_PROTECTION and retry the MOVE once. The
// retry's result is what gets returned.

TEST_CASE("Motor::write returns NotEnabled on refused MOVE when auto_clear is off") {
    auto p = make_paired_transport();
    RawDriver d(p.transport, 0x01, 100'000);
    Motor m(d);
    // auto_clear defaults to false

    std::thread t([&] {
        // Read MOVE_ABS_AXIS request (11 bytes), respond with refusal (0x00).
        (void)read_n(p.peer_fd, 11);
        std::uint8_t body[] = {0xFB, 0x01, 0xF5, 0x00};  // payload 0x00 = refused
        std::uint8_t frame[5];
        std::memcpy(frame, body, 4);
        frame[4] = mks_servo::checksum8(body, 4);
        mock_write(p.peer_fd, frame, 5);
    });

    auto r = m.write(45.0, {}, /*blocking=*/false);
    t.join();
    CHECK_FALSE(r.ok());
    CHECK(r.status == MotorStatusEx::NotEnabled);
    ::close(p.peer_fd);
}

TEST_CASE("Motor::write recovers via release_protection + retry when auto_clear is on") {
    auto p = make_paired_transport();
    RawDriver d(p.transport, 0x01, 100'000);
    Motor m(d);
    m.set_auto_clear_protection(true);

    std::thread t([&] {
        // 1. First MOVE — respond with refusal (0x00).
        auto req1 = read_n(p.peer_fd, 11);
        REQUIRE(req1.size() == 11);
        CHECK(req1[2] == 0xF5);  // MOVE_ABS_AXIS
        {
            std::uint8_t body[] = {0xFB, 0x01, 0xF5, 0x00};
            std::uint8_t frame[5];
            std::memcpy(frame, body, 4);
            frame[4] = mks_servo::checksum8(body, 4);
            mock_write(p.peer_fd, frame, 5);
        }

        // 2. RELEASE_PROTECTION (0x3D, 4 bytes request).
        auto req2 = read_n(p.peer_fd, 4);
        REQUIRE(req2.size() == 4);
        CHECK(req2[2] == 0x3D);
        {
            std::uint8_t body[] = {0xFB, 0x01, 0x3D, 0x01};
            std::uint8_t frame[5];
            std::memcpy(frame, body, 4);
            frame[4] = mks_servo::checksum8(body, 4);
            mock_write(p.peer_fd, frame, 5);
        }

        // 3. Second MOVE_ABS_AXIS — respond with success (0x01).
        auto req3 = read_n(p.peer_fd, 11);
        REQUIRE(req3.size() == 11);
        CHECK(req3[2] == 0xF5);
        {
            std::uint8_t body[] = {0xFB, 0x01, 0xF5, 0x01};
            std::uint8_t frame[5];
            std::memcpy(frame, body, 4);
            frame[4] = mks_servo::checksum8(body, 4);
            mock_write(p.peer_fd, frame, 5);
        }
    });

    auto r = m.write(45.0, {}, /*blocking=*/false);
    t.join();
    CHECK(r.ok());
    CHECK(r.value);
    ::close(p.peer_fd);
}

TEST_CASE("Motor::write surfaces NotEnabled if the retry also gets refused (auto_clear on)") {
    auto p = make_paired_transport();
    RawDriver d(p.transport, 0x01, 100'000);
    Motor m(d);
    m.set_auto_clear_protection(true);

    std::thread t([&] {
        // Both MOVEs respond with refusal, plus the release in between.
        for (int i = 0; i < 2; ++i) {
            auto req = read_n(p.peer_fd, 11);
            REQUIRE(req.size() == 11);
            std::uint8_t body[] = {0xFB, 0x01, 0xF5, 0x00};
            std::uint8_t frame[5];
            std::memcpy(frame, body, 4);
            frame[4] = mks_servo::checksum8(body, 4);
            mock_write(p.peer_fd, frame, 5);
            if (i == 0) {
                auto rel = read_n(p.peer_fd, 4);
                REQUIRE(rel.size() == 4);
                std::uint8_t rb[] = {0xFB, 0x01, 0x3D, 0x01};
                std::uint8_t rf[5];
                std::memcpy(rf, rb, 4);
                rf[4] = mks_servo::checksum8(rb, 4);
                mock_write(p.peer_fd, rf, 5);
            }
        }
    });

    auto r = m.write(45.0, {}, /*blocking=*/false);
    t.join();
    CHECK_FALSE(r.ok());
    CHECK(r.status == MotorStatusEx::NotEnabled);
    ::close(p.peer_fd);
}
