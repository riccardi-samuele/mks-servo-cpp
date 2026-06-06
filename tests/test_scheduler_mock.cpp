// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for Scheduler + MotorProfile.
//
// Covers:
//   * MotorProfile defaults + the two shipped presets carry the
//     documented values
//   * Scheduler::add installs the default profile; set_motor_profile
//     replaces it; motor_profile reads it back
//   * sched.move applies the per-motor profile to every MoveState it
//     allocates (settle_drain_ms, predispatch_drain_ms,
//     inter_move_rest_us, consecutive_in_window, expected_duration_ms)
//   * MoveHandle trigger primitives compile and set the right
//     TriggerKind on the underlying state
//   * One-move sched.run() against a socketpair-backed mock motor
//     completes within the timeout (smoke test of the worker thread)
//   * Race regression: ten consecutive single-move sched.run() calls
//     against the mock all complete (the queue-clear race used to
//     hang here from run 2 onward)
//
// The mock motor "teleports": when asked for encoder, it always
// reports the most recent move target. So sched.run() exits as soon
// as the worker has polled encoder once and seen it in-window.

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
#include "mks_servo/scheduler.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::checksum8;
using mks_servo::Mechanical;
using mks_servo::Motor;
using mks_servo::MotorProfile;
using mks_servo::MotorStatusEx;
using mks_servo::MoveParams;
using mks_servo::MoveStatus;
using mks_servo::RawDriver;
using mks_servo::Scheduler;
using mks_servo::TriggerKind;
using mks_servo::Transport;

namespace {

// Capturing write() return (vs a bare `(void)::write(...)`) satisfies
// glibc's warn_unused_result attribute on hardened toolchains.
inline void mock_write(int fd, const void* buf, std::size_t n) noexcept {
    const ssize_t r = ::write(fd, buf, n);
    (void)r;
}

// Build an encoder addition response carrying `counts` (cmd 0x31, 6-byte
// signed big-endian payload). Mirrors the firmware's reply layout.
std::array<std::uint8_t, 10> make_encoder_response(std::int64_t counts) {
    std::uint8_t buf[10];
    buf[0] = 0xFB;
    buf[1] = 0x01;
    buf[2] = 0x31;
    const std::uint64_t u = static_cast<std::uint64_t>(counts) & 0x0000FFFFFFFFFFFFull;
    for (int i = 0; i < 6; ++i) {
        buf[3 + i] = static_cast<std::uint8_t>((u >> ((5 - i) * 8)) & 0xFF);
    }
    buf[9] = checksum8(buf, 9);
    std::array<std::uint8_t, 10> out;
    std::memcpy(out.data(), buf, 10);
    return out;
}

// 5-byte ack frame for a MOVE_* command: FB 01 cmd 01 chk
std::array<std::uint8_t, 5> make_ack(std::uint8_t cmd, std::uint8_t value = 0x01) {
    std::uint8_t buf[5] = {0xFB, 0x01, cmd, value, 0x00};
    buf[4] = checksum8(buf, 4);
    std::array<std::uint8_t, 5> out;
    std::memcpy(out.data(), buf, 5);
    return out;
}

// Mock motor thread: services one peer fd.
//
// Per request type:
//   READ_ENCODER (0x31, 4-byte frame): reply with the current "position",
//     which is the current target counts (mock teleports instantly).
//   MOVE_REL_AXIS (0xF4, 11-byte frame): parse the signed 4-byte counts
//     payload, update target_counts, reply with cmd ack.
//   anything else: ack with value 0x01 so SET_*/enable/etc. don't stall.
//
// Stops when stop_flag is set.
void mock_motor_thread(int fd,
                       std::atomic<bool>& stop_flag,
                       std::atomic<std::int64_t>& target_counts) {
    std::uint8_t buf[64];
    std::size_t buf_n = 0;
    while (!stop_flag.load(std::memory_order_acquire)) {
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        const int pr = ::poll(&pfd, 1, 50);
        if (pr <= 0) continue;
        const ssize_t r = ::read(fd, buf + buf_n, sizeof(buf) - buf_n);
        if (r <= 0) continue;
        buf_n += static_cast<std::size_t>(r);

        // Try to consume frames from the start of buf.
        std::size_t i = 0;
        while (i < buf_n) {
            if (buf[i] != 0xFA) { ++i; continue; }
            if (buf_n - i < 4) break;  // minimum frame size
            const std::uint8_t cmd = buf[i + 2];
            std::size_t frame_len = 0;
            switch (cmd) {
                case 0x31: frame_len = 4;  break;  // READ_ENCODER (no data)
                case 0xF4: frame_len = 11; break;  // MOVE_REL_AXIS (rpm[2]+acc[1]+counts[4])
                case 0xF3: frame_len = 5;  break;  // ENABLE (1B data)
                default:   frame_len = 4;  break;  // best-effort
            }
            if (buf_n - i < frame_len) break;

            if (cmd == 0x31) {
                const auto resp = make_encoder_response(
                    target_counts.load(std::memory_order_acquire));
                mock_write(fd, resp.data(), resp.size());
            } else if (cmd == 0xF4) {
                // Parse signed 4-byte counts at buf[i + 6..i+9]
                std::uint32_t u_delta = 0;
                for (std::size_t k = 0; k < 4; ++k) {
                    u_delta = static_cast<std::uint32_t>(u_delta << 8)
                            | static_cast<std::uint32_t>(buf[i + 6 + k]);
                }
                const auto delta = static_cast<std::int32_t>(u_delta);
                target_counts.fetch_add(delta, std::memory_order_acq_rel);
                const auto resp = make_ack(0xF4);
                mock_write(fd, resp.data(), resp.size());
            } else {
                const auto resp = make_ack(cmd);
                mock_write(fd, resp.data(), resp.size());
            }
            i += frame_len;
        }
        if (i > 0) {
            std::memmove(buf, buf + i, buf_n - i);
            buf_n -= i;
        }
    }
}

struct MockRig {
    Transport  transport;
    RawDriver* raw     = nullptr;
    Motor*     motor   = nullptr;
    int        peer_fd = -1;
    std::atomic<bool>         stop{false};
    std::atomic<std::int64_t> target{0};
    std::thread               thread;

    MockRig() {
        int fds[2] = {-1, -1};
        REQUIRE(::socketpair(AF_UNIX, SOCK_STREAM, 0, fds) == 0);
        transport = Transport(fds[0]);
        peer_fd   = fds[1];
        raw   = new RawDriver(transport, /*addr=*/1, /*timeout=*/200'000);
        motor = new Motor(*raw, Mechanical{1.0, 0});
        thread = std::thread(mock_motor_thread, peer_fd,
                             std::ref(stop), std::ref(target));
    }

    ~MockRig() {
        stop.store(true, std::memory_order_release);
        if (thread.joinable()) thread.join();
        delete motor;
        delete raw;
        if (peer_fd >= 0) ::close(peer_fd);
    }
};

}  // namespace

TEST_CASE("MotorProfile default values match library-wide defaults") {
    const MotorProfile p;
    CHECK(p.settle_drain_ms == 30);
    CHECK(p.predispatch_drain_ms == 5);
    CHECK(p.inter_move_rest_us == 100'000);
    CHECK(p.consecutive_in_window == 2);
    CHECK(p.t_90deg_ms == doctest::Approx(0.0));
}

TEST_CASE("MotorProfile presets carry the documented values") {
    SUBCASE("V1.0.9 SR_CLOSE") {
        const auto p = MotorProfile::for_v1_0_9_sr_close();
        CHECK(p.settle_drain_ms == 30);
        CHECK(p.predispatch_drain_ms == 5);
        CHECK(p.inter_move_rest_us == 5'000);     // 20x shorter
        CHECK(p.consecutive_in_window == 2);
        CHECK(p.t_90deg_ms == doctest::Approx(40.0));
    }
    SUBCASE("V1.0.8 SR_vFOC") {
        const auto p = MotorProfile::for_v1_0_8_sr_vfoc();
        CHECK(p.settle_drain_ms == 30);
        CHECK(p.predispatch_drain_ms == 5);
        CHECK(p.inter_move_rest_us == 100'000);
        CHECK(p.consecutive_in_window == 2);
        CHECK(p.t_90deg_ms == doctest::Approx(43.0));
    }
}

TEST_CASE("Scheduler::add installs the default profile") {
    MockRig rig;
    Scheduler sched;
    sched.add(*rig.motor);
    const auto& p = sched.motor_profile(*rig.motor);
    CHECK(p.inter_move_rest_us == 100'000);
    CHECK(p.t_90deg_ms == doctest::Approx(0.0));
}

TEST_CASE("Scheduler::set_motor_profile replaces the per-motor profile") {
    MockRig rig;
    Scheduler sched;
    sched.add(*rig.motor);
    sched.set_motor_profile(*rig.motor, MotorProfile::for_v1_0_9_sr_close());
    const auto& p = sched.motor_profile(*rig.motor);
    CHECK(p.inter_move_rest_us == 5'000);
    CHECK(p.t_90deg_ms == doctest::Approx(40.0));
}

TEST_CASE("Scheduler::move copies the profile values onto every MoveState") {
    MockRig rig;
    Scheduler sched;
    sched.add(*rig.motor);

    SUBCASE("default profile applied") {
        auto h = sched.move(*rig.motor, 45.0, MoveParams{2000, 255});
        const auto* s = h.state();
        CHECK(s->settle_drain_ms == 30);
        CHECK(s->predispatch_drain_ms == 5);
        CHECK(s->inter_move_rest_us == 100'000);
        CHECK(s->consecutive_in_window == 2);
        CHECK(s->expected_duration_ms == doctest::Approx(0.0));  // unknown
    }

    SUBCASE("V1.0.9 preset applied; expected_duration scales with angle") {
        sched.set_motor_profile(*rig.motor,
                                MotorProfile::for_v1_0_9_sr_close());
        auto h = sched.move(*rig.motor, 180.0, MoveParams{2000, 255});
        const auto* s = h.state();
        CHECK(s->inter_move_rest_us == 5'000);
        // 180° at 40 ms/90° = 80 ms
        CHECK(s->expected_duration_ms == doctest::Approx(80.0));
    }
}

TEST_CASE("MoveHandle trigger primitives set TriggerKind on state") {
    MockRig rig;
    Scheduler sched;
    sched.add(*rig.motor);

    SUBCASE(".after") {
        auto h0 = sched.move(*rig.motor, 90, {2000, 255});
        auto h1 = sched.move(*rig.motor, 90, {2000, 255}).after(h0);
        CHECK(h1.state()->trigger_kind == TriggerKind::AfterCompleted);
        CHECK(h1.state()->trigger_ref == h0.state());
    }
    SUBCASE(".at_progress") {
        auto h0 = sched.move(*rig.motor, 90, {2000, 255});
        auto h1 = sched.move(*rig.motor, 90, {2000, 255}).at_progress(h0, 0.5);
        CHECK(h1.state()->trigger_kind == TriggerKind::AtProgress);
        CHECK(h1.state()->trigger_val == doctest::Approx(0.5));
    }
    SUBCASE(".at_time_after_start") {
        auto h0 = sched.move(*rig.motor, 90, {2000, 255});
        auto h1 = sched.move(*rig.motor, 90, {2000, 255})
                       .at_time_after_start(h0, 20.0);
        CHECK(h1.state()->trigger_kind == TriggerKind::AtTimeAfterStart);
        CHECK(h1.state()->trigger_val == doctest::Approx(20.0));
    }
    SUBCASE(".at_time_before_end") {
        auto h0 = sched.move(*rig.motor, 90, {2000, 255})
                       .with_expected_duration_ms(40.0);
        auto h1 = sched.move(*rig.motor, 90, {2000, 255})
                       .at_time_before_end(h0, 5.0);
        CHECK(h1.state()->trigger_kind == TriggerKind::AtTimeBeforeEnd);
        CHECK(h1.state()->trigger_val == doctest::Approx(5.0));
    }
}

TEST_CASE("Scheduler::run completes a single move against a mock motor") {
    MockRig rig;
    Scheduler sched;
    sched.add(*rig.motor);
    // Tighten inter_move_rest so the test doesn't burn 100 ms.
    auto prof = sched.motor_profile(*rig.motor);
    prof.inter_move_rest_us = 1'000;
    sched.set_motor_profile(*rig.motor, prof);

    auto h = sched.move(*rig.motor, 90.0, MoveParams{2000, 255});
    const auto worst = sched.run();
    CHECK(worst == MotorStatusEx::OK);
    CHECK(h.state()->status.load() == MoveStatus::Completed);
    CHECK(h.state()->t_end_us.load() > h.state()->t_start_us.load());
}

TEST_CASE("Scheduler: ten back-to-back single-move runs do not hang "
          "(queue-clear race regression)") {
    MockRig rig;
    Scheduler sched;
    sched.add(*rig.motor);
    auto prof = sched.motor_profile(*rig.motor);
    prof.inter_move_rest_us = 1'000;
    sched.set_motor_profile(*rig.motor, prof);

    for (int i = 0; i < 10; ++i) {
        sched.reset();
        auto h = sched.move(*rig.motor, 90.0 * ((i % 2) ? -1 : +1),
                            MoveParams{2000, 255});
        const auto worst = sched.run();
        REQUIRE(worst == MotorStatusEx::OK);
        REQUIRE(h.state()->status.load() == MoveStatus::Completed);
    }
}
