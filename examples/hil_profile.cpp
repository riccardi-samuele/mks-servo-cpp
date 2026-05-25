// SPDX-License-Identifier: Apache-2.0
//
// HIL example: construct a Profile in code, apply it to a Motor, and
// drive the motor through it.
//
// The library doesn't bundle a YAML / JSON parser. The Profile struct
// is plain data, so any source — a YAML file via yaml-cpp, a JSON
// payload via nlohmann/json, hard-coded constants like in this file,
// or a database row — can fill it.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "mks_servo/motor.hpp"
#include "mks_servo/profile.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::apply_profile_to_motor;
using mks_servo::build_frame;
using mks_servo::Direction;
using mks_servo::Mechanical;
using mks_servo::Motor;
using mks_servo::MoveParams;
using mks_servo::OnViolation;
using mks_servo::PositionLimits;
using mks_servo::Profile;
using mks_servo::RawDriver;
using mks_servo::SpeedLimits;
using mks_servo::Transport;
using mks_servo::WorkMode;
using mks_servo::op::SET_BAUD;

static void sleep_ms(int ms) {
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1'000'000L;
    ::nanosleep(&ts, nullptr);
}

static bool send_set_baud(const char* dev, int baud, std::uint8_t code) {
    Transport t;
    if (t.open(dev, baud) != Transport::Status::OK) return false;
    std::uint8_t req[16];
    std::uint8_t data[1] = {code};
    const auto n = build_frame(req, sizeof(req), 1, SET_BAUD, data, 1);
    if (t.write_all(req, n) != Transport::Status::OK) return false;
    sleep_ms(200);
    return true;
}

int main(int argc, char** argv) {
    const char* dev  = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const int   addr = (argc > 2) ? std::atoi(argv[2]) : 1;

    // Hand-rolled Profile, same shape as the Python lib's YAML profile.
    // In a real application you'd load this from a file via your
    // YAML / JSON / TOML library of choice.
    Profile prof;
    prof.transport.port   = dev;
    prof.transport.baud   = 256000;
    prof.config.mode      = WorkMode::SR_vFOC;
    prof.config.microsteps      = 16;
    prof.config.work_current_ma = 1500;
    prof.config.direction       = Direction::CW;
    prof.config.slave_addr      = static_cast<std::uint8_t>(addr);
    prof.mechanical.gear_ratio  = 1.0;
    // Soft limits: refuse motion outside ±180°, clamp speed at 1500 rpm.
    prof.position_limits.enabled = true;
    prof.position_limits.min_deg = -180.0;
    prof.position_limits.max_deg = +180.0;
    prof.position_limits.policy  = OnViolation::Reject;
    prof.speed_limits.enabled    = true;
    prof.speed_limits.max_rpm    = 1500;
    prof.speed_limits.policy     = OnViolation::Clamp;

    send_set_baud(dev, 38400, 0x07);
    Transport t;
    if (t.open(dev, prof.transport.baud) != Transport::Status::OK) return 1;
    RawDriver raw(t, prof.config.slave_addr);
    Motor m(raw, Mechanical{prof.mechanical});
    raw.enable(true);
    if (!m.set_origin().ok()) return 2;

    // Apply the in-memory parts of the profile.
    apply_profile_to_motor(prof, m);
    std::printf("Profile applied:\n");
    std::printf("  gear_ratio:      %.3f\n", m.mechanical().gear_ratio);
    std::printf("  position limits: [%+.1f, %+.1f] (%s)\n",
                m.position_limits().min_deg,
                m.position_limits().max_deg,
                m.position_limits().policy == OnViolation::Reject ? "reject"
                : m.position_limits().policy == OnViolation::Clamp ? "clamp"
                : "warn");
    std::printf("  speed limit:     %u rpm (%s)\n",
                m.speed_limits().max_rpm,
                m.speed_limits().policy == OnViolation::Reject ? "reject"
                : m.speed_limits().policy == OnViolation::Clamp ? "clamp"
                : "warn");

    // Demo 1: a legal move (within position limits) — should succeed.
    std::printf("\nmoving to +90° at rpm=600 (legal)…\n");
    auto r1 = m.write(90.0, MoveParams{600, 200},
                      /*blocking=*/true,
                      /*tolerance=*/50,
                      /*timeout_us=*/3'000'000);
    std::printf("  status=%d  reached=%+.3f°\n",
                static_cast<int>(r1.status), m.read().value);

    // Demo 2: a move OUTSIDE the position limit (190°). Reject policy
    // should refuse before any wire activity.
    std::printf("\nrequesting +190° (outside ±180° reject window)…\n");
    auto r2 = m.write(190.0, MoveParams{600, 200},
                      /*blocking=*/false);
    std::printf("  status=%d  ok=%s  (LimitExceeded == 3)\n",
                static_cast<int>(r2.status),
                r2.ok() ? "yes" : "no");

    // Demo 3: a move at rpm above speed limit. Clamp policy should
    // silently lower it to the limit and proceed.
    std::printf("\nrequesting move at rpm=2500 (above 1500 clamp)…\n");
    auto r3 = m.write(0.0, MoveParams{2500, 200},
                      /*blocking=*/true,
                      /*tolerance=*/50,
                      /*timeout_us=*/3'000'000);
    std::printf("  status=%d  reached=%+.3f°\n",
                static_cast<int>(r3.status), m.read().value);

    raw.enable(false);
    t.close();
    send_set_baud(dev, 256000, 0x04);
    return 0;
}
