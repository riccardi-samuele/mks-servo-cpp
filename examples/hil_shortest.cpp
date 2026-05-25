// SPDX-License-Identifier: Apache-2.0
//
// HIL example: validate Motor::move_relative_shortest by asking for
// rotations that have a more efficient equivalent (e.g. +270° → -90°)
// and verifying the motor takes the shorter path.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "mks_servo/motor.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::build_frame;
using mks_servo::Mechanical;
using mks_servo::Motor;
using mks_servo::MoveParams;
using mks_servo::RawDriver;
using mks_servo::Transport;
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

    send_set_baud(dev, 38400, 0x07);
    Transport t;
    if (t.open(dev, 256000) != Transport::Status::OK) return 1;
    RawDriver raw(t, static_cast<std::uint8_t>(addr));
    Motor m(raw, Mechanical{1.0, 0});
    raw.enable(true);
    if (!m.set_origin().ok()) return 2;

    const MoveParams mp{/*rpm=*/600, /*acc=*/200};

    // Each pair: (requested delta, expected shortened delta).
    // We verify by reading the position before & after and checking the
    // actual rotation matches the SHORTENED version (mod 360°).
    struct Case { double request; double expect; };
    const Case cases[] = {
        {  +90.0,   +90.0},   // no shortening needed
        {  -90.0,   -90.0},   // no shortening needed
        { +270.0,   -90.0},   // long way → short way the other direction
        { -270.0,   +90.0},
        { +540.0,  -180.0},   // 1.5 turns → 0.5 turn (algorithm picks -180)
        { -180.0,  -180.0},   // edge: 180° stays 180°
    };

    int passed = 0;
    int total  = 0;

    for (auto c : cases) {
        ++total;
        const auto before = m.read();
        if (!before.ok()) {
            std::fprintf(stderr, "read before failed\n");
            continue;
        }
        std::printf("requested %+7.1f°  expected motion %+7.1f°  ",
                    c.request, c.expect);

        const auto r = m.move_relative_shortest(c.request, mp,
                                                /*blocking=*/true,
                                                /*tolerance=*/50,
                                                /*timeout_us=*/3'000'000);
        if (!r.ok()) {
            std::printf("→ FAILED (status=%d)\n", static_cast<int>(r.status));
            continue;
        }
        const auto after = m.read();
        if (!after.ok()) {
            std::printf("→ FAILED (read after)\n");
            continue;
        }
        const double actual = after.value - before.value;
        const double err    = std::fabs(actual - c.expect);
        const bool   ok     = err < 2.0;  // 2° tolerance
        std::printf("→ actual %+7.3f°  err %.3f°  %s\n",
                    actual, err, ok ? "OK" : "MISMATCH");
        if (ok) ++passed;
    }

    std::printf("\n%d/%d cases passed\n", passed, total);

    raw.enable(false);
    t.close();
    send_set_baud(dev, 256000, 0x04);
    return (passed == total) ? 0 : 3;
}
