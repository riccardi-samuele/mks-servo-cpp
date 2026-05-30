// SPDX-License-Identifier: Apache-2.0
//
// HIL test for MotorGroup. Uses a single physical motor as a group of N=1
// — the dispatch/wait paths are identical to the multi-motor case, only
// the cross-motor synchronisation skew is undefined here (which is fine,
// because with N=1 there's nothing to synchronise).
//
// Switches to 256k baud + back, performs three rounds of dispatch_all +
// wait_all_settled, prints per-round status, restores baud.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "mks_servo/motor.hpp"
#include "mks_servo/motor_group.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::build_frame;
using mks_servo::Mechanical;
using mks_servo::Motor;
using mks_servo::MotorGroup;
using mks_servo::MotorStatusEx;
using mks_servo::MoveSpec;
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

    std::printf("switch to 256k\n");
    send_set_baud(dev, 38400, 0x07);

    Transport t;
    if (t.open(dev, 256000) != Transport::Status::OK) {
        std::fprintf(stderr, "open 256k failed\n");
        return 1;
    }

    RawDriver raw(t, static_cast<std::uint8_t>(addr));
    Motor     m(raw, Mechanical{1.0, 0});

    MotorGroup g;
    g.add(m);
    std::printf("group size = %zu\n", g.size());

    // enable_all
    auto en = g.enable_all(true);
    if (!en.ok()) { std::fprintf(stderr, "enable_all failed\n"); return 2; }

    // set_origin on the single member
    auto origin = m.set_origin();
    if (!origin.ok()) { std::fprintf(stderr, "set_origin failed\n"); return 3; }

    // Three rounds of dispatch_all + wait_all_settled, with per-round status.
    const MoveSpec rounds[3][1] = {
        {{ 90.0, {600, 200}}},
        {{-90.0, {600, 200}}},
        {{  0.0, {600, 200}}},
    };

    for (int r = 0; r < 3; ++r) {
        std::int32_t targets[1] = { m.angle_to_counts(rounds[r][0].angle_deg) };

        MotorStatusEx ds_per[1];
        const auto ds = g.dispatch_all(rounds[r], ds_per);
        if (ds != MotorStatusEx::OK && ds != MotorStatusEx::LimitWarned) {
            std::fprintf(stderr, "round %d dispatch failed (status=%d, motor[0]=%d)\n",
                         r, static_cast<int>(ds), static_cast<int>(ds_per[0]));
            return 4;
        }

        MotorStatusEx ws_per[1];
        const auto ws = g.wait_all_settled(targets,
                                           /*tolerance=*/32,
                                           /*timeout_us=*/3'000'000,
                                           /*consecutive=*/2,
                                           /*poll_interval_us=*/1000,
                                           ws_per);
        if (ws != MotorStatusEx::OK) {
            std::fprintf(stderr, "round %d wait failed (motor[0] = %d)\n",
                          r, static_cast<int>(ws_per[0]));
            return 5;
        }
        const auto reached = m.read();
        std::printf("  round %d: target=%+.1f°  reached=%+.3f°\n",
                    r, rounds[r][0].angle_deg, reached.value);
    }

    // Disable and restore baud.
    g.enable_all(false);
    t.close();

    std::printf("restore 38400\n");
    send_set_baud(dev, 256000, 0x04);
    return 0;
}
