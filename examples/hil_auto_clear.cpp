// SPDX-License-Identifier: Apache-2.0
//
// HIL example: validate Motor's auto_clear_protection option.
//
// Two passes over the same provocation:
//   A) auto_clear OFF (default): if the firmware latches protection
//      during the move, Motor::write returns NotEnabled and stays
//      stuck — subsequent moves keep failing until the caller clears
//      the latch.
//   B) auto_clear ON: if the firmware latches, Motor internally calls
//      release_protection and retries the MOVE once. The caller sees
//      a clean return.
//
// If the firmware never latches on this particular rig (the unloaded
// development NEMA17 is inconsistent about it), both passes succeed
// and the example just confirms the API path is wire-safe. The real
// value of auto_clear shows up under load.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "mks_servo/diagnostics.hpp"
#include "mks_servo/motor.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::build_frame;
using mks_servo::Diagnostics;
using mks_servo::Mechanical;
using mks_servo::Motor;
using mks_servo::MotorStatusEx;
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

static const char* mstatus(MotorStatusEx s) {
    switch (s) {
        case MotorStatusEx::OK:             return "OK";
        case MotorStatusEx::TransportError: return "TransportError";
        case MotorStatusEx::ParseError:     return "ParseError";
        case MotorStatusEx::LimitExceeded:  return "LimitExceeded";
        case MotorStatusEx::LimitWarned:    return "LimitWarned";
        case MotorStatusEx::Timeout:        return "Timeout";
        case MotorStatusEx::NotEnabled:     return "NotEnabled";
    }
    return "?";
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
    Diagnostics d(m);

    // Provocation: aggressive params that have been observed to latch
    // stall protection on the unloaded NEMA17 + 12V rig.
    const MoveParams provoke{/*rpm=*/3000, /*acc=*/255};
    const double     target = 540.0;

    auto run_one = [&](const char* label, bool auto_clear) {
        std::printf("\n=== %s (auto_clear=%s) ===\n",
                    label, auto_clear ? "true" : "false");
        m.set_auto_clear_protection(auto_clear);
        // Make sure we start clean.
        d.clear_protection();
        sleep_ms(100);
        m.set_origin();

        for (int i = 0; i < 3; ++i) {
            const auto r = m.write(target * ((i % 2) ? -1.0 : 1.0),
                                   provoke,
                                   /*blocking=*/true,
                                   /*tolerance=*/50,
                                   /*timeout_us=*/3'000'000);
            const auto latched = d.is_protection_latched();
            std::printf("  attempt %d: status=%s  latched_after=%s\n",
                        i, mstatus(r.status),
                        latched.ok() ? (latched.value ? "yes" : "no") : "?");
            sleep_ms(200);
        }
    };

    run_one("A: no auto-clear", false);
    run_one("B: with auto-clear", true);

    raw.enable(false);
    t.close();
    send_set_baud(dev, 256000, 0x04);
    return 0;
}
