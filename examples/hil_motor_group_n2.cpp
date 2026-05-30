// SPDX-License-Identifier: Apache-2.0
//
// HIL example: real multi-motor MotorGroup over N=2 independent USB-RS485
// buses. This is the canonical pattern to copy when you have multiple
// motors each on its own adapter — the "N motors, N buses" topology of
// the v1 library.
//
// What this example demonstrates:
//   1. Independent Transport per bus, probed for whichever baud each
//      motor is at (motors may legitimately differ — one freshly power-
//      cycled at 38400, another configured at 256000).
//   2. Building a MotorGroup over those motors, no shared bus.
//   3. enable_all + per-motor set_origin + move_all with per-motor
//      outcome inspection — the right way to drive a group.
//   4. How error reporting surfaces when one motor refuses a command
//      (firmware ack 0x00 -> NotEnabled): the failure is caught at
//      dispatch in milliseconds, NOT discovered as a timeout after the
//      full wait_all_settled budget.
//
// Power: this example uses gentle move params (rpm=600 acc=128) so two
// motors accelerating simultaneously stay within a modest current
// budget. If you have a beefier supply, bump acc/rpm after verifying
// thermal/electrical headroom.
//
// Stable identification: on a multi-adapter rig, the OS-assigned
// /dev/ttyUSBn order is NOT stable across reboots / hot-plug. Two
// stable schemes exist under /dev/serial/:
//   - by-id/usb-FTDI_FT232R_USB_UART_<SERIAL>-if00-port0
//   - by-path/pci-...-usb-0:<PORT>:1.0-port0
// PREFER by-path: many low-cost FTDI clones ship with the SAME factory
// serial number (e.g. "A5069RR4") across units, so by-id can collide
// and silently point to "one of them" non-deterministically. by-path
// is keyed on the physical USB port and is always unique. Pass these
// stable paths as argv to make "motor A = bus X" deterministic.
//
// Usage:
//   hil_motor_group_n2 [dev0] [dev1] [rounds]
// Defaults: /dev/ttyUSB0 /dev/ttyUSB1 4

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "mks_servo/motor.hpp"
#include "mks_servo/motor_group.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::Mechanical;
using mks_servo::Motor;
using mks_servo::MotorGroup;
using mks_servo::MotorStatusEx;
using mks_servo::MoveParams;
using mks_servo::MoveSpec;
using mks_servo::RawDriver;
using mks_servo::Result;
using mks_servo::Transport;

static std::uint64_t now_us() {
    struct timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000ull
         + static_cast<std::uint64_t>(ts.tv_nsec) / 1000ull;
}

static void sleep_ms(int ms) {
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1'000'000L;
    ::nanosleep(&ts, nullptr);
}

static const char* status_text(MotorStatusEx s) {
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

// Probe which baud the motor at `dev` is responding at. Tries 256000
// first (the operating point most callers normalise to), then the
// firmware default 38400. Returns 0 on no response.
static int probe_baud(const char* dev) {
    Transport t;
    for (int b : {256000, 38400}) {
        if (t.open(dev, b) != Transport::Status::OK) continue;
        RawDriver p(t, 1);
        auto e = p.read_encoder_addition();
        if (e.ok()) return b;
        t.close();
    }
    return 0;
}

int main(int argc, char** argv) {
    const char* dev0     = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const char* dev1     = (argc > 2) ? argv[2] : "/dev/ttyUSB1";
    const int   rounds_n = (argc > 3) ? std::atoi(argv[3]) : 4;

    // ── 1. Probe each bus independently ──────────────────────────────
    std::printf("=== probing buses ===\n");
    const int b0 = probe_baud(dev0);
    const int b1 = probe_baud(dev1);
    if (!b0 || !b1) {
        std::fprintf(stderr,
            "no response: %s=%d, %s=%d (check wiring/power/baud)\n",
            dev0, b0, dev1, b1);
        return 1;
    }
    std::printf("  %s @ %d baud\n", dev0, b0);
    std::printf("  %s @ %d baud\n", dev1, b1);

    // ── 2. One Transport per bus, one Motor on each. ─────────────────
    // Each Transport opens at its own baud; this example does NOT try
    // to normalise to 256k. If you want all motors at the same baud,
    // send SET_BAUD (firmware cmd 0x8A) to each before this stage.
    Transport t0, t1;
    if (t0.open(dev0, b0) != Transport::Status::OK) {
        std::fprintf(stderr, "open %s failed\n", dev0); return 2;
    }
    if (t1.open(dev1, b1) != Transport::Status::OK) {
        std::fprintf(stderr, "open %s failed\n", dev1); return 2;
    }

    RawDriver r0(t0, /*addr=*/1);
    RawDriver r1(t1, /*addr=*/1);
    Motor m0(r0, Mechanical{1.0, 0});
    Motor m1(r1, Mechanical{1.0, 0});

    MotorGroup g;
    g.add(m0);
    g.add(m1);
    std::printf("group size = %zu\n", g.size());

    // ── 3. enable_all with per-motor inspection ──────────────────────
    // enable_all stops at the first transport-level failure (since
    // those almost always mean a wiring / power problem that blocks
    // everything) but RECORDS firmware-level refusals for every motor
    // and keeps going. Inspect the per-motor array to know which
    // motors are actually enabled.
    std::printf("\n=== enable_all ===\n");
    Result<bool> en_per[2];
    auto en = g.enable_all(true, en_per);
    std::printf("  aggregate: ok=%d value=%d\n", en.ok(), (int)en.value);
    bool any_enabled = false;
    for (std::size_t i = 0; i < g.size(); ++i) {
        std::printf("  motor[%zu]: ok=%d firmware_ack=%d\n",
                    i, en_per[i].ok(), (int)en_per[i].value);
        any_enabled = any_enabled || (en_per[i].ok() && en_per[i].value);
    }
    if (!any_enabled) {
        std::fprintf(stderr, "no motor enabled; aborting\n");
        return 3;
    }

    // ── 4. set_origin per motor (no group method by design) ──────────
    // SET_ZERO_POINT is a per-motor decision; the group doesn't try to
    // batch it because zero-point semantics are mechanical, not part of
    // a coordinated motion command.
    std::printf("\n=== set_origin per motor ===\n");
    for (std::size_t i = 0; i < g.size(); ++i) {
        auto o = g[i].set_origin();
        std::printf("  motor[%zu]: status=%s value=%d\n",
                    i, status_text(o.status), (int)o.value);
    }

    // ── 5. N rounds of synchronized move_all (gentle params) ─────────
    // Power-safe defaults: rpm=600 acc=128 keeps current peaks modest
    // even with both motors accelerating at the same instant. The
    // dispatch is sequential (~1.4 ms per motor at 256 kbaud) so the
    // motors start within a couple of ms of each other; for sub-ms
    // sync you'd run one thread per motor and release them on a
    // barrier — that's caller code, not MotorGroup's job.
    const MoveParams mp{600, 128};
    int dir = +1;
    int ok_rounds = 0, dispatch_fail_rounds = 0, wait_fail_rounds = 0;

    std::printf("\n=== %d rounds of move_all (rpm=%d acc=%d) ===\n",
                rounds_n, (int)mp.rpm, (int)mp.acc);
    for (int r = 0; r < rounds_n; ++r) {
        // Read each motor's current position; compute target = cur + 90*dir
        // independently per motor (since they may be at different starts).
        auto p0 = g[0].read();
        auto p1 = g[1].read();
        const double a0 = p0.ok() ? p0.value : 0.0;
        const double a1 = p1.ok() ? p1.value : 0.0;
        const MoveSpec specs[2] = {
            { a0 + 90.0 * dir, mp },
            { a1 + 90.0 * dir, mp },
        };

        MotorStatusEx per[2];
        const std::uint64_t t_start = now_us();
        const auto agg = g.move_all(specs,
                                    /*tol=*/50,
                                    /*timeout_us=*/3'000'000,
                                    /*consec=*/2,
                                    /*poll_us=*/0,
                                    per);
        const std::uint64_t t_end = now_us();
        const double wall_ms = static_cast<double>(t_end - t_start) / 1000.0;

        auto end0 = g[0].read();
        auto end1 = g[1].read();
        std::printf("  round %d (dir=%+d) wall=%7.2f ms  agg=%s\n",
                    r, dir, wall_ms, status_text(agg));
        std::printf("    motor[0]: per=%-15s tgt=%+7.1f° actual=%+7.1f°\n",
                    status_text(per[0]), specs[0].angle_deg,
                    end0.ok() ? end0.value : -9999.0);
        std::printf("    motor[1]: per=%-15s tgt=%+7.1f° actual=%+7.1f°\n",
                    status_text(per[1]), specs[1].angle_deg,
                    end1.ok() ? end1.value : -9999.0);

        if (agg == MotorStatusEx::OK || agg == MotorStatusEx::LimitWarned) {
            ++ok_rounds;
        } else if (per[0] == MotorStatusEx::OK
                && per[1] == MotorStatusEx::OK) {
            // Shouldn't happen, but defensive.
            ++ok_rounds;
        } else {
            // Categorise the failure for the summary.
            const bool dispatch_failed =
                (per[0] == MotorStatusEx::NotEnabled
              || per[0] == MotorStatusEx::TransportError
              || per[0] == MotorStatusEx::ParseError
              || per[1] == MotorStatusEx::NotEnabled
              || per[1] == MotorStatusEx::TransportError
              || per[1] == MotorStatusEx::ParseError);
            if (dispatch_failed) ++dispatch_fail_rounds; else ++wait_fail_rounds;
        }
        dir = -dir;
        sleep_ms(150);
    }

    std::printf("\n=== summary ===\n");
    std::printf("  successful rounds:        %d / %d\n", ok_rounds, rounds_n);
    std::printf("  dispatch-failure rounds:  %d\n", dispatch_fail_rounds);
    std::printf("  wait-failure rounds:      %d\n", wait_fail_rounds);

    g.enable_all(false);
    return ok_rounds == rounds_n ? 0 : 4;
}
