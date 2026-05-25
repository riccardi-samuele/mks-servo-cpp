// SPDX-License-Identifier: Apache-2.0
//
// HIL example: exercise the Diagnostics surface against a live driver.
//
//   1. Connect / enable, verify protection is not latched at startup.
//   2. Issue a deliberately aggressive MOVE at acc=255 + rpm=3000 on the
//      unloaded shaft. With a typical unloaded NEMA17 + 12V this trips
//      the firmware's stall-protection latch.
//   3. Read protection status → expect latched.
//   4. Call clear_protection → verify it returns 0x01.
//   5. Read protection status again → expect cleared.
//   6. Read pulses_received and status_text.
//
// If your motor doesn't trip stall protection on step 2 (e.g. you're
// running a stiffer rig), the test still exercises the read paths but
// won't witness the latch-and-clear cycle.

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
    if (t.open(dev, 256000) != Transport::Status::OK) {
        std::fprintf(stderr, "open failed\n");
        return 1;
    }
    RawDriver raw(t, static_cast<std::uint8_t>(addr));
    Motor m(raw, Mechanical{1.0, 0});
    raw.enable(true);
    if (!m.set_origin().ok()) {
        std::fprintf(stderr, "set_origin failed\n");
        return 2;
    }

    Diagnostics d(m);

    // 1. Initial state ─────────────────────────────────────────────
    std::printf("=== initial state ===\n");
    auto latched0 = d.is_protection_latched();
    if (!latched0.ok()) {
        std::fprintf(stderr, "  is_protection_latched failed\n");
        return 3;
    }
    std::printf("  protection latched: %s\n",
                latched0.value ? "yes" : "no");
    std::printf("  status_text:        %s\n", d.status_text());
    auto pulses0 = d.pulses_received();
    if (pulses0.ok()) {
        std::printf("  pulses received:    %d\n", pulses0.value);
    }

    // Exercise clear_protection unconditionally. The firmware accepts
    // this even when nothing is latched, so it's a safe smoke test of
    // the API path (and clears any residual latch from a prior session).
    std::printf("\n=== clear_protection (unconditional) ===\n");
    auto clr0 = d.clear_protection();
    std::printf("  returned: ok=%s value=%s\n",
                clr0.ok() ? "yes" : "no",
                clr0.value ? "true" : "false");
    sleep_ms(200);
    auto latched_after_clr = d.is_protection_latched();
    std::printf("  protection latched after clear: %s\n",
                latched_after_clr.ok()
                    ? (latched_after_clr.value ? "yes" : "no")
                    : "(read failed)");

    // 2. Provoke a stall: aggressive params on an unloaded shaft ───
    std::printf("\n=== provoking stall (acc=255 rpm=3000) ===\n");
    const MoveParams provoke{/*rpm=*/3000, /*acc=*/255};
    auto move_r = m.write(720.0, provoke,
                          /*blocking=*/true,
                          /*tolerance=*/32,
                          /*timeout_us=*/3'000'000);
    std::printf("  move status: %d (ok=%s)\n",
                static_cast<int>(move_r.status),
                move_r.ok() ? "yes" : "no");
    sleep_ms(300);

    // 3. Read latch ────────────────────────────────────────────────
    auto latched1 = d.is_protection_latched();
    std::printf("  protection latched after stall: %s\n",
                latched1.ok() ? (latched1.value ? "yes" : "no") : "(read failed)");
    std::printf("  status_text:                    %s\n", d.status_text());

    if (latched1.ok() && latched1.value) {
        // 4. Clear ─────────────────────────────────────────────────
        std::printf("\n=== clearing protection ===\n");
        auto clr = d.clear_protection();
        std::printf("  clear_protection returned: %s (value=%s)\n",
                    clr.ok() ? "ok" : "fail",
                    clr.value ? "true" : "false");
        sleep_ms(200);

        // 5. Verify clear ──────────────────────────────────────────
        auto latched2 = d.is_protection_latched();
        std::printf("  protection latched after clear: %s\n",
                    latched2.ok() ? (latched2.value ? "still latched"
                                                    : "cleared")
                                  : "(read failed)");

        // 6. Issue a gentle move to confirm motor is happy again ──
        std::printf("\n=== recovery move (gentle params) ===\n");
        auto rec = m.write(0.0, MoveParams{600, 200},
                           /*blocking=*/true,
                           /*tolerance=*/32,
                           /*timeout_us=*/3'000'000);
        std::printf("  recovery move status: %d (ok=%s)\n",
                    static_cast<int>(rec.status), rec.ok() ? "yes" : "no");
        const auto rd = m.read();
        if (rd.ok()) std::printf("  position now: %+.3f°\n", rd.value);
    } else {
        std::printf("\n(stall did not trip on this rig — that's ok, the\n"
                    "Diagnostics read paths were still exercised.)\n");
    }

    raw.enable(false);
    t.close();
    send_set_baud(dev, 256000, 0x04);
    return 0;
}
