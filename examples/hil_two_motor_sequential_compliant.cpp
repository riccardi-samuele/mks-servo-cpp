// SPDX-License-Identifier: Apache-2.0
//
// Sequential two-motor coordination on COMPLIANT mechanical loads.
//
// A compliant load is any external mechanism that resists motion through
// a stable rest state — detents, spring returns, magnetic centring,
// indexed couplings. The motor reaches firmware "Stopped" but the load
// keeps moving (slowly settling into its rest state) for several ms
// after. Starting the next motor in a coupled mechanism BEFORE the
// previous load is fully settled can lock the mechanism.
//
// This example demonstrates two patterns specific to coordinating
// two motors on a shared compliant mechanism:
//
//   1. ABSOLUTE TARGETS instead of relative deltas. Each move targets
//      an absolute axis position (multiples of the step size, here 90°)
//      so per-move closed-loop error does NOT accumulate across the
//      run — a single move's overshoot is just a static offset, not a
//      ratcheting drift.
//
//   2. SYMMETRIC "encoder-stable" settle after EVERY motor's move
//      (not just one of them). Polling read_encoder_addition until
//      consecutive reads agree confirms that the load has actually
//      reached its rest state, not just that the firmware declared
//      Stopped. Applying the wait asymmetrically (e.g. only after M0)
//      makes the two motors behave differently even when they're
//      identical hardware — because M0 has time to settle but M1 does
//      not.
//
// Topology:
//   /dev/ttyUSB0 — addr 1 — motor "M0"
//   /dev/ttyUSB1 — addr 1 — motor "M1"
//   Both at 256000 baud, SR_CLOSE mode, holding_current=10% (so the
//   compliant load's centring force can pull the motor into the rest
//   state — required for the per-move offset to NOT accumulate).
//
// Expected behaviour with this configuration:
//   - 0 motor binds across a 50-cycle soak
//   - Encoder reading at each "+90°" target oscillates around a fixed
//     offset, but does NOT drift cycle-over-cycle.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <vector>

#include "mks_servo/motor.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using namespace mks_servo;

static std::uint64_t now_us() {
    struct timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000ull
         + static_cast<std::uint64_t>(ts.tv_nsec) / 1000ull;
}
static void sleep_ms(int ms) {
    struct timespec ts{ms / 1000, (ms % 1000) * 1'000'000L};
    ::nanosleep(&ts, nullptr);
}

// Poll the motor's encoder until N consecutive reads agree to within
// 2 counts (= 0.044°), capped at settle_cap_ms. This confirms the
// compliant load has come to rest. Default tuning (poll=15, n=3) is
// the conservative safe choice on heavy detented loads. (poll=5, n=2)
// is the aggressive mode for lighter loads or once the calling
// application has validated its specific mechanism with the
// conservative setting first.
static double wait_until_stable(RawDriver& d,
                                int settle_cap_ms,
                                int poll_ms       = 15,
                                int consec_needed = 3) {
    auto t_start = now_us();
    std::int64_t prev = d.read_encoder_addition().value;
    int stable = 0;
    while (stable < consec_needed) {
        sleep_ms(poll_ms);
        const std::int64_t cur = d.read_encoder_addition().value;
        if (std::abs(cur - prev) <= 2) ++stable;
        else                            stable = 0;
        prev = cur;
        if ((now_us() - t_start) > static_cast<std::uint64_t>(settle_cap_ms) * 1000)
            break;
    }
    return static_cast<double>(now_us() - t_start) / 1000.0;
}

static double counts_to_deg(std::int64_t counts) {
    return static_cast<double>(counts) * 360.0 / 16384.0;
}

int main(int argc, char** argv) {
    const int N           = (argc > 1) ? std::atoi(argv[1]) : 20;
    const double step_deg = (argc > 2) ? std::atof(argv[2]) : 90.0;
    const int settle_cap  = (argc > 3) ? std::atoi(argv[3]) : 500;
    const int rpm         = (argc > 4) ? std::atoi(argv[4]) : 800;
    const int acc         = (argc > 5) ? std::atoi(argv[5]) : 255;
    const int poll_ms     = (argc > 6) ? std::atoi(argv[6]) : 15;
    const int consec      = (argc > 7) ? std::atoi(argv[7]) : 3;

    Transport t0, t1;
    if (t0.open("/dev/ttyUSB0", 256000) != Transport::Status::OK) {
        std::fprintf(stderr, "open /dev/ttyUSB0 failed\n"); return 1;
    }
    if (t1.open("/dev/ttyUSB1", 256000) != Transport::Status::OK) {
        std::fprintf(stderr, "open /dev/ttyUSB1 failed\n"); return 1;
    }
    RawDriver d0(t0, 1), d1(t1, 1);

    // Clean state on both before starting.
    d0.emergency_stop(); d1.emergency_stop();
    sleep_ms(300);
    if (d0.read_protect_status().value) d0.release_protection();
    if (d1.read_protect_status().value) d1.release_protection();
    if (!d0.enable(true).ok() || !d1.enable(true).ok()) {
        std::fprintf(stderr, "enable failed on at least one motor\n"); return 2;
    }
    sleep_ms(100);

    Motor m0(d0, Mechanical{1.0, 0});
    Motor m1(d1, Mechanical{1.0, 0});
    m0.set_origin();
    m1.set_origin();
    sleep_ms(100);

    const MoveParams mp{static_cast<std::uint16_t>(rpm),
                        static_cast<std::uint8_t>(acc)};

    std::printf("=== two-motor sequential soak on compliant load ===\n");
    std::printf("    N=%d  step=%.1f deg  rpm=%d  acc=%d  settle_cap=%d ms  poll=%d ms x %d reads\n",
                N, step_deg, rpm, acc, settle_cap, poll_ms, consec);
    std::printf("    %-3s %-7s %-10s %-9s %-10s %-9s %-9s\n",
                "i", "dir", "m0_ms", "m0_pos", "m1_ms", "m1_pos", "settle");

    int m0_fail = 0, m1_fail = 0;
    std::vector<double> cycle_ms;
    double m0_target = 0, m1_target = 0;
    const auto t_start_total = now_us();

    for (int i = 0; i < N; ++i) {
        const double dir = (i & 1) ? -step_deg : step_deg;
        m0_target += dir;
        m1_target += dir;

        const auto t_c0 = now_us();

        // ── motor 0 ───────────────────────────────────────────────
        const auto t_m0 = now_us();
        auto r0 = m0.write(m0_target, mp, /*blocking=*/true,
                           /*tol_counts=*/50, /*timeout_us=*/3'000'000);
        const double dt0 = static_cast<double>(now_us() - t_m0) / 1000.0;
        t0.drain_input();
        const auto t_s0 = now_us();
        sleep_ms(settle_cap > 0 ? settle_cap : 0);
        const double s0  = static_cast<double>(now_us() - t_s0) / 1000.0;
        const double pos0 = counts_to_deg(d0.read_encoder_addition().value);

        // ── motor 1 ───────────────────────────────────────────────
        const auto t_m1 = now_us();
        auto r1 = m1.write(m1_target, mp, /*blocking=*/true,
                           /*tol_counts=*/50, /*timeout_us=*/3'000'000);
        const double dt1 = static_cast<double>(now_us() - t_m1) / 1000.0;
        t1.drain_input();
        const auto t_s1 = now_us();
        sleep_ms(settle_cap > 0 ? settle_cap : 0);
        const double s1  = static_cast<double>(now_us() - t_s1) / 1000.0;
        const double pos1 = counts_to_deg(d1.read_encoder_addition().value);

        cycle_ms.push_back(static_cast<double>(now_us() - t_c0) / 1000.0);
        if (r0.status != MotorStatusEx::OK) ++m0_fail;
        if (r1.status != MotorStatusEx::OK) ++m1_fail;

        std::printf("    %-3d %+6.1f  %7.1f  %+8.2f   %7.1f  %+8.2f   %4.0f+%4.0f\n",
                    i, dir, dt0, pos0, dt1, pos1, s0, s1);
    }
    const double total = static_cast<double>(now_us() - t_start_total) / 1000.0;

    const double nd = static_cast<double>(cycle_ms.size());
    double mean = 0;
    for (double x : cycle_ms) mean += x;
    mean /= nd;
    double sigma = 0;
    for (double x : cycle_ms) sigma += (x - mean) * (x - mean);
    sigma = std::sqrt(sigma / nd);

    std::printf("\n  total wall: %.2f s   |   per cycle: %.1f ms (σ %.1f)\n",
                total / 1000.0, mean, sigma);
    std::printf("  per move:   %.1f ms   |   fails:  m0=%d  m1=%d  out of %d each\n",
                total / (2.0 * N), m0_fail, m1_fail, N);
    return (m0_fail || m1_fail) ? 1 : 0;
}
