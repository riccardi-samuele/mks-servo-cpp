// SPDX-License-Identifier: Apache-2.0
//
// HIL experiment: probe how the MKS SERVO42D firmware handles a
// MOVE_ABS_AXIS sent while a previous MOVE_ABS_AXIS is still active.
// The firmware spec claims "supports real-time updates" — this
// example characterizes WHEN that's actually true.
//
// Four scenarios with identical FINAL target (720°):
//   A) Baseline: single MOVE_ABS_AXIS(720°).
//   B) Mid retarget: MOVE_ABS_AXIS(360°), then 150 ms later
//      MOVE_ABS_AXIS(720°).
//   C) Early retarget: same as B but the second MOVE goes out at 50 ms.
//   D) Late retarget: same as B but the second MOVE goes out at 400 ms
//      (close to when the firmware would otherwise start decelerating
//      for the 360° target).
//
// Encoder is sampled at ~1 ms (256k baud) throughout each move.
//
// Result on the development rig (NEMA17 + 12V/3A, firmware V1.0.6):
//   A  ≈ 1062 ms  (baseline)
//   B  ≈ 2700 ms  (firmware enters slow creep mode)
//   C  ≈ 2700 ms  (firmware enters slow creep mode)
//   D  ≈   940 ms (firmware honors the update; slightly FASTER than A
//                  because the deceleration phase is skipped)
//
// Conclusion: mid-motion update is real but only works in a narrow
// window — specifically when the second command lands close to the
// time the firmware would naturally start decelerating. See
// docs/design.md §6 for the takeaway and the implication for users.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <vector>

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

// Run one move scenario.
//   - If `chained_intermediate_deg` > 0, dispatch the move to that target
//     first, then partway through (at `chain_after_us`) dispatch the
//     final target.
//   - Otherwise dispatch directly to final_deg.
// Samples encoder + timestamp throughout; returns the time-to-target.
struct ScenarioResult {
    double      total_ms = 0;
    std::size_t samples  = 0;
    double      peak_rpm = 0;
    bool        ok       = false;
};

static ScenarioResult run_scenario(Motor&       m,
                                   RawDriver&   raw,
                                   const char*  label,
                                   double       final_deg,
                                   MoveParams   mp,
                                   double       chained_intermediate_deg = 0.0,
                                   std::uint64_t chain_after_us           = 0) {
    ScenarioResult r;
    std::printf("\n=== scenario %s: target=%+.0f°", label, final_deg);
    if (chained_intermediate_deg != 0.0) {
        std::printf(", intermediate=%+.0f° then retarget after %llu us",
                    chained_intermediate_deg,
                    static_cast<unsigned long long>(chain_after_us));
    }
    std::printf(" ===\n");

    if (!m.set_origin().ok()) {
        std::fprintf(stderr, "set_origin failed\n");
        return r;
    }

    const std::int32_t target_counts = m.angle_to_counts(final_deg);
    const std::int32_t tolerance     = 32;

    // Capture samples throughout the motion.
    struct Sample {
        std::uint64_t t_us;
        std::int64_t  counts;
    };
    std::vector<Sample> samples;
    samples.reserve(4096);

    bool dispatched_final = (chained_intermediate_deg == 0.0);

    const std::uint64_t t0 = now_us();

    // Dispatch first move.
    Transport::Status s;
    if (chained_intermediate_deg != 0.0) {
        const std::int32_t mid_counts = m.angle_to_counts(chained_intermediate_deg);
        s = raw.dispatch_move_absolute_axis(mid_counts, mp.rpm, mp.acc);
    } else {
        s = raw.dispatch_move_absolute_axis(target_counts, mp.rpm, mp.acc);
    }
    if (s != Transport::Status::OK) {
        std::fprintf(stderr, "dispatch failed\n");
        return r;
    }

    // Polling loop.
    int  in_window = 0;
    std::uint64_t deadline = t0 + 3'000'000ull;  // 3 s safety
    while (true) {
        const std::uint64_t now = now_us();
        if (now > deadline) {
            std::fprintf(stderr, "timeout\n");
            return r;
        }

        // Mid-motion retarget?
        if (!dispatched_final && (now - t0) >= chain_after_us) {
            const auto s2 = raw.dispatch_move_absolute_axis(target_counts,
                                                            mp.rpm, mp.acc);
            if (s2 != Transport::Status::OK) {
                std::fprintf(stderr, "retarget dispatch failed\n");
                return r;
            }
            std::printf("  retarget sent at t=%llu us\n",
                        static_cast<unsigned long long>(now - t0));
            dispatched_final = true;
        }

        const auto er = raw.read_encoder_addition();
        if (!er.ok()) {
            // Could be a stray firmware ack at the bus boundary; tolerate one.
            continue;
        }
        samples.push_back({now - t0, er.value});

        const std::int64_t diff  = er.value - target_counts;
        const std::int64_t adiff = diff < 0 ? -diff : diff;
        if (adiff <= tolerance) {
            if (++in_window >= 2) break;
        } else {
            in_window = 0;
        }
    }
    const std::uint64_t t1 = now_us();
    raw.transport_drain_settle(20);

    r.total_ms = static_cast<double>(t1 - t0) / 1000.0;
    r.samples  = samples.size();
    r.ok       = true;

    // Compute peak rpm from samples: max |Δcounts/Δt| × 60 / 16384.
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const std::int64_t dc = samples[i].counts - samples[i - 1].counts;
        const std::uint64_t dt = samples[i].t_us - samples[i - 1].t_us;
        if (dt == 0) continue;
        const double rpm = std::abs(static_cast<double>(dc)) / 16384.0
                         * 60'000'000.0 / static_cast<double>(dt);
        if (rpm > r.peak_rpm) r.peak_rpm = rpm;
    }

    std::printf("  total time:    %.2f ms\n", r.total_ms);
    std::printf("  samples:       %zu\n", r.samples);
    std::printf("  peak rpm est.: %.0f\n", r.peak_rpm);
    std::printf("  encoder traj (every ~50 ms):\n");
    const std::uint64_t step_us = 50'000;
    std::uint64_t next_print = 0;
    for (const auto& s_ : samples) {
        if (s_.t_us >= next_print) {
            const double angle = m.counts_to_angle(s_.counts);
            std::printf("    t=%5llu us  enc=%6lld  angle=%+8.2f°\n",
                        static_cast<unsigned long long>(s_.t_us),
                        static_cast<long long>(s_.counts), angle);
            next_print = s_.t_us + step_us;
        }
    }
    return r;
}

int main(int argc, char** argv) {
    const char* dev  = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const int   addr = (argc > 2) ? std::atoi(argv[2]) : 1;

    send_set_baud(dev, 38400, 0x07);  // → 256k

    Transport t;
    if (t.open(dev, 256000) != Transport::Status::OK) {
        std::fprintf(stderr, "open 256k failed\n");
        return 1;
    }
    RawDriver raw(t, static_cast<std::uint8_t>(addr));
    Motor m(raw, Mechanical{1.0, 0});

    raw.enable(true);

    // Conservative params that work in soak (avoid stall protection).
    const MoveParams mp{/*rpm=*/600, /*acc=*/200};

    // Final target = 720° (two revolutions). Far enough that the motor
    // reaches stable cruise; intermediate at 360° (one revolution) is
    // roughly half the move.
    const double final_deg = 720.0;
    const double mid_deg   = 360.0;

    // Baseline: one move to 720°.
    auto a = run_scenario(m, raw, "A baseline (single 720°)",
                          final_deg, mp);
    sleep_ms(500);

    // Chained variant 1: retarget at t=150 ms (mid-motion).
    auto b = run_scenario(m, raw, "B chained mid (retarget at 150 ms)",
                          final_deg, mp,
                          /*intermediate=*/mid_deg,
                          /*chain_after=*/150'000ull);
    sleep_ms(500);

    // Chained variant 2: retarget early (t=50 ms — barely past dispatch).
    auto c = run_scenario(m, raw, "C chained early (retarget at 50 ms)",
                          final_deg, mp,
                          /*intermediate=*/mid_deg,
                          /*chain_after=*/50'000ull);
    sleep_ms(500);

    // Chained variant 3: retarget very late (t=400 ms — past midpoint).
    auto d = run_scenario(m, raw, "D chained late (retarget at 400 ms)",
                          final_deg, mp,
                          /*intermediate=*/mid_deg,
                          /*chain_after=*/400'000ull);

    raw.enable(false);
    t.close();
    send_set_baud(dev, 256000, 0x04);

    std::printf("\n=== summary ===\n");
    if (a.ok) std::printf("  A (single 720°):                 %7.2f ms  peak %.0f rpm\n",
                          a.total_ms, a.peak_rpm);
    if (b.ok) std::printf("  B (retarget at 150 ms): %7.2f ms  peak %.0f rpm  Δ %+.2f ms\n",
                          b.total_ms, b.peak_rpm, b.total_ms - a.total_ms);
    if (c.ok) std::printf("  C (retarget at  50 ms): %7.2f ms  peak %.0f rpm  Δ %+.2f ms\n",
                          c.total_ms, c.peak_rpm, c.total_ms - a.total_ms);
    if (d.ok) std::printf("  D (retarget at 400 ms): %7.2f ms  peak %.0f rpm  Δ %+.2f ms\n",
                          d.total_ms, d.peak_rpm, d.total_ms - a.total_ms);
    std::printf("\n  if any of B/C/D is within ~30 ms of A, mid-motion update works\n"
                "  for that timing window. larger Δ = firmware did not honor it.\n");
    return 0;
}
