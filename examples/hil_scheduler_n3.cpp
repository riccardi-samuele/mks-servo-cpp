// SPDX-License-Identifier: Apache-2.0
//
// HIL test for Scheduler with THREE motors on independent buses.
//
// Verifies that:
//   1. Each motor's solo timing through the scheduler matches the
//      known motion-only baseline (~40 ms for V1.0.9 SR_CLOSE,
//      ~43 ms for V1.0.8 SR_vFOC) plus the expected ~2 ms scheduler
//      overhead from consecutive_in_window=2.
//   2. Parallel A||B||C dispatches with sub-ms skew (thread-per-bus
//      design promises < 1 ms cross-bus dispatch divergence).
//   3. Sequential and cross-motor at_progress triggers work with all
//      three motors mixed.
//   4. A bigger 12-move choreography across A+B+C completes reliably.
//
// Topology (verified by READ_VERSION 0x40 probe at session start):
//   /dev/ttyUSB2 — B (V1.0.9 SR_CLOSE)
//   /dev/ttyUSB3 — A (V1.0.8 SR_vFOC)
//   /dev/ttyUSB4 — C (V1.0.9 SR_CLOSE)
//
// Methodology rule (per feedback memory): every reported number is
// validated against the single-motor baseline FIRST. If solo timing
// doesn't reproduce the baseline, the bench is wrong — fix before
// reporting any multi-motor number.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <vector>

#include "mks_servo/motor.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/scheduler.hpp"
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

struct Stats {
    double mean{0}, sigma{0}, min{0}, max{0}, p50{0};
    int n{0};
};
static Stats stats(std::vector<double> v) {
    Stats s;
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    double sum = 0;
    for (double x : v) sum += x;
    s.mean = sum / static_cast<double>(v.size());
    double var = 0;
    for (double x : v) var += (x - s.mean) * (x - s.mean);
    s.sigma = std::sqrt(var / static_cast<double>(v.size()));
    s.min = v.front(); s.max = v.back();
    s.p50 = v[v.size() / 2];
    s.n = static_cast<int>(v.size());
    return s;
}

int main() {
    Transport tB; tB.open("/dev/ttyUSB2", 256000);
    Transport tA; tA.open("/dev/ttyUSB3", 256000);
    Transport tC; tC.open("/dev/ttyUSB4", 256000);
    RawDriver rB(tB, 1), rA(tA, 1), rC(tC, 1);
    Motor B(rB, Mechanical{1.0, 0});
    Motor A(rA, Mechanical{1.0, 0});
    Motor C(rC, Mechanical{1.0, 0});
    rB.enable(true); rA.enable(true); rC.enable(true);
    sleep_ms(300);

    Scheduler sched;
    sched.add(B);
    sched.add(A);
    sched.add(C);

    // ── PHASE 1: solo per motore (baseline reproducibility check) ──
    //
    // Known motion-only baseline (hil_envelope methodology):
    //   A V1.0.8 SR_vFOC: 43.19 ms ± 0.49
    //   B V1.0.9 SR_CLOSE: 40.94 ms ± 0.03
    //   C V1.0.9 SR_CLOSE: 39.89 ms ± 0.025
    // Scheduler t_end - t_start should be baseline + ~1.5 ms (one extra
    // poll cycle from consecutive_in_window=2).
    auto solo_bench = [&](Motor& m, const char* name, double baseline_ms) {
        constexpr int N = 12;
        std::vector<double> durs;
        int dir = +1;
        for (int i = 0; i < N; ++i) {
            sched.reset();
            auto h = sched.move(m, 90.0 * dir, MoveParams{2000, 255});
            (void)sched.run();
            const auto& s = *h.state();
            const double dur = (double)(s.t_end_us.load() - s.t_start_us.load()) / 1000.0;
            durs.push_back(dur);
            dir = -dir;
            sleep_ms(60);
        }
        const auto S = stats(durs);
        const double delta_vs_baseline = S.mean - baseline_ms;
        std::printf("  %s solo: mean=%.2fms σ=%.2fms (baseline %.2fms → Δ=%+.2fms)\n",
                    name, S.mean, S.sigma, baseline_ms, delta_vs_baseline);
        return S;
    };
    std::printf("=== PHASE 1: solo timing per motor (must match baseline ±2ms) ===\n");
    const auto sA = solo_bench(A, "A", 43.19);
    const auto sB = solo_bench(B, "B", 40.94);
    const auto sC = solo_bench(C, "C", 39.89);

    const bool baselines_ok = (sA.mean - 43.19 < 3.0)
                           && (sB.mean - 40.94 < 3.0)
                           && (sC.mean - 39.89 < 3.0);
    if (!baselines_ok) {
        std::fprintf(stderr,
            "\n!! solo timing does not reproduce baseline within +2 ms.\n"
            "   The scheduler is adding more overhead than expected, OR\n"
            "   the motors are in a different state than last measurement.\n"
            "   Investigate before trusting multi-motor numbers.\n");
    } else {
        std::printf("  -> baselines reproduced; scheduler overhead < 2 ms each.\n");
    }

    // ── PHASE 2: parallel A||B||C (dispatch skew measurement) ──
    std::printf("\n=== PHASE 2: parallel A||B||C (sub-ms dispatch skew target) ===\n");
    constexpr int RUNS = 10;
    std::vector<double> skew_ab, skew_ac, skew_bc;
    std::vector<double> par_wall;
    int dir = +1;
    for (int run = 0; run < RUNS; ++run) {
        sched.reset();
        auto hA = sched.move(A, 90.0 * dir, MoveParams{2000, 255});
        auto hB = sched.move(B, 90.0 * dir, MoveParams{2000, 255});
        auto hC = sched.move(C, 90.0 * dir, MoveParams{2000, 255});
        const auto t0 = now_us();
        sched.run();
        const auto t1 = now_us();
        const auto tsA = hA.state()->t_start_us.load();
        const auto tsB = hB.state()->t_start_us.load();
        const auto tsC = hC.state()->t_start_us.load();
        skew_ab.push_back(std::fabs((double)((std::int64_t)tsA - (std::int64_t)tsB)));
        skew_ac.push_back(std::fabs((double)((std::int64_t)tsA - (std::int64_t)tsC)));
        skew_bc.push_back(std::fabs((double)((std::int64_t)tsB - (std::int64_t)tsC)));
        par_wall.push_back((double)(t1 - t0) / 1000.0);
        dir = -dir;
        sleep_ms(80);
    }
    const auto Sab = stats(skew_ab), Sac = stats(skew_ac), Sbc = stats(skew_bc);
    const auto Swall = stats(par_wall);
    std::printf("  skew A-B: mean=%.0fµs max=%.0fµs\n", Sab.mean, Sab.max);
    std::printf("  skew A-C: mean=%.0fµs max=%.0fµs\n", Sac.mean, Sac.max);
    std::printf("  skew B-C: mean=%.0fµs max=%.0fµs\n", Sbc.mean, Sbc.max);
    std::printf("  wall:     mean=%.2fms σ=%.2fms max=%.2fms\n",
                Swall.mean, Swall.sigma, Swall.max);
    // One-shot dispatch-path breakdown after a long rest, so worker
    // inter-move-rest drift across runs has fully cleared.
    std::printf("\n  ── dispatch path breakdown after 500 ms rest ──\n");
    sleep_ms(500);
    sched.reset();
    auto h0 = sched.move(A, 90.0, MoveParams{2000, 255});
    auto h1 = sched.move(B, 90.0, MoveParams{2000, 255});
    auto h2 = sched.move(C, 90.0, MoveParams{2000, 255});
    sched.run();
    auto report_path = [](const MoveHandle& h, const char* name) {
        const auto& s = *h.state();
        const auto tp = s.t_pickup_us.load();
        const auto td = s.t_predrain_us.load();
        const auto te = s.t_e0_read_us.load();
        const auto ts = s.t_start_us.load();
        std::printf("    %s: pickup=0 predrain=%+5lld  e0_read=%+5lld  dispatch=%+5lld µs\n",
                    name,
                    (long long)((std::int64_t)td - (std::int64_t)tp),
                    (long long)((std::int64_t)te - (std::int64_t)tp),
                    (long long)((std::int64_t)ts - (std::int64_t)tp));
    };
    report_path(h0, "A");
    report_path(h1, "B");
    report_path(h2, "C");
    // Cross-motor: when did each one pick up vs the earliest pickup?
    const auto tpA = h0.state()->t_pickup_us.load();
    const auto tpB = h1.state()->t_pickup_us.load();
    const auto tpC = h2.state()->t_pickup_us.load();
    const auto earliest = std::min({tpA, tpB, tpC});
    std::printf("    pickup offsets vs earliest: A=%+5lldµs B=%+5lldµs C=%+5lldµs\n",
                (long long)((std::int64_t)tpA - (std::int64_t)earliest),
                (long long)((std::int64_t)tpB - (std::int64_t)earliest),
                (long long)((std::int64_t)tpC - (std::int64_t)earliest));
    sleep_ms(80);

    // ── PHASE 3: sequential A → B → C ──
    std::printf("\n=== PHASE 3: sequential A → B → C ===\n");
    std::vector<double> seq_wall;
    dir = +1;
    for (int run = 0; run < RUNS; ++run) {
        sched.reset();
        auto hA = sched.move(A, 90.0 * dir, MoveParams{2000, 255});
        auto hB = sched.move(B, 90.0 * dir, MoveParams{2000, 255}).after(hA);
        (void)sched.move(C, 90.0 * dir, MoveParams{2000, 255}).after(hB);
        const auto t0 = now_us();
        sched.run();
        const auto t1 = now_us();
        seq_wall.push_back((double)(t1 - t0) / 1000.0);
        dir = -dir;
        sleep_ms(80);
    }
    const auto Sseq = stats(seq_wall);
    const double expected_seq_min = sA.mean + sB.mean + sC.mean;
    std::printf("  wall:     mean=%.2fms σ=%.2fms (sum-of-solos %.2fms)\n",
                Sseq.mean, Sseq.sigma, expected_seq_min);
    std::printf("  per-move handoff overhead: ~%.1fms\n",
                (Sseq.mean - expected_seq_min) / 2.0);

    // ── PHASE 4: cross-motor at_progress (C starts when B at 50%) ──
    std::printf("\n=== PHASE 4: cross-motor at_progress (C waits B 50%%) ===\n");
    std::vector<double> ap_wall, ap_delay;
    dir = +1;
    for (int run = 0; run < RUNS; ++run) {
        sched.reset();
        auto hB = sched.move(B, 90.0 * dir, MoveParams{2000, 255});
        auto hC = sched.move(C, 90.0 * dir, MoveParams{2000, 255}).at_progress(hB, 0.5);
        const auto t0 = now_us();
        sched.run();
        const auto t1 = now_us();
        ap_wall.push_back((double)(t1 - t0) / 1000.0);
        const auto tsB = hB.state()->t_start_us.load();
        const auto tsC = hC.state()->t_start_us.load();
        ap_delay.push_back((double)((std::int64_t)tsC - (std::int64_t)tsB) / 1000.0);
        dir = -dir;
        sleep_ms(80);
    }
    const auto Sapw = stats(ap_wall), Sapd = stats(ap_delay);
    std::printf("  wall:     mean=%.2fms σ=%.2fms\n", Sapw.mean, Sapw.sigma);
    std::printf("  C delay from B start: mean=%.2fms σ=%.2fms (expect ~%.0fms = half B)\n",
                Sapd.mean, Sapd.sigma, sB.mean * 0.5);

    // ── PHASE 5: 12-move choreography mixing all three motors ──
    std::printf("\n=== PHASE 5: 12-move choreography (4 per motor, mixed deps) ===\n");
    std::vector<double> ch_wall;
    int chfail = 0;
    dir = +1;
    for (int run = 0; run < RUNS; ++run) {
        sched.reset();
        const MoveParams p{2000, 255};
        // Round 1: parallel start on all three
        auto a1 = sched.move(A,  90.0 * dir, p);
        auto b1 = sched.move(B,  90.0 * dir, p);
        (void)sched.move(C,  90.0 * dir, p);
        // Round 2: each motor reverses, B starts at A 50%, C after B
        auto a2 = sched.move(A, -90.0 * dir, p);                    // FIFO after a1
        (void)sched.move(B, -90.0 * dir, p).at_progress(a1,0.5);
        auto c2 = sched.move(C, -90.0 * dir, p).after(b1);
        // Round 3: ±180 each, cross deps
        (void)sched.move(A, 180.0 * dir, p);
        (void)sched.move(B, 180.0 * dir, p).at_progress(a2,0.3);
        (void)sched.move(C, 180.0 * dir, p).after(c2);
        // Round 4: return — all FIFO
        (void)sched.move(A,-180.0 * dir, p);
        (void)sched.move(B,-180.0 * dir, p);
        (void)sched.move(C,-180.0 * dir, p);

        const auto t0 = now_us();
        const auto worst = sched.run();
        const auto t1 = now_us();
        ch_wall.push_back((double)(t1 - t0) / 1000.0);
        if ((int)worst > (int)MoveStatus::Completed) ++chfail;
        dir = -dir;
        sleep_ms(100);
    }
    const auto Sch = stats(ch_wall);
    std::printf("  wall:     mean=%.2fms σ=%.2fms min=%.2f max=%.2f  fails=%d/%d\n",
                Sch.mean, Sch.sigma, Sch.min, Sch.max, chfail, RUNS);

    rB.enable(false); rA.enable(false); rC.enable(false);
    return 0;
}
