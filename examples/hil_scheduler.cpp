// SPDX-License-Identifier: Apache-2.0
//
// HIL test for Scheduler: exercise every dependency primitive on two
// motors connected over independent buses. Reports per-pattern timing
// and start skew, then compares with the bare single-motor solo time.
//
// Topology assumed:
//   /dev/ttyUSB0 — addr 1 — motor "M0"
//   /dev/ttyUSB1 — addr 1 — motor "M1"
//   Both at 256000 baud (auto-detected). Configuration is left to the
//   user (this example does not touch work_mode / current / etc.).
//
// Test sequence (each pattern N=4 iterations alternating ±90°):
//   1. Solo M0  — baseline
//   2. Solo M1  — baseline
//   3. Parallel — M0 and M1 simultaneously
//   4. Sequential — M0, then M1 after M0 completes
//   5. AtProgress(0.5) — M1 starts when M0 reaches 50% of delta
//   6. AtTimeAfterStart(20ms) — M1 starts 20 ms after M0 dispatches
//   7. AtTimeBeforeEnd(5ms)   — M1 starts so M0 has 5 ms remaining

#include <cmath>
#include <cstdint>
#include <cstdio>
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

struct Bench {
    double t0_ms{}, t1_ms{}, total_ms{};
    double m1_start_delay_ms{};  // M1 start time relative to M0 start
};

static Bench summarize(const MoveState& m0, const MoveState& m1) {
    Bench b{};
    const auto m0_start = m0.t_start_us.load();
    const auto m0_end   = m0.t_end_us.load();
    const auto m1_start = m1.t_start_us.load();
    const auto m1_end   = m1.t_end_us.load();
    b.t0_ms = (double)(m0_end - m0_start) / 1000.0;
    b.t1_ms = (double)(m1_end - m1_start) / 1000.0;
    const auto first = std::min(m0_start, m1_start);
    const auto last  = std::max(m0_end,   m1_end);
    b.total_ms = (double)(last - first) / 1000.0;
    b.m1_start_delay_ms = (double)((std::int64_t)m1_start - (std::int64_t)m0_start) / 1000.0;
    return b;
}

int main() {
    Transport t0; t0.open("/dev/ttyUSB0", 256000);
    Transport t1; t1.open("/dev/ttyUSB1", 256000);
    RawDriver  r0(t0, 1);
    RawDriver  r1(t1, 1);
    Motor      m0(r0, Mechanical{1.0, 0});
    Motor      m1(r1, Mechanical{1.0, 0});
    r0.enable(true); r1.enable(true);
    sleep_ms(300);

    Scheduler sched;
    sched.add(m0);
    sched.add(m1);

    const int N = 4;
    int dir = +1;

    auto run_pattern = [&](const char* label, auto submit) {
        std::printf("\n=== %s ===\n", label);
        std::vector<Bench> results;
        for (int i = 0; i < N; ++i) {
            sched.reset();
            auto handles = submit(dir);
            std::uint64_t wall_start = now_us();
            sched.run();
            std::uint64_t wall_end = now_us();
            Bench b = summarize(*handles.first.state(), *handles.second.state());
            results.push_back(b);
            std::printf("  iter %d dir%+d: m0=%.2fms m1=%.2fms (m1_delay=%.2fms) total=%.2fms wall=%.2fms\n",
                        i, dir, b.t0_ms, b.t1_ms, b.m1_start_delay_ms, b.total_ms,
                        (double)(wall_end - wall_start) / 1000.0);
            dir = -dir;
            sleep_ms(80);
        }
        double sum_total = 0, sum_delay = 0;
        for (auto& b : results) { sum_total += b.total_ms; sum_delay += b.m1_start_delay_ms; }
        std::printf("  mean total: %.2fms   mean m1_delay: %.2fms\n",
                    sum_total / N, sum_delay / N);
    };

    auto solo = [&](Motor& motor, const char* label) {
        std::printf("\n=== %s ===\n", label);
        for (int i = 0; i < N; ++i) {
            sched.reset();
            auto h = sched.move(motor, 90.0 * dir, MoveParams{2000, 255});
            sched.run();
            const auto& s = *h.state();
            std::printf("  iter %d dir%+d: %.2fms\n", i, dir,
                        (double)(s.t_end_us.load() - s.t_start_us.load()) / 1000.0);
            dir = -dir;
            sleep_ms(80);
        }
    };

    solo(m0, "Solo M0 (90 deg)");
    solo(m1, "Solo M1 (90 deg)");

    run_pattern("Parallel (M0 || M1)", [&](int d){
        auto h0 = sched.move(m0, 90.0 * d, MoveParams{2000, 255});
        auto h1 = sched.move(m1, 90.0 * d, MoveParams{2000, 255});
        return std::pair(h0, h1);
    });

    run_pattern("Sequential (M1 after M0)", [&](int d){
        auto h0 = sched.move(m0, 90.0 * d, MoveParams{2000, 255});
        auto h1 = sched.move(m1, 90.0 * d, MoveParams{2000, 255}).after(h0);
        return std::pair(h0, h1);
    });

    run_pattern("AtProgress 0.5 (M1 at M0 50%)", [&](int d){
        auto h0 = sched.move(m0, 90.0 * d, MoveParams{2000, 255});
        auto h1 = sched.move(m1, 90.0 * d, MoveParams{2000, 255}).at_progress(h0, 0.5);
        return std::pair(h0, h1);
    });

    run_pattern("AtTimeAfterStart 20ms", [&](int d){
        auto h0 = sched.move(m0, 90.0 * d, MoveParams{2000, 255});
        auto h1 = sched.move(m1, 90.0 * d, MoveParams{2000, 255}).at_time_after_start(h0, 20.0);
        return std::pair(h0, h1);
    });

    run_pattern("AtTimeBeforeEnd 5ms (settle hiding)", [&](int d){
        auto h0 = sched.move(m0, 90.0 * d, MoveParams{2000, 255});
        // Tell the scheduler what M0's expected duration is so the trigger
        // can compute "X ms before end". Use the measured solo number.
        h0.with_expected_duration_ms(43.0);
        auto h1 = sched.move(m1, 90.0 * d, MoveParams{2000, 255}).at_time_before_end(h0, 5.0);
        return std::pair(h0, h1);
    });

    r0.enable(false); r1.enable(false);
    return 0;
}
