// SPDX-License-Identifier: Apache-2.0
//
// HIL test for Scheduler under a longer, mixed-pattern choreography.
// Exercises:
//   - FIFO per motor (multiple moves on the same motor with default deps)
//   - Cross-motor dependencies in various directions
//   - Long chains (move A1 -> wait for B1 at 60% -> move A2 -> ...)
//   - Repeated execution of the same plan (reuse the same Scheduler)
//
// Reports total wall time per run and per-move start/end timestamps so
// you can verify the resolution order matches the declared DAG.

#include <algorithm>
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

int main() {
    Transport t0; t0.open("/dev/ttyUSB0", 256000);
    Transport t1; t1.open("/dev/ttyUSB1", 256000);
    RawDriver r0(t0, 1), r1(t1, 1);
    Motor m0(r0, Mechanical{1.0, 0});
    Motor m1(r1, Mechanical{1.0, 0});
    r0.enable(true); r1.enable(true);
    sleep_ms(300);

    Scheduler sched;
    sched.add(m0);
    sched.add(m1);

    constexpr int RUNS = 10;
    std::vector<double> wall_times;
    int dir = +1;

    for (int run = 0; run < RUNS; ++run) {
        sched.reset();

        // Choreography (8 moves total):
        //   A: m0 +90       (no dep)               — 0 ms
        //   B: m1 +90       (no dep)               — par with A
        //   C: m0 -90       (FIFO after A)         — after A done
        //   D: m1 -90       (after B at 50%)       — encoder trigger
        //   E: m0 +180      (FIFO after C)
        //   F: m1 +180      (FIFO after D)         — par with E
        //   G: m0 -180      (after F at 30%)       — cross-motor encoder trigger
        //   H: m1 -180      (FIFO after F)
        const MoveParams p{2000, 255};
        auto A = sched.move(m0,  90.0 * dir, p);
        auto B = sched.move(m1,  90.0 * dir, p);
        auto C = sched.move(m0, -90.0 * dir, p);                  // FIFO
        auto D = sched.move(m1, -90.0 * dir, p).at_progress(B, 0.5);
        auto E = sched.move(m0, 180.0 * dir, p);                  // FIFO after C
        auto F = sched.move(m1, 180.0 * dir, p);                  // FIFO after D
        auto G = sched.move(m0, -180.0 * dir, p).at_progress(F, 0.3);
        auto H = sched.move(m1, -180.0 * dir, p);                 // FIFO after F

        const std::uint64_t wall_start = now_us();
        const auto worst = sched.run();
        const std::uint64_t wall_end = now_us();
        const double wall_ms = (double)(wall_end - wall_start) / 1000.0;
        wall_times.push_back(wall_ms);

        const auto rel_us = [&](std::uint64_t t){
            return (double)((std::int64_t)t - (std::int64_t)wall_start) / 1000.0;
        };
        std::printf("--- run %d dir%+d wall=%.2fms worst=%d ---\n",
                    run, dir, wall_ms, (int)worst);
        auto report = [&](const MoveHandle& h, const char* name) {
            const auto& s = *h.state();
            const double ts = rel_us(s.t_start_us.load());
            const double te = rel_us(s.t_end_us.load());
            std::printf("  %s  start=%6.2fms  end=%6.2fms  dur=%5.2fms  status=%d\n",
                        name, ts, te, te - ts, (int)s.status.load());
        };
        report(A, "A m0 +90 ");
        report(B, "B m1 +90 ");
        report(C, "C m0 -90 ");
        report(D, "D m1 -90 ");
        report(E, "E m0 +180");
        report(F, "F m1 +180");
        report(G, "G m0 -180");
        report(H, "H m1 -180");
        std::printf("\n");
        dir = -dir;
        sleep_ms(100);
    }

    // Stats
    std::sort(wall_times.begin(), wall_times.end());
    double sum = 0;
    for (double v : wall_times) sum += v;
    const double n = static_cast<double>(wall_times.size());
    const double mean = sum / n;
    double var = 0;
    for (double v : wall_times) var += (v - mean) * (v - mean);
    std::printf("=== Wall time stats over %d runs ===\n", RUNS);
    std::printf("  mean=%.2fms sigma=%.2fms min=%.2f max=%.2f p99=%.2f\n",
                mean, std::sqrt(var / n),
                wall_times.front(), wall_times.back(),
                wall_times[wall_times.size() * 99 / 100]);

    r0.enable(false); r1.enable(false);
    return 0;
}
