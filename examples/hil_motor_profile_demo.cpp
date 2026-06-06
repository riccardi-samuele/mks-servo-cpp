// SPDX-License-Identifier: Apache-2.0
//
// MotorProfile API demo: probes each registered motor, prints the
// measured t_90deg, and runs a 12-move 3-motor choreography under:
//   1. Library defaults (everything conservative)
//   2. HIL-validated presets per motor (V1.0.9 SR_CLOSE for B & C,
//      V1.0.8 SR_vFOC for A)
//   3. Probe-derived expected_duration on every move
//
// Compares wall times and σ to show the perf win from per-motor
// tuning. Intended both as a smoke test for the MotorProfile API
// and as a runnable demo of the recommended setup pattern.

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
    struct timespec ts; ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1'000'000ull + ts.tv_nsec/1000;
}
static void sleep_ms(int ms) {
    struct timespec ts{ms / 1000, (ms % 1000) * 1'000'000L};
    ::nanosleep(&ts, nullptr);
}

struct W { double mean, sigma, min, max; int fails; };

static W run_pattern(Scheduler& sched, Motor& A, Motor& B, Motor& C,
                     int runs) {
    std::vector<double> walls;
    int fails = 0;
    int dir = +1;
    sleep_ms(500);
    for (int run = 0; run < runs; ++run) {
        sched.reset();
        const MoveParams p{2000, 255};
        auto a1 = sched.move(A,  90.0 * dir, p);
        auto b1 = sched.move(B,  90.0 * dir, p);
        (void)sched.move(C,  90.0 * dir, p);
        auto a2 = sched.move(A, -90.0 * dir, p);
        (void)sched.move(B, -90.0 * dir, p).at_progress(a1, 0.5);
        auto c2 = sched.move(C, -90.0 * dir, p).after(b1);
        (void)sched.move(A, 180.0 * dir, p);
        (void)sched.move(B, 180.0 * dir, p).at_progress(a2, 0.3);
        (void)sched.move(C, 180.0 * dir, p).after(c2);
        (void)sched.move(A, -180.0 * dir, p);
        (void)sched.move(B, -180.0 * dir, p);
        (void)sched.move(C, -180.0 * dir, p);
        const uint64_t t0 = now_us();
        const auto worst = sched.run();
        const uint64_t t1 = now_us();
        walls.push_back((double)(t1 - t0) / 1000.0);
        if ((int)worst > (int)MoveStatus::Completed) ++fails;
        dir = -dir;
        sleep_ms(100);
    }
    std::sort(walls.begin(), walls.end());
    double sum = 0; for (double v : walls) sum += v;
    const double mean = sum / walls.size();
    double var = 0; for (double v : walls) var += (v - mean) * (v - mean);
    return {mean, std::sqrt(var / walls.size()), walls.front(), walls.back(), fails};
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

    constexpr int RUNS = 10;

    std::printf("=== Step 1: probe each motor (measure t_90deg) ===\n");
    const double tA_ms = sched.probe_motor(A);
    const double tB_ms = sched.probe_motor(B);
    const double tC_ms = sched.probe_motor(C);
    std::printf("  A t_90deg = %.2f ms\n", tA_ms);
    std::printf("  B t_90deg = %.2f ms\n", tB_ms);
    std::printf("  C t_90deg = %.2f ms\n", tC_ms);

    std::printf("\n=== Step 2: baseline run with library defaults ===\n");
    const auto baseline = run_pattern(sched, A, B, C, RUNS);
    std::printf("  wall: mean=%.2fms σ=%.2fms min=%.2f max=%.2f fails=%d\n",
                baseline.mean, baseline.sigma, baseline.min, baseline.max,
                baseline.fails);

    std::printf("\n=== Step 3: install HIL-validated presets ===\n");
    sched.set_motor_profile(A, MotorProfile::for_v1_0_8_sr_vfoc());
    sched.set_motor_profile(B, MotorProfile::for_v1_0_9_sr_close());
    sched.set_motor_profile(C, MotorProfile::for_v1_0_9_sr_close());
    std::printf("  A: V1.0.8 SR_vFOC preset\n");
    std::printf("  B: V1.0.9 SR_CLOSE preset (inter_move_rest 5 ms)\n");
    std::printf("  C: V1.0.9 SR_CLOSE preset (inter_move_rest 5 ms)\n");

    std::printf("\n=== Step 4: run with presets ===\n");
    const auto tuned = run_pattern(sched, A, B, C, RUNS);
    std::printf("  wall: mean=%.2fms σ=%.2fms min=%.2f max=%.2f fails=%d\n",
                tuned.mean, tuned.sigma, tuned.min, tuned.max, tuned.fails);

    std::printf("\n=== summary ===\n");
    std::printf("  defaults: %.2f ± %.2f ms\n", baseline.mean, baseline.sigma);
    std::printf("  presets:  %.2f ± %.2f ms  (%+.2f%% wall)\n",
                tuned.mean, tuned.sigma,
                (tuned.mean - baseline.mean) / baseline.mean * 100.0);

    rB.enable(false); rA.enable(false); rC.enable(false);
    return 0;
}
