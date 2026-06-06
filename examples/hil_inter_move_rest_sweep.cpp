// SPDX-License-Identifier: Apache-2.0
//
// Inter-move-rest A/B sweep: runs the same 12-move 3-motor
// choreography under several inter_move_rest_us values to find the
// lowest rest that preserves 10/10 reliability.
//
// Note: settle_drain_ms (bus cleanup after target reached) is NOT
// tunable on a per-motor basis from physical settle time alone —
// the drain absorbs the firmware's late "complete" ack frame which
// arrives ~25-30 ms after motion regardless of work_mode. We leave
// settle_drain at the library default.
//
// inter_move_rest_us is wall-clock between worker's process_move
// returns and its next dispatch. Was hardcoded 100 ms historically;
// after the n3 bench surfaced this as the source of cross-run
// accumulated drift (and hence parallel skew), tuning it is the
// biggest available wall-time win.
//
// Methodology: don't trust the upper bound — find empirically. For
// each candidate, run the choreography RUNS times, count failures,
// measure wall mean. A regression in fail count means the candidate
// is too aggressive for this fleet.
//
// Topology:
//   /dev/ttyUSB2 — B (V1.0.9 SR_CLOSE)
//   /dev/ttyUSB3 — A (V1.0.8 SR_vFOC) — keep settle_drain=30 ms
//   /dev/ttyUSB4 — C (V1.0.9 SR_CLOSE)
//
// We hold A at the library default (V1.0.8 emits late-ack frames that
// the predispatch_drain on the NEXT move had to absorb, 5 ms measured).
// We sweep B and C settle_drain together to find their minimum safe
// value.

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

    // A stays at library default — V1.0.8 firmware emits late acks that
    // need the 100 ms rest. We only sweep B and C (both V1.0.9).
    const int candidates_us[] = {100'000, 50'000, 20'000, 10'000, 5'000, 2'000};

    constexpr int RUNS = 10;
    std::printf("=== inter_move_rest_us sweep (B & C; A at default 100 ms) ===\n");
    std::printf("  %7s  %7s  %7s  %7s  %s\n",
                "B/C_us", "wall_mn", "wall_sg", "wall_mx", "fails");

    for (int cand_us : candidates_us) {
        MotorProfile prof_bc;
        prof_bc.settle_drain_ms      = 30;        // hold default
        prof_bc.predispatch_drain_ms = 5;
        prof_bc.inter_move_rest_us   = cand_us;   // <-- the variable
        prof_bc.consecutive_in_window = 2;
        sched.set_motor_profile(B, prof_bc);
        sched.set_motor_profile(C, prof_bc);

        std::vector<double> walls;
        int fails = 0;
        int dir = +1;
        // Long rest to clear inter-move-rest drift accumulated previously
        sleep_ms(500);
        for (int run = 0; run < RUNS; ++run) {
            sched.reset();
            const MoveParams p{2000, 255};
            // Same 12-move choreography as hil_scheduler_n3 phase 5.
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

            const std::uint64_t t0 = now_us();
            const auto worst = sched.run();
            const std::uint64_t t1 = now_us();
            walls.push_back((double)(t1 - t0) / 1000.0);
            if ((int)worst > (int)MoveStatus::Completed) ++fails;
            dir = -dir;
            sleep_ms(100);
        }

        std::sort(walls.begin(), walls.end());
        double sum = 0;
        for (double v : walls) sum += v;
        const double mean = sum / static_cast<double>(walls.size());
        double var = 0;
        for (double v : walls) var += (v - mean) * (v - mean);
        const double sigma = std::sqrt(var / static_cast<double>(walls.size()));
        std::printf("  %7d  %7.2f  %7.2f  %7.2f  %d/%d\n",
                    cand_us, mean, sigma, walls.back(), fails, RUNS);
    }

    rB.enable(false); rA.enable(false); rC.enable(false);
    return 0;
}
