// SPDX-License-Identifier: Apache-2.0
//
// HIL example: run the empirical characterization suite against a live
// driver and print the results.
//
// Switches the motor to 256k baud, runs all four tests (P1, P3, P5, S2),
// then restores 38400. Use this once per motor + load + supply combo to
// learn its operating envelope; persist the numbers somewhere (a YAML
// profile, a constants header, etc.) and feed them back to your
// application's motion planner.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "mks_servo/characterize.hpp"
#include "mks_servo/motor.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::build_frame;
using mks_servo::CharacterizationSuite;
using mks_servo::Mechanical;
using mks_servo::Motor;
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

    CharacterizationSuite suite(m);

    // Small pause between tests; the firmware needs ms-scale settle
    // time between bursts of different-mode commands.
    auto settle = [] { sleep_ms(200); };

    // P1: precision at a single target ─────────────────────────────
    std::printf("\n=== P1: precision (5 moves to 0°) ===\n");
    auto p1 = suite.run_p1_precision(/*target=*/0.0,
                                     /*iterations=*/5,
                                     /*rpm=*/300,
                                     /*acc=*/200);
    if (!p1.ok) {
        std::fprintf(stderr, "P1 failed\n");
    } else {
        std::printf("  samples (deg):");
        for (double s : p1.samples_deg) std::printf(" %+.3f", s);
        std::printf("\n");
        std::printf("  mean  = %+.4f°\n", p1.mean_deg);
        std::printf("  sigma = %.4f° (population stdev)\n", p1.sigma_deg);
        std::printf("  peak  = %.4f° (max |sample - target|)\n", p1.peak_deg);
    }
    settle();

    // P3: error vs RPM ─────────────────────────────────────────────
    // RPMs chosen to be reachable on an unloaded NEMA17 @ 12V/3A. A
    // loaded shaft and/or 24V supply can sustain higher values; tune
    // for the actual setup.
    std::printf("\n=== P3: error vs RPM (5 moves per RPM, target=90°) ===\n");
    auto p3 = suite.run_p3_error_vs_rpm({50, 100, 200, 300, 500},
                                        /*samples_per_rpm=*/5,
                                        /*target=*/90.0,
                                        /*acc=*/200);
    if (!p3.ok && p3.rpm_samples.empty()) {
        std::fprintf(stderr, "P3 failed: no RPM completed\n");
    } else {
        std::printf("  %5s  %s\n", "rpm", "rms err [deg]");
        for (std::size_t i = 0; i < p3.rpm_samples.size(); ++i) {
            std::printf("  %5u  %.4f\n", p3.rpm_samples[i], p3.rms_error_deg[i]);
        }
        if (!p3.failed_rpms.empty()) {
            std::printf("  failed RPMs (skipped):");
            for (auto r : p3.failed_rpms) std::printf(" %u", r);
            std::printf("\n");
        }
    }
    settle();

    // P5: follow error during continuous sweep ─────────────────────
    std::printf("\n=== P5: follow error during 360° sweep @ 60 rpm ===\n");
    auto p5 = suite.run_p5_follow_error(/*rpm=*/60,
                                        /*duration_s=*/2.0,
                                        /*sweep=*/360.0,
                                        /*interval=*/0.05,
                                        /*acc=*/200);
    if (!p5.ok) {
        std::fprintf(stderr, "P5 failed\n");
    } else {
        std::printf("  samples:    %zu\n", p5.samples);
        std::printf("  max follow: %.4f°\n", p5.max_follow_err_deg);
        std::printf("  rms follow: %.4f°\n", p5.rms_follow_err_deg);
    }
    settle();

    // S2: acceleration curves ──────────────────────────────────────
    // Velocity-mode top speed depends heavily on supply voltage and
    // mechanical load. The unloaded NEMA17 + 12V/3A rig peaks around
    // 100 rpm in MOVE_SPEED mode (sustained), so the demo target is
    // set low. With 24V + a loaded shaft the same hardware can sustain
    // 1000+ rpm — override for your setup.
    std::printf("\n=== S2: time-to-target vs acc (target=50 rpm) ===\n");
    auto s2 = suite.run_s2_acceleration(/*target_rpm=*/50,
                                        {1, 50, 100, 200, 255},
                                        /*samples_per_acc=*/100,
                                        /*sample_interval=*/0.01,
                                        /*cool_s=*/1.0);
    if (!s2.ok) {
        std::fprintf(stderr, "S2 failed\n");
    } else {
        std::printf("  %5s  %s\n", "acc", "time-to-target [ms]");
        for (std::size_t i = 0; i < s2.accs.size(); ++i) {
            const double t_ms = s2.time_to_target_ms[i];
            if (t_ms < 0) std::printf("  %5u  (did not reach 95%% within window)\n",
                                       s2.accs[i]);
            else          std::printf("  %5u  %.2f\n", s2.accs[i], t_ms);
        }
        std::printf("  max observed rpm across runs: %u\n", s2.max_observed_rpm);
    }

    // Ensure motor is stopped before exit.
    raw.move_speed(0, 255, mks_servo::Direction::CW);
    sleep_ms(200);
    raw.enable(false);
    t.close();
    send_set_baud(dev, 256000, 0x04);
    return 0;
}
