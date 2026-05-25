// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 mks-servo-cpp contributors
//
// CharacterizationSuite: empirical motor characterization tests, ported
// from the Python `mks-servo` library's CharacterizationSuite.
//
// Four tests, each returning a struct with the numbers the test produces:
//
//   P1  Precision / repeatability
//       Move to the same target N times and measure the read-back spread
//       (mean, sigma, peak deviation from target).
//
//   P3  Error vs commanded RPM
//       For a list of RPMs, do K moves to a target at that RPM and report
//       the RMS of (read - target).
//
//   P5  Follow error during a sweep
//       Run a continuous sweep at constant RPM and sample the firmware's
//       internal angle-error register throughout. Reports max and RMS.
//
//   S2  Acceleration curves
//       For a list of `acc` parameter values, command MOVE_SPEED at a
//       fixed target RPM and time how long it takes to reach 95% of
//       target. Reports time-to-target per acc and the maximum observed
//       RPM across the runs.
//
// All tests assume the motor is enabled and the origin has been set.
// They use moderate parameters by default to avoid the firmware's
// stall-protection latch (see docs/design.md §4); callers can override.
//
// This module isn't on the hot path — it does dynamic allocation
// (std::vector) and is not noexcept-friendly. It's a tool you run once
// per motor at calibration time.

#ifndef MKS_SERVO_CHARACTERIZE_HPP
#define MKS_SERVO_CHARACTERIZE_HPP

#include <cmath>
#include <cstdint>
#include <ctime>
#include <vector>

#include "mks_servo/motor.hpp"
#include "mks_servo/raw_driver.hpp"

namespace mks_servo {

struct P1Result {
    double              target_deg   = 0.0;
    std::size_t         iterations   = 0;
    std::vector<double> samples_deg;
    double              mean_deg     = 0.0;
    double              sigma_deg    = 0.0;  // population stdev
    double              peak_deg     = 0.0;  // max |sample - target|
    bool                ok           = false;
};

struct P3Result {
    std::vector<std::uint16_t> rpm_samples;       // RPMs that completed all iterations
    std::vector<double>        rms_error_deg;
    std::vector<std::uint16_t> failed_rpms;       // RPMs where at least one move failed
    bool                       ok = false;
};

struct P5Result {
    std::uint16_t rpm                = 0;
    double        duration_s         = 0.0;
    double        max_follow_err_deg = 0.0;
    double        rms_follow_err_deg = 0.0;
    std::size_t   samples            = 0;
    bool          ok                 = false;
};

struct S2Result {
    std::uint16_t              target_rpm        = 0;
    std::vector<std::uint8_t>  accs;
    // -1 means: never reached 95% within samples_per_acc.
    std::vector<double>        time_to_target_ms;
    std::uint16_t              max_observed_rpm  = 0;
    bool                       ok                = false;
};

struct SuiteResult {
    P1Result p1;
    P3Result p3;
    P5Result p5;
    S2Result s2;
};

class CharacterizationSuite {
public:
    explicit CharacterizationSuite(Motor& m) noexcept : motor_(m) {}

    // ─── P1 ────────────────────────────────────────────────────────
    P1Result run_p1_precision(double        target_deg = 0.0,
                              std::size_t   iterations = 5,
                              std::uint16_t rpm        = 300,
                              std::uint8_t  acc        = 200) {
        P1Result res;
        res.target_deg = target_deg;
        res.iterations = iterations;
        res.samples_deg.reserve(iterations);

        for (std::size_t i = 0; i < iterations; ++i) {
            const auto w = motor_.write(target_deg,
                                        MoveParams{rpm, acc},
                                        /*blocking=*/true);
            if (!w.ok()) return res;
            const auto rd = motor_.read();
            if (!rd.ok()) return res;
            res.samples_deg.push_back(rd.value);
        }
        compute_stats_(res.samples_deg, target_deg,
                       res.mean_deg, res.sigma_deg, res.peak_deg);
        res.ok = true;
        return res;
    }

    // ─── P3 ────────────────────────────────────────────────────────
    // RPMs that complete all iterations cleanly land in rpm_samples +
    // rms_error_deg. RPMs where at least one move or read failed land
    // in failed_rpms with no rms recorded — the test moves on to the
    // next RPM instead of bailing. `ok` is true if at least one RPM
    // succeeded.
    P3Result run_p3_error_vs_rpm(const std::vector<std::uint16_t>& rpms
                                       = {50, 100, 300, 500, 1000},
                                 std::size_t samples_per_rpm = 5,
                                 double      target_deg      = 90.0,
                                 std::uint8_t acc            = 200) {
        P3Result res;

        for (auto rpm : rpms) {
            std::vector<double> errs;
            errs.reserve(samples_per_rpm);
            bool rpm_ok = true;
            for (std::size_t i = 0; i < samples_per_rpm; ++i) {
                const auto w = motor_.write(target_deg,
                                            MoveParams{rpm, acc},
                                            /*blocking=*/true);
                if (!w.ok()) { rpm_ok = false; break; }
                const auto rd = motor_.read();
                if (!rd.ok()) { rpm_ok = false; break; }
                errs.push_back(rd.value - target_deg);
            }
            if (rpm_ok) {
                res.rpm_samples.push_back(rpm);
                res.rms_error_deg.push_back(rms_(errs));
            } else {
                res.failed_rpms.push_back(rpm);
            }
        }
        res.ok = !res.rpm_samples.empty();
        return res;
    }

    // ─── P5 ────────────────────────────────────────────────────────
    P5Result run_p5_follow_error(std::uint16_t rpm                = 60,
                                 double        duration_s         = 2.0,
                                 double        sweep_deg          = 360.0,
                                 double        sample_interval_s  = 0.05,
                                 std::uint8_t  acc                = 200) {
        P5Result res;
        res.rpm        = rpm;
        res.duration_s = duration_s;

        // Setup: move to start (blocking).
        if (!motor_.write(0.0, MoveParams{rpm, acc},
                          /*blocking=*/true).ok()) {
            return res;
        }

        // Dispatch the sweep without waiting for completion, then sample.
        const auto disp = motor_.dispatch_write(sweep_deg,
                                                MoveParams{rpm, acc});
        if (disp != Transport::Status::OK) return res;

        std::vector<double> samples;
        const std::uint64_t deadline_us = now_us_() +
            static_cast<std::uint64_t>(duration_s * 1'000'000.0);
        while (now_us_() < deadline_us) {
            const auto er = motor_.error();
            if (!er.ok()) break;
            samples.push_back(std::abs(er.value));
            sleep_us_(static_cast<std::uint64_t>(sample_interval_s * 1'000'000.0));
        }

        if (!samples.empty()) {
            res.max_follow_err_deg = samples[0];
            for (double v : samples) {
                if (v > res.max_follow_err_deg) res.max_follow_err_deg = v;
            }
            res.rms_follow_err_deg = rms_(samples);
        }
        res.samples = samples.size();
        res.ok      = true;
        return res;
    }

    // ─── S2 ────────────────────────────────────────────────────────
    S2Result run_s2_acceleration(std::uint16_t target_rpm = 1500,
                                 const std::vector<std::uint8_t>& accs
                                       = {1, 50, 100, 200, 255},
                                 std::size_t samples_per_acc   = 50,
                                 double      sample_interval_s = 0.01,
                                 double      cool_s            = 1.0) {
        S2Result res;
        res.target_rpm = target_rpm;
        res.accs       = accs;
        res.time_to_target_ms.reserve(accs.size());

        RawDriver& raw = motor_.raw();

        for (auto acc : accs) {
            // Stop the motor (rpm=0, max acc) and let it settle.
            (void)raw.move_speed(0, 255, Direction::CW);
            sleep_us_(static_cast<std::uint64_t>(cool_s * 1'000'000.0));

            // Start the ramp.
            const auto r = raw.move_speed(target_rpm, acc, Direction::CW);
            if (!r.ok()) {
                res.time_to_target_ms.push_back(-1.0);
                continue;
            }

            const std::uint64_t t0 = now_us_();
            double reached_ms = -1.0;
            for (std::size_t i = 0; i < samples_per_acc; ++i) {
                const auto sr = raw.read_speed_rpm();
                if (!sr.ok()) break;
                const std::uint16_t v = static_cast<std::uint16_t>(std::abs(sr.value));
                if (v > res.max_observed_rpm) res.max_observed_rpm = v;
                if (v >= static_cast<std::uint16_t>(target_rpm * 0.95)) {
                    reached_ms = static_cast<double>(now_us_() - t0) / 1000.0;
                    break;
                }
                sleep_us_(static_cast<std::uint64_t>(sample_interval_s * 1'000'000.0));
            }
            res.time_to_target_ms.push_back(reached_ms);

            // Bring the motor back to rest before the next acc value.
            (void)raw.move_speed(0, 255, Direction::CW);
        }
        res.ok = true;
        return res;
    }

    // ─── convenience: run everything ───────────────────────────────
    SuiteResult run_mvp() {
        SuiteResult s;
        s.p1 = run_p1_precision();        sleep_us_(200'000);
        s.p3 = run_p3_error_vs_rpm();     sleep_us_(200'000);
        s.p5 = run_p5_follow_error();     sleep_us_(200'000);
        s.s2 = run_s2_acceleration();
        return s;
    }

private:
    static void compute_stats_(const std::vector<double>& samples,
                               double                     target,
                               double& mean, double& sigma, double& peak) {
        if (samples.empty()) { mean = sigma = peak = 0.0; return; }
        double sum = 0.0;
        for (double v : samples) sum += v;
        mean = sum / static_cast<double>(samples.size());
        double var = 0.0;
        double peak_abs = 0.0;
        for (double v : samples) {
            const double d = v - mean;
            var += d * d;
            const double pa = std::abs(v - target);
            if (pa > peak_abs) peak_abs = pa;
        }
        sigma = std::sqrt(var / static_cast<double>(samples.size()));
        peak  = peak_abs;
    }

    static double rms_(const std::vector<double>& v) {
        if (v.empty()) return 0.0;
        double s = 0.0;
        for (double x : v) s += x * x;
        return std::sqrt(s / static_cast<double>(v.size()));
    }

    static std::uint64_t now_us_() noexcept {
        struct timespec ts;
        ::clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000ull
             + static_cast<std::uint64_t>(ts.tv_nsec) / 1000ull;
    }

    static void sleep_us_(std::uint64_t us) noexcept {
        if (us == 0) return;
        struct timespec ts;
        ts.tv_sec  = static_cast<time_t>(us / 1'000'000ull);
        ts.tv_nsec = static_cast<long>((us % 1'000'000ull) * 1000ull);
        while (::nanosleep(&ts, &ts) != 0) {}
    }

    Motor& motor_;
};

}  // namespace mks_servo

#endif  // MKS_SERVO_CHARACTERIZE_HPP
