// SPDX-License-Identifier: Apache-2.0
//
// Envelope characterisation: discover what THIS motor + supply + load
// combination is actually capable of, end-to-end. Different from
// `characterize` (which measures servo quality — precision, follow
// error): this measures the OPERATING ENVELOPE — how fast can you
// move, how far up does steady RPM go before the firmware saturates,
// how much overshoot you pay for maximum acceleration, and how
// reliably it all repeats.
//
// Why it exists: a user dropping the lib into their setup needs to
// know "what numbers should I pass to write() / move_relative() to
// get the most out of my motor?". Without this, they either
// under-utilise (slow, safe) or over-push (stalls, retries). Running
// this once per setup gives the operating point.
//
// Methodology (each test states its validity criterion):
//   1. Comms latency        — encoder round-trip jitter
//   2. 90° vs acceleration  — time-to-window across acc parameter
//   3. Max RPM (steady)     — MOVE_SPEED + plateau detection
//   4. 90° vs commanded RPM — at acc=255, find the knee
//   5. Overshoot            — post-window encoder evolution
//   6. Soak                 — 100 alternating moves at max params
//
// Setup required:
//   - Motor enabled-capable (responds to ENABLE with firmware_ack=1)
//   - Shaft FREE (unloaded). With a load, max RPM and 90° times will
//     be lower than reported here. Re-run with load to characterise
//     the loaded envelope.
//   - Supply within MKS spec (9-28V, current sufficient for the
//     motor's configured work_current; >5A recommended).
//
// Usage:
//   hil_envelope [device] [slave_addr] [--json /path/to/out.json]
// Defaults: /dev/ttyUSB0  1  (no JSON output)
//
// Total runtime: ~80 seconds.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include "mks_servo/motor.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::build_frame;
using mks_servo::Direction;
using mks_servo::Mechanical;
using mks_servo::Motor;
using mks_servo::MotorStatusEx;
using mks_servo::MoveParams;
using mks_servo::RawDriver;
using mks_servo::Transport;
using mks_servo::op::SET_BAUD;

// ─── timing helpers ─────────────────────────────────────────────────
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

// ─── stats helpers ──────────────────────────────────────────────────
struct Stats {
    double mean = 0, sigma = 0, p50 = 0, p99 = 0, mn = 0, mx = 0;
    std::size_t n = 0;
};
static Stats compute(std::vector<double> v) {
    Stats s;
    s.n = v.size();
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    double sum = 0; for (double x : v) sum += x;
    s.mean = sum / static_cast<double>(v.size());
    double var = 0; for (double x : v) var += (x - s.mean) * (x - s.mean);
    s.sigma = std::sqrt(var / static_cast<double>(v.size()));
    auto pct = [&](double p) {
        const std::size_t i = std::min(
            static_cast<std::size_t>(static_cast<double>(v.size()) * p / 100.0),
            v.size() - 1);
        return v[i];
    };
    s.p50 = pct(50);
    s.p99 = pct(99);
    s.mn  = v.front();
    s.mx  = v.back();
    return s;
}

// ─── one 90° move with separate motion / settle / overshoot metrics ─
//
// Uses MOVE_REL_AXIS (delta-in-counts) instead of MOVE_ABS_AXIS to be
// fully agnostic of firmware-frame state. MOVE_ABS_AXIS targets are
// interpreted in the firmware's internal "axis position" frame, which
// is reset by SET_ZERO_POINT but NOT in sync with the encoder
// addition register (the one the lib reads via m.read()) once
// MOVE_SPEED has rotated the motor. A bench that mixes MOVE_SPEED and
// MOVE_ABS_AXIS hits frame drift and produces nonsense timings. Going
// through raw_driver's relative form sidesteps that entirely: the
// target is computed from the live encoder reading.
struct Move90 {
    bool   ok               = false;
    double t_window_ms      = 0;  // time from dispatch to first encoder-in-window
    double t_settle_ms      = 0;  // time spent draining the trailing "complete" ack
    double overshoot_counts = 0;  // signed (in direction of motion) counts past target at window-entry
};
static Move90 do_90deg(Motor& m, int dir, MoveParams mp,
                       std::int32_t tol_counts = 50,
                       std::uint64_t timeout_us = 5'000'000) {
    Move90 r;
    auto& raw = m.raw();
    auto e0 = raw.read_encoder_addition();
    if (!e0.ok()) return r;
    const std::int32_t QTR_COUNTS = 4096;  // 16384 / 4 = 90° exactly
    const std::int32_t delta = QTR_COUNTS * dir;
    const std::int64_t target = e0.value + delta;

    const std::uint64_t t0 = now_us();
    auto disp = raw.move_relative_axis(delta, mp.rpm, mp.acc);
    if (!disp.ok() || !disp.value) return r;

    // Poll encoder until within tol_counts of target, with the same
    // tight loop wait_for_position uses but no settle.
    const std::uint64_t deadline = now_us() + timeout_us;
    bool reached = false;
    while (now_us() < deadline) {
        auto e = raw.read_encoder_addition();
        if (!e.ok()) return r;
        const std::int64_t diff = e.value - target;
        const std::int64_t adiff = diff < 0 ? -diff : diff;
        if (adiff <= tol_counts) { reached = true; break; }
    }
    const std::uint64_t t1 = now_us();
    if (!reached) return r;

    auto immediate = raw.read_encoder_addition();
    const double overshoot = immediate.ok()
        ? static_cast<double>(immediate.value - target) * static_cast<double>(dir)
        : 0;

    const std::uint64_t s0 = now_us();
    raw.transport_drain_settle(20);
    const std::uint64_t s1 = now_us();

    r.ok              = true;
    r.t_window_ms     = static_cast<double>(t1 - t0) / 1000.0;
    r.t_settle_ms     = static_cast<double>(s1 - s0) / 1000.0;
    r.overshoot_counts = overshoot;
    return r;
}

// ─── plateau-detected RPM measurement ───────────────────────────────
//
// Sample encoder-derived rpm in 250ms windows. Declare plateau when 3
// consecutive samples are within 3% of each other. Cap at 16 windows
// (~4s). Returns -1 if no plateau (motor never reached steady state).
static double measure_rpm_plateau(RawDriver& raw, int window_ms = 250,
                                  int max_windows = 16) {
    auto sample = [&]() -> double {
        auto e0 = raw.read_encoder_addition();
        const std::uint64_t t0 = now_us();
        sleep_ms(window_ms);
        auto e1 = raw.read_encoder_addition();
        const std::uint64_t t1 = now_us();
        if (!e0.ok() || !e1.ok()) return -1;
        double dc = static_cast<double>(e1.value - e0.value);
        if (dc < 0) dc = -dc;
        return (dc / 16384.0) / (static_cast<double>(t1 - t0) / 1e6) * 60.0;
    };

    double prev = -1, prev2 = -1;
    for (int k = 0; k < max_windows; ++k) {
        const double r = sample();
        if (r < 0) return -1;
        if (prev > 0 && prev2 > 0) {
            const double lo = std::min({r, prev, prev2});
            const double hi = std::max({r, prev, prev2});
            if (hi > 0 && (hi - lo) / hi < 0.03) {
                return (r + prev + prev2) / 3.0;
            }
        }
        prev2 = prev;
        prev  = r;
    }
    return -1;  // no plateau
}

// ─── tests ──────────────────────────────────────────────────────────
struct EnvelopeReport {
    int    baud                 = 0;
    // Test 1
    Stats  comms_latency_us;
    bool   comms_valid          = false;
    // Test 2  (per-acc stats)
    std::vector<std::uint8_t> t2_accs;
    std::vector<Stats>         t2_window_ms;
    std::vector<int>           t2_ok;
    bool   t2_valid             = false;
    // Test 3
    std::vector<std::uint16_t> t3_cmds;
    std::vector<double>        t3_real_rpm;
    double t3_max_steady_rpm    = 0;
    std::uint16_t t3_knee_cmd   = 0;
    bool   t3_valid             = false;
    // Test 4
    std::vector<std::uint16_t> t4_rpms;
    std::vector<Stats>         t4_window_ms;
    std::uint16_t t4_flat_rpm   = 0;
    bool   t4_valid             = false;
    // Test 5
    double t5_peak_overshoot_deg = 0;
    double t5_settle_to_05deg_ms = 0;
    bool   t5_valid              = false;
    // Test 6
    int    t6_n            = 0;
    int    t6_successes    = 0;
    Stats  t6_window_ms;
    Stats  t6_settle_ms;
    bool   t6_valid        = false;
};

static void run_test1_comms(RawDriver& raw, EnvelopeReport& rep) {
    std::printf("[1/6] COMMS LATENCY (encoder round-trip, N=500)...\n");
    std::fflush(stdout);
    std::vector<double> us;
    us.reserve(500);
    int errors = 0;
    for (int i = 0; i < 500; ++i) {
        const std::uint64_t t0 = now_us();
        auto e = raw.read_encoder_addition();
        const std::uint64_t t1 = now_us();
        if (!e.ok()) { ++errors; continue; }
        us.push_back(static_cast<double>(t1 - t0));
    }
    rep.comms_latency_us = compute(us);
    // Validity: p99 < 3 * mean
    rep.comms_valid = (rep.comms_latency_us.n >= 450)
                   && (rep.comms_latency_us.p99 < 3.0 * rep.comms_latency_us.mean);
    std::printf("       mean=%.0fus  p50=%.0fus  p99=%.0fus  max=%.0fus  errors=%d  %s\n",
        rep.comms_latency_us.mean, rep.comms_latency_us.p50,
        rep.comms_latency_us.p99, rep.comms_latency_us.mx,
        errors, rep.comms_valid ? "VALID" : "INVALID");
}

static void run_test2_acc_sweep(Motor& m, EnvelopeReport& rep) {
    std::printf("[2/6] 90deg TIME vs ACCELERATION (rpm=2000, N=8 each acc)...\n");
    std::fflush(stdout);
    const std::uint8_t accs[] = {16, 64, 128, 200, 255};
    bool all_valid = true;
    for (std::uint8_t acc : accs) {
        std::vector<double> times;
        int ok_count = 0, dir = +1;
        for (int i = 0; i < 8; ++i) {
            auto r = do_90deg(m, dir, MoveParams{2000, acc});
            if (r.ok) { times.push_back(r.t_window_ms); ++ok_count; }
            dir = -dir;
            sleep_ms(80);
        }
        Stats s = compute(times);
        rep.t2_accs.push_back(acc);
        rep.t2_window_ms.push_back(s);
        rep.t2_ok.push_back(ok_count);
        const bool valid = (ok_count >= 6)
                        && (s.mean > 0)
                        && (s.sigma / s.mean < 0.20);
        if (!valid) all_valid = false;
        std::printf("       acc=%3d:  mean=%.2fms  sigma=%.2fms  min=%.2f  max=%.2f  ok=%d/8  %s\n",
            (int)acc, s.mean, s.sigma, s.mn, s.mx, ok_count,
            valid ? "VALID" : "INVALID");
    }
    rep.t2_valid = all_valid;
}

static void run_test3_max_rpm(RawDriver& raw, EnvelopeReport& rep) {
    std::printf("[3/6] MAX RPM (steady, plateau detection, acc=255)...\n");
    std::fflush(stdout);
    const std::uint16_t cmds[] = {300, 600, 900, 1200, 1500, 1800, 2100, 2400, 2700, 3000};
    double prev_real = 0;
    std::uint16_t knee = 0;
    bool any_valid = false;
    for (std::uint16_t c : cmds) {
        auto mv = raw.move_speed(c, /*acc=*/255, Direction::CW);
        if (!mv.ok()) {
            std::printf("       cmd=%4d:  move_speed FAILED\n", (int)c);
            continue;
        }
        const double real = measure_rpm_plateau(raw);
        rep.t3_cmds.push_back(c);
        rep.t3_real_rpm.push_back(real);
        if (real > 0) {
            any_valid = true;
            const double ratio = real / static_cast<double>(c);
            // Knee = first commanded RPM where real ratio drops below 0.95
            if (knee == 0 && ratio < 0.95 && prev_real > 0) {
                knee = c;
            }
            if (real > rep.t3_max_steady_rpm) rep.t3_max_steady_rpm = real;
            std::printf("       cmd=%4d:  real=%6.1f  ratio=%.3f%s\n",
                (int)c, real, ratio, ratio < 0.95 ? "  (not tracking)" : "");
            prev_real = real;
        } else {
            std::printf("       cmd=%4d:  no plateau\n", (int)c);
        }
    }
    raw.move_speed(0, 255, Direction::CW);
    sleep_ms(600);
    rep.t3_knee_cmd = knee;
    rep.t3_valid = any_valid;
    std::printf("       >>> max steady RPM: %.0f", rep.t3_max_steady_rpm);
    if (knee) std::printf("  (knee at cmd=%d)", (int)knee);
    std::printf("\n");
}

static void run_test4_rpm_sweep(Motor& m, EnvelopeReport& rep) {
    std::printf("[4/6] 90deg TIME vs COMMANDED RPM (acc=255, N=8 each)...\n");
    std::fflush(stdout);
    const std::uint16_t rpms[] = {300, 600, 900, 1200, 1500, 1800, 2100, 2400};
    double prev_mean = 1e9;
    std::uint16_t flat_at = 0;
    bool all_valid = true;
    for (std::uint16_t rpm : rpms) {
        std::vector<double> times;
        int ok_count = 0, dir = +1;
        for (int i = 0; i < 8; ++i) {
            auto r = do_90deg(m, dir, MoveParams{rpm, 255});
            if (r.ok) { times.push_back(r.t_window_ms); ++ok_count; }
            dir = -dir;
            sleep_ms(80);
        }
        Stats s = compute(times);
        rep.t4_rpms.push_back(rpm);
        rep.t4_window_ms.push_back(s);
        const bool valid = (ok_count >= 6)
                        && (s.mean > 0)
                        && (s.sigma / s.mean < 0.20);
        if (!valid) all_valid = false;
        // Flat region: when this mean is within 5% of previous mean
        if (flat_at == 0 && s.mean > 0 && std::fabs(s.mean - prev_mean) / s.mean < 0.05) {
            flat_at = rpm;
        }
        std::printf("       rpm=%4d:  mean=%.2fms  sigma=%.2fms  ok=%d/8  %s\n",
            (int)rpm, s.mean, s.sigma, ok_count, valid ? "VALID" : "INVALID");
        prev_mean = s.mean;
    }
    rep.t4_flat_rpm = flat_at;
    rep.t4_valid = all_valid;
    if (flat_at) std::printf("       >>> 90deg time flattens at rpm=%d\n", (int)flat_at);
}

static void run_test5_overshoot(Motor& m, EnvelopeReport& rep) {
    std::printf("[5/6] OVERSHOOT (acc=255 rpm=2000, sampled every ~5ms post-window)...\n");
    std::fflush(stdout);
    auto& raw = m.raw();
    auto e0 = raw.read_encoder_addition();
    if (!e0.ok()) { std::printf("       read failed; skipping\n"); return; }
    const std::int32_t QTR_COUNTS = 4096;
    const std::int32_t delta = QTR_COUNTS;  // forward 90°
    const std::int64_t target = e0.value + delta;

    auto disp = raw.move_relative_axis(delta, 2000, 255);
    if (!disp.ok() || !disp.value) {
        std::printf("       dispatch failed (ok=%d val=%d)\n", (int)disp.ok(), (int)disp.value);
        return;
    }
    // Wait for first window entry
    const std::uint64_t deadline = now_us() + 3'000'000ull;
    bool reached = false;
    while (now_us() < deadline) {
        auto e = raw.read_encoder_addition();
        if (!e.ok()) { std::printf("       read failed during wait\n"); return; }
        const std::int64_t d = e.value - target;
        if ((d < 0 ? -d : d) <= 50) { reached = true; break; }
    }
    if (!reached) { std::printf("       never reached window\n"); return; }

    // Capture encoder evolution post-window. Each read ~1ms; sleep ~4ms → 5ms cadence.
    const int N_SAMPLES = 60;
    std::vector<double> delta_counts;
    delta_counts.reserve(static_cast<std::size_t>(N_SAMPLES));
    std::vector<std::uint64_t> times;
    times.reserve(static_cast<std::size_t>(N_SAMPLES));
    const std::uint64_t t_origin = now_us();
    for (int i = 0; i < N_SAMPLES; ++i) {
        auto r = raw.read_encoder_addition();
        const std::uint64_t t = now_us();
        if (r.ok()) {
            delta_counts.push_back(static_cast<double>(r.value - target));
            times.push_back(t - t_origin);
        }
        struct timespec ts{0, 4'000'000L};
        ::nanosleep(&ts, nullptr);
    }

    // Move back to origin to keep cumulative position sane
    raw.move_relative_axis(-delta, 2000, 200);
    sleep_ms(300);

    constexpr double COUNTS_TO_DEG = 360.0 / 16384.0;
    double peak_counts = 0;
    for (double d : delta_counts) {
        if (std::fabs(d) > std::fabs(peak_counts)) peak_counts = d;
    }
    // Settle: first time after which |delta| stays < 0.5° (~23 counts) permanently
    const double SETTLE_TOL_COUNTS = 0.5 / COUNTS_TO_DEG;
    double settle_t = -1;
    for (std::size_t i = 0; i < delta_counts.size(); ++i) {
        bool stays = true;
        for (std::size_t j = i; j < delta_counts.size(); ++j) {
            if (std::fabs(delta_counts[j]) > SETTLE_TOL_COUNTS) { stays = false; break; }
        }
        if (stays) { settle_t = static_cast<double>(times[i]) / 1000.0; break; }
    }
    rep.t5_peak_overshoot_deg  = peak_counts * COUNTS_TO_DEG;
    rep.t5_settle_to_05deg_ms  = settle_t;
    rep.t5_valid = !delta_counts.empty();
    std::printf("       peak overshoot: %+.2f deg  settle-to-0.5deg: %s ms\n",
        rep.t5_peak_overshoot_deg,
        settle_t < 0 ? "(did not settle within window)"
                     : std::to_string(static_cast<int>(settle_t)).c_str());
}

static void run_test6_soak(Motor& m, EnvelopeReport& rep, int N = 100) {
    std::printf("[6/6] SOAK (%d alternating 90deg, acc=255 rpm=2000)...\n", N);
    std::fflush(stdout);
    rep.t6_n = N;
    std::vector<double> windows, settles;
    windows.reserve(static_cast<std::size_t>(N));
    settles.reserve(static_cast<std::size_t>(N));
    int dir = +1;
    for (int i = 0; i < N; ++i) {
        auto r = do_90deg(m, dir, MoveParams{2000, 255});
        if (r.ok) {
            windows.push_back(r.t_window_ms);
            settles.push_back(r.t_settle_ms);
            ++rep.t6_successes;
        }
        dir = -dir;
        sleep_ms(100);
        if ((i + 1) % 20 == 0) {
            std::printf("       progress: %d/%d  ok=%d\n", i + 1, N, rep.t6_successes);
            std::fflush(stdout);
        }
    }
    rep.t6_window_ms = compute(windows);
    rep.t6_settle_ms = compute(settles);
    rep.t6_valid = (rep.t6_successes >= (9 * N) / 10);
    std::printf("       successes=%d/%d  window mean=%.2fms sigma=%.2fms p99=%.2fms  "
                "settle mean=%.2fms  %s\n",
        rep.t6_successes, N,
        rep.t6_window_ms.mean, rep.t6_window_ms.sigma, rep.t6_window_ms.p99,
        rep.t6_settle_ms.mean,
        rep.t6_valid ? "VALID" : "INVALID");
}

static void print_summary(const EnvelopeReport& r) {
    std::printf("\n=== ENVELOPE SUMMARY ===\n");
    std::printf("comms p99:              %.0f us\n", r.comms_latency_us.p99);
    std::printf("max steady RPM:         %.0f", r.t3_max_steady_rpm);
    if (r.t3_knee_cmd) std::printf("  (firmware knee at cmd=%d)", (int)r.t3_knee_cmd);
    std::printf("\n");
    if (!r.t2_window_ms.empty()) {
        // Best (lowest mean) acc
        std::size_t best_i = 0;
        for (std::size_t i = 1; i < r.t2_window_ms.size(); ++i) {
            if (r.t2_window_ms[i].mean > 0
                && (r.t2_window_ms[best_i].mean == 0
                 || r.t2_window_ms[i].mean < r.t2_window_ms[best_i].mean)) {
                best_i = i;
            }
        }
        std::printf("min 90deg time:         %.2f ms  (at acc=%d rpm=2000)\n",
            r.t2_window_ms[best_i].mean, (int)r.t2_accs[best_i]);
    }
    if (r.t4_flat_rpm) {
        std::printf("90deg time flat above:  rpm=%d  (no benefit pushing rpm higher)\n",
            (int)r.t4_flat_rpm);
    }
    std::printf("overshoot @ max-acc:    %+.2f deg  (settle %s ms)\n",
        r.t5_peak_overshoot_deg,
        r.t5_settle_to_05deg_ms < 0 ? ">300"
            : std::to_string(static_cast<int>(r.t5_settle_to_05deg_ms)).c_str());
    if (r.t6_n > 0) {
        std::printf("soak %d×:               %d/%d ok  sigma=%.2fms  settle mean=%.2fms\n",
            r.t6_n, r.t6_successes, r.t6_n,
            r.t6_window_ms.sigma, r.t6_settle_ms.mean);
    }

    std::printf("\nRECOMMENDED OPERATING POINT (rough heuristics, verify under your load):\n");
    if (r.t6_valid) {
        std::printf("  rpm=2000  acc=255    -> %.0f ms per 90deg  (PROVEN under soak)\n",
            r.t6_window_ms.mean);
    } else {
        // Fall back to acc that had best stable time in test 2
        std::printf("  (soak did not validate max params — re-run with less load)\n");
    }
}

static void maybe_write_json(const EnvelopeReport& r, const char* path) {
    if (!path) return;
    FILE* f = std::fopen(path, "w");
    if (!f) { std::fprintf(stderr, "could not open %s\n", path); return; }
    std::fprintf(f, "{\n");
    std::fprintf(f, "  \"baud\": %d,\n", r.baud);
    std::fprintf(f, "  \"comms_latency_us\": {\"mean\": %.1f, \"p50\": %.1f, \"p99\": %.1f, \"max\": %.1f, \"valid\": %s},\n",
        r.comms_latency_us.mean, r.comms_latency_us.p50, r.comms_latency_us.p99,
        r.comms_latency_us.mx, r.comms_valid ? "true" : "false");
    std::fprintf(f, "  \"acc_sweep\": [");
    for (std::size_t i = 0; i < r.t2_accs.size(); ++i) {
        std::fprintf(f, "%s{\"acc\": %d, \"mean_ms\": %.2f, \"sigma_ms\": %.2f, \"ok\": %d}",
            i ? ", " : "", (int)r.t2_accs[i], r.t2_window_ms[i].mean,
            r.t2_window_ms[i].sigma, r.t2_ok[i]);
    }
    std::fprintf(f, "],\n");
    std::fprintf(f, "  \"rpm_sweep\": {\"max_steady\": %.0f, \"knee\": %d, \"points\": [",
        r.t3_max_steady_rpm, (int)r.t3_knee_cmd);
    for (std::size_t i = 0; i < r.t3_cmds.size(); ++i) {
        std::fprintf(f, "%s{\"cmd\": %d, \"real\": %.1f}",
            i ? ", " : "", (int)r.t3_cmds[i], r.t3_real_rpm[i]);
    }
    std::fprintf(f, "]},\n");
    std::fprintf(f, "  \"rpm_vs_time\": {\"flat_rpm\": %d, \"points\": [",
        (int)r.t4_flat_rpm);
    for (std::size_t i = 0; i < r.t4_rpms.size(); ++i) {
        std::fprintf(f, "%s{\"rpm\": %d, \"mean_ms\": %.2f, \"sigma_ms\": %.2f}",
            i ? ", " : "", (int)r.t4_rpms[i], r.t4_window_ms[i].mean,
            r.t4_window_ms[i].sigma);
    }
    std::fprintf(f, "]},\n");
    std::fprintf(f, "  \"overshoot\": {\"peak_deg\": %.2f, \"settle_to_0.5deg_ms\": %.1f},\n",
        r.t5_peak_overshoot_deg, r.t5_settle_to_05deg_ms);
    std::fprintf(f, "  \"soak\": {\"n\": %d, \"successes\": %d, \"window_mean_ms\": %.2f, "
                    "\"window_sigma_ms\": %.2f, \"settle_mean_ms\": %.2f}\n",
        r.t6_n, r.t6_successes, r.t6_window_ms.mean,
        r.t6_window_ms.sigma, r.t6_settle_ms.mean);
    std::fprintf(f, "}\n");
    std::fclose(f);
    std::printf("\nJSON report written to %s\n", path);
}

// ─── main ───────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    const char* dev    = "/dev/ttyUSB0";
    int         addr   = 1;
    const char* json   = nullptr;
    // Positional: [dev] [addr]    Optional: --json <path>
    int positional = 0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--json") == 0 && i + 1 < argc) {
            json = argv[++i];
        } else if (argv[i][0] != '-') {
            if (positional == 0) dev = argv[i];
            else if (positional == 1) addr = std::atoi(argv[i]);
            ++positional;
        }
    }

    EnvelopeReport rep;

    // Probe baud (try 256k first, then 38400)
    int baud = 0;
    {
        Transport t;
        for (int b : {256000, 38400}) {
            if (t.open(dev, b) != Transport::Status::OK) continue;
            RawDriver p(t, static_cast<std::uint8_t>(addr));
            auto e = p.read_encoder_addition();
            if (e.ok()) { baud = b; break; }
            t.close();
        }
    }
    if (!baud) {
        std::fprintf(stderr, "no response from %s @ 256k or 38400\n", dev);
        return 1;
    }
    if (baud != 256000) {
        std::printf("switching motor to 256000 baud (was %d)...\n", baud);
        send_set_baud(dev, baud, 0x07);
    }
    rep.baud = 256000;

    Transport t;
    if (t.open(dev, 256000) != Transport::Status::OK) {
        std::fprintf(stderr, "open at 256k failed\n");
        return 2;
    }
    RawDriver raw(t, static_cast<std::uint8_t>(addr));
    Motor m(raw, Mechanical{1.0, 0});

    auto en = raw.enable(true);
    if (!en.ok() || !en.value) {
        std::fprintf(stderr, "enable failed (transport_ok=%d firmware_ack=%d)\n",
                     (int)en.ok(), (int)en.value);
        return 3;
    }
    if (!m.set_origin().ok()) {
        std::fprintf(stderr, "set_origin failed\n");
        raw.enable(false);
        return 4;
    }

    std::printf("=== mks-servo envelope characterisation ===\n");
    std::printf("device: %s @ %d baud  addr=%d\n", dev, rep.baud, addr);
    std::printf("estimated runtime: ~80s\n\n");

    // Order matters: MOVE_SPEED (test 3) leaves the firmware's axis-position
    // frame badly out of sync with the encoder addition register, breaking
    // any MOVE_ABS_AXIS that follows in the same session. We run all the
    // relative-move tests first (which are frame-agnostic, since do_90deg
    // goes through raw.move_relative_axis), and put MOVE_SPEED last.
    const std::uint64_t total_start = now_us();
    run_test1_comms(raw, rep);
    run_test2_acc_sweep(m, rep);
    run_test4_rpm_sweep(m, rep);
    run_test5_overshoot(m, rep);
    run_test6_soak(m, rep);
    run_test3_max_rpm(raw, rep);
    const std::uint64_t total_end = now_us();
    std::printf("\ntotal runtime: %.1f s\n",
        static_cast<double>(total_end - total_start) / 1e6);

    print_summary(rep);
    maybe_write_json(rep, json);

    raw.enable(false);
    t.close();
    if (baud != 256000) send_set_baud(dev, 256000, 0x04);
    return 0;
}
