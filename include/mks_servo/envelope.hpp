// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 mks-servo-cpp contributors
//
// Envelope + auto_calibrate: discover the safe-fast operating point of
// this motor + supply + load combination at startup, so user code can
// just write `m.write(angle)` instead of guessing rpm/acc parameters.
//
// Philosophy: the lib should be a toy that finds its own limits. The
// envelope captures the four numbers that matter for picking
// MoveParams: max steady RPM, a working recommended RPM, a working
// recommended acceleration, and the measured 90° time at those params.
// Plus comms-latency and a 10-move mini-soak result for confidence.
//
// Cost: ~3 seconds. Motor will rotate up to ~3 full revolutions during
// the max-RPM probe — make sure the load can take it. Doesn't write to
// flash; safe to call repeatedly.
//
// Typical usage:
//   Motor m(raw);
//   raw.enable(true);
//   m.set_origin();
//   auto env = auto_calibrate(m);
//   if (env.valid) {
//       // use env.recommended_rpm / env.recommended_acc going forward
//       m.write(90, MoveParams{env.recommended_rpm, env.recommended_acc});
//   }
//
// Persist `env` to disk (it's plain-old-data) for instant reuse on
// next startup; recharacterise when the setup (load / supply / motor)
// changes.

#ifndef MKS_SERVO_ENVELOPE_HPP
#define MKS_SERVO_ENVELOPE_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>

#include "mks_servo/motor.hpp"
#include "mks_servo/raw_driver.hpp"

namespace mks_servo {

struct Envelope {
    // Validity: true iff the calibration ran end-to-end and the motor
    // accepted at least the safe-conservative params (rpm=1000 acc=150).
    // false means either the motor didn't respond or it stalls even at
    // gentle params — caller should treat the recommendation as untrusted.
    bool           valid                = false;

    // Comms
    double         comms_latency_us_p50 = 0;
    double         comms_latency_us_p99 = 0;

    // Steady-state speed envelope (firmware-limited at given VIN/current)
    std::uint16_t  max_steady_rpm       = 0;   // cmd RPM at which real RPM saturates

    // Recommended operating point: the most aggressive {rpm, acc} that
    // passed the mini-soak (10 alternating 90° moves, all successful).
    // Falls back to safer values if the aggressive tier had failures.
    std::uint16_t  recommended_rpm      = 0;
    std::uint8_t   recommended_acc      = 0;

    // Measured time for one 90° move at the recommended params (motion
    // only, time-to-window) and the worst overshoot observed.
    double         t_90deg_ms_mean      = 0;
    double         t_90deg_ms_sigma     = 0;
    double         overshoot_peak_deg   = 0;

    // Mini-soak result at recommended params (count of successes out of
    // soak_n). With healthy hardware this is soak_n/soak_n; partial means
    // either the load isn't supporting the params, or the supply is sagging.
    int            soak_n               = 0;
    int            soak_successes       = 0;
};

// ─── implementation ────────────────────────────────────────────────

namespace detail {

inline std::uint64_t ec_now_us() noexcept {
    struct timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000ull
         + static_cast<std::uint64_t>(ts.tv_nsec) / 1000ull;
}

inline void ec_sleep_ms(int ms) noexcept {
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1'000'000L;
    ::nanosleep(&ts, nullptr);
}

// One 90° relative move with motion-only timing + overshoot snapshot.
// Frame-agnostic by going through move_relative_axis (no abs-frame
// dependency on prior MOVE_SPEED state).
struct EcMove90 {
    bool   ok                 = false;
    double t_window_ms        = 0;
    double overshoot_counts   = 0;
};
inline EcMove90 ec_do_90deg(Motor& m, int dir, MoveParams mp,
                            std::int32_t tol_counts = 50,
                            std::uint64_t timeout_us = 3'000'000) {
    EcMove90 r;
    auto& raw = m.raw();
    auto e0 = raw.read_encoder_addition();
    if (!e0.ok()) return r;
    constexpr std::int32_t QTR = 4096;  // 16384/4
    const std::int32_t delta = QTR * dir;
    const std::int64_t target = e0.value + delta;

    const std::uint64_t t0 = ec_now_us();
    auto disp = raw.move_relative_axis(delta, mp.rpm, mp.acc);
    if (!disp.ok() || !disp.value) return r;

    const std::uint64_t deadline = ec_now_us() + timeout_us;
    bool reached = false;
    while (ec_now_us() < deadline) {
        auto e = raw.read_encoder_addition();
        if (!e.ok()) return r;
        const std::int64_t diff  = e.value - target;
        const std::int64_t adiff = diff < 0 ? -diff : diff;
        if (adiff <= tol_counts) { reached = true; break; }
    }
    const std::uint64_t t1 = ec_now_us();
    if (!reached) return r;

    auto imm = raw.read_encoder_addition();
    if (imm.ok()) {
        r.overshoot_counts = static_cast<double>(imm.value - target)
                           * static_cast<double>(dir);
    }
    raw.transport_drain_settle(15);

    r.ok           = true;
    r.t_window_ms  = static_cast<double>(t1 - t0) / 1000.0;
    return r;
}

// 250ms-window plateau-detected real RPM via MOVE_SPEED.
// Returns -1 if no plateau in max_windows iterations.
inline double ec_measure_rpm_plateau(RawDriver& raw, std::uint16_t cmd_rpm,
                                     std::uint8_t acc = 255,
                                     int window_ms = 250,
                                     int max_windows = 10) {
    auto mv = raw.move_speed(cmd_rpm, acc, Direction::CW);
    if (!mv.ok() || !mv.value) return -1;

    auto sample = [&]() -> double {
        auto a = raw.read_encoder_addition();
        const std::uint64_t t0 = ec_now_us();
        ec_sleep_ms(window_ms);
        auto b = raw.read_encoder_addition();
        const std::uint64_t t1 = ec_now_us();
        if (!a.ok() || !b.ok()) return -1;
        double dc = static_cast<double>(b.value - a.value);
        if (dc < 0) dc = -dc;
        return (dc / 16384.0) / (static_cast<double>(t1 - t0) / 1e6) * 60.0;
    };

    double prev = -1, prev2 = -1, plateau = -1;
    for (int k = 0; k < max_windows; ++k) {
        const double r = sample();
        if (r < 0) break;
        if (prev > 0 && prev2 > 0) {
            const double lo = std::min({r, prev, prev2});
            const double hi = std::max({r, prev, prev2});
            if (hi > 0 && (hi - lo) / hi < 0.03) {
                plateau = (r + prev + prev2) / 3.0;
                break;
            }
        }
        prev2 = prev;
        prev  = r;
    }
    raw.move_speed(0, acc, Direction::CW);
    ec_sleep_ms(500);
    raw.transport_drain_settle(15);
    return plateau;
}

}  // namespace detail

// auto_calibrate: ~3 second characterisation, picks the best params
// that pass a 10-move mini-soak.
//
// Assumes:
//   - Motor is enabled (raw.enable(true) called by caller)
//   - The shaft can rotate freely (or under a load that tolerates the
//     candidate move params)
//
// Side effects:
//   - Performs ~10 relative quarter-turn moves at various param sets
//   - Performs one MOVE_SPEED probe for ~2.5s rotating ~50-100 revs
//   - Returns motor to enabled, idle state; motor will not be at any
//     specific position (use set_origin afterwards if needed)
//
// The function is noexcept; on any unrecoverable failure (transport
// error, persistent firmware refusal) it returns Envelope{valid=false}.
inline Envelope auto_calibrate(Motor& m) noexcept {
    Envelope env;
    auto& raw = m.raw();

    // ── Step 1: comms latency (100 samples) ───────────────────────────
    {
        std::vector<double> us;
        us.reserve(100);
        for (int i = 0; i < 100; ++i) {
            const std::uint64_t t0 = detail::ec_now_us();
            auto e = raw.read_encoder_addition();
            const std::uint64_t t1 = detail::ec_now_us();
            if (e.ok()) us.push_back(static_cast<double>(t1 - t0));
        }
        if (us.empty()) return env;  // motor not responding
        std::sort(us.begin(), us.end());
        env.comms_latency_us_p50 = us[us.size() / 2];
        env.comms_latency_us_p99 = us[std::min(us.size() * 99 / 100, us.size() - 1)];
    }

    // ── Step 2: try param tiers, from aggressive down to safe ────────
    // Find the highest tier that passes a tiny pre-soak (4 alternating
    // 90° moves all ok with sigma < 1ms).
    struct Tier { std::uint16_t rpm; std::uint8_t acc; };
    static constexpr Tier TIERS[] = {
        {2000, 255},  // aggressive (40-41ms at 24V)
        {2000, 200},  // mid-aggressive
        {1500, 200},  // safe-fast
        {1000, 150},  // conservative
        { 600, 128},  // gentle (last resort)
    };

    Tier picked{0, 0};
    double picked_t_mean = 0, picked_t_sigma = 0, picked_overshoot = 0;
    for (const Tier& tier : TIERS) {
        std::vector<double> times;
        std::vector<double> overshoots;
        int ok_count = 0;
        for (int i = 0; i < 4; ++i) {
            auto r = detail::ec_do_90deg(m, (i % 2 ? -1 : +1),
                                         MoveParams{tier.rpm, tier.acc});
            if (r.ok) {
                times.push_back(r.t_window_ms);
                overshoots.push_back(r.overshoot_counts);
                ++ok_count;
            }
            detail::ec_sleep_ms(80);
        }
        if (ok_count < 4) continue;
        double sum = 0; for (double x : times) sum += x;
        const double mean = sum / static_cast<double>(times.size());
        double var = 0; for (double x : times) var += (x - mean) * (x - mean);
        const double sigma = std::sqrt(var / static_cast<double>(times.size()));
        if (sigma / mean > 0.05) continue;  // too noisy at this tier

        // Tier is good. Record and break.
        picked = tier;
        picked_t_mean = mean;
        picked_t_sigma = sigma;
        for (double o : overshoots) {
            if (std::fabs(o) > std::fabs(picked_overshoot)) picked_overshoot = o;
        }
        break;
    }
    if (picked.rpm == 0) return env;  // no tier worked

    env.recommended_rpm     = picked.rpm;
    env.recommended_acc     = picked.acc;
    env.t_90deg_ms_mean     = picked_t_mean;
    env.t_90deg_ms_sigma    = picked_t_sigma;
    constexpr double COUNTS_TO_DEG = 360.0 / 16384.0;
    env.overshoot_peak_deg  = picked_overshoot * COUNTS_TO_DEG;

    // ── Step 3: max steady RPM (quick plateau probe at cmd=2400) ─────
    // 2400 is comfortably above the typical 24V firmware knee (~2100)
    // so we observe saturation in a single shot. If real RPM ≈ cmd,
    // the motor isn't yet saturated → try higher (cmd=3000).
    {
        double r = detail::ec_measure_rpm_plateau(raw, 2400);
        if (r > 0) {
            env.max_steady_rpm = static_cast<std::uint16_t>(r);
            if (r > 2300) {
                // Not saturated; try one step higher.
                double r2 = detail::ec_measure_rpm_plateau(raw, 3000);
                if (r2 > 0 && r2 > r) env.max_steady_rpm = static_cast<std::uint16_t>(r2);
            }
        }
    }

    // ── Step 4: mini-soak at recommended params (10 alternating moves) ─
    {
        env.soak_n = 10;
        for (int i = 0; i < env.soak_n; ++i) {
            auto r = detail::ec_do_90deg(m, (i % 2 ? -1 : +1),
                                         MoveParams{env.recommended_rpm,
                                                    env.recommended_acc});
            if (r.ok) ++env.soak_successes;
            detail::ec_sleep_ms(80);
        }
    }

    // ── Clean up: clear any latched stall protection from aggressive
    // testing, drain the bus, and give the closed-loop time to fully
    // settle before handing back to the caller. This keeps the motor in
    // a known-good state so the first user m.write after auto_calibrate
    // doesn't trip on a lingering ack or a still-oscillating rotor.
    (void)raw.release_protection();
    raw.transport_drain_settle(15);
    detail::ec_sleep_ms(200);
    // If mini-soak passed at <80% success, drop one tier and re-record
    // the recommendation as the safer one (don't re-soak — keep cost
    // bounded to one extra ~3s call if caller wants more confidence).
    if (env.soak_successes * 10 < env.soak_n * 8) {
        // Find current tier's index and step down one
        for (std::size_t i = 0; i + 1 < sizeof(TIERS)/sizeof(TIERS[0]); ++i) {
            if (TIERS[i].rpm == env.recommended_rpm
             && TIERS[i].acc == env.recommended_acc) {
                env.recommended_rpm = TIERS[i + 1].rpm;
                env.recommended_acc = TIERS[i + 1].acc;
                break;
            }
        }
    }

    env.valid = true;
    return env;
}

// ─── persistence ────────────────────────────────────────────────────
//
// Binary, fixed-layout format — no JSON parser, no version drift
// games. The Envelope struct itself is POD, so we just write the
// magic + version + sizeof + raw bytes.
//
// Why binary: the file is a CACHE, not a config. Users don't edit it
// by hand; the lib writes and reads it. Plain bytes mean
// load_envelope can validate the file in 4 ifs, no parser.

constexpr std::uint32_t ENVELOPE_MAGIC   = 0x4D4B4543;  // "MKEC" little-endian
constexpr std::uint32_t ENVELOPE_VERSION = 1;

struct EnvelopeFileHeader {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t struct_bytes;  // sizeof(Envelope) at write time
    std::uint32_t reserved;
};

// Write `env` to `path` in binary form. Returns true on success.
// Overwrites the file if it exists. Safe to call with any Envelope
// (including {valid=false}) — the caller decides whether persisting
// a bad result is useful.
inline bool save_envelope(const Envelope& env, const char* path) noexcept {
    std::FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    EnvelopeFileHeader h{
        ENVELOPE_MAGIC,
        ENVELOPE_VERSION,
        static_cast<std::uint32_t>(sizeof(Envelope)),
        0,
    };
    bool ok = std::fwrite(&h, sizeof(h), 1, f) == 1
           && std::fwrite(&env, sizeof(env), 1, f) == 1;
    std::fclose(f);
    return ok;
}

// Load an Envelope previously written by save_envelope. Returns
// Envelope{valid=false} on any failure (missing file, wrong magic,
// version mismatch, size mismatch, short read). The caller should
// re-run auto_calibrate in that case.
inline Envelope load_envelope(const char* path) noexcept {
    Envelope env;
    std::FILE* f = std::fopen(path, "rb");
    if (!f) return env;
    EnvelopeFileHeader h{};
    if (std::fread(&h, sizeof(h), 1, f) != 1) { std::fclose(f); return env; }
    if (h.magic        != ENVELOPE_MAGIC)        { std::fclose(f); return env; }
    if (h.version      != ENVELOPE_VERSION)      { std::fclose(f); return env; }
    if (h.struct_bytes != sizeof(Envelope))      { std::fclose(f); return env; }
    if (std::fread(&env, sizeof(env), 1, f) != 1) {
        // Partial read — re-default-init to be safe and mark invalid.
        env = Envelope{};
        std::fclose(f);
        return env;
    }
    std::fclose(f);
    return env;
}

// Cache-friendly helper: load envelope from `path` and run a quick
// sanity probe to confirm the motor is still reachable. If yes, return
// the cached envelope (instant, ~50 ms). Otherwise run a full
// auto_calibrate, save the result, and return it.
//
// Validation: ONLY comms latency, not motion. A MOVE_SPEED-based
// recalibration probe would rotate the motor for several seconds (a
// non-instant cache that no one wants), AND leave the firmware in a
// state where SET_ZERO_POINT misbehaves immediately after — every
// downstream m.write would have a small window of unreliable behavior.
// If your setup (load, supply, motor) changes between runs, the new
// behavior will show up as overshoot or stalls; force a fresh
// calibration by calling auto_calibrate(m) directly (or remove the
// cache file).
//
// Returns Envelope{valid=false} only if both load AND fresh
// auto_calibrate fail.
inline Envelope auto_calibrate_cached(Motor& m, const char* path) noexcept {
    Envelope cached = load_envelope(path);
    if (cached.valid) {
        auto& raw = m.raw();
        // Cheap comms-only validation. ~50ms total.
        std::vector<double> us;
        us.reserve(50);
        for (int i = 0; i < 50; ++i) {
            const std::uint64_t t0 = detail::ec_now_us();
            auto e = raw.read_encoder_addition();
            const std::uint64_t t1 = detail::ec_now_us();
            if (e.ok()) us.push_back(static_cast<double>(t1 - t0));
        }
        if (us.empty()) {
            // Motor unreachable now; cached data is useless either way.
            cached.valid = false;
            return cached;
        }
        std::sort(us.begin(), us.end());
        const double p99_now = us[us.size() * 99 / 100];
        const bool comms_ok = p99_now < 1.5 * cached.comms_latency_us_p99;

        if (comms_ok) {
            return cached;  // cache hit — instant
        }
    }

    // Cache miss / stale / first run: full calibration.
    Envelope fresh = auto_calibrate(m);
    if (fresh.valid) (void)save_envelope(fresh, path);
    return fresh;
}

}  // namespace mks_servo

#endif  // MKS_SERVO_ENVELOPE_HPP
