// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 mks-servo-cpp contributors
//
// Motor: Level 0 + Level 1 ergonomic API on top of RawDriver.
//
// The same role as `mks_servo.motor.Motor` in the Python reference:
//   - angle_deg ↔ encoder counts conversion (gear_ratio + origin_offset)
//   - write(angle) / read() / move_relative(delta)
//   - set_origin() / set_origin_offset_counts()
//   - optional soft limits on position and speed (in-memory, not persisted)
//   - blocking variants poll the encoder (NOT motor_status, which is
//     unreliable per HIL findings — see Python lib docstring)
//
// Design choices specific to C++:
//   - No exceptions: every method that can fail returns Result<T> or
//     Transport::Status.
//   - No allocation in the hot path.
//   - The mechanical config is value-typed (`Mechanical`) and copyable; a
//     full Profile (with YAML I/O) layers on top in a future header.

#ifndef MKS_SERVO_MOTOR_HPP
#define MKS_SERVO_MOTOR_HPP

#include <cmath>
#include <cstdint>
#include <ctime>

#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

namespace mks_servo {

inline constexpr std::int32_t ENCODER_COUNTS_PER_REV = 0x4000;  // 16384
inline constexpr std::int32_t ANGLE_ERROR_UNITS_PER_REV = 51200;

// What to do when a soft limit is exceeded.
enum class OnViolation : std::uint8_t {
    Reject,  // refuse the operation, return Status::LimitExceeded
    Clamp,   // silently clamp to the limit
    Warn,    // proceed but signal the violation via Status::LimitWarned
};

// Mechanical / configuration data. Mirrors the Python ProfileSection
// fields that are needed for math; full Profile lives elsewhere.
struct Mechanical {
    double       gear_ratio              = 1.0;    // motor_rev / output_rev
    std::int32_t origin_offset_counts    = 0;
};

struct PositionLimits {
    bool         enabled = false;
    double       min_deg = 0.0;
    double       max_deg = 0.0;
    OnViolation  policy  = OnViolation::Reject;
};

struct SpeedLimits {
    bool          enabled    = false;
    std::uint16_t max_rpm    = 0;
    OnViolation   policy     = OnViolation::Clamp;
};

// Move parameters with sensible defaults that match the Python lib.
struct MoveParams {
    std::uint16_t rpm = 300;
    std::uint8_t  acc = 50;
};

// Extended status enum for Motor-level operations: includes the
// soft-limit-related outcomes that don't exist at the Transport level.
enum class MotorStatusEx : std::uint8_t {
    OK              = 0,
    TransportError  = 1,
    ParseError      = 2,
    LimitExceeded   = 3,  // rejected by Reject policy
    LimitWarned     = 4,  // proceeded despite Warn policy
    Timeout         = 5,  // wait_until_idle gave up
    NotEnabled      = 6,  // the driver was never enabled
};

template <typename T>
struct MResult {
    T              value   = T{};
    MotorStatusEx  status  = MotorStatusEx::OK;

    bool ok() const noexcept {
        return status == MotorStatusEx::OK || status == MotorStatusEx::LimitWarned;
    }
};

// ─── Motor ─────────────────────────────────────────────────────────
class Motor {
public:
    Motor(RawDriver& raw, Mechanical mech = {}) noexcept
        : raw_(raw)
        , mech_(mech) {}

    // ─── Configuration ─────────────────────────────────────────
    const Mechanical&     mechanical()      const noexcept { return mech_; }
    void set_mechanical(const Mechanical& m) noexcept { mech_ = m; }

    void set_gear_ratio(double r)                  noexcept { mech_.gear_ratio = r; }
    void set_origin_offset_counts(std::int32_t c)  noexcept { mech_.origin_offset_counts = c; }

    // Auto-clear stall protection on dispatch refusal.
    // If true, Motor::write detects when the firmware refuses a MOVE
    // (ack payload 0x00 — usually because stall protection latched),
    // issues RELEASE_PROTECTION, and retries the MOVE once. The retry
    // result is what gets returned to the caller. Off by default so
    // applications make an explicit opt-in to silent recovery.
    bool auto_clear_protection() const noexcept { return auto_clear_protection_; }
    void set_auto_clear_protection(bool on) noexcept { auto_clear_protection_ = on; }

    const PositionLimits& position_limits() const noexcept { return pos_lim_; }
    void set_position_limits(double min_deg, double max_deg,
                             OnViolation policy = OnViolation::Reject) noexcept {
        pos_lim_.enabled = true;
        pos_lim_.min_deg = min_deg;
        pos_lim_.max_deg = max_deg;
        pos_lim_.policy  = policy;
    }
    void clear_position_limits() noexcept { pos_lim_.enabled = false; }

    const SpeedLimits&    speed_limits()    const noexcept { return spd_lim_; }
    void set_speed_limit_rpm(std::uint16_t max_rpm,
                             OnViolation policy = OnViolation::Clamp) noexcept {
        spd_lim_.enabled = true;
        spd_lim_.max_rpm = max_rpm;
        spd_lim_.policy  = policy;
    }
    void clear_speed_limit() noexcept { spd_lim_.enabled = false; }

    // ─── Conversions ───────────────────────────────────────────
    std::int32_t angle_to_counts(double angle_deg) const noexcept {
        const double motor_deg = angle_deg * mech_.gear_ratio;
        const double counts    = motor_deg * ENCODER_COUNTS_PER_REV / 360.0;
        const std::int32_t rounded = static_cast<std::int32_t>(
            counts >= 0 ? counts + 0.5 : counts - 0.5);
        return static_cast<std::int32_t>(rounded - mech_.origin_offset_counts);
    }

    double counts_to_angle(std::int64_t counts) const noexcept {
        const double motor_counts = static_cast<double>(counts - mech_.origin_offset_counts);
        const double motor_deg    = motor_counts * 360.0 / ENCODER_COUNTS_PER_REV;
        return motor_deg / mech_.gear_ratio;
    }

    // ─── Level 0: write / read ─────────────────────────────────
    //
    // Move to an absolute output-axis angle. If blocking, polls the encoder
    // until the target is reached within `tolerance_counts`, or until
    // `timeout_us` elapses.
    //
    // Polling uses read_encoder_addition (safe during motion) — never
    // read_motor_status or read_speed_rpm (which sabotage closed-loop moves
    // when polled aggressively; see HIL findings in benchmark-mks-baseline).
    MResult<bool> write(double         angle_deg,
                        MoveParams     mp                  = {},
                        bool           blocking            = true,
                        std::int32_t   tolerance_counts    = 16,   // ~0.35°
                        std::uint64_t  timeout_us          = 5'000'000) noexcept {
        MResult<bool> out;

        // Apply position policy (mutates angle_deg if Clamp).
        const auto pos_outcome = apply_position_policy(angle_deg);
        if (pos_outcome == MotorStatusEx::LimitExceeded) {
            out.status = MotorStatusEx::LimitExceeded;
            return out;
        }

        // Apply speed policy.
        std::uint16_t rpm = mp.rpm;
        const auto spd_outcome = apply_speed_policy(rpm);
        if (spd_outcome == MotorStatusEx::LimitExceeded) {
            out.status = MotorStatusEx::LimitExceeded;
            return out;
        }

        const std::int32_t target_counts = angle_to_counts(angle_deg);
        // Use the ack-waiting MOVE: it reads the firmware's "started"
        // response (the 1st of two it emits per move). This leaves only
        // the "complete" response in the bus buffer, which drain_input on
        // the next transaction discards cleanly. Without this read, both
        // responses race against subsequent encoder polls and the scan
        // loop occasionally exceeds its byte budget under timing noise.
        // Users who need maximum dispatch speed can call dispatch_write()
        // and manage the bus state themselves.
        auto r = raw_.move_absolute_axis(target_counts, rpm, mp.acc);
        if (!r.ok()) {
            out.status = (r.t_status != Transport::Status::OK)
                ? MotorStatusEx::TransportError
                : MotorStatusEx::ParseError;
            return out;
        }
        if (!r.value && auto_clear_protection_) {
            // Firmware refused — likely stall protection latched. Try to
            // recover by clearing the latch and re-issuing the MOVE.
            (void)raw_.release_protection();
            sleep_us(200'000);
            r = raw_.move_absolute_axis(target_counts, rpm, mp.acc);
            if (!r.ok()) {
                out.status = (r.t_status != Transport::Status::OK)
                    ? MotorStatusEx::TransportError
                    : MotorStatusEx::ParseError;
                return out;
            }
        }
        if (!r.value) {
            out.status = MotorStatusEx::NotEnabled;
            return out;
        }

        if (blocking) {
            const auto wait_status = wait_for_position(target_counts,
                                                       tolerance_counts,
                                                       timeout_us);
            if (wait_status != MotorStatusEx::OK) {
                out.status = wait_status;
                return out;
            }
            // wait_for_position has its own settle_drain pass that absorbs
            // the firmware's trailing "complete" ack — no extra step needed.
        }

        out.value  = true;
        out.status = (pos_outcome == MotorStatusEx::LimitWarned
                   || spd_outcome == MotorStatusEx::LimitWarned)
                       ? MotorStatusEx::LimitWarned
                       : MotorStatusEx::OK;
        return out;
    }

    // Fire-and-forget move (no ack, no blocking wait). For RT use cases.
    Transport::Status dispatch_write(double angle_deg, MoveParams mp = {}) noexcept {
        // Clamp policy only; reject becomes a no-op here (no Result to signal).
        if (pos_lim_.enabled && pos_lim_.policy == OnViolation::Clamp) {
            if (angle_deg < pos_lim_.min_deg) angle_deg = pos_lim_.min_deg;
            if (angle_deg > pos_lim_.max_deg) angle_deg = pos_lim_.max_deg;
        }
        std::uint16_t rpm = mp.rpm;
        if (spd_lim_.enabled && rpm > spd_lim_.max_rpm
            && spd_lim_.policy == OnViolation::Clamp) {
            rpm = spd_lim_.max_rpm;
        }
        const std::int32_t counts = angle_to_counts(angle_deg);
        return raw_.dispatch_move_absolute_axis(counts, rpm, mp.acc);
    }

    MResult<double> read() noexcept {
        MResult<double> out;
        const auto r = raw_.read_encoder_addition();
        if (!r.ok()) {
            out.status = (r.t_status != Transport::Status::OK)
                ? MotorStatusEx::TransportError
                : MotorStatusEx::ParseError;
            return out;
        }
        out.value = counts_to_angle(r.value);
        return out;
    }

    // Following error in output-axis degrees (driver units → degrees).
    MResult<double> error() noexcept {
        MResult<double> out;
        const auto r = raw_.read_angle_error();
        if (!r.ok()) {
            out.status = (r.t_status != Transport::Status::OK)
                ? MotorStatusEx::TransportError
                : MotorStatusEx::ParseError;
            return out;
        }
        const double motor_deg = static_cast<double>(r.value) * 360.0
                                 / static_cast<double>(ANGLE_ERROR_UNITS_PER_REV);
        out.value = motor_deg / mech_.gear_ratio;
        return out;
    }

    MResult<bool> move_relative(double delta_deg,
                                MoveParams mp = {},
                                bool blocking = true,
                                std::int32_t tolerance_counts = 16,
                                std::uint64_t timeout_us = 5'000'000) noexcept {
        const auto current = read();
        if (!current.ok()) {
            MResult<bool> out;
            out.status = current.status;
            return out;
        }
        return write(current.value + delta_deg, mp, blocking,
                     tolerance_counts, timeout_us);
    }

    // Shortest-path variant of move_relative. Given a desired rotation
    // delta_deg, picks the equivalent rotation in [-180, 180] before
    // executing — so e.g. move_relative_shortest(+270°) actually rotates
    // -90°, ending up at the same orientation modulo 360°.
    //
    // ONLY safe when the motor's mechanical range is unbounded (no end
    // stops). If position limits are configured, those still apply to
    // the chosen (shorter) rotation; an absolute target outside the
    // limits will be Reject / Clamp / Warn per policy.
    //
    // Useful for cyclic / orientation tasks where what matters is the
    // final pose modulo 360°, not the path taken to get there (typical
    // for revolute joints, turret-style mechanisms, gripper rotation).
    MResult<bool> move_relative_shortest(double         delta_deg,
                                         MoveParams     mp                = {},
                                         bool           blocking          = true,
                                         std::int32_t   tolerance_counts  = 16,
                                         std::uint64_t  timeout_us        = 5'000'000) noexcept {
        // Wrap to [-180, 180]. fmod handles signs, then we shift by 180,
        // mod again, and subtract 180 to land in the symmetric range.
        double d = std::fmod(delta_deg + 180.0, 360.0);
        if (d < 0.0) d += 360.0;
        d -= 180.0;
        return move_relative(d, mp, blocking, tolerance_counts, timeout_us);
    }

    // ─── Origin ───────────────────────────────────────────────
    // Set the current physical position as the new zero (cmd 0x92).
    //
    // This is the "hard" variant: it writes the firmware's internal
    // absolute-position counter and persists in flash. Required before
    // MOVE_ABS_AXIS commands make sense in user-axis degrees, because
    // MOVE_ABS_AXIS targets are interpreted in the firmware's reference
    // frame — not the encoder accumulation register.
    //
    // After this call, origin_offset_counts is also reset to 0 so the
    // software-side math agrees with the firmware.
    //
    // The MKS firmware needs ~200 ms of settling time around flash writes
    // (matches the Python reference's handling). We sleep both before and
    // after the SET_ZERO_POINT call so subsequent commands don't race.
    //
    // Use set_origin_soft() if you only want to shift the in-software
    // zero without touching firmware (rare; mostly for tests).
    MResult<bool> set_origin() noexcept {
        MResult<bool> out;
        sleep_us(200'000);
        auto r = raw_.set_zero_point();
        if (r.t_status == Transport::Status::ReadTimeout) {
            // Single retry, as the Python reference does.
            sleep_us(500'000);
            r = raw_.set_zero_point();
        }
        if (!r.ok()) {
            out.status = (r.t_status != Transport::Status::OK)
                ? MotorStatusEx::TransportError
                : MotorStatusEx::ParseError;
            return out;
        }
        if (!r.value) {
            out.status = MotorStatusEx::NotEnabled;
            return out;
        }
        mech_.origin_offset_counts = 0;
        sleep_us(200'000);
        out.value = true;
        return out;
    }

    // Software-only origin shift: captures the current encoder counts as
    // the new in-memory zero. Does NOT touch firmware. Use with caution —
    // MOVE_ABS_AXIS will still be interpreted in the firmware's frame, so
    // mixing this with absolute moves produces surprising results.
    MResult<bool> set_origin_soft() noexcept {
        MResult<bool> out;
        const auto r = raw_.read_encoder_addition();
        if (!r.ok()) {
            out.status = (r.t_status != Transport::Status::OK)
                ? MotorStatusEx::TransportError
                : MotorStatusEx::ParseError;
            return out;
        }
        mech_.origin_offset_counts = static_cast<std::int32_t>(r.value);
        out.value = true;
        return out;
    }

    // ─── Convenience pass-throughs ─────────────────────────────
    Result<bool> enable(bool on) noexcept { return raw_.enable(on); }
    RawDriver&   raw()           noexcept { return raw_; }

    // ─── Encoder-based idle wait (the only reliable signal) ────
    //
    // Polls read_encoder_addition until the value is within
    // `tolerance_counts` of `target_counts` for `consecutive_in_window`
    // consecutive reads (so a value drifting through the window doesn't
    // false-trigger). Returns Timeout if the window never holds.
    //
    // poll_interval_us throttles the gap between polls. There is no
    // universally-correct value — the right interval depends on the bus
    // baud rate (each encoder transact takes ~ FRAME_BYTES * 10 / baud
    // seconds). For 256k baud, 0 (back-to-back) is optimal. For 38400 you
    // need ≥5 ms to avoid saturating the firmware. The Motor default is
    // conservative; benchmark callers should override.
    //
    // settle_drain_ms: after the target is reached, drain the bus for up
    // to this many ms to absorb any pending "complete" ack from the
    // firmware. Defaults to 20 — necessary for chained MOVE_* calls to
    // avoid the next move picking up the stray ack as its own response.
    // Set to 0 if you'll handle drain yourself (e.g. high-frequency
    // pipelined use).
    MotorStatusEx wait_for_position(std::int32_t   target_counts,
                                    std::int32_t   tolerance_counts,
                                    std::uint64_t  timeout_us,
                                    int            consecutive_in_window = 2,
                                    std::uint64_t  poll_interval_us = 50'000,
                                    int            settle_drain_ms = 20) noexcept {
        const std::uint64_t deadline = monotonic_us() + timeout_us;
        int in_window = 0;
        while (true) {
            const auto r = raw_.read_encoder_addition();
            if (!r.ok()) {
                return (r.t_status != Transport::Status::OK)
                    ? MotorStatusEx::TransportError
                    : MotorStatusEx::ParseError;
            }
            const std::int64_t diff = r.value - target_counts;
            const std::int64_t adiff = diff < 0 ? -diff : diff;
            if (adiff <= tolerance_counts) {
                if (++in_window >= consecutive_in_window) {
                    if (settle_drain_ms > 0) {
                        raw_.transport_drain_settle(settle_drain_ms);
                    }
                    return MotorStatusEx::OK;
                }
            } else {
                in_window = 0;
            }
            if (monotonic_us() >= deadline) return MotorStatusEx::Timeout;
            if (poll_interval_us > 0) sleep_us(poll_interval_us);
        }
    }

private:
    static std::uint64_t monotonic_us() noexcept {
        struct timespec ts;
        ::clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000ull
             + static_cast<std::uint64_t>(ts.tv_nsec) / 1000ull;
    }

    static void sleep_us(std::uint64_t us) noexcept {
        struct timespec ts;
        ts.tv_sec  = static_cast<time_t>(us / 1'000'000ull);
        ts.tv_nsec = static_cast<long>((us % 1'000'000ull) * 1000ull);
        while (::nanosleep(&ts, &ts) != 0) {
            // EINTR: ts is updated with remaining time; loop.
        }
    }

    // Returns OK / LimitWarned / LimitExceeded. Mutates `angle_deg` on Clamp.
    MotorStatusEx apply_position_policy(double& angle_deg) const noexcept {
        if (!pos_lim_.enabled) return MotorStatusEx::OK;
        if (angle_deg >= pos_lim_.min_deg && angle_deg <= pos_lim_.max_deg) {
            return MotorStatusEx::OK;
        }
        switch (pos_lim_.policy) {
            case OnViolation::Reject: return MotorStatusEx::LimitExceeded;
            case OnViolation::Clamp:
                if (angle_deg < pos_lim_.min_deg) angle_deg = pos_lim_.min_deg;
                if (angle_deg > pos_lim_.max_deg) angle_deg = pos_lim_.max_deg;
                return MotorStatusEx::OK;
            case OnViolation::Warn:   return MotorStatusEx::LimitWarned;
        }
        return MotorStatusEx::OK;
    }

    // Returns OK / LimitWarned / LimitExceeded. Mutates `rpm` on Clamp.
    MotorStatusEx apply_speed_policy(std::uint16_t& rpm) const noexcept {
        if (!spd_lim_.enabled) return MotorStatusEx::OK;
        if (rpm <= spd_lim_.max_rpm) return MotorStatusEx::OK;
        switch (spd_lim_.policy) {
            case OnViolation::Reject: return MotorStatusEx::LimitExceeded;
            case OnViolation::Clamp:  rpm = spd_lim_.max_rpm; return MotorStatusEx::OK;
            case OnViolation::Warn:   return MotorStatusEx::LimitWarned;
        }
        return MotorStatusEx::OK;
    }

    RawDriver&     raw_;
    Mechanical     mech_{};
    PositionLimits pos_lim_{};
    SpeedLimits    spd_lim_{};
    bool           auto_clear_protection_ = false;
};

}  // namespace mks_servo

#endif  // MKS_SERVO_MOTOR_HPP
