// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 mks-servo-cpp contributors
//
// Diagnostics: read-only views over the driver's debug state plus the
// only "fix it" action that doesn't touch flash (release_protection).
//
// All methods are thin wrappers over RawDriver — exposed here as a
// distinct surface because they're the operations you actually need
// when a robot is misbehaving and you want to find out why.
//
// Usage:
//   Motor m(raw);
//   Diagnostics d(m);
//   if (d.is_protection_latched().value) {
//       d.clear_protection();
//   }
//   const char* st = d.status_text();   // "STOPPED", "SPEED_UP", ...

#ifndef MKS_SERVO_DIAGNOSTICS_HPP
#define MKS_SERVO_DIAGNOSTICS_HPP

#include "mks_servo/motor.hpp"
#include "mks_servo/raw_driver.hpp"

namespace mks_servo {

class Diagnostics {
public:
    explicit Diagnostics(Motor& m) noexcept : motor_(m) {}

    // True if the driver's stall-protection latch is set. Latching
    // happens when the closed-loop FOC sees the rotor failing to track
    // the commanded trajectory (e.g. mechanical stall, or transient
    // vibration on an unloaded shaft at aggressive acc/rpm). While
    // latched, MOVE_* commands return 0x00 ("refused").
    Result<bool> is_protection_latched() noexcept {
        return motor_.raw().read_protect_status();
    }

    // Clear the stall-protection latch. Required before issuing further
    // MOVE_* commands after a stall has been detected.
    Result<bool> clear_protection() noexcept {
        return motor_.raw().release_protection();
    }

    // Cumulative pulses received by the driver since power-on. Useful
    // to verify the host has sent what it thinks it sent, especially
    // when debugging frame-level issues.
    Result<std::int32_t> pulses_received() noexcept {
        return motor_.raw().read_pulses();
    }

    // Most recent MotorStatus enum value.
    Result<MotorStatus> motor_status() noexcept {
        return motor_.raw().read_motor_status();
    }

    // Human-readable form of motor_status(). Returns "?" if the read
    // failed or the value is outside the documented enum range. Safe
    // to log directly.
    const char* status_text() noexcept {
        const auto r = motor_status();
        if (!r.ok()) return "?";
        return status_to_text(r.value);
    }

    // Static lookup — exposed so callers who already have a MotorStatus
    // value can stringify it without going to the wire.
    static const char* status_to_text(MotorStatus s) noexcept {
        switch (s) {
            case MotorStatus::QueryFail:   return "QUERY_FAIL";
            case MotorStatus::Stopped:     return "STOPPED";
            case MotorStatus::SpeedUp:     return "SPEED_UP";
            case MotorStatus::SpeedDown:   return "SPEED_DOWN";
            case MotorStatus::FullSpeed:   return "FULL_SPEED";
            case MotorStatus::Homing:      return "HOMING";
            case MotorStatus::Calibrating: return "CALIBRATING";
        }
        return "?";
    }

private:
    Motor& motor_;
};

}  // namespace mks_servo

#endif  // MKS_SERVO_DIAGNOSTICS_HPP
