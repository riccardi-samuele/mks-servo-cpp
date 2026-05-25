// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 mks-servo-cpp contributors
//
// Profile: a value-typed aggregate of everything you'd want to know
// about a motor — mechanical config, limits, origin, characterization
// results — in one struct that's easy to fill from any source (a YAML
// file, a JSON payload, a constants header, hard-coded literals).
//
// The library does NOT bundle a YAML parser. Schema is intentionally
// 1:1 with the Python `mks-servo` profile schema so a YAML file
// written by the Python `mks-servo profile` CLI can be loaded by any
// YAML / JSON / etc. library in user code and copied field-by-field
// into this struct. See `examples/profile_example.cpp` for the
// hand-rolled equivalent.

#ifndef MKS_SERVO_PROFILE_HPP
#define MKS_SERVO_PROFILE_HPP

#include <cstdint>
#include <string>

#include "mks_servo/motor.hpp"
#include "mks_servo/raw_driver.hpp"

namespace mks_servo {

struct Profile {
    // [transport]
    struct Transport {
        std::string  port;            // e.g. "/dev/ttyUSB0"; empty = caller-managed
        int          baud      = 38400;
        double       timeout_s = 3.0;
    } transport;

    // [config] — what to push into firmware flash
    struct Config {
        WorkMode      mode              = WorkMode::SR_vFOC;
        std::uint16_t microsteps        = 16;
        std::uint16_t work_current_ma   = 1500;
        std::uint8_t  hold_current_pct  = 50;            // 10..90 step 10
        Direction     direction         = Direction::CW;
        std::uint8_t  slave_addr        = 1;
    } config;

    // [mechanical] — output-axis math
    Mechanical mechanical{};

    // [limits] — soft limits for position / speed / current
    PositionLimits position_limits{};
    SpeedLimits    speed_limits{};
    struct CurrentLimits {
        std::uint16_t max_ma = 3000;
    } current_limits;

    // [origin] — where the user-axis zero is
    struct Origin {
        bool          set_in_firmware       = true;
        std::int32_t  encoder_offset_counts = 0;
    } origin;

    // [characterization] — empirical results from CharacterizationSuite,
    //                      stored alongside the motor so a motion planner
    //                      can read them without having to re-run the suite.
    struct Characterization {
        // P1
        double sigma_deg                       = 0.0;
        double peak_deg                        = 0.0;
        // S2
        std::uint16_t max_measured_rpm         = 0;
        // Free-form info field (timestamp, notes)
        std::string   notes;
    } characterization;
};

// Push the in-memory parts of a Profile to a Motor without touching the
// firmware. Sets mechanical, position/speed limits, origin offset (the
// software side — does NOT call SET_ZERO_POINT). Useful at startup when
// you've just constructed a Motor and want to load its profile.
//
// To also push mode / microsteps / current / hold_current to the firmware
// flash, call apply_profile_to_firmware separately — those are flash
// writes with the usual 200 ms settle penalties and are intentionally
// not bundled into a single call.
inline void apply_profile_to_motor(const Profile& p, Motor& m) noexcept {
    m.set_mechanical(p.mechanical);
    if (p.position_limits.enabled) {
        m.set_position_limits(p.position_limits.min_deg,
                              p.position_limits.max_deg,
                              p.position_limits.policy);
    } else {
        m.clear_position_limits();
    }
    if (p.speed_limits.enabled) {
        m.set_speed_limit_rpm(p.speed_limits.max_rpm,
                              p.speed_limits.policy);
    } else {
        m.clear_speed_limit();
    }
    m.set_origin_offset_counts(p.origin.encoder_offset_counts);
}

// Push the firmware-side config fields (mode / microsteps / current /
// hold_current) to the driver. Each is a flash write; the firmware
// reports back via the standard ack mechanism. Returns the first
// non-OK result, or an all-OK Result.
//
// Note: changing mode requires a driver restart in some firmware
// revisions — see the Python reference for the proven pattern. This
// helper does NOT trigger a restart; if you need to change the mode,
// call raw.restart() yourself after.
inline Result<bool> apply_profile_to_firmware(const Profile& p,
                                              RawDriver& raw) noexcept {
    Result<bool> r;
    r.value = true;

    auto step = [&](Result<bool> step_r, const char*) {
        if (!step_r.ok() || !step_r.value) {
            r = step_r;
            return false;
        }
        return true;
    };

    if (!step(raw.set_work_mode(p.config.mode),           "work_mode"))      return r;
    if (!step(raw.set_subdivision(p.config.microsteps),   "microsteps"))     return r;
    if (!step(raw.set_work_current_ma(p.config.work_current_ma),
              "work_current_ma"))                                            return r;
    return r;
}

}  // namespace mks_servo

#endif  // MKS_SERVO_PROFILE_HPP
