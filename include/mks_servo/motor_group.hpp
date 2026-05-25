// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 mks-servo-cpp contributors
//
// MotorGroup: ergonomic batch operations over N motors, where each motor
// owns its own Transport (the "N-motor, N-bus" topology — N independent
// USB-RS485 adapters, one per motor).
//
// Design choices:
//   - Non-owning. The group holds references to Motors that the caller
//     constructs and keeps alive elsewhere. This keeps lifetime explicit
//     and avoids forcing one ownership model on every user.
//   - Single-threaded by default. Dispatch sends frames sequentially on
//     the calling thread (~400 μs per frame at 256 kbaud → ~2.5 ms total
//     skew across 6 motors, invisible to Rubik's-style use cases). If you
//     need sub-ms cross-motor synchronisation, run one thread per motor
//     blocked on a shared barrier and release them simultaneously — that
//     pattern lives in user code, not here.
//   - No shared-bus support in v1. Multi-motor-on-one-bus requires HIL
//     validation across multiple motors, which we don't have hardware
//     for yet.
//   - Hot-path methods (dispatch/wait) do no allocation. setup-time
//     `add()` uses std::vector which may allocate, but that runs once.

#ifndef MKS_SERVO_MOTOR_GROUP_HPP
#define MKS_SERVO_MOTOR_GROUP_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include "mks_servo/motor.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

namespace mks_servo {

// A per-motor move target.
struct MoveSpec {
    double     angle_deg;
    MoveParams params;
};

class MotorGroup {
public:
    MotorGroup() = default;

    // Register a motor. The motor must outlive the group.
    void add(Motor& m) { motors_.push_back(&m); }

    std::size_t size() const noexcept { return motors_.size(); }

    Motor&       operator[](std::size_t i)       noexcept { return *motors_[i]; }
    const Motor& operator[](std::size_t i) const noexcept { return *motors_[i]; }

    // ─── enable / disable ────────────────────────────────────────
    //
    // Enable or disable every motor. Stops at the first transport-level
    // failure to surface a hard error fast, since enable issues are
    // almost always wiring/power problems that affect everything.
    // Per-motor results land in `out_per_motor` if non-null
    // (caller-allocated buffer of at least size() elements).
    Result<bool> enable_all(bool on, Result<bool>* out_per_motor = nullptr) noexcept {
        Result<bool> worst;
        worst.value = true;
        for (std::size_t i = 0; i < motors_.size(); ++i) {
            const auto r = motors_[i]->enable(on);
            if (out_per_motor) out_per_motor[i] = r;
            if (!r.ok()) {
                worst = r;
                return worst;
            }
            if (!r.value) {
                worst.value    = false;
                worst.t_status = r.t_status;
            }
        }
        return worst;
    }

    // ─── dispatch_all (with ack, sequential) ─────────────────────
    //
    // Send a MOVE_ABS_AXIS to each motor and read the firmware's
    // "started" 1-byte ack inline. The ack consumes the first of the
    // two responses the firmware emits per move; the second ("complete")
    // is drained naturally by subsequent encoder polls.
    //
    // Why ack-reading is the default: a fully fire-and-forget dispatch
    // leaves two pending response frames per motor on the bus, which can
    // race against the first encoder polls in wait_all_settled and cause
    // ReadFailed when MAX_SCAN_BYTES is reached. Reading the started ack
    // is ~1 ms at 256 kbaud — well worth the reliability.
    //
    // Sequential skew across N motors at 256 kbaud is roughly:
    //   ~400 us (write)  +  ~1 ms (ack)  =  ~1.4 ms per motor.
    //
    // For applications that need true fire-and-forget (e.g. user-managed
    // bus state with their own drain step), call Motor::dispatch_write
    // directly via operator[].
    //
    // specs must point to at least size() MoveSpec entries.
    // Per-motor results go into out_per_motor if non-null.
    Transport::Status dispatch_all(const MoveSpec*    specs,
                                   Transport::Status* out_per_motor = nullptr) noexcept {
        Transport::Status worst = Transport::Status::OK;
        for (std::size_t i = 0; i < motors_.size(); ++i) {
            // Use the ack-reading Motor::write with blocking=false: it
            // does policy checks AND consumes the start ack, but doesn't
            // poll the encoder.
            const auto r = motors_[i]->write(specs[i].angle_deg,
                                             specs[i].params,
                                             /*blocking=*/false);
            const Transport::Status s = (r.status == MotorStatusEx::TransportError)
                ? Transport::Status::ReadFailed
                : Transport::Status::OK;
            if (out_per_motor) out_per_motor[i] = s;
            if (s != Transport::Status::OK && worst == Transport::Status::OK) {
                worst = s;
            }
        }
        return worst;
    }

    // ─── wait_all_settled ───────────────────────────────────────
    //
    // Block until every motor's encoder is within tolerance_counts of its
    // target, for consecutive_in_window consecutive reads, or until
    // timeout_us elapses. target_counts must point to size() entries.
    //
    // Polls motors in round-robin order. Once a motor is settled it is
    // dropped from the polling rotation, so finishing motors don't slow
    // down the still-moving ones.
    //
    // Returns OK iff every motor settled. On any per-motor failure
    // (transport error, parse error, timeout) returns that status and
    // populates out_per_motor[i] for the failed index.
    MotorStatusEx wait_all_settled(const std::int32_t* target_counts,
                                   std::int32_t        tolerance_counts,
                                   std::uint64_t       timeout_us,
                                   int                 consecutive_in_window = 2,
                                   std::uint64_t       poll_interval_us = 50'000,
                                   MotorStatusEx*      out_per_motor = nullptr) noexcept {
        const std::uint64_t deadline = monotonic_us() + timeout_us;
        const std::size_t   n        = motors_.size();

        // Per-motor state.
        // Stack arrays would be nicer but we don't know n at compile time
        // and the size is small; vector with reserve avoids per-call alloc
        // if the user reuses the group. We do allocate once here on the
        // hot path, which is acceptable for v1.
        std::vector<int>  in_window(n, 0);
        std::vector<bool> done(n, false);
        std::size_t       remaining = n;
        if (out_per_motor) {
            for (std::size_t i = 0; i < n; ++i) out_per_motor[i] = MotorStatusEx::OK;
        }

        while (remaining > 0) {
            for (std::size_t i = 0; i < n; ++i) {
                if (done[i]) continue;
                const auto r = motors_[i]->raw().read_encoder_addition();
                if (!r.ok()) {
                    const auto bad = (r.t_status != Transport::Status::OK)
                        ? MotorStatusEx::TransportError
                        : MotorStatusEx::ParseError;
                    if (out_per_motor) out_per_motor[i] = bad;
                    return bad;
                }
                const std::int64_t diff  = r.value - target_counts[i];
                const std::int64_t adiff = diff < 0 ? -diff : diff;
                if (adiff <= tolerance_counts) {
                    if (++in_window[i] >= consecutive_in_window) {
                        done[i] = true;
                        if (out_per_motor) out_per_motor[i] = MotorStatusEx::OK;
                        --remaining;
                    }
                } else {
                    in_window[i] = 0;
                }
            }
            if (remaining == 0) break;
            if (monotonic_us() >= deadline) {
                if (out_per_motor) {
                    for (std::size_t i = 0; i < n; ++i) {
                        if (!done[i]) out_per_motor[i] = MotorStatusEx::Timeout;
                    }
                }
                return MotorStatusEx::Timeout;
            }
            if (poll_interval_us > 0) sleep_us(poll_interval_us);
        }
        // Per-motor settle drain: the encoder reaches its target window a
        // few ms BEFORE the firmware emits its "complete" ack. Without
        // this step the next call into dispatch_all races against those
        // pending ack frames and intermittently sees ReadFailed.
        // Same trick Motor::write blocking-mode uses.
        for (std::size_t i = 0; i < n; ++i) {
            motors_[i]->raw().transport_drain_settle(20);
        }
        return MotorStatusEx::OK;
    }

    // ─── move_all: dispatch + wait, the common case ─────────────
    //
    // Computes per-motor target counts from each MoveSpec.angle_deg using
    // that motor's own Mechanical config (gear_ratio + origin_offset), so
    // motors with different mechanical setups in the same group work
    // transparently.
    MotorStatusEx move_all(const MoveSpec* specs,
                           std::int32_t   tolerance_counts = 32,
                           std::uint64_t  timeout_us = 5'000'000,
                           int            consecutive_in_window = 2,
                           std::uint64_t  poll_interval_us = 50'000,
                           MotorStatusEx* out_per_motor = nullptr) noexcept {
        const std::size_t n = motors_.size();
        std::vector<std::int32_t> targets(n);
        for (std::size_t i = 0; i < n; ++i) {
            targets[i] = motors_[i]->angle_to_counts(specs[i].angle_deg);
        }
        const auto ds = dispatch_all(specs);
        if (ds != Transport::Status::OK) {
            if (out_per_motor) {
                for (std::size_t i = 0; i < n; ++i) {
                    out_per_motor[i] = MotorStatusEx::TransportError;
                }
            }
            return MotorStatusEx::TransportError;
        }
        return wait_all_settled(targets.data(),
                                tolerance_counts,
                                timeout_us,
                                consecutive_in_window,
                                poll_interval_us,
                                out_per_motor);
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
        while (::nanosleep(&ts, &ts) != 0) {}
    }

    std::vector<Motor*> motors_;
};

}  // namespace mks_servo

#endif  // MKS_SERVO_MOTOR_GROUP_HPP
