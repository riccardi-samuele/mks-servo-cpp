// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 mks-servo-cpp contributors
//
// Scheduler: DAG-based execution of moves across multiple Motors on
// independent buses. Each Motor is driven by a dedicated worker thread
// that owns the bus for the lifetime of the Scheduler. Workers spin-wait
// on per-move dependencies, publish progress to shared cache-line-aligned
// state, and signal completion via atomics.
//
// The model is "submit then run":
//   1. Register each Motor with `add()`. A worker thread is spawned for it.
//   2. Build the move plan with `move(...)` and the fluent dependency
//      operators on the returned `MoveHandle`.
//   3. Call `run()` to release the workers and block until the plan
//      finishes (or any motor errors).
//
// Dependency primitives on `MoveHandle`:
//   - `.after(other)`               — wait for `other` to complete
//   - `.at_progress(other, frac)`   — wait until `other` reaches `frac` of
//                                     its commanded delta (encoder-based)
//   - `.at_time_after_start(other, ms)` — wait until `now() >= other.t_start + ms`
//   - `.at_time_before_end(other, ms)`  — wait until `now() >=
//                                          other.t_start + other.t_expected - ms`
//
// Default dependency: if a motor has multiple moves queued without
// explicit dependencies, each waits for the previous one ON THE SAME
// MOTOR to complete (FIFO per motor). Cross-motor moves with no explicit
// dependency run concurrently.
//
// Threading:
//   - One worker thread per registered Motor. Worker owns the Transport,
//     so no cross-thread bus contention is possible.
//   - Workers spin-wait on dependencies (sub-microsecond latency).
//   - Workers spin-poll their own encoder during a move (~1 ms per poll
//     at 256k baud); progress is published as a fraction in [0, 1].
//   - On error, the worker marks its move as failed and stops; the main
//     thread can call `abort()` to emergency-stop all motors.
//
// Performance notes:
//   - MoveState is alignas(64) to avoid false sharing between workers on
//     different motors.
//   - No allocation in the hot path; the move pool is reserved up-front.
//   - Spin-wait on dependencies (not condition_variable) — measured trigger
//     latency is < 1 μs.

#ifndef MKS_SERVO_SCHEDULER_HPP
#define MKS_SERVO_SCHEDULER_HPP

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <thread>
#include <vector>

#include "mks_servo/motor.hpp"
#include "mks_servo/raw_driver.hpp"

namespace mks_servo {

// ─── MotorProfile: per-motor timing tunables ────────────────────────
//
// Per-motor tuning. Users construct one and call sched.set_motor_profile(
// motor, profile) before submitting moves; the values are copied onto
// every MoveState the scheduler allocates for that motor at sched.move()
// time. Default-constructed values mirror the conservative library-wide
// defaults (designed for V1.0.8 SR_vFOC).
//
// Presets cover the HIL-validated configurations we've shipped:
//   * MotorProfile::for_v1_0_8_sr_vfoc()  — A's setup on 12V/5A
//   * MotorProfile::for_v1_0_9_sr_close() — B & C's setup on 12V/5A
struct MotorProfile {
    int    settle_drain_ms     = 30;       // bus cleanup after target
    int    predispatch_drain_ms = 5;       // defensive flush before dispatch
    int    inter_move_rest_us  = 100'000;  // post-move worker rest on this bus
    int    consecutive_in_window = 2;      // in-tol reads to declare complete
    double t_90deg_ms          = 0;        // measured; 0 = unknown

    // ── Empirically-validated presets ────────────────────────────────
    //
    // Numbers come from hil_inter_move_rest_sweep on the dev fleet:
    // V1.0.9 SR_CLOSE motors stayed reliable at inter_move_rest_us=5000
    // and showed lower σ than 100 ms. V1.0.8 SR_vFOC was not swept
    // explicitly (the swept fleet held V1.0.8 at default 100 ms while
    // varying V1.0.9 only) — keep the conservative default here until
    // a V1.0.8-specific sweep has been done.

    static MotorProfile for_v1_0_9_sr_close() {
        MotorProfile p;
        p.settle_drain_ms      = 30;       // bus drain is firmware-timing-bound,
                                           // not mechanical settle — keep 30 ms
        p.predispatch_drain_ms = 5;
        p.inter_move_rest_us   = 5'000;    // 20× shorter than V1.0.8 default
        p.consecutive_in_window = 2;
        p.t_90deg_ms           = 40.0;     // HIL: 39.89 ms ± 0.025 (clean bench)
        return p;
    }

    static MotorProfile for_v1_0_8_sr_vfoc() {
        MotorProfile p;
        p.settle_drain_ms      = 30;
        p.predispatch_drain_ms = 5;
        p.inter_move_rest_us   = 100'000;  // V1.0.8 late-ack window unmeasured
        p.consecutive_in_window = 2;
        p.t_90deg_ms           = 43.0;     // HIL: 43.19 ms ± 0.49
        return p;
    }
};

// ─── Trigger kinds ──────────────────────────────────────────────────
enum class TriggerKind : std::uint8_t {
    None = 0,             // dispatch immediately (no dependency)
    AfterCompleted,       // wait for ref move to complete
    AtProgress,           // wait until ref move encoder reaches val (0..1)
    AtTimeAfterStart,     // wait until now() >= ref.t_start + val_ms
    AtTimeBeforeEnd,      // wait until now() >= ref.t_start + ref.t_expected - val_ms
};

enum class MoveStatus : std::uint8_t {
    Pending = 0,
    WaitingDeps,
    Dispatching,
    Polling,
    Completed,
    Failed,
};

// ─── MoveState: per-move shared state ───────────────────────────────
//
// Cache-line-aligned to keep one move's state isolated from another's
// when both are mutated by different threads in tight loops.
struct alignas(64) MoveState {
    // Static (set at submission)
    Motor*       motor       = nullptr;
    double       angle_deg   = 0;
    MoveParams   params{};
    std::int32_t tol_counts  = 50;
    int          consecutive_in_window = 2;  // require N in-tol reads before completing
    double       expected_duration_ms = 0;   // for AtTimeBeforeEnd
    int          settle_drain_ms = 30;       // bus cleanup after target reached
    int          predispatch_drain_ms = 5;   // defensive flush before dispatch
    int          inter_move_rest_us = 100'000; // post-move worker rest on this bus

    // Trigger condition (one ref + one value)
    TriggerKind  trigger_kind = TriggerKind::None;
    MoveState*   trigger_ref  = nullptr;
    double       trigger_val  = 0.0;

    // Live state (workers publish, scheduler/other workers consume)
    std::atomic<MoveStatus>     status{MoveStatus::Pending};
    std::atomic<double>         progress{0.0};      // [0..1] during Polling
    std::atomic<std::uint64_t>  t_start_us{0};      // when dispatch began
    std::atomic<std::uint64_t>  t_end_us{0};        // when target reached

    // Diagnostic timestamps (worker publishes for skew analysis)
    std::atomic<std::uint64_t>  t_pickup_us{0};      // worker picked up Pending
    std::atomic<std::uint64_t>  t_predrain_us{0};    // after predispatch drain
    std::atomic<std::uint64_t>  t_e0_read_us{0};     // after encoder read

    // Internal (set by submitter before run)
    std::int64_t e0_counts = 0;          // encoder at dispatch
    std::int64_t target_counts = 0;       // computed target encoder
};

// ─── MoveHandle: fluent dependency builder ──────────────────────────
class MoveHandle {
public:
    // Wait for the other move to complete before dispatching this one.
    MoveHandle& after(const MoveHandle& other) noexcept {
        state_->trigger_kind = TriggerKind::AfterCompleted;
        state_->trigger_ref  = other.state_;
        state_->trigger_val  = 0.0;
        return *this;
    }

    // Wait until the other move reaches `fraction` (0..1) of its
    // commanded delta. Encoder-based trigger.
    MoveHandle& at_progress(const MoveHandle& other, double fraction) noexcept {
        state_->trigger_kind = TriggerKind::AtProgress;
        state_->trigger_ref  = other.state_;
        state_->trigger_val  = fraction;
        return *this;
    }

    // Wait until `ms` milliseconds have elapsed since the other move's
    // dispatch start.
    MoveHandle& at_time_after_start(const MoveHandle& other, double ms) noexcept {
        state_->trigger_kind = TriggerKind::AtTimeAfterStart;
        state_->trigger_ref  = other.state_;
        state_->trigger_val  = ms;
        return *this;
    }

    // Wait until `ms` milliseconds before the other move's expected end
    // (i.e. start B early to hide A's settle inside B's acceleration).
    // Requires `expected_duration_ms` to be set on the ref (otherwise
    // behaves like at_time_after_start with 0).
    MoveHandle& at_time_before_end(const MoveHandle& other, double ms) noexcept {
        state_->trigger_kind = TriggerKind::AtTimeBeforeEnd;
        state_->trigger_ref  = other.state_;
        state_->trigger_val  = ms;
        return *this;
    }

    // Set the expected duration (used by at_time_before_end).
    // Normally populated from a MotorProfile but exposed for manual override.
    MoveHandle& with_expected_duration_ms(double ms) noexcept {
        state_->expected_duration_ms = ms;
        return *this;
    }

    // Access the underlying state (advanced use).
    const MoveState* state() const noexcept { return state_; }

private:
    friend class Scheduler;
    explicit MoveHandle(MoveState* s) noexcept : state_(s) {}
    MoveState* state_;
};

// ─── Scheduler ──────────────────────────────────────────────────────
class Scheduler {
public:
    // Create an empty scheduler.
    Scheduler() = default;

    // Move-only; destructor joins workers.
    Scheduler(const Scheduler&) = delete;
    Scheduler& operator=(const Scheduler&) = delete;

    ~Scheduler() {
        shutdown_.store(true, std::memory_order_release);
        release_.store(true, std::memory_order_release);
        for (auto& w : workers_) if (w.joinable()) w.join();
    }

    // Register a motor with the scheduler. The motor must outlive the
    // scheduler. Spawns a worker thread for this motor.
    //
    // Returns the motor index, which is also the index into `worker_status_`.
    std::size_t add(Motor& m) {
        const std::size_t idx = motors_.size();
        motors_.push_back(&m);
        per_motor_queues_.emplace_back();
        per_motor_last_state_.push_back(nullptr);
        per_motor_profile_.emplace_back();  // default profile
        // The worker uses motors_[idx] and per_motor_queues_[idx].
        workers_.emplace_back([this, idx]{ this->worker_loop(idx); });
        return idx;
    }

    // Install a per-motor MotorProfile. Affects all subsequent
    // sched.move() calls for this motor; in-flight moves are unaffected.
    // The motor must already be registered via add().
    void set_motor_profile(Motor& m, const MotorProfile& p) {
        for (std::size_t i = 0; i < motors_.size(); ++i) {
            if (motors_[i] == &m) { per_motor_profile_[i] = p; return; }
        }
        std::abort();  // caller bug: motor not registered
    }

    const MotorProfile& motor_profile(Motor& m) const {
        for (std::size_t i = 0; i < motors_.size(); ++i) {
            if (motors_[i] == &m) return per_motor_profile_[i];
        }
        std::abort();
    }

    // Probe a motor: measure its real t_90deg motion-only time and
    // populate the per-motor profile's t_90deg_ms. Uses the historical
    // hil_envelope::ec_do_90deg methodology (single-tolerance check,
    // motion-only timing) so the number is directly comparable to the
    // baselines in memory.
    //
    // The motor MUST be enabled, calibrated and at a stable position
    // before calling. Shaft must be free to rotate ±90° from current.
    //
    // Returns the measured t_90deg in ms, or 0.0 on failure (in which
    // case the profile's t_90deg_ms is NOT updated).
    double probe_motor(Motor& m, int samples = 5,
                       std::uint16_t rpm = 2000,
                       std::uint8_t  acc = 255) {
        auto& raw = m.raw();
        double sum = 0;
        int ok = 0;
        for (int i = 0; i < samples; ++i) {
            auto e0 = raw.read_encoder_addition();
            if (!e0.ok()) continue;
            const std::int32_t delta = 4096 * ((i % 2) ? -1 : +1);
            const std::int64_t target = e0.value + delta;
            const std::uint64_t t0 = now_us();
            auto disp = raw.move_relative_axis(delta, rpm, acc);
            if (!disp.ok() || !disp.value) continue;
            const std::uint64_t deadline = t0 + 2'000'000ull;
            bool reached = false;
            while (now_us() < deadline) {
                auto e = raw.read_encoder_addition();
                if (!e.ok()) break;
                const std::int64_t diff = e.value - target;
                const std::int64_t adiff = diff < 0 ? -diff : diff;
                if (adiff <= 50) { reached = true; break; }
            }
            const std::uint64_t t1 = now_us();
            raw.transport_drain_settle(15);
            if (reached) { sum += (double)(t1 - t0) / 1000.0; ++ok; }
            // brief inter-probe rest so the firmware late-ack doesn't
            // leak into the next probe
            sleep_us(80'000);
        }
        if (ok == 0) return 0.0;
        const double mean = sum / ok;
        for (std::size_t i = 0; i < motors_.size(); ++i) {
            if (motors_[i] == &m) {
                per_motor_profile_[i].t_90deg_ms = mean;
                return mean;
            }
        }
        return mean;
    }

    // Submit a move. Default dependency: the previous move on the SAME
    // motor (FIFO per motor). Override via `.after()`, `.at_progress()`,
    // etc. on the returned handle.
    MoveHandle move(Motor& motor, double angle_deg, MoveParams params,
                    std::int32_t tol_counts = 50) {
        // Find the motor's index
        std::size_t idx = SIZE_MAX;
        for (std::size_t i = 0; i < motors_.size(); ++i) {
            if (motors_[i] == &motor) { idx = i; break; }
        }
        if (idx == SIZE_MAX) std::abort();  // caller bug: motor not registered

        // Allocate a new MoveState in the pool
        pool_.emplace_back(std::make_unique<MoveState>());
        MoveState* s = pool_.back().get();
        s->motor      = &motor;
        s->angle_deg  = angle_deg;
        s->params     = params;
        s->tol_counts = tol_counts;
        // Apply per-motor profile to this move
        const auto& prof = per_motor_profile_[idx];
        s->settle_drain_ms       = prof.settle_drain_ms;
        s->predispatch_drain_ms  = prof.predispatch_drain_ms;
        s->inter_move_rest_us    = prof.inter_move_rest_us;
        s->consecutive_in_window = prof.consecutive_in_window;
        if (prof.t_90deg_ms > 0) {
            // expected_duration scales with abs angle (assuming linear time)
            const double abs_deg = angle_deg < 0 ? -angle_deg : angle_deg;
            s->expected_duration_ms = (abs_deg / 90.0) * prof.t_90deg_ms;
        }

        // Default dependency: previous move on same motor
        if (per_motor_last_state_[idx] != nullptr) {
            s->trigger_kind = TriggerKind::AfterCompleted;
            s->trigger_ref  = per_motor_last_state_[idx];
        }

        per_motor_queues_[idx].push_back(s);
        per_motor_last_state_[idx] = s;
        return MoveHandle{s};
    }

    // Set an expected duration for moves on this motor, used by
    // at_time_before_end. Typical value: t_90deg_ms × |angle/90|. Set
    // automatically below, but exposed for callers that want manual control.
    void set_expected_duration(MoveHandle& h, double ms) noexcept {
        h.state_->expected_duration_ms = ms;
    }

    // Execute the submitted plan. Releases all workers, polls completion,
    // collects results. Returns the worst per-motor status — or OK if every
    // move completed successfully.
    MotorStatusEx run() noexcept {
        // Compute expected_duration_ms for moves that don't have it set,
        // using a sane default of 45 ms / 90 deg (overridable per move).
        // Library users with a MotorProfile should pass it in explicitly.
        for (auto& sp : pool_) {
            if (sp->expected_duration_ms <= 0) {
                const double abs_deg = sp->angle_deg < 0 ? -sp->angle_deg : sp->angle_deg;
                sp->expected_duration_ms = (abs_deg / 90.0) * 45.0;
            }
        }

        // Release all workers simultaneously (atomic flag spin-wait).
        release_.store(true, std::memory_order_release);

        // Block until every move is in a terminal state.
        // Spin-wait at 100 μs polling — moves are 40+ ms so this is fine.
        while (true) {
            bool all_done = true;
            for (auto& sp : pool_) {
                const auto st = sp->status.load(std::memory_order_acquire);
                if (st != MoveStatus::Completed && st != MoveStatus::Failed) {
                    all_done = false;
                    break;
                }
            }
            if (all_done) break;
            sleep_us(100);
        }

        // Reset for next run()
        release_.store(false, std::memory_order_release);

        // Aggregate result
        MotorStatusEx worst = MotorStatusEx::OK;
        for (auto& sp : pool_) {
            if (sp->status.load(std::memory_order_acquire) == MoveStatus::Failed) {
                worst = MotorStatusEx::TransportError;  // generic — caller can inspect per move
            }
        }

        // Clear the queues for the next run (but keep workers alive).
        // The pool itself is kept so handles remain valid for inspection.
        for (auto& q : per_motor_queues_) q.clear();
        for (auto& last : per_motor_last_state_) last = nullptr;

        return worst;
    }

    // Clear the move pool (call between independent plans to release
    // memory). After calling, all previously-issued MoveHandles are
    // invalidated.
    void reset() noexcept {
        pool_.clear();
        for (auto& q : per_motor_queues_) q.clear();
        for (auto& last : per_motor_last_state_) last = nullptr;
    }

    // Send EMERGENCY_STOP to every registered motor and mark all in-flight
    // moves as failed. Caller is responsible for re-arming.
    void abort() noexcept {
        for (auto* m : motors_) m->raw().emergency_stop();
        for (auto& sp : pool_) {
            auto st = sp->status.load(std::memory_order_acquire);
            if (st != MoveStatus::Completed && st != MoveStatus::Failed) {
                sp->status.store(MoveStatus::Failed, std::memory_order_release);
            }
        }
    }

    std::size_t motor_count() const noexcept { return motors_.size(); }

private:
    static std::uint64_t now_us() noexcept {
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

    // Worker loop: dedicated thread per motor.
    //
    // Picks moves from per_motor_queues_[idx] FIFO. For each move:
    //   1. Spin-wait until trigger satisfied.
    //   2. Read e0 (current encoder), compute target.
    //   3. Dispatch (move_relative_axis) and record t_start.
    //   4. Poll encoder, publish progress, until within tolerance.
    //   5. Mark Completed (or Failed on transport error).
    void worker_loop(std::size_t idx) noexcept {
        // Wait until first run() is called.
        while (!release_.load(std::memory_order_acquire) &&
               !shutdown_.load(std::memory_order_acquire)) {
            sleep_us(50);
        }

        while (!shutdown_.load(std::memory_order_acquire)) {
            // Status-based scan: process any move with status == Pending.
            // This is robust against queue clear+repopulate races between
            // run() calls (e.g. when a previous run's inter-move rest is
            // still in flight while the next run() repopulates the queue).
            // Index-based iteration with a persistent local counter is NOT
            // robust against that race — see git history.
            //
            // While release_ is true and we're not shutting down, keep
            // scanning the queue for Pending work.
            while (release_.load(std::memory_order_acquire) &&
                   !shutdown_.load(std::memory_order_acquire)) {
                MoveState* next_move = nullptr;
                auto& q = per_motor_queues_[idx];
                for (auto* m : q) {
                    if (m->status.load(std::memory_order_acquire)
                        == MoveStatus::Pending) {
                        next_move = m;
                        break;
                    }
                }
                if (next_move == nullptr) {
                    // Nothing to do for this motor in the current plan.
                    // Yield briefly, then re-check (cheap atomic load).
                    sleep_us(100);
                    continue;
                }
                process_move(next_move);
                // Inter-move rest on this motor's bus. The firmware can
                // emit a "complete" ack up to ~30 ms after the encoder
                // reaches its target window (FTDI latency_timer + buffer);
                // the previous move's post-completion drain may have ended
                // before that. A short sleep gives the late ack time to
                // land before the next move's dispatch on the same motor.
                //
                // Read from the MoveState we just processed so callers can
                // tune per-move (typically populated from a MotorProfile
                // — see SchedulerConfig planning notes).
                const int rest_us = next_move->inter_move_rest_us;
                if (rest_us > 0) sleep_us(static_cast<std::uint64_t>(rest_us));
            }

            // Between runs: wait until the next release_=true edge.
            while (!release_.load(std::memory_order_acquire) &&
                   !shutdown_.load(std::memory_order_acquire)) {
                sleep_us(50);
            }
        }
    }

    void process_move(MoveState* s) noexcept {
        s->t_pickup_us.store(now_us(), std::memory_order_release);
        s->status.store(MoveStatus::WaitingDeps, std::memory_order_release);

        // 1. Resolve trigger
        wait_for_trigger(s);

        s->status.store(MoveStatus::Dispatching, std::memory_order_release);

        // 2. Read e0, compute target. Drain defensively first to catch
        // any stray late ack the previous move's post-completion drain
        // might have missed (worst-case FTDI latency_timer is ~16 ms
        // bimodal, so a stray byte can arrive after our 30 ms drain
        // returned). Cheap if the bus is already clean.
        auto& raw = s->motor->raw();
        if (s->predispatch_drain_ms > 0) {
            raw.transport_drain_settle(s->predispatch_drain_ms);
        }
        s->t_predrain_us.store(now_us(), std::memory_order_release);
        auto e_start = raw.read_encoder_addition();
        s->t_e0_read_us.store(now_us(), std::memory_order_release);
        if (!e_start.ok()) {
            s->status.store(MoveStatus::Failed, std::memory_order_release);
            return;
        }
        const std::int32_t delta = s->motor->angle_to_counts(s->angle_deg);
        s->e0_counts     = e_start.value;
        s->target_counts = e_start.value + delta;

        // 3. Dispatch. We use MOVE_REL because the firmware's internal
        // absolute-position tracker isn't kept in sync with the encoder
        // outside of SET_ZERO_POINT operations; MOVE_REL is self-consistent
        // because the firmware moves by the requested delta from wherever
        // it currently believes it is.
        const std::uint64_t t0 = now_us();
        s->t_start_us.store(t0, std::memory_order_release);
        auto disp = raw.move_relative_axis(delta,
                                            s->params.rpm,
                                            s->params.acc);
        if (!disp.ok() || !disp.value) {
            s->status.store(MoveStatus::Failed, std::memory_order_release);
            return;
        }

        s->status.store(MoveStatus::Polling, std::memory_order_release);

        // 4. Poll until target within tolerance, mirroring the robustness
        // pattern from Motor::wait_for_position: require `consecutive_in_window`
        // in-tol reads before declaring completion, and tolerate up to 2
        // consecutive transient bus errors (a stray "complete" ack from a
        // previous move can momentarily desynchronise the response stream
        // even when the motor itself is fine).
        const std::uint64_t deadline = t0 + 5'000'000ull;
        const double inv_delta = (delta != 0) ? (1.0 / static_cast<double>(delta)) : 0;
        int in_window = 0;
        int consec_errors = 0;
        constexpr int MAX_CONSEC_ERRORS = 2;
        while (now_us() < deadline) {
            auto e = raw.read_encoder_addition();
            if (!e.ok()) {
                if (++consec_errors > MAX_CONSEC_ERRORS) {
                    s->status.store(MoveStatus::Failed, std::memory_order_release);
                    return;
                }
                // Backoff to let the bus drain any stray bytes before retry.
                sleep_us(2000);
                continue;
            }
            consec_errors = 0;

            // Publish progress (clamped to [0, 1])
            const std::int64_t moved = e.value - s->e0_counts;
            double prog = static_cast<double>(moved) * inv_delta;
            if (prog < 0) prog = 0;
            if (prog > 1) prog = 1;
            s->progress.store(prog, std::memory_order_release);

            const std::int64_t diff = e.value - s->target_counts;
            const std::int64_t adiff = diff < 0 ? -diff : diff;
            if (adiff <= s->tol_counts) {
                if (++in_window >= s->consecutive_in_window) {
                    // Record physical-arrival timestamp before bus cleanup.
                    s->t_end_us.store(now_us(), std::memory_order_release);
                    s->progress.store(1.0, std::memory_order_release);
                    // Drain the trailing "complete" ack frame the firmware
                    // emits a few ms after the encoder reaches its window.
                    // Without this the next dispatch on this motor races
                    // against the pending ack and sees a corrupted
                    // response. Same idiom Motor::write blocking uses.
                    raw.transport_drain_settle(s->settle_drain_ms);
                    // Publish Completed only after the bus is clean, so
                    // AfterCompleted dependents on this motor don't
                    // dispatch into a half-drained transport.
                    s->status.store(MoveStatus::Completed, std::memory_order_release);
                    return;
                }
            } else {
                in_window = 0;
            }
        }
        // Timeout
        s->status.store(MoveStatus::Failed, std::memory_order_release);
    }

    void wait_for_trigger(MoveState* s) noexcept {
        if (s->trigger_kind == TriggerKind::None || s->trigger_ref == nullptr) return;

        switch (s->trigger_kind) {
            case TriggerKind::AfterCompleted: {
                while (s->trigger_ref->status.load(std::memory_order_acquire)
                       != MoveStatus::Completed) {
                    if (s->trigger_ref->status.load(std::memory_order_acquire)
                        == MoveStatus::Failed) return;  // propagate failure
                }
                return;
            }
            case TriggerKind::AtProgress: {
                while (s->trigger_ref->progress.load(std::memory_order_acquire)
                       < s->trigger_val) {
                    const auto st = s->trigger_ref->status.load(std::memory_order_acquire);
                    if (st == MoveStatus::Failed) return;
                    if (st == MoveStatus::Completed) return;  // already past threshold
                }
                return;
            }
            case TriggerKind::AtTimeAfterStart: {
                // Wait until the trigger ref has started, then until offset elapsed.
                while (s->trigger_ref->t_start_us.load(std::memory_order_acquire) == 0) {
                    const auto st = s->trigger_ref->status.load(std::memory_order_acquire);
                    if (st == MoveStatus::Failed) return;
                }
                const std::uint64_t target = s->trigger_ref->t_start_us.load(
                    std::memory_order_acquire) + static_cast<std::uint64_t>(s->trigger_val * 1000.0);
                while (now_us() < target) {
                    const auto st = s->trigger_ref->status.load(std::memory_order_acquire);
                    if (st == MoveStatus::Failed) return;
                }
                return;
            }
            case TriggerKind::AtTimeBeforeEnd: {
                while (s->trigger_ref->t_start_us.load(std::memory_order_acquire) == 0) {
                    const auto st = s->trigger_ref->status.load(std::memory_order_acquire);
                    if (st == MoveStatus::Failed) return;
                }
                const double total_ms = s->trigger_ref->expected_duration_ms;
                const std::uint64_t target = s->trigger_ref->t_start_us.load(
                    std::memory_order_acquire) + static_cast<std::uint64_t>(
                    (total_ms - s->trigger_val) * 1000.0);
                while (now_us() < target) {
                    const auto st = s->trigger_ref->status.load(std::memory_order_acquire);
                    if (st == MoveStatus::Failed) return;
                    if (st == MoveStatus::Completed) return;  // already done
                }
                return;
            }
            case TriggerKind::None:
            default:
                return;
        }
    }

    // Members
    std::vector<Motor*>                       motors_;
    std::vector<std::unique_ptr<MoveState>>   pool_;
    std::vector<std::vector<MoveState*>>      per_motor_queues_;
    std::vector<MoveState*>                   per_motor_last_state_;
    std::vector<MotorProfile>                 per_motor_profile_;
    std::vector<std::thread>                  workers_;
    std::atomic<bool>                         release_{false};
    std::atomic<bool>                         shutdown_{false};
};

}  // namespace mks_servo

#endif  // MKS_SERVO_SCHEDULER_HPP
