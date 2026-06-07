# Changelog

All notable changes to `mks-servo-cpp` are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
- `examples/hil_two_motor_sequential_compliant.cpp` — coordination
  example for two motors driving a compliant or detented mechanical
  load (couplings with spring return, indexed mechanisms, multi-motor
  setups joined through a common mechanism). Demonstrates the three
  patterns documented in `docs/scheduler_guide.md` § "Coordinating
  motors on compliant or detented loads": absolute targets via
  `Motor::write` (no relative-drift accumulation), encoder-stable
  adaptive settle (poll `read_encoder_addition` until consecutive
  reads agree, instead of trusting firmware "Stopped"), and symmetric
  application of the settle wait to all motors.

### Documented
- `docs/scheduler_guide.md` — new section on compliant/detented
  loads: why `move_relative(±step)` accumulates drift on coupled
  setups, why an asymmetric settle makes identical motors look
  different, and why `holding_current = 10 %` is the safer default
  when the load has detent self-recovery.
- `docs/setup_guide.md` — new section "Flash settings:
  apply-immediately vs apply-on-boot". HIL-validated: `SET_BAUD` and
  `SET_HOLDING_CURRENT` ack=1 but require a power-cycle to take
  effect. `SET_WORK_CURRENT` applies live. Verifying any
  apply-on-boot change without rebooting can produce misleading
  results.


## [0.4.0] — 2026-06-06

### Added
- `mks_servo/scheduler.hpp` — DAG-based multi-motor execution. One
  worker thread per motor (no shared locks across buses), four
  trigger primitives: `.after(other)`, `.at_progress(other, frac)`,
  `.at_time_after_start(other, ms)`, `.at_time_before_end(other, ms)`.
  Status-based worker scan makes queue clear+repopulate races between
  `sched.run()` calls safe (fixes the hang seen with single-move-per-
  run workloads). Per-move tunables (`settle_drain_ms`,
  `predispatch_drain_ms`, `inter_move_rest_us`, `consecutive_in_window`)
  on every `MoveState`, copied from the per-motor profile at
  `sched.move()` time.
- `MotorProfile` + `Scheduler::set_motor_profile()` — per-motor timing
  tunables for heterogeneous fleets. HIL-validated presets:
  `MotorProfile::for_v1_0_9_sr_close()` (settle_drain 5 ms,
  inter_move_rest 5 ms, measured t_90deg 40 ms) and
  `MotorProfile::for_v1_0_8_sr_vfoc()` (settle_drain 30 ms,
  inter_move_rest 100 ms, measured t_90deg 43 ms). On a 12-move
  3-motor choreography the V1.0.9 preset took σ from 137 ms to 23 ms
  vs the conservative 100 ms default — same mean, much tighter. On a
  20-move sequential B↔C chain the V1.0.9 preset combined with
  `.at_progress(prev, 0.90)` triggers lands at 805 ms wall (0.6 %
  above the 800 ms theoretical floor of 20 × t_90deg) at σ 3.66 ms,
  100 % reliable over a 1000-move soak.
- `Scheduler::probe_motor()` — runs the canonical hil_envelope
  motion-only timing on a motor (5 ±90° samples by default), populates
  `MotorProfile::t_90deg_ms`, returns the measured mean. Replaces
  manual baseline measurements at fleet bring-up.
- Per-move diagnostic timestamps (`t_pickup_us`, `t_predrain_us`,
  `t_e0_read_us`) published by the worker during `process_move`,
  letting callers attribute parallel-dispatch skew to specific stages
  (pickup wake, bus drain, encoder read, firmware ack). Cost: three
  cheap atomic stores per move.
- `examples/hil_scheduler_n3.cpp` — 3-motor Scheduler bench across
  five phases (solo baseline reproduction, parallel A‖B‖C, sequential
  A→B→C, cross-motor `at_progress`, 12-move mixed choreography). The
  test that surfaced the queue-clear race during development.
- `examples/hil_inter_move_rest_sweep.cpp` — empirical tuning sweep
  of `inter_move_rest_us` across {100k, 50k, 20k, 10k, 5k, 2k} µs on
  a 12-move 3-motor choreography. The sweep produced the 5000 µs
  recommendation in `MotorProfile::for_v1_0_9_sr_close()`.
- `examples/hil_motor_profile_demo.cpp` — end-to-end usable template
  for fleet bring-up: `probe_motor()` each registered motor, then
  install per-motor presets, then run.
- `examples/hil_single_motor_bench.cpp` — per-motor characterisation
  (firmware version probe, SR_CLOSE + 256k + calibrate setup, t_90deg
  sweep across {1000/200, 1500/255, 2000/255, 2500/255, 3000/255}
  combos with motion-only timing matching the hil_envelope baseline,
  max sustained RPM via MOVE_SPEED). HIL-confirmed motor C
  (smaller-body V1.0.9) at 39.89 ms ± 0.025 ms — matches motor B
  within natural variation.
- `docs/scheduler_guide.md` — user-facing guide to the Scheduler:
  trigger primitives walkthrough, `MotorProfile` fields and presets,
  the `inter_move_rest_us` sweep table that produced the 5 ms
  recommendation, dispatch-path diagnostic timestamps cheat-sheet,
  and a heterogeneous-fleet recipe for mixed V1.0.8 + V1.0.9 setups.
- `tests/test_scheduler_mock.cpp` — 8 cases / 67 assertions against
  a socketpair-backed mock motor (teleport semantics — instant
  encoder/ack reply, no wall-clock dependency). Covers default and
  preset MotorProfile values, profile install / lookup, per-MoveState
  field propagation, MoveHandle trigger fluent API, single-move
  `sched.run()` smoke, and a regression test for the queue-clear
  race (ten consecutive single-move runs must all complete).

### Fixed
- `Scheduler` worker race: the previous index-based scan kept a
  persistent `next` counter and a 100 ms inter-move sleep that could
  outlive the run that scheduled the move. A subsequent `sched.run()`
  that cleared and repopulated the queue while the worker was still
  asleep would leave the worker spinning forever on `release_=false`.
  The new status-based scan iterates the queue each pass looking for
  `Pending` moves; queue resets and repopulates are safe.

## [0.3.0] — 2026-05-31

### Added
- `docs/setup_guide.md` — user-facing setup guide answering "12 V or
  24 V?" and "what work_current?". Includes HIL-measured comparison
  tables across three PSUs (12V wall-wart, 12V lab, 24V lab) and a
  quick-pick table for common scenarios. Captures the non-obvious
  finding that the firmware default work_current=1600 mA caps the
  motor at ~1800 RPM at 24V — raising it to 2000 mA unlocks the full
  3000 RPM firmware-spec cap.
- `examples/hil_voltage_setup.cpp` — tuning helper that sweeps
  work_current candidates (1000-3000 mA) at the current supply
  voltage, runs a high-RPM probe at each, and recommends the lowest
  work_current that reaches the firmware-spec 3000 RPM cap. Restores
  the factory default (1600 mA) at the end. Runtime ~1-2 minutes.
- `Motor::firmware_target_for(angle)` and `Motor::encoder_target_for(angle)`
  — public conversion methods that explicitly split the two coordinate
  frames that the existing `angle_to_counts` conflated. Internal lib
  code (`Motor::write`, `dispatch_write`, `MotorGroup::move_all`) now
  uses these to send the correct MOVE_ABS_AXIS target AND poll the
  correct encoder reading in `wait_for_position`. See the `Conversions`
  doc block in `motor.hpp` for the full rationale.
- `save_envelope` / `load_envelope` / `auto_calibrate_cached` in
  `envelope.hpp` — binary persistence (magic + version + struct
  bytes; ~80 bytes/file) and a cache-friendly wrapper that loads
  from disk + does a cheap comms-only sanity probe (~50 ms) and
  falls through to a full `auto_calibrate` only on a real cache
  miss. HIL: cache-hit second-run total time drops from ~3 seconds
  to ~1.3 s (mostly baud-probe and enable, not the calibration
  itself).
- `mks_servo/envelope.hpp` — `Envelope` struct + `auto_calibrate(Motor&)`
  function. ~3-6 s self-characterisation that discovers the safe-fast
  operating point of the motor + supply + load and returns a plain-
  old-data Envelope (max steady RPM, recommended {rpm, acc} that
  passed a mini-soak, measured 90° time + sigma, peak overshoot,
  comms latency p50/p99). The "lib as a toy that finds its own
  limits" pattern: user calls `auto_calibrate` once at startup, then
  passes `MoveParams{env.recommended_rpm, env.recommended_acc}` to
  subsequent moves. Persist `env` to disk (POD, just memcpy) for
  instant subsequent startups. HIL-validated on NEMA17 42mm at 24V:
  picks rpm=2000/acc=255 reliably; 5 consecutive runs produced
  identical recommendations.
- `examples/hil_auto_calibrate.cpp` — runs auto_calibrate on a live
  motor and dispatches one round-trip move at the discovered params
  to demonstrate the "after calibration, just use m.write(angle)"
  pattern.
- `examples/hil_envelope.cpp` — end-to-end operating-envelope
  characterisation. Six tests (comms latency, 90° vs acceleration,
  90° vs commanded RPM, overshoot, soak, max steady RPM) report what
  THIS motor + supply + load can actually deliver, with explicit
  validity criteria per test. Optional `--json <path>` produces a
  machine-readable report intended as the input for the planned
  `Motor::auto_calibrate()` / `Envelope` API. Distinct from
  `characterize` (servo quality — precision, follow error) — this
  measures the operating envelope (how fast / how reliably).
  Validated on NEMA17 42mm at 24V/10A: 40.66±0.02ms per 90° at
  acc=255 rpm=2000, max steady RPM 1856, 99/100 soak success.

### Changed
- `Motor::write`/`move_relative`/`move_relative_shortest` default
  `tolerance_counts` 16 → 50 (~0.35° → ~1.1°). HIL-validated: with
  the old tight default, a user calling `m.write(angle,
  MoveParams{2000, 255})` had 14/20 timeouts because the firmware's
  transient overshoot at max acceleration (~3.5° peak) couldn't
  settle within 0.35° fast enough; the new default produces 20/20 ok
  in ~56 ms per 90° move. Same value the Python reference and our
  bench_quarter_turn use. Pass a tighter tolerance explicitly when
  you've verified your load doesn't overshoot that much.
- `Motor::wait_for_position` defaults:
  - `poll_interval_us` 50'000 → 0 (back-to-back at 256k baud is safe
    — each encoder transact is ~1 ms — and the old 50 ms gap meant
    `m.write` blocking spent up to 50 ms idle between polls). Users
    on 38 400 baud should pass ≥5 000 explicitly.
  - `settle_drain_ms` 20 → 15 (HIL-measured worst-case "complete" ack
    arrival is ~15.6 ms post-window; 15 ms covers it without the
    spare 5 ms the old default wasted on every move).
  Combined impact on a 90° move via `Motor::write(angle,
  MoveParams{2000, 255})` blocking with defaults: 205 ms → 56 ms
  (3.7× faster, same code) — HIL-measured on NEMA17 42mm at 24V.

### Fixed
- `Motor::write`/`dispatch_write` and `MotorGroup::move_all` now use
  separate firmware-frame and encoder-frame targets. Previously a
  single `angle_to_counts(angle_deg)` value was passed both as the
  MOVE_ABS_AXIS target (firmware frame, reset to 0 by SET_ZERO_POINT)
  AND as the `wait_for_position` reference (encoder frame, NOT reset
  by SET_ZERO_POINT). The two frames differ by the origin_offset_counts
  captured at set_origin time — a value that is large after any
  preceding MOVE_SPEED rotation. Symptom on HIL: a 90° move
  immediately after `auto_calibrate_cached` (whose RPM probe used to
  rotate the motor for ~3 s) ran the motor for ~6 revolutions in the
  wrong direction before timing out. The fix wires the two frames
  through the right targets via the new `firmware_target_for` /
  `encoder_target_for` methods. Public `angle_to_counts` is unchanged
  for back-compat with user code that calls it directly (it's the
  encoder-frame value — useful for diagnostics, not for dispatch).
- `Motor::wait_for_position` now tolerates up to 2 consecutive isolated
  encoder-read failures before propagating an error. Back-to-back
  polling at 256 k baud (the new default) occasionally races with a
  stray "complete" ack frame from a previous move, producing a
  transient ReadFailed at transport level. Previously the first such
  glitch was fatal — Motor::write blocking returned TransportError
  even though the motor had landed at target correctly. With the
  counter, isolated glitches are absorbed (one ~2 ms backoff + retry);
  3+ consecutive failures still surface immediately. HIL: 5/5 demo
  moves after auto_calibrate now report status=0 (was 1-2/5 with
  transient TransportError/Timeout/NotEnabled mixed).
- `Motor::set_origin` now anchors `origin_offset_counts` to the encoder
  reading captured immediately after SET_ZERO_POINT, instead of blindly
  resetting it to 0. The encoder addition register is NEVER reset by
  SET_ZERO_POINT on the firmware variants tested — it keeps accumulating
  from power-on. The old behavior left `m.read()` returning a huge
  cumulative angle right after set_origin, breaking any subsequent
  `m.write(angle)` (which dispatched MOVE_ABS_AXIS targets computed
  from the wrong frame). The fix is a one-line change in the hot path
  (an extra encoder read post-SET_ZERO_POINT) and HIL-validated:
  m.read() returns ~0 even when prior MOVE_REL_AXIS has accumulated
  millions of encoder counts. New mock regression test added.
- `MotorGroup::dispatch_all` no longer silently masks firmware refusals.
  Previously only `MotorStatusEx::TransportError` was surfaced; firmware
  ack `0x00` (`NotEnabled`, e.g. stall-protection latched or coil driver
  not ready), `ParseError` and `LimitExceeded` collapsed to `OK`, and
  the downstream `wait_all_settled` would then poll a never-moving
  motor until the full timeout. HIL-confirmed with one broken motor in
  a 2-motor group: error-detection latency went from 3007 ms to 6.7 ms.
- `MotorGroup::move_all` no longer clobbers `out_per_motor` with a
  uniform `TransportError` on dispatch failure; the per-motor outcome
  recorded by `dispatch_all` (some motors `OK`, the broken one
  `NotEnabled`, …) is preserved for the caller. Also, the returned
  aggregate now reflects the actual failure cause instead of
  unconditionally `TransportError`.

### Changed
- `MotorGroup::dispatch_all` signature: returns `MotorStatusEx` (was
  `Transport::Status`), `out_per_motor` is `MotorStatusEx*` (was
  `Transport::Status*`). Breaking for callers, but pre-1.0; required
  to carry per-motor `NotEnabled` / `LimitExceeded` info that the old
  type couldn't represent.
- `MotorGroup::move_all` is now fail-fast on dispatch failure: returns
  immediately with the failure status without entering
  `wait_all_settled`. CAVEAT: motors that dispatched OK are still
  physically executing their moves when the call returns; the caller
  owns recovery (emergency-stop, or wait on the subset). Documented in
  the header.

## [0.2.0] — 2026-05-25

### Added
- `mks_servo/profile.hpp` — value-typed `Profile` aggregate matching
  the Python lib's YAML profile schema 1:1 (transport / config /
  mechanical / limits / origin / characterization). Two helpers:
  `apply_profile_to_motor()` (in-memory copy, no wire activity) and
  `apply_profile_to_firmware()` (pushes work_mode, microsteps,
  work_current_ma via the SET_* opcodes). No YAML parser bundled —
  users wire in their own format.
- `Motor::move_relative_shortest(delta_deg)` — picks the equivalent
  rotation in [-180, 180] before executing, for cyclic / orientation
  use cases (turret joints, gripper rotation). Only safe with no
  position limits.
- `Motor::set_auto_clear_protection(bool)` — when on, a refused MOVE
  (firmware ack 0x00, typically stall protection) triggers
  RELEASE_PROTECTION + one retry. Off by default.
- `mks_servo/diagnostics.hpp` — `Diagnostics` helper exposing the
  driver's debug surface: `is_protection_latched()`,
  `clear_protection()`, `pulses_received()`, `motor_status()`, and a
  `status_text()` enum-to-string. Useful when a robot is misbehaving
  and you need to find out what the firmware thinks is happening.
- `RawDriver::read_protect_status()` — cmd 0x3E wrapper (the read
  half of release_protection).
- `mks_servo/characterize.hpp` — `CharacterizationSuite` ported from
  the Python reference. Four tests:
  Python reference. Four tests:
    - P1 (precision): repeatability spread (mean / sigma / peak) over N
      moves to the same target.
    - P3 (error vs RPM): RMS of (read − target) at each commanded RPM.
      Tolerant of per-RPM failures: failed RPMs land in `failed_rpms`,
      successful ones in `rpm_samples` + `rms_error_deg`.
    - P5 (follow error): max / RMS of the firmware's angle-error
      register during a continuous sweep at constant RPM.
    - S2 (acceleration): time-to-target across a list of acc parameter
      values via MOVE_SPEED (velocity mode); reports max observed RPM.
- `RawDriver::move_speed(rpm, acc, direction)` — cmd 0xF6 wrapper used
  by S2; not previously exposed.
- `examples/characterize.cpp` — HIL example that runs all four tests
  and prints the results.
- `examples/hil_chained_moves.cpp` — HIL experiment that probes the
  firmware's "real-time MOVE_ABS_AXIS update" feature with three
  retarget timings. Surfaced a bimodal behaviour: the update is
  honored cleanly when sent close to the first target's deceleration
  point, but causes a long low-velocity creep when sent earlier. See
  `docs/design.md` §6 for the full writeup.

### Changed
- `examples/hil_soak.cpp` defaults adjusted to match the bench
  example (poll_interval=0, consecutive=1) which is the reliable
  configuration at 256k baud. The previous conservative defaults
  (poll=2ms, consecutive=2) ran into intermittent ReadFailed on the
  development rig after extended sessions; the new defaults run
  200/200 cleanly.

### HIL-validated on the development NEMA17 + 12V/3A rig
- Mock suite: 6 binaries / ~55 cases / >200 assertions, 100% green
- `hil_motor_group`: 5/5 consecutive runs, sub-2° accuracy
- `hil_soak`: 200/200 quarter-turns continuous, mean 317 ms,
  p99 = 320 ms, 0 failures
- `bench_quarter_turn`: 10/10, mean 55 ms at 256 kbaud
- `hil_shortest`: 6/6 cases including wrap-around at ±180°
- `hil_profile`: position-reject / speed-clamp / legal-move paths all
  exercised correctly
- `hil_diagnostics`: every read path returns expected values; clear
  accepted by firmware
- `characterize`: P1 sigma 0.009°, P3 rms 0.066-0.131° (50-500 rpm),
  P5 max follow 0.66°, S2 reaches the velocity-mode ceiling for the
  unloaded rig (~100 rpm) — limit documented in design.md §7.

## [0.1.0] — initial scaffolding & HIL-validated v1.0 candidate

First implementation. Six header-only modules, six HIL-validated
examples, ~55 mock cases + 215 assertions, 200/200 continuous-move soak
green on a NEMA17 + 12V/3A development rig.

### Added — library
- `protocol.hpp` — frame build/parse, CRC-8, zero allocation, `constexpr`.
  Wire-compatible with the Python `mks-servo` reference (test vectors
  taken verbatim).
- `transport.hpp` — termios2 raw serial I/O with `BOTHER` for
  non-standard baud rates (256000). `poll()`-based read with microsecond
  timeout. Frame-scanning `transact()` that survives stray firmware
  acks left in the bus buffer.
- `raw_driver.hpp` — 1:1 wrapper over the MKS firmware opcode set.
  Read primitives (`read_encoder_addition`, `read_speed_rpm`, etc.) and
  move primitives (`move_relative_axis`, `move_absolute_axis`,
  `move_speed`) with both ack-reading and fire-and-forget variants.
- `motor.hpp` — Level 0/1 ergonomic API. `write(angle_deg)` / `read()`
  with gear ratio and origin offset, soft position/speed limits
  (Reject / Clamp / Warn policies), `set_origin()` (hard firmware
  reset) and `set_origin_soft()` (in-memory). Encoder-based blocking
  wait with built-in settle drain.
- `motor_group.hpp` — batch operations over N motors each on their own
  Transport. `dispatch_all`, `wait_all_settled` (round-robin, settled
  motors drop out), `move_all` convenience. Single-threaded by design;
  RT-thread orchestration lives in user code.
- `rt.hpp` — optional helpers: `lock_memory()` (mlockall),
  `set_realtime_priority(n)` (SCHED_FIFO), `pin_thread_to_core(c)`
  (pthread_setaffinity_np), and an `install({...})` convenience that
  applies all three. Library never calls these internally.

### Added — build
- CMake 3.16+ with `find_package(mks_servo)` and a `mks_servo::mks_servo`
  interface target. Install rules, version config, namespaced exports.
- GitHub Actions matrix: Linux × {GCC, Clang} × C++{17, 20}, plus
  macOS and Windows smoke builds.
- `MKS_SERVO_WERROR=ON` opt-in for treat-warnings-as-errors. Strict
  flag set: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion
  -Wsign-conversion`.

### Added — examples
- `read_encoder` — HIL hello world: open the bus, read the encoder.
- `move_quarter_turn` — Motor::write/read against the live motor.
- `bench_quarter_turn` — apples-to-apples vs Python `bench_tmin.py`.
  Result: mean 44.3 ms / min 43.7 ms vs Python's 41.7 ms (delta is
  the cost of frame-scanning robustness).
- `stress_read` — 5000 isolated encoder reads, 0 failures.
- `hil_motor_group` — MotorGroup over a single physical motor.
- `hil_soak` — 200+ alternating quarter-turns continuous; flushes
  out rare comm faults and state-accumulation bugs.
- `rt_demo` — show what `rt::install` can apply under the running uid.

### Added — docs
- `README.md` with quickstart, API levels, validation summary.
- `docs/design.md` with architecture decisions, the firmware quirks
  the design works around, and measured numbers.
- `CONTRIBUTING.md` with dev setup, HIL test policy, commit style.

### Validated against MKS SERVO42D V1.0.6 firmware
- 6 doctest binaries: 55 cases / 215 assertions, 100% green
- `hil_motor_group`: 5/5 runs, accuracy ±2°
- `hil_soak`: 200/200 quarter-turns continuous, 0 failures, 65.7 s
- `bench_quarter_turn`: 10/10 runs at 256k baud, mean 44.3 ms

### Known limitations (planned for v1.1+)
- Shared-bus (multi-motor on one USB-RS485) not yet supported — needs
  multi-motor hardware for HIL validation.
- Profile YAML loading not yet wired (deferred — pulls in a YAML
  dependency).
- `transport.hpp` is Linux-only (`termios2`). macOS / Windows backends
  are on the roadmap; the protocol/raw_driver/motor/motor_group layers
  are portable.
- `MOVE_ABS_AXIS` real-time target updates (firmware-documented but
  not yet HIL-validated against the chained-move use case).
