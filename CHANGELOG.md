# Changelog

All notable changes to `mks-servo-cpp` are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
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
