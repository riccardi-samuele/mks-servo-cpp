# Changelog

All notable changes to `mks-servo-cpp` are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
- `mks_servo/diagnostics.hpp` — `Diagnostics` helper exposing the
  driver's debug surface: `is_protection_latched()`,
  `clear_protection()`, `pulses_received()`, `motor_status()`, and a
  `status_text()` enum-to-string. Useful when a robot is misbehaving
  and you need to find out what the firmware thinks is happening.
- `RawDriver::read_protect_status()` — cmd 0x3E wrapper (the read
  half of release_protection).
- `examples/hil_diagnostics.cpp` — exercises every Diagnostics path
  on the live driver, including a deliberate stall provoke.

### Changed
- `examples/hil_soak.cpp` defaults adjusted to match the bench
  example (poll_interval=0, consecutive=1) which is the reliable
  configuration at 256k baud. The previous conservative defaults
  (poll=2ms, consecutive=2) ran into intermittent ReadFailed on the
  development rig after extended sessions; the new defaults run
  200/200 cleanly.

### Added (continued)
- `mks_servo/characterize.hpp` — `CharacterizationSuite` ported from the
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
