# Contributing to mks-servo-cpp

Thanks for your interest. This is a small focused library; the
contribution bar is straightforward but real:

- **HIL-validate every code change that touches motor state.** Mock
  tests catch encoding bugs; only the real motor catches firmware-state
  bugs (the trailing "complete" ack, polling-rate saturation, stall
  protection). If you can't run on hardware, say so in the PR and the
  maintainer will validate before merge.
- **Keep `MKS_SERVO_WERROR=ON` green on Linux × {GCC, Clang} × C++{17, 20}.**
  CI enforces this on every PR.
- **No exceptions in I/O paths.** Return enums or `Result<T>`. Throwing
  from a header is fine in argument validation; never from a transact.
- **No allocation in the hot path.** Stack buffers, `std::array`, no
  `std::string`/`std::vector` per call. Setup-time allocation
  (`MotorGroup::add`) is fine.
- **Wire-compatible with the Python `mks-servo` reference.** Wire-format
  tests use vectors lifted verbatim from the Python suite, and they
  must pass.

## Dev setup

```bash
git clone https://github.com/riccardi-samuele/mks-servo-cpp.git
cd mks-servo-cpp

cmake -S . -B build -DMKS_SERVO_BUILD_TESTS=ON -DMKS_SERVO_WERROR=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Requirements: CMake ≥ 3.16, a C++17 compiler (GCC 9+ or Clang 10+). The
test suite fetches doctest automatically via `FetchContent`.

To run the HIL examples against a connected motor:

```bash
ls /dev/ttyUSB*                                       # confirm the adapter
build/examples/read_encoder       /dev/ttyUSB0 38400 1
build/examples/move_quarter_turn  /dev/ttyUSB0 38400 1
build/examples/hil_motor_group    /dev/ttyUSB0 1
build/examples/hil_soak           /dev/ttyUSB0 200 1
```

The HIL examples are non-destructive (they restore the original baud at
exit) but they DO move the motor — make sure the shaft is free.

## Adding a new module

1. Implement the header in `include/mks_servo/<name>.hpp`. Header-only,
   zero alloc on hot path, no exceptions in I/O, returns an enum.
2. Add a mock test in `tests/test_<name>.cpp` using doctest. Wire it
   into `tests/CMakeLists.txt`. Use socketpair-based fake transports if
   the module talks to a `Transport`.
3. Add an HIL example in `examples/hil_<name>.cpp` that exercises the
   module against the real motor. Wire it into `examples/CMakeLists.txt`.
4. Run the example at least 5 times in a row — intermittent failures
   are the failure mode for firmware-state bugs.
5. Document the module's section in `README.md` and any non-obvious
   design choice in `docs/design.md`.

## Commit style

Imperative subject line, short paragraph describing **why**, then bullet
points for **what** and **how**. No automated co-author lines or AI
attribution — commits should look like they were written by a human
contributor.

Example:

```
Fix stray-ack race in MotorGroup chained dispatch

The firmware's trailing "complete" ack landed AFTER the encoder reached
its target window, so the next MOVE_ABS_AXIS dispatch picked up the
stray as its own response and reported NotEnabled.

- move the settle drain into wait_for_position so any caller benefits
- read the start ack inline in dispatch_all instead of fire-and-forget
```

## Reporting bugs

Open a GitHub issue with:
- the firmware version on your MKS SERVO42D (`READ_ALL_CONFIG` /
  `mks-servo profile from-driver` if you have the Python lib),
- the USB-RS485 adapter chip (`lsusb`),
- the supply voltage and current,
- a minimal reproducer (one of the examples is ideal).

Intermittent bugs need the firmware state to drift, so include how many
iterations it took to hit.

## License

By contributing you agree that your contribution is licensed under the
[Apache License 2.0](LICENSE), the same as the rest of the project.
