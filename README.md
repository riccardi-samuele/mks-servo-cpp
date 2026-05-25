# mks-servo-cpp

[![tests](https://github.com/riccardi-samuele/mks-servo-cpp/actions/workflows/test.yml/badge.svg)](https://github.com/riccardi-samuele/mks-servo-cpp/actions)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![license](https://img.shields.io/badge/license-Apache--2.0-green.svg)](LICENSE)

Header-only C++17 library for MKS SERVO42D RS-485 stepper drivers
(NEMA17 / NEMA23). Zero allocation in the hot path, no exceptions in
the I/O path, real-time scheduling friendly. Wire-protocol-compatible
with the Python [`mks-servo`](https://github.com/riccardi-samuele/mks-servo)
reference, byte-for-byte.

## What this library provides

A small, focused surface for talking to MKS SERVO42D drivers from C++
without dragging in a Python runtime, exceptions, dynamic allocation,
or a GIL. It fills a gap in the MKS ecosystem: a community-grade C++
binding suitable for real-time control loops, robotics frameworks
(ROS 2, ros2_control, custom orchestrators), embedded targets, and
any other context where the Python reference can't be embedded.

The design priorities, in order:

1. **Wire-correct.** Frame encoding / decoding matches the firmware
   manual and is HIL-validated against a live driver.
2. **Predictable.** No exceptions, no heap allocations on the hot
   path, no hidden global state. Every failure path is an enum
   return that the caller can branch on.
3. **Real-time friendly.** Optional `rt.hpp` helpers for `mlockall`,
   `SCHED_FIFO`, and core pinning; the library never calls them
   internally so callers stay in control.
4. **Layered.** Five levels (`protocol` → `transport` → `raw_driver`
   → `motor` → `motor_group`) so users can pick the abstraction that
   matches their problem.

## Quick start

```cmake
include(FetchContent)
FetchContent_Declare(
    mks_servo
    GIT_REPOSITORY https://github.com/riccardi-samuele/mks-servo-cpp.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(mks_servo)

add_executable(my_robot main.cpp)
target_link_libraries(my_robot PRIVATE mks_servo::mks_servo)
```

```cpp
#include <mks_servo/motor.hpp>
#include <mks_servo/transport.hpp>
#include <mks_servo/raw_driver.hpp>

using namespace mks_servo;

int main() {
    Transport t;
    t.open("/dev/ttyUSB0", 256000);

    RawDriver raw(t, /*slave_addr=*/1);
    Motor     m(raw, Mechanical{/*gear_ratio=*/1.0, /*offset=*/0});

    raw.enable(true);
    m.set_origin();                          // current position = 0°
    m.write(90.0, {/*rpm=*/600, /*acc=*/200}); // blocking; polls encoder
    auto pos = m.read();                     // → ~90.0
}
```

## API levels

Each level builds on the one below; pick the lowest that lets you express
what you need.

| Level | Header | Surface | When to use |
|-------|--------|---------|-------------|
| 0 + 1 | `motor.hpp` | `Motor::write/read/move_relative`, soft limits, set_origin | Day-to-day robotics |
| Group | `motor_group.hpp` | `MotorGroup::dispatch_all`, `wait_all_settled` | N motors, N dedicated buses |
| 3 | `raw_driver.hpp` | `RawDriver` — 1:1 wrapper over every firmware opcode | Power-user / debugging |
| Wire | `protocol.hpp` | `build_frame`, `parse_frame`, `checksum8` | Custom framing, tests |
| RT | `rt.hpp` (optional) | `lock_memory`, `set_realtime_priority`, `pin_thread_to_core` | Sub-ms determinism |

## Topology

v1.0 supports **N motors on N dedicated USB-RS485 buses** — one motor per
adapter, in parallel. Shared-bus (multiple motors on one adapter) is on
the roadmap but isn't HIL-validated yet, so it isn't shipped.

```
host  ──/dev/ttyUSB0── motor 0
      ──/dev/ttyUSB1── motor 1
      ──/dev/ttyUSB2── motor 2          ── independent state, no
      ──/dev/ttyUSB3── motor 3             shared locks, no GIL
      ──/dev/ttyUSB4── motor 4             contention
      ──/dev/ttyUSB5── motor 5
```

## Design principles

| | |
|---|---|
| Header-only | Drop into any project via `FetchContent` or `find_package`. |
| Zero allocation in the hot path | All buffers are `std::array` on stack — no `std::string`, no `std::vector` per call. |
| No exceptions in I/O | Errors are returned as enums (`Transport::Status`, `MotorStatusEx`). |
| `constexpr` where free | Frame helpers usable at compile time for static tests. |
| C++17 baseline, C++20-clean | Builds and tests under both standards. |
| Wire-compatible with the Python lib | Mock-suite vectors are taken from the Python tests verbatim. |
| Linux first | macOS and Windows backends are planned. Library compiles on macOS/Windows but the transport layer is currently Linux-only. |

## Layout

```
include/mks_servo/
  protocol.hpp      Frame build/parse + CRC, zero-alloc, constexpr
  transport.hpp     termios2 raw I/O + frame-scanning transact()
  raw_driver.hpp    Level 3 1:1 opcode wrapper + dispatch_* (fire-and-forget)
  motor.hpp         Level 0/1 ergonomic API, soft limits, set_origin
  motor_group.hpp   N-bus N-motor batch operations
  rt.hpp            Optional helpers: mlockall, SCHED_FIFO, core pinning

examples/
  read_encoder.cpp        — minimal HIL hello-world
  move_quarter_turn.cpp   — Motor::write/read demo
  bench_quarter_turn.cpp  — apples-to-apples bench vs the Python reference
  hil_motor_group.cpp     — MotorGroup against the live motor
  hil_soak.cpp            — 200+ moves continuous; rare-fault detection
  stress_read.cpp         — encoder polling at line rate
  rt_demo.cpp             — show what rt.hpp can install on this user

tests/
  test_protocol.cpp        — frame build/parse vectors from Python suite
  test_transport_compile.cpp
  test_raw_driver_mock.cpp — socketpair-based, opcode-level
  test_motor.cpp           — math, limits, set_origin (mock)
  test_motor_group.cpp     — multi-motor batch ops (mock)
  test_rt.cpp              — argument validation + privilege paths
```

## Validation

Every module has both mock and HIL coverage:

| Area | Mock | HIL on live NEMA17 + 12V supply |
|---|---|---|
| Protocol / wire format | 55 cases, 215 assertions | byte-for-byte vs Python lib |
| Transport (termios2, baud, scan) | smoke + frame-scan corner cases | 5000-read stress, 0 failures |
| RawDriver | 9 cases incl. stray frames | every opcode used by Motor |
| Motor (Level 0/1) | 10 cases | move_quarter_turn 5/5 |
| MotorGroup | 6 cases | hil_motor_group 5/5 |
| Soak | — | **200/200 moves continuous, 0 failures** |
| Benchmark vs Python | — | 44.3 ms mean (vs Python 41.7 ms) at 256k baud |

CI runs Linux × {GCC, Clang} × C++{17, 20}, plus macOS / Windows smoke.

## Building

```bash
cmake -S . -B build -DMKS_SERVO_BUILD_TESTS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Optional: `-DMKS_SERVO_WERROR=ON` to treat warnings as errors,
`-DMKS_SERVO_BUILD_EXAMPLES=OFF` to skip examples.

## License

[Apache 2.0](LICENSE). The patent grant is intentional: this library is
meant to be safe to use in hardware and robotics products.

## See also

- [`mks-servo`](https://github.com/riccardi-samuele/mks-servo) — the
  Python sibling library. Production-stable, HIL-validated.
- [`docs/design.md`](docs/design.md) — architecture decisions and the
  firmware quirks the design works around.
- [MKS SERVO42D firmware manual](https://github.com/makerbase-motor/MKS-SERVO42D-57D)
