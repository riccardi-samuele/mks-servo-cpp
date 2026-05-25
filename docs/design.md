# Design notes

This document captures the non-obvious decisions in `mks-servo-cpp` and
the firmware quirks the design works around. It's intended for
contributors and curious users; everyday users only need the
[README](../README.md) and the inline header docs.

## API levels

Same architecture as the Python `mks-servo` reference. From top to
bottom:

```
┌────────────────────────────────────────────────────────────┐
│  Motor             ergonomic angle_deg / encoder-poll wait │  motor.hpp
├────────────────────────────────────────────────────────────┤
│  MotorGroup        N motors on N buses, batch dispatch     │  motor_group.hpp
├────────────────────────────────────────────────────────────┤
│  RawDriver         one method per firmware opcode (Level 3)│  raw_driver.hpp
├────────────────────────────────────────────────────────────┤
│  Transport         termios2 raw I/O + frame-scanning       │  transport.hpp
├────────────────────────────────────────────────────────────┤
│  protocol          build_frame / parse_frame / checksum8   │  protocol.hpp
└────────────────────────────────────────────────────────────┘
   rt.hpp adds optional mlockall / SCHED_FIFO / core pinning
   to any thread; the library doesn't call it internally.
```

Each level is usable independently. A power user can call `protocol` to
build a frame and write it to any fd; a typical user just touches
`Motor` and `MotorGroup`.

## Zero exceptions in the hot path

Every method that touches I/O returns an enum (`Transport::Status`,
`ParseStatus`, `MotorStatusEx`) or a `Result<T>` aggregate that bundles
the value with its status. No `throw` anywhere in headers. Reasons:

- Predictable cost. RT code can't tolerate stack unwinding mid-control
  loop.
- Easier to audit. Every failure point has an explicit branch.
- C linkage friendly. The header could be wrapped in a `extern "C"` API
  for FFI without changing the implementation.

Users who want exception ergonomics can wrap calls at the API edge:

```cpp
auto must_read(Motor& m) {
    auto r = m.read();
    if (!r.ok()) throw std::runtime_error("read failed");
    return r.value;
}
```

## Zero allocation in the hot path

All frame buffers are `std::array<std::uint8_t, MAX_FRAME_SIZE>` on the
stack (16 bytes). `RawDriver`, `Transport`, and `Motor` carry no
container fields. The only heap allocation in the library is
`MotorGroup`'s `std::vector<Motor*>`, which is touched only at setup
time (`add()`), never on the hot path.

`build_frame` and `checksum8` are `constexpr`-evaluable when their
inputs are constants — the protocol tests exercise this with
`static_assert`.

## Firmware quirks the implementation works around

These are all real behaviours discovered during HIL bring-up on the
MKS SERVO42D V1.0.6 firmware. Understanding them is necessary to
understand why some methods do what they do.

### 1. Two responses per MOVE_*

Every `MOVE_REL_AXIS` / `MOVE_ABS_AXIS` / `MOVE_*_PULSES` command emits
**two** frames back:

- `0x01` ("started") within a few ms of the command landing
- `0x02` ("complete") when the motor's encoder has settled at the
  target — milliseconds to seconds later

Both frames share the same opcode (e.g. `0xF5`) and differ only in the
1-byte payload. The Python reference reads one ack synchronously and
its slow (200 ms) status polling effectively absorbs the second one
between calls. The C++ port polls encoder positions much faster, so we
need to be explicit:

- `RawDriver::move_*_axis` reads the FIRST ack inline (consumes
  "started").
- `Motor::wait_for_position` calls `Transport::drain_input(20)` after
  the encoder reaches its window, to absorb the "complete" before
  returning to the caller. This is what makes chained moves race-free.

### 2. `read_motor_status` and `read_speed_rpm` are *not* safe during motion

Quoting the Python lib's own warning, confirmed in our HIL tests: polling
these registers during a closed-loop move can sabotage the move
(truncated frames, the motor undershooting). The library uses
`read_encoder_addition` everywhere it needs in-motion polling, because
that one is a pure register read and doesn't interfere with the
control loop.

### 3. Polling rate is bounded by the bus baud rate

Every encoder transact is roughly `frame_bytes * 10 / baud` seconds of
wire time:

- 38400 baud → ~5 ms per transact → max ~200 polls/s
- 256000 baud → ~1 ms per transact → max ~1000 polls/s

Polling faster than this causes the firmware to drop frames. The
library default `poll_interval_us = 50'000` (50 ms = 20 Hz) is
conservative — safe at any supported baud. Benchmark and tight-loop
callers should override based on their bus speed; see
`bench_quarter_turn.cpp` for the 256k example (poll_interval_us = 0).

### 4. Stall protection on unloaded motors

Aggressive moves (`acc=255`, `rpm=2000+`) on an unloaded NEMA17
sometimes trigger the firmware's stall-protection latch — transient
rotor vibration looks like a stall to the closed-loop FOC. After
protection trips, MOVE commands return `0x00` (refused) until you call
`RELEASE_PROTECTION` (cmd `0x3D`, exposed as
`RawDriver::release_protection()`). Mount the motor with realistic
inertia before pushing the parameters.

### 5. Flash writes need a settle window

`SET_ZERO_POINT`, `SET_BAUD`, `SET_WORK_CURRENT`, etc. are written to
the driver's flash. The firmware is briefly unresponsive immediately
after one of these (~200 ms). `Motor::set_origin` therefore sleeps
200 ms before and after the `SET_ZERO_POINT` call, with a one-time
retry on read-timeout — matching the Python reference's proven pattern.

### 6. `MOVE_ABS_AXIS` "real-time target update" is conditional

The firmware spec lists MOVE_ABS_AXIS as supporting real-time target
updates (send a new MOVE_ABS_AXIS while the motor is already moving,
the firmware re-targets without stopping). The
[`hil_chained_moves` example](../examples/hil_chained_moves.cpp)
HIL-tested this on the development rig and found the behaviour to be
bimodal:

| When the second MOVE_ABS_AXIS is sent | Total time to final target | Behaviour |
|---|---|---|
| Before the motor enters its deceleration phase for the first target | ~2.7 × baseline | Firmware appears confused; enters a long low-velocity creep |
| After the motor would start decelerating (i.e. close to the first target) | ~0.9 × baseline (actually faster than baseline) | Firmware honors the new target cleanly; the original deceleration phase is replaced by continued cruise |

Practical consequence: the documented "real-time update" feature is
real but only works in a narrow window — specifically, when the
firmware is *about* to start decelerating for the original target. A
naive "retarget mid-motion" API would do the wrong thing most of the
time, so the library does **not** ship one in v1.0. Users who want to
chain moves on the same motor without a deceleration penalty can call
`RawDriver::dispatch_move_absolute_axis` directly with timing
estimated from a measured motion profile (peak rpm, acc time);
characterizing those numbers for the actual motor + load is part of
what the library expects callers to do upstream.

If you find a different firmware revision behaves differently, the
hil_chained_moves example can be re-run as a quick regression check.

### 7. Velocity-mode top speed is supply- and load-dependent

`MOVE_SPEED` (cmd 0xF6) runs the motor continuously at a commanded RPM
rather than to a position target. The firmware accepts targets up to
3000 RPM in vFOC work mode, but the actual sustained speed on a real
rig is bounded by supply voltage and load. The development
NEMA17 + 12V/3A + unloaded shaft caps out around 100 RPM in velocity
mode regardless of acc parameter; 24V + a representative load lifts
the same hardware to 1000+ RPM cleanly.

Practical consequence: do not assume `MOVE_SPEED(N)` reaches `N`.
Measure with `CharacterizationSuite::run_s2_acceleration()` on the
real setup and persist the achievable RPM in the application's motion
plan. Position-mode moves (`MOVE_ABS_AXIS`, `MOVE_REL_AXIS`) are NOT
subject to the same limit — they comfortably push the same hardware
above 2000 RPM transient.

## Why frame-scanning transact

The naive transact reads exactly `expected_bytes` from the fd and
parses them. That works if the bus is clean before every call. It
fails the moment any of the above quirks leave a stray frame in the
buffer — the naive parser reads the stray as the response and gets a
CRC mismatch.

`transact()` instead does a **sliding-window scan**:

1. Read enough bytes for one full frame.
2. Look for `HEAD_UP` (`0xFB`) at the current scan position.
3. Try `parse_frame` there.
4. If CRC and opcode match, return.
5. Otherwise advance one byte and retry, refilling the buffer as needed.

Capped by `MAX_SCAN_BYTES = 256` to prevent spinning on line noise.

This adds maybe a microsecond of overhead per call on a clean bus and
makes the library robust to up to ~30 bytes of stray frames per
transact. Mock tests cover both the clean-bus path and the
stray-frame path.

## Topology decisions

### v1.0: N motors, N buses (one motor per USB-RS485 adapter)

This is the only multi-motor topology supported in v1.0. Every motor
gets its own `Transport`, its own `RawDriver`, and its own `Motor`,
with no shared state between them. `MotorGroup` is a thin batch wrapper
over independent instances.

Skew across N motors when `dispatch_all` is called sequentially:
- ~1.4 ms per motor at 256k baud (frame write + start-ack read)
- 6 motors → ~8.4 ms first-to-last start skew
- At rpm=3000 that's ~150° of rotor difference — invisible to most
  applications

For sub-ms cross-motor synchronisation, run one thread per motor with
`rt::set_realtime_priority` + a shared `eventfd`/`futex` barrier, and
release them simultaneously. The library doesn't ship this pattern — it
lives in user code.

### Shared-bus (multiple motors on one fd) is **not** in v1.0

Implementable in principle (the firmware supports addressing up to 16
slaves on the same RS-485 line), but it requires a mutex over the
shared `Transport` and HIL coverage against multiple physical motors
on one bus. v1.0 ships only what has been HIL-validated; shared-bus
support is on the v1.1 roadmap once a multi-motor rig is available
for validation.

## Measured numbers (development NEMA17 + 12V/3A supply)

All values are mean over multiple HIL runs.

| Operation | At 38400 baud | At 256000 baud |
|---|---:|---:|
| Single read_encoder transact | 4967 μs | 993 μs |
| Single MOVE dispatch (no ack) | ~5000 μs | ~1000 μs |
| Single MOVE dispatch + ack read | ~10000 μs | ~2000 μs |
| 90° quarter-turn, acc=255 | (saturates with poll) | **44.3 ms** |
| Fire-and-forget command queueing | n/a | **141 μs** |
| Soak — 200 moves continuous | n/a | 200/200, max 329 ms |

The 41–44 ms quarter-turn floor is firmware-limited (the acceleration
profile at `acc=255` reaches only ~730 rpm of peak velocity during a
90° turn — see [bench_quarter_turn](../examples/bench_quarter_turn.cpp)).
A higher-supply-voltage rig (24V) and a mechanically loaded shaft
would change these numbers; characterize your own setup before
publishing comparisons.

## What deliberately isn't here in v1.0

- **Profile YAML** — same schema as the Python lib, deferred to v1.1
  because it brings in a YAML parser dependency.
- **`SharedTransport`** — multiple motors on one bus. Deferred until
  multi-motor hardware is available for HIL.
- **`CharacterizationSuite`** — the Python lib's empirical motor
  characterizer. Same reason as profile YAML.
- **macOS / Windows transport** — header-only API is portable but
  `transport.hpp` currently uses Linux termios2. Patches welcome.

## Acknowledgements

This port stands on the shoulders of the Python
[`mks-servo`](https://github.com/riccardi-samuele/mks-servo) library —
its protocol code, test vectors, and HIL learnings (especially the
firmware-quirk docstrings) directly informed every design decision
here. Wire-format compatibility with that library is enforced by the
test suite and is a load-bearing design constraint.
