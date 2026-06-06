# Scheduler guide: choreographing multiple motors on independent buses

The `Scheduler` (Level 6 of the API) is for orchestrating moves across
multiple motors where moves have explicit dependencies — "B starts when
A reaches 50 %", "C starts 20 ms before B finishes", "D runs FIFO after
A". For "all motors do the same thing at the same time" use
`MotorGroup` instead; it's simpler and faster for that one pattern.

This guide walks through the API by use case, then covers
`MotorProfile` tuning for heterogeneous fleets (mixed firmware versions
or work modes), the empirical numbers behind the shipped presets, and
the diagnostics the worker publishes for tuning your own setup.

## TL;DR

```cpp
#include <mks_servo/scheduler.hpp>
using namespace mks_servo;

Transport tA, tB, tC;
tA.open("/dev/ttyUSB0", 256000);
tB.open("/dev/ttyUSB1", 256000);
tC.open("/dev/ttyUSB2", 256000);
RawDriver rA(tA, 1), rB(tB, 1), rC(tC, 1);
Motor A(rA, {}), B(rB, {}), C(rC, {});
rA.enable(true); rB.enable(true); rC.enable(true);

Scheduler sched;
sched.add(A);
sched.add(B);
sched.add(C);

// Recommended for a typical V1.0.9 SR_CLOSE fleet on 12V/5A:
sched.set_motor_profile(A, MotorProfile::for_v1_0_9_sr_close());
sched.set_motor_profile(B, MotorProfile::for_v1_0_9_sr_close());
sched.set_motor_profile(C, MotorProfile::for_v1_0_9_sr_close());

// Submit moves:
auto hA = sched.move(A, 90.0, MoveParams{2000, 255});
auto hB = sched.move(B, 90.0, MoveParams{2000, 255}).at_progress(hA, 0.5);
auto hC = sched.move(C, 90.0, MoveParams{2000, 255}).after(hB);

// Execute the whole DAG. Returns the worst per-motor status.
auto worst = sched.run();
```

That's it. Each motor gets its own dedicated worker thread, no shared
locks, no GIL.

## Trigger primitives

`MoveHandle` returned by `sched.move()` exposes four trigger methods.
Default (no trigger) is **FIFO per motor**: the next move on the same
motor waits for the previous one to complete.

| Method | When this move dispatches | Use case |
|---|---|---|
| `.after(other)` | when `other` is in `Completed` state | strict ordering across motors |
| `.at_progress(other, fraction)` | when `other`'s encoder has covered `fraction` of its delta (0-1) | start B while A is still moving |
| `.at_time_after_start(other, ms)` | `ms` after `other` dispatched | timing-locked staggers |
| `.at_time_before_end(other, ms)` | `ms` before `other`'s expected end | hide A's settle inside B's accel |

`at_time_before_end` needs `expected_duration_ms` populated on the
referenced move. The Scheduler auto-populates it when a `MotorProfile`
with `t_90deg_ms > 0` is installed (so call `probe_motor()` or use a
preset); otherwise call `.with_expected_duration_ms()` on the handle
manually.

### Examples

**Pipeline-style overlap** — B starts halfway through A so the bus is
never idle:

```cpp
auto hA = sched.move(A, 90, {2000, 255});
auto hB = sched.move(B, 90, {2000, 255}).at_progress(hA, 0.5);
// FIFO chain on motor A:
auto hA2 = sched.move(A, -90, {2000, 255});  // implicit .after(hA)
```

**Synchronized stop** — both motors finish at the same wall time:

```cpp
// A's profile reports t_90deg_ms; the at_time_before_end stagger
// is computed against A's expected duration.
sched.set_motor_profile(A, MotorProfile::for_v1_0_9_sr_close());
auto hA = sched.move(A, 90, {2000, 255});
auto hB = sched.move(B, 90, {2000, 255}).at_time_before_end(hA, 0.0);
```

## MotorProfile: per-motor tunables

Every registered motor gets a `MotorProfile` (defaults to conservative
values matching V1.0.8 SR_vFOC). Override via
`sched.set_motor_profile(motor, profile)`.

Fields:

| Field | Default | What it controls |
|---|---|---|
| `settle_drain_ms` | 30 | bus drain time after target reached, absorbing the firmware's late "complete" ack frame |
| `predispatch_drain_ms` | 5 | defensive bus flush before each move dispatches |
| `inter_move_rest_us` | 100 000 | worker-thread sleep between consecutive moves on this motor |
| `consecutive_in_window` | 2 | encoder reads within tolerance required before declaring complete |
| `t_90deg_ms` | 0 | measured 90° move time; 0 = unknown (disables `at_time_before_end` auto-compute) |

The shipped presets cover the two HIL-validated configurations:

```cpp
sched.set_motor_profile(m, MotorProfile::for_v1_0_9_sr_close());
// settle_drain 30, inter_move_rest 5 000 µs, t_90deg 40 ms

sched.set_motor_profile(m, MotorProfile::for_v1_0_8_sr_vfoc());
// settle_drain 30, inter_move_rest 100 000 µs, t_90deg 43 ms
```

The big difference is `inter_move_rest_us`: V1.0.9 SR_CLOSE drivers
need 5 ms while V1.0.8 SR_vFOC needs 100 ms. The V1.0.8 firmware
emits a late-ack frame ~ 25 ms after motion completes (over and above
the 30 ms `settle_drain`) that the next move's `predispatch_drain` then
has to absorb — keeping the inter-move rest high gives the late ack
time to arrive before the next dispatch.

### Auto-tune at startup with `probe_motor`

If you don't know whether a motor is V1.0.8 or V1.0.9 (heterogeneous
fleet, fresh AliExpress order, etc.), `probe_motor` measures `t_90deg`
empirically and populates the profile:

```cpp
sched.set_motor_profile(A, MotorProfile::for_v1_0_9_sr_close());
const double measured = sched.probe_motor(A);  // ~5 ±90° samples
std::printf("A t_90deg = %.2f ms\n", measured);
```

This is also a fleet-health sanity check: a motor that reports
`t_90deg ≈ 43 ms` is V1.0.8, `≈ 40 ms` is V1.0.9 SR_CLOSE, and
substantial deviation (e.g. 70 ms) means the motor isn't actually at
SR_CLOSE 256 k baud, or the supply isn't delivering, or the shaft has
load.

## Tuning `inter_move_rest_us`

The shipped V1.0.9 preset uses 5 000 µs. That number comes from
`hil_inter_move_rest_sweep` on the dev fleet. Reproducible numbers
from the sweep on a 12-move 3-motor choreography:

| inter_move_rest_us | wall mean | wall σ | wall max | fails |
|---|---|---|---|---|
| 100 000 (default) | 945 ms | 137 ms | — | 0/10 |
| 20 000 | 925 ms | 143 ms | 1335 ms | 0/10 |
| 10 000 | 850 ms | 31 ms | 912 ms | 0/10 |
| **5 000** | **849 ms** | **23 ms** | **879 ms** | **0/10** |
| 2 000 | 2844 ms | 3255 ms | 10 904 ms | 0/10 |

The shape is **non-monotonic**: above the threshold (≈ 10 ms) σ is
high because cross-run inter-move-rest drift accumulates and shows up
as parallel-dispatch skew; below 5 ms bus collisions start (the firmware
late-ack hasn't drained before the next dispatch tries to read encoder).

If your fleet is different (different firmware version, supply, load,
USB-RS485 adapter, OS scheduler tuning), re-run
`hil_inter_move_rest_sweep` to find your own optimum. The sweep takes
about 5 minutes.

## Dispatch-path diagnostics

Each `MoveState` publishes per-stage timestamps. Read them via the
`MoveHandle::state()` pointer after `sched.run()` returns:

| Field | When the worker stamps it |
|---|---|
| `t_pickup_us` | first instruction inside `process_move` (release_=true → worker entered process_move) |
| `t_predrain_us` | after `predispatch_drain_ms` returned |
| `t_e0_read_us` | after the pre-dispatch encoder read |
| `t_start_us` | right before `move_relative_axis` dispatch |
| `t_end_us` | when encoder is in-window for `consecutive_in_window` reads |

Subtracting two timestamps tells you which stage cost what.
`hil_scheduler_n3.cpp` does this for the parallel A‖B‖C phase to
attribute skew to specific causes (worker wake jitter, bus drain
absorbing a stale frame, encoder read latency).

## Heterogeneous fleet recipe

Recommended setup for a mix of V1.0.8 and V1.0.9 motors:

```cpp
Scheduler sched;
sched.add(A_v1_0_8);
sched.add(B_v1_0_9);
sched.add(C_v1_0_9);

// Install per-motor presets according to each motor's firmware.
// (You can read each motor's firmware version once at startup via
// the raw cmd 0x40 — RawDriver doesn't wrap it yet but the wire
// format is FA addr 40 chk; the V1.0.9 reply length is 9 B,
// V1.0.8 is 8 B.)
sched.set_motor_profile(A_v1_0_8, MotorProfile::for_v1_0_8_sr_vfoc());
sched.set_motor_profile(B_v1_0_9, MotorProfile::for_v1_0_9_sr_close());
sched.set_motor_profile(C_v1_0_9, MotorProfile::for_v1_0_9_sr_close());

// (Optional) refine t_90deg with a probe so at_time_before_end is
// accurate per motor.
sched.probe_motor(A_v1_0_8);
sched.probe_motor(B_v1_0_9);
sched.probe_motor(C_v1_0_9);
```

V1.0.8 motors are the slow link in a mixed fleet (43 vs 40 ms
t_90deg, longer late-ack window). For best wall-time on the
choreography, design DAGs so V1.0.8 moves overlap with V1.0.9 moves
via `at_progress` rather than waiting in FIFO chains.

See `examples/hil_motor_profile_demo.cpp` for a runnable template of
this pattern.

## Performance numbers (HIL, 3 motors, 12V/5A)

From `hil_scheduler_n3`, V1.0.9 SR_CLOSE fleet, 256 k baud:

| Pattern | Wall mean | σ |
|---|---|---|
| Solo (any single motor) | 40-42 ms | ~0.03 ms |
| Sequential A→B→C | ~200 ms | 10 ms |
| Parallel A‖B‖C | ~100 ms | 5 ms |
| Cross-motor at_progress(B, 0.5) | ~85 ms | 3 ms |
| 12-move mixed choreography | **849 ms** | **23 ms** |

Choreography wall vs sum-of-solo-times tells you how much
parallelism the DAG actually achieves; reasonable DAGs are 30-50 %
of sequential wall time.

## See also

- `examples/hil_scheduler.cpp` — basic 2-motor trigger primitives demo
- `examples/hil_scheduler_choreography.cpp` — 8-move 2-motor choreography
- `examples/hil_scheduler_n3.cpp` — 3-motor full bench
- `examples/hil_motor_profile_demo.cpp` — probe + presets end-to-end
- `examples/hil_inter_move_rest_sweep.cpp` — empirical tuning sweep
- [`docs/setup_guide.md`](setup_guide.md) — supply / work_current
- [`docs/design.md`](design.md) — architecture decisions
