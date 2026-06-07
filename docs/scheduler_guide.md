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
// settle_drain 5, inter_move_rest 5 000 µs, t_90deg 40 ms

sched.set_motor_profile(m, MotorProfile::for_v1_0_8_sr_vfoc());
// settle_drain 30, inter_move_rest 100 000 µs, t_90deg 43 ms
```

The two big differences between the V1.0.8 and V1.0.9 presets are
`settle_drain_ms` (30 → 5) and `inter_move_rest_us` (100 000 → 5 000).
The V1.0.8 firmware emits a late-ack frame ~ 25 ms after motion
completes that the next move's `predispatch_drain` has to absorb,
which is why the conservative settle and high inter-move rest are
needed. V1.0.9 SR_CLOSE emits the ack much sooner — empirically
settle_drain=5 ms passed a 1000-move soak with 0 failures (and
settle_drain=2 ms also passed at 0 failures, but 5 ms keeps a
margin against edge motor / cable conditions).

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

## Fast sequential chains: `at_progress(prev, 0.90)` over `.after(prev)`

When you have a chain of moves where each waits for the previous to
finish — e.g. a 20-move B↔C sequence — `.after(prev)` is the obvious
choice but it stalls the next dispatch for the FULL `settle_drain_ms`
of the previous move (the worker only marks `Completed` after the
drain returns). `.at_progress(prev, 0.90)` instead fires when the
previous move's encoder has covered 90 % of its delta, which is
typically a few ms before motion ends — and the `settle_drain` of the
previous move then overlaps with the acceleration ramp of the next
one, completely hidden.

Measured on a 20-move sequential B↔C chain (V1.0.9 SR_CLOSE,
12 V/5 A, soak N=50):

| Pattern | Wall mean | σ | min | max |
|---|---|---|---|---|
| `.after(prev)` + settle=30 (old preset) | 1343 ms | 32.49 | 1279 | 1369 |
| `.after(prev)` + settle=5 (new preset) | 989 ms | 3.60 | 984 | 995 |
| `.at_progress(prev, 0.90)` + settle=30 | 851 ms | 3.28 | 847 | 855 |
| **`.at_progress(prev, 0.90)` + settle=5 (recommended)** | **805 ms** | **3.66** | **797** | **813** |
| Theoretical floor (20 × t_90deg) | 800 ms | — | — | — |

The recommended combination lands within 0.6 % of the floor. All
configs above were 100 % reliable across the 1000-move soak.

If your application has true strict ordering (the next move's
parameters depend on the previous one's outcome), keep `.after()`.
For "always do A then B then …" choreographies, `.at_progress(0.90)`
is essentially free.

## Coordinating motors on compliant or detented loads

This section covers a class of mechanical setups where the standard
"firmware reports Stopped → next move dispatches" pattern fails:

- **Detented loads** — Geneva drives, indexed couplings, magnetic
  centring, click-detent mechanisms. The load has stable rest states
  separated by an unstable transition zone.
- **Compliant loads** — couplings with spring return, tendon drives,
  loaded gear trains where the load keeps moving for several ms after
  the motor has electrically stopped.
- **Mechanically coupled multi-motor setups** — two or more motors
  whose loads are joined through a common mechanism that requires all
  loads to be at a rest state before any single motor can move next.

On such setups, three things go wrong with naive sequential
coordination:

1. **`firmware Stopped ≠ load at rest`**. The motor's PI loop declares
   Stopped while the load is still settling toward its rest state.
   Dispatching the next motor's move during this window can jam the
   common mechanism.
2. **`move_relative(±step_deg)` accumulates drift**. Each move's
   tolerance-bounded overshoot becomes the starting point of the next
   move. After N cycles the encoder is N × overshoot off from the
   nominal axis position, even though the physical load may still be
   near its detent.
3. **Asymmetric settle handling makes identical motors look different.**
   If you wait for M0's load to settle after M0 moves but skip the wait
   after M1, then M0 always starts its next move from a fully-settled
   state and M1 doesn't — so M1 appears to drift more even when the
   hardware is identical.

The three patterns below address each:

### 1. Use absolute targets, not relative deltas

Instead of `m.move_relative(±step_deg)` per cycle, track the expected
axis position in your code and use `m.write(absolute_target_deg, …)`:

```cpp
double target = 0.0;
for (int i = 0; i < N; ++i) {
    target += (i & 1) ? -90.0 : 90.0;          // accumulates in user code
    m.write(target, mp, /*blocking=*/true, /*tol_counts=*/50);
}
```

`Motor::write` dispatches `MOVE_ABS_AXIS` (cmd `0xF5`), so each move
targets the absolute axis position regardless of where the previous
one happened to land. Single-move overshoot becomes a static offset,
not a ratcheting drift.

### 2. Confirm load-at-rest by polling the encoder, not by trusting "Stopped"

For compliant loads, replace blind `sleep_ms(settle)` calls with an
adaptive wait that polls `read_encoder_addition` until consecutive
reads agree to within a small threshold:

```cpp
static double wait_until_stable(RawDriver& d, int cap_ms) {
    auto t0 = now_us();
    std::int64_t prev = d.read_encoder_addition().value;
    int stable = 0;
    while (stable < 3) {
        sleep_ms(15);
        std::int64_t cur = d.read_encoder_addition().value;
        if (std::abs(cur - prev) <= 2) ++stable;
        else                            stable = 0;
        prev = cur;
        if ((now_us() - t0) > (uint64_t)cap_ms * 1000) break;
    }
    return (now_us() - t0) / 1000.0;
}
```

Typical wait on a free-running load: 15-30 ms (one or two poll
intervals). On a heavy compliant load: 80-150 ms. The cap_ms ceiling
protects against pathological cases where the load never reaches a
rest state (e.g. a true mechanical bind).

### 3. Apply the settle wait symmetrically to every motor

Sequential coordination across N motors needs **every** motor's load
to be at rest before the next motor dispatches. Apply the wait after
each motor's move, not only after the first one. The bench example
in `examples/hil_two_motor_sequential_compliant.cpp` walks through a
2-motor variant of this pattern and a 50-cycle soak.

### Holding current and the detent-recovery tradeoff

A detented load with **low holding current** (e.g. 10 %) will
self-correct into its detent after each move: the detent spring
overpowers the motor's holding torque and pulls the rotor through the
last fraction of a degree. The motor's encoder shows a constant
per-move offset, but the load is consistently at its detent rest
state. This is the regime where the absolute-target pattern is
self-healing — drift on the encoder doesn't translate into drift of
the load.

With **high holding current** (≥ 30-50 %), the motor wins against the
detent spring and locks the rotor at the firmware's "good enough"
stop position. The load stays slightly off its rest state. On a
mechanically coupled multi-motor setup this accumulates across cycles
and eventually jams the common mechanism.

For detented loads on this firmware, **`holding_current = 10 %` is
the safer default** for sequential coordination — the lower
mechanical determinism is offset by the load's intrinsic
self-recovery, and the lower idle current keeps the motor cooler.
Note that `SET_HOLDING_CURRENT` requires a power-cycle to take
effect (see `docs/setup_guide.md`).

## Performance numbers (HIL, 3 motors, 12V/5A)

From `hil_scheduler_n3` (V1.0.9 SR_CLOSE) on a freshly idle fleet:

| Pattern | Wall mean | σ |
|---|---|---|
| Solo (any single motor) | 40-42 ms | ~0.03 ms |
| Sequential A→B→C (`.after()`, chain of 3) | ~200 ms | 10 ms |
| Parallel A‖B‖C | ~100 ms | 5 ms |
| Cross-motor `at_progress(B, 0.5)` | ~85 ms | 3 ms |
| 12-move mixed choreography (3-motor) | 794 ms | 2.87 ms |
| 20-move B↔C `.at_progress(0.90)` chain | **805 ms** | **3.66 ms** |

Choreography wall vs sum-of-solo-times tells you how much
parallelism the DAG actually achieves; reasonable DAGs are 30-50 %
of sequential wall time.

## See also

- `examples/hil_scheduler.cpp` — basic 2-motor trigger primitives demo
- `examples/hil_scheduler_choreography.cpp` — 8-move 2-motor choreography
- `examples/hil_scheduler_n3.cpp` — 3-motor full bench
- `examples/hil_motor_profile_demo.cpp` — probe + presets end-to-end
- `examples/hil_inter_move_rest_sweep.cpp` — empirical tuning sweep
- `examples/hil_two_motor_sequential_compliant.cpp` — coordination on compliant/detented loads
- [`docs/setup_guide.md`](setup_guide.md) — supply / work_current
- [`docs/design.md`](design.md) — architecture decisions
