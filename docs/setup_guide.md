# Setup guide: matching your supply voltage and work_current

This guide answers two questions that aren't obvious from the MKS
documentation:

1. **What supply voltage should I use?** 12 V or 24 V?
2. **What `work_current` should I set?** The default is 1600 mA, but
   is that right for my supply?

The short answer is in [Quick-pick table](#quick-pick) below. The rest of
the document explains how we arrived at it (HIL-measured on NEMA17 42mm
unloaded, with `examples/hil_envelope`), and how to verify your own
setup.

## TL;DR

For a single motor doing position moves (the typical use case):

- **12 V/3 A** is fine: full performance up to ~2350 RPM steady, 42 ms
  per 90° move at `acc=255`.
- **24 V/10 A** is fine too, with one caveat: **you must explicitly set
  `work_current` to ≥ 2000 mA** (via `SET_WORK_CURRENT` / menu) to
  unlock the firmware's full 3000 RPM cap. With the factory default the
  motor caps around 1800 RPM at 24 V — counter-intuitive but real.

For multi-motor projects (≥ 3 motors on one supply), **prefer 24 V**:
the higher voltage halves the current needed per motor on the bus, and
gives you headroom for the ~115 W peak that six NEMA17s at full work
current draw together.

90° moves are firmware-acc-limited (~42 ms), so **voltage doesn't change
short-move timing**. The 24 V advantage shows up at:

- High steady RPM (>2000): 24 V tops out at 3022 RPM vs 2389 at 12 V
- Overshoot magnitude: ~2° at 24 V vs ~5° at 12 V (acc=255, peak)
- Timing determinism: σ ≈ 30 µs at 24 V vs ~300-600 µs at 12 V

## Quick-pick

| Your scenario | Supply | work_current |
|---|---|---|
| Single motor, 12 V wall-wart, hobbyist | 12 V / ≥ 3 A | 1600 mA (default) |
| Single motor, want full RPM range | 24 V / ≥ 3 A | **2000 mA** |
| Single motor, lab/precision (lowest overshoot) | 24 V / ≥ 3 A | 1600-2000 mA |
| 3-6 motors, 90° moves only (e.g. Rubik's robot) | 24 V / ≥ 10 A | 1600 mA |
| 3-6 motors, mixed (some high RPM) | 24 V / ≥ 10 A | **2000 mA** |
| Heavy load (significant torque required) | 24 V / sized to load | 2500-3000 mA + thermal monitoring |

## The non-obvious part: work_current at 24 V

This is the gotcha. The MKS SERVO42D's firmware default for
`work_current` is **1600 mA**. That setting works fine at 12 V — you
get the motor's full back-EMF-limited 2350 RPM. But at 24 V the same
1600 mA setting caps the motor at only ~1800 RPM, because the firmware
runs a constant-power chopper: 12 V × 1.6 A and 24 V × 0.8 A are both
~12-19 W of input power, and at the 24 V duty cycle the chopper
delivers less torque-producing current at high RPM (the duty cycle is
so low that the chopper PWM struggles to maintain target current
through the inductance during the small ON-time slices).

Raising `work_current` to 2000 mA unlocks the full firmware-spec
3000 RPM cap at 24 V. Going above 2000 mA gives no further benefit at
no load — you've hit the firmware cap, not the motor cap.

HIL-measured at 24 V (NEMA17 42mm unloaded, work_current set EXPLICITLY
via `set_work_current_ma()`):

| work_current | max steady RPM | tracking at cmd=3000 |
|---|---|---|
| 1000 mA | 1190 | 0.40 (saturated) |
| 1600 mA (manufacturer's nominal default) | **2385** | 0.80 (saturated) |
| 2000 mA | **3175** | **1.06** (reaches firmware cap) |
| 2500 mA | 3174 | 1.06 (no extra gain) |
| 3000 mA | (no extra gain) | (no extra gain) |

> ⚠️ Some boards ship with a factory setting BELOW the nominal 1600 mA
> default. We've observed first-boot behavior capping at ~1840 RPM
> instead of the 2385 above, which means the actual factory value was
> lower than spec on that unit. Always call `set_work_current_ma()`
> explicitly before benchmarking or running production code — don't
> trust the implicit factory value.

> ⚠️ Thermal note: at 2500-3000 mA the motor coils dissipate more I²R
> heat (a NEMA17 with 2 Ω/phase × 2 phases × 3 A² ≈ 36 W when both
> phases are at full current). Under continuous high-duty operation
> the motor can reach 70°C+. For short-duty applications (position
> moves with idle periods) this is not a problem; for continuous
> rotation, monitor temperature or stick to 2000 mA.

## The benchmark numbers (HIL-measured, NEMA17 42mm, unloaded)

All three setups, same motor, same library, same `examples/hil_envelope`:

| Metric | 12 V/3 A wall | 12 V/3 A lab | 24 V/10 A lab |
|---|---|---|---|
| Comm round-trip (p99) | 1044 µs | 1055 µs | 1035 µs |
| 90° time @ acc=255 rpm=2000 | 41.85 ± 0.31 ms | 42.95 ± 0.66 ms | 41.73 ± 0.03 ms |
| 90° @ acc=200 | 305 ms | 307 ms | 312 ms |
| 90° @ acc=128 | 412 ms | 415 ms | 420 ms |
| 90° @ acc=64 | 467 ms | 474 ms | 479 ms |
| 90° @ acc=16 | 500 ms | 503 ms | 506 ms |
| max steady RPM (factory implicit) | 2346 | 2391 | 1838 (board shipped < 1600 mA) |
| max steady RPM (EXPLICIT 1600 mA) | (same) | (same) | **2385** |
| max steady RPM (EXPLICIT 2000+ mA) | (same physics) | (same physics) | **3175** |
| overshoot peak @ acc=255 | +4.86° | +4.33° | +2.24° |
| soak 100× | 99/100 | 99/100 | 99/100 |
| settle drain mean | 9.96 ms | 11.46 ms | 13.74 ms |

## Three things to remember

1. **Short moves (≤ 360°) are firmware-acc-limited**, not voltage-limited.
   Switching voltage doesn't change timing on a 90° move. Optimise
   `acc` for these (the lib's recommended default is acc=255).

2. **Max RPM IS voltage-limited at the right work_current**. If you
   need sustained high speed, use 24 V AND set work_current ≥ 2000 mA.
   With default 1600 mA at 24 V you'll see less RPM than at 12 V —
   counter-intuitive but reproducible.

3. **For multi-motor power budget, 24 V is almost always the right
   call**. Two NEMA17s pulling 1.6 A each at 12 V is ~38 W of input
   power, requiring 3.2 A from the supply. Six motors need ~10 A at
   12 V — supplies that big are heavier/pricier than the equivalent
   24 V/5 A.

## How to set work_current

From C++ code (writes to flash, persistent across reboots):

```cpp
#include "mks_servo/raw_driver.hpp"

// ... after Transport + RawDriver setup ...
auto r = raw.set_work_current_ma(2000);  // mA, 0-3000
if (!r.ok() || !r.value) {
    // handle error (firmware rejected, transport failed)
}
// Setting is now in flash; takes effect immediately.
```

From the board menu: navigate to `Set → Ma` and set the value (each
unit on the dial = 200 mA on most firmware versions).

## How to verify your setup is configured correctly

Run `examples/hil_envelope` and check the **MAX RPM** section:

```
[3/6] MAX RPM (steady, plateau detection, acc=255)...
       cmd=2400:  real=2418.5  ratio=1.008
       cmd=2700:  real=2722.6  ratio=1.008
       cmd=3000:  real=3022.0  ratio=1.007
       >>> max steady RPM: 3022
```

If `ratio` is close to 1.0 at cmd=2400+, your work_current is high
enough for your voltage. If `ratio` drops below 0.95 at cmd=2100 or
below, you're hitting the chopper-power limit — bump work_current.

Alternatively, run `examples/hil_voltage_setup` (sweeps work_current
values and prints the recommended setting for your supply).
