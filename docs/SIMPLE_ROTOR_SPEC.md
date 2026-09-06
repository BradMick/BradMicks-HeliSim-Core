# Simple rotor - what we are building towards

The code is back at `pre-rotor-consolidation`. This is the target, and what was
learned attempting it, so the next attempt does not rediscover it.

---

## The goal

**One rotor function, called like every other force generator in the mod.**

`fn_wingUpdate` reads parallel arrays and loops, calling `fn_wing` per surface.
The BET rotor does the same. The simple rotor should too:

```
fn_simpleRotorUpdate   reads numRotors and the per-rotor arrays, loops
fn_simpleRotor         one rotor, parameters passed in
```

`fn_coreUpdateFlightModel` calls `simpleRotorUpdate` once. No main/tail split.

**A main rotor and a tail rotor are the same thing.** The only difference is
which control input drives them. Everything else - ground effect, cyclic, the
power draw - is data: a rotor with `groundEffectGain = 0` has no ground effect,
without needing a flag or a branch.

**Everything works in the rotor's own frame.** `rotation[]` next to
`position[]`, both live-tunable. Resolve the velocity into that frame once;
the forces then come out in it and one transform puts them in model space.
No per-force rotation, no hardcoded `[1,0,0]`, no `set [1, ...]` on a moment.

**Airspeed in knots** in config, converted once at init.

**Core keeps what is Core's.** `VEL_*` and the damage thresholds are model
constants and belong in `core.hpp`/`systems.hpp`, not in the aircraft pack.
The pack declares airframe data only.

---

## Target table count

Four per rotor:

| table | axis |
|---|---|
| `thrustVsCollective` | control input (x altitude) |
| `thrustVsAirspeed` | airspeed |
| `powerVsCollective` | collective |
| `powerVsAirspeed` | airspeed |

Plus support tables - ground effect, tip loss, autorotation, climb - which
still need reducing and were never specced.

---

## What was proven, and what was not

These were established by computation against the original and are worth
keeping:

**A 2-D interpolator built on `mathLinearInterp`.** Both passes call the
existing function, so clamping matches every other table. `fn_perfData`
already does this by hand and could use it.

**`thrustVsCollective` collapses to a 2-D surface.** The blade-pitch ramp, its
altitude-indexed floor, and the per-Nr altitude table are one surface in
collective x pressure altitude. Worst error 123 N on 102306.

  - blade pitch min/max become derivation INPUTS, not config fields. The
    spreadsheet needs them; the model does not.

**The two induced-power speed curves merge into one two-column table.**
`mathLinearInterp` already returns multiple columns. Exact.

**The power chain does NOT collapse to a 2-D surface.** Collective appears
twice, the second time divided by a speed-dependent term, and there is a clamp
whose position moves with airspeed. A bilinear grid plateaus at ~134 kW of
2133 regardless of density, because the kink cuts diagonally across cells.
Keep the two speed curves as a table and do the arithmetic in the model.

**`thrustVsAirspeed` does NOT collapse with the velocity exponent.** The
airspeed table is applied TWICE - beside the exponent, and again on the final
thrust, except below 5 kt where the IGE/OGE hover blend substitutes for it.
Fold the exponent in and it squares on the second use. Leave it out and
nothing is gained. Three variants were tried; each broke a different regime.

  - **Before collapsing anything, count how many times the value is used.**
    That check would have saved an hour.

**The tail rotor's two airspeed terms DO collapse.** Flat authority table
times `(1 + V/Vbe)^0.4`, one curve, worst 0.166%. The tail's table is used
once, which is why it works there and not on the main rotor.

---

## How to do it

One rotor, one change, flown before the next. Never both rotors at once, never
a collapse in the same commit as a move, never a reorganisation folded into
either.

The order that worked:

1. literals to config, math untouched
2. collapse what genuinely collapses
3. rotor frame
4. then, with both rotors ported, the combined `simpleRotorUpdate`

Verify by computing the old and new chains against each other across the
envelope - not at one condition, and not only the term being changed.
