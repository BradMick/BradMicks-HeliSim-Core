# Simple Rotor Redesign — Specification

**Branch:** `rotor-consolidation`
**Fallback:** tag `pre-rotor-consolidation` (commit `ab0b0ca`)
**Status:** proposed. No code written.

---

## What the simple rotor is for

A force generator. It produces a thrust vector and a torque, and it should be
authorable by someone with a flight manual and a spreadsheet — not by someone
willing to reverse-engineer fifteen interacting lookup tables.

Simple rotor is the one you tune by eye.

---

## What it produces

The entire model has exactly two physical outputs, plus two published values:

| output | consumer |
|---|---|
| `addForce` thrust vector | Arma |
| `addTorque` moment `[pitch, roll, yaw]` | Arma |
| `bmkhs_rtrThrust[i]` | debug overlay, tail VRS |
| `bmkhs_reqEngTorque[i]` | engine, transmission, drivetrain |

Sixty-odd hard-coded values currently compute those. That is the whole problem.

**The interface does not change.** `bmkhs_rtrThrust[]` and `bmkhs_reqEngTorque[]`
keep their shape and meaning, so `fn_engine2`,
`fn_transmissionUpdate` and the debug overlay need no edits.

---

## Thrust and torque stay decoupled

This is deliberate and it survives the redesign.

Today the two chains are fully independent — thrust never enters the power
calculation, and torque never scales thrust. Both read collective and airspeed,
neither reads the other. That is what lets collective feel and torque loading be
tuned separately, and merging them would destroy the knob.

So the tables come in two groups, and a designer reasons about them separately:

- **Thrust chain** — how much lift do I get
- **Power chain** — how much power does it cost

---

## Considered and rejected: calculating torque

Torque could be derived rather than tabulated. Blade count, chord and radius
give solidity `σ = Nb·c / πR`, and momentum theory then yields
`Cq = κ·Ct^1.5/√2 + σ·Cd0/8` — every input is already config or state. That
would delete both power tables, leave two physical coefficients in their place,
and make torque respond correctly to blade count for free.

**Rejected deliberately.** It recouples torque to thrust, destroying the knob
this model exists to provide, and momentum theory misses the high-speed power
rise so a correction table comes back anyway. Derivation is not what this model
is for.

Simple means simple: a designer types numbers off a chart and flies.

---

## The four tables

Every one is a curve that can be read off a flight manual or sketched by hand.
That is the test for whether a table belongs in this list.

### 1. `thrustVsAirspeed[]` — airspeed (kt) → multiplier

Translational lift. The ETL bucket and its recovery.

```cpp
thrustVsAirspeed[] = {
    {  0, 1.164},
    { 20, 1.059},
    { 40, 0.953},
    { 70, 0.848},
    { 90, 0.889},
    {100, 0.890},
    {120, 0.947},
    {130, 0.990},
    {140, 1.043}
};
```

**Replaces:** `_rtrThrustScalarTable`, `_velocityThrustExponentTable`, the
`(1 + V/VBE)^n` exponent form, and the IGE/OGE hover blend (which is currently
`1.0` blended to `1.0` — a no-op wearing a costume).

### 2. `thrustVsCollective[]` — control input (0–1, or −1..1 for a tail) → multiplier

Collective (or pedal) to thrust. Non-linear, and **asymmetric for a tail rotor**.

```cpp
// main rotor — currently a straight line, expressible as two points
thrustVsCollective[] = {
    {0.00, 0.032},
    {1.00, 1.000}
};

// tail rotor — the real curve, 2:1 asymmetric (left pedal authority
// is double right pedal, which is the airframe's pitch range)
thrustVsCollective[] = {
    {-1.00,  2.000}, {-0.90, 1.960}, {-0.80, 1.850}, {-0.70, 1.700},
    {-0.60,  1.500}, {-0.50, 1.250}, {-0.40, 0.960}, {-0.30, 0.680},
    {-0.20,  0.400}, {-0.10, 0.170}, { 0.00, 0.000}, { 0.10, -0.170},
    { 0.20, -0.320}, { 0.30, -0.460}, { 0.40, -0.590}, { 0.50, -0.700},
    { 0.60, -0.790}, { 0.70, -0.860}, { 0.80, -0.920}, { 0.90, -0.960},
    { 1.00, -1.000}
};
```

**Replaces:** `_bladePitchInducedThrustScalar` (main, linear),
`_bladePitchInducedThrustTable` (tail), `_rtrThrustScalarTable_min/_max`.

### 3. `powerVsAirspeed[]` — airspeed (kt) → fraction of `maxPower`

**The power-required curve. This is a chart out of the −10.** The power bucket:
high in the hover, minimum around best-endurance speed, rising again toward Vne.

```cpp
powerVsAirspeed[] = {
    {  0, 0.94}, {  5, 0.93}, { 10, 0.90}, { 15, 0.87},
    { 20, 0.82}, { 25, 0.78}, { 40, 0.62}, { 50, 0.54},
    { 60, 0.50}, { 70, 0.49}, { 80, 0.50}, { 90, 0.52},
    {100, 0.56}, {110, 0.64}, {120, 0.72}, {130, 0.85},
    {140, 1.01}, {150, 1.18}
};
```

**Replaces:** `_cruiseTqTable`, `_inducedPowerVelocityScalarTable`,
`_inducedPowerCollectiveCorrectionTable`, `_profile_min`/`_profile_max`.

A designer transcribes this. No tuning, no guessing.

### 4. `powerVsCollective[]` — collective (0–1) → multiplier

The deliberate torque-tuning knob. Shapes how power demand grows with
collective, independently of thrust.

```cpp
powerVsCollective[] = {
    {0.000, 0.000},
    {0.050, 0.760},
    {0.225, 0.798},
    {0.250, 1.000},
    {0.850, 1.000},
    {1.000, 1.500}
};
```

**Replaces:** `_collectiveTorqueCorrectionTable`, `_tqRoCTable`.

---

## The scalars

```cpp
class Rotors {
    class MainRotor {
        // ---- identity ----
        rotorType        = "MAIN";        // MAIN | TAIL
        thrustAxis       = "Z";           // Z up (main), X lateral (tail)
        position[]       = {0.0, 0.5, 1.8};
        direction        = "CCW";         // CCW | CW - sets torque reaction sign

        // ---- geometry (real AH-64 values) ----
        bladeRadius      = 7.315;         // m - disc area, tip speed, ground effect
        rotorInertia     = 5152;          // kg.m^2 about the mast - see below
        bladePitchMin    = 1.0;           // deg
        bladePitchMax    = 19.0;          // deg
        designRpm        = 289.0;
        gearRatio        = 72.291;        // shared with the transmission model
        rpmTrimVal       = 1.01;
        heightAgl        = 3.606;         // m, hub height on the wheels

        // ---- force scale ----
        baseThrust       = 102306;        // N, max gross weight * g
        maxPower         = 2133;          // kW at power fraction 1.0
        refRpm           = 20900;         // transmission output RPM at 100% Nr

        // ---- tuning scalars ----
        groundEffectGain = 0.31;          // thrust gain at ground; falls off over 1 diameter
        climbGain        = 0.73;          // excess-torque -> climb thrust
        autoroTorque     = 5.0;           // driving torque per m/s of descent

        // ---- control authority ----
        // These three ALREADY EXIST in the AH-64 pack, declared in Nm with the
        // comment "Control torques the simple rotor model applies" - and Core
        // reads none of them. It uses hardcoded 2.50*1.3 and 0.75*1.3 against a
        // 100000 base instead. Wiring these up is part of the redesign, not a
        // new surface: the pack already declares what it wants.
        cyclicPitchTorque = 4500.0;       // Nm
        cyclicRollTorque  = 1500.0;       // Nm
        pedalYawTorque    = 5000.0;       // Nm

        rollCouple       = 0.0;           // fraction of thrust moment coupled into roll
        thrustTiltRoll   = -6.0;          // deg of thrust-vector tilt per unit roll input
        flapbackLon      = 0.0;           // disc tilt vs advance ratio (longitudinal)
        flapbackLat      = 0.0;           // (lateral)

        // ---- envelope ----
        vne              = 250;           // kt - clamp ceiling
        vbe              = 75;            // kt - best endurance, the power bucket minimum
        etl              = 24;            // kt - effective translational lift onset

        // ---- the four tables ----
        thrustVsAirspeed[]   = { ... };
        thrustVsCollective[] = { ... };
        powerVsAirspeed[]    = { ... };
        powerVsCollective[]  = { ... };
    };

    class TailRotor {
        rotorType        = "TAIL";
        thrustAxis       = "X";
        position[]       = {-0.87, -6.98, -0.075};
        numBlades        = 4;
        bladeRadius      = 1.402;         // m
        bladeChord       = 0.253;         // m
        designRpm        = 1403.0;
        gearRatio        = 14.90;
        rpmTrimVal       = 1.01;
        baseThrust       = 10230;         // N
        torqueScalar     = 0.008;         // Nm of engine torque per N of tail thrust (~8% of total power)
        rollCouple       = 0.25;          // tail thrust acts above the roll axis
        // no ground effect, no cyclic, no climb term
    };
};
```

**Units are declared and consistent: airspeed in knots, power in kW, thrust in
newtons, angles in degrees.** Core converts once at init. Today the tables are
in m/s, which nobody reads a flight manual in.

---

## The tail rotor draws power

Today it does not. `fn_simpleRotorTail.sqf` never writes `bmkhs_reqEngTorque`,
so the tail rotor is free — no engine load, no Nr droop on a pedal input. A real
tail rotor is 5–15% of total power and spikes hard when you stomp a pedal.

The fix is one scalar, because the plumbing already exists:
`bmkhs_reqEngTorque` is indexed per rotor and `fn_transmissionUpdate` sums the
whole array. Index 1 is simply never written.

```
tailTorque = abs(tailThrust) × torqueScalar
```

Scaling off the tail's **own thrust** rather than off main rotor torque means it
responds to pedal for free — and because `thrustVsCollective[]` is 2:1
asymmetric on a tail, left pedal costs twice the torque of right pedal without
anybody authoring that. One number, correct behaviour.

---

## Rotor inertia: one number, not four

The simple rotor currently declares `numBlades`, `bladeChord`, `bladeMass` and
`bladeHingeOffset` for exactly one purpose — computing `_Jtot`, which becomes
`bmkhs_rtrMoi`, whose only consumer is `fn_transmissionUpdate` for `deltaRpm`
(spin-up, coastdown, Nr droop).

Run the AH-64's real numbers through that formula and the breakdown is:

| term | formula | value | share |
|---|---|---|---|
| `Icm` | ⅓·m·r² | 1286.1 | **99.86%** |
| `Iy` | ¹⁄₁₂·m·c² | 1.707 | 0.13% |
| `md2` | m·e² | 0.104 | 0.01% |
| **`J`** | `(Iy + Icm + md2) × Nb` | **5151.8** | |

Chord and hinge offset contribute 0.14% between them. They are noise.

There is also a latent unit bug: the config comments `mainRtrBladeHingeOff =
0.038` as "fraction of blade radius" while the code squares it as metres.
Correcting it moves `J` by 0.4% — which is why nobody has noticed.

**So the simple rotor declares `rotorInertia` directly.** Four fields become
one, the unit-bug surface disappears, and the designer gets a knob that maps
straight onto what they actually tune: how the rotor spools and droops.

`bladeRadius` stays — disc area (`πr²`) drives induced velocity and the whole
VRS band, plus tip speed, ground-effect falloff and the debug disc.

**The spreadsheet computes it.** Enter blade mass, radius and blade count and
it emits `rotorInertia`, so anyone working from blade data still gets there —
the arithmetic just happens once, in a cell, instead of every frame in SQF.

---

## What Core computes and config never sees

These are physics or model behaviour. They stay in Core:

| value | formula | why |
|---|---|---|
| induced velocity | `sqrt(T / 2ρA)` | momentum theory |
| VRS band | `inducedVelocity × {0.23, 1.25}` | VRS theory, scales with the above |
| ground effect shape | `1 − h/D` | dimensionless, universal |
| disc area | `πr²` | geometry |
| moment of inertia | `⅓mr² + ¹⁄₁₂mc² + md²` | geometry |
| tip speed | `ωr` | geometry |
| air density ratio | `ρ / ρ_ISA` | atmosphere |

**Vortex ring state is fully derived.** Induced velocity comes from this frame's
thrust, air density and disc area — all config or state — so the VRS band scales
with the airframe automatically. There is nothing to declare.

This also **fixes the tail rotor**, which today uses a fixed `VEL_VRS` macro
(24.384 m/s) sized for one airframe instead of deriving its own. Unifying the
two rotors gives the tail momentum theory for free, and drops `VEL_VRS` from
the config surface entirely.

---

## One function, N rotors

`fn_simpleRotorMain.sqf` + `fn_simpleRotorTail.sqf` → **`fn_simpleRotor.sqf`**,
looped over a rotor index.

A tail rotor is not a different model. It is the same force generator with:

- `thrustAxis = "X"` instead of `"Z"`
- no ground effect
- no cyclic moments
- no climb-thrust term
- pedal instead of collective on the same `thrustVsCollective[]` curve

Declaring a NOTAR, a coaxial or a tandem becomes a config change rather than a
new SQF file.

---

## Defaults: neutral, not AH-64

Core ships **neutral** defaults — flat `1.0` curves, `groundEffectGain = 0.0`.

An aircraft that declares nothing flies badly but honestly. It does not silently
inherit Apache handling. "Flies like a brick" is diagnosable; "flies
suspiciously well" is not.

The AH-64's real numbers live in the AH-64 pack, where they belong. Core knows
no aircraft — including the one it was born from.

---

## The spreadsheet

`tools/simple_rotor_tables.xlsx`, shipped with the mod.

Enter gross weight, rotor radius, blade count, blade mass, engine power, Vne and
the hover/cruise power figures from the flight manual. It emits:

- the four tables as paste-ready `.hpp` text
- **`rotorInertia`**, computed as `⅓ · bladeMass · r² · numBlades` — so anyone
  working from blade data still gets there without Core carrying the arithmetic
- **`baseThrust`**, as max gross weight × g
- each curve plotted, so the power bucket's shape is visible before flying it
- sanity warnings (bucket minimum far from Vbe, non-monotonic thrust curve)

Three sheets: **Inputs** (airframe data in, derived scalars out), **Thrust** and
**Power** — the last two matching the two chains. Google Sheets opens `.xlsx`
natively, so one file serves both.

---

## Bugs found, fixed separately

These are pre-existing and get their own commits. Folding them into the
extraction would make any regression unattributable.

| bug | where |
|---|---|
| retreating-blade-stall computed, never used | `fn_simpleRotorMain.sqf:386-387` |
| `bmkhs_rtrTqTable`, `bmkhs_mainThrustTable` published, no consumers | `fn_simpleRotorMain.sqf:127,143` |
| longitudinal flapback computed, discarded (pitch arg hardcoded `0.0`) | `fn_simpleRotorMain.sqf:437-443` |

---

## The cost, stated plainly

**This changes how the AH-64 flies.** Collapsing six overlapping curves into
four cannot reproduce their interaction exactly. The AH-64 needs a retune
against the new surface — flight time, not code time.

`docs/CONFIG_PLAN.md` says extraction must be lossless and never combined with
retuning. **This redesign deliberately breaks that rule**, because the goal is
not to preserve the current surface but to replace it. The tag
`pre-rotor-consolidation` is what makes that safe: the old behaviour is one
checkout away.

The upside: four legible curves instead of fifteen interacting ones, and the
retune is bounded — within each chain the merged tables were genuinely
redundant (three thrust-vs-airspeed curves multiplying together, two
induced-power curves doing one job).

---

## Sequence

1. This spec, agreed
2. `simpleRotor.hpp` field reference beside the code, in `components.hpp` style
3. `fn_simpleRotorVariables.sqf` reads the new surface, neutral defaults
4. `fn_simpleRotor.sqf` — unified, N-rotor
5. AH-64 pack config written against the new surface
6. Fly it, tune it
7. Delete `fn_simpleRotorMain.sqf` / `fn_simpleRotorTail.sqf`
8. Spreadsheet
9. `AIRCRAFT_GUIDE.md` rotor section; correct `CONFIG_PLAN.md`, which currently
   claims the simple rotor already reads from config - it does not

Bugs above land as separate commits, before or after, never inside step 4.

---

## Open questions

1. ~~**Table x-axis in knots**~~ — **settled.** Every breakpoint in the current
   code converts to a round knot value (10.29 = 20 kt, 36.01 = 70 kt, 77.17 =
   150 kt, and so on). The tables were authored in knots and converted to m/s
   once; the m/s literals are the artifact, not the source. Knots it is.
2. ~~**`rollCouple` default**~~ — **settled.** Core defaults `0.0` (neutral);
   the AH-64 pack declares `0.25` on the tail, preserving the value committed
   as `a3021a9`. Revisit once it has had air time.
3. ~~**Does the AH-64 pack live in this repo?**~~ — **settled.** It lives at
   `E:\AH-64D`, branch `BMKHS-HeliSim-Core-Refactor`, tagged
   `pre-rotor-consolidation` to match this repo. Config goes in
   `addons/fza_ah64_helisim/config/bmkhs_config/helisim_simpleRotor.hpp`,
   already included by `bmkhs_ah64_config.hpp:26`. Built with `scons` (wraps
   HEMTT). The two repos move together: a Core rollback without the matching
   config leaves the pack declaring a surface Core no longer reads.
4. **`thrustTiltRoll = -6.0`** is a hardcoded control-power gain sitting next to
   the config-driven flapback gains. Included as a scalar above; worth asking
   whether it should instead fold into `flapbackLat`. Note the AH-64 pack
   already sets `mainRtrFlapbackLat = 10.0` with a comment saying its sign was
   never verified in the air - so two knobs currently fight over roll response.
   Kept separate so one can be verified at a time.
