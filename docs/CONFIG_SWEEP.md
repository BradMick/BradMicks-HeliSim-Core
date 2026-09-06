# HeliSim Core — Full Constant Sweep

Every numeric literal in Core's SQF, classified. Trivial structural values
(0, 1, 0.5, 100, 360) excluded. **129 hits across 24 files.**

Verdict key:
- **CONFIG** — per-aircraft, externalised to the aircraft config
- **CORE** — model tuning or maths, stays fixed in Core
- **MOD** — belongs to the aircraft mod, not Core at all
- **CHECK** — needs a decision

---

## Simple rotor — 34 values, the largest gap

`fn_simpleRotorMain.sqf`

| Value | Current | Verdict |
|---|---|---|
| `_rtrHeightAGL` | 3.606 m | CONFIG |
| `_rtrDesignRPM` | 289.0 | CONFIG |
| `_rtrRPMTrimVal` | 1.01 | CONFIG |
| `_rtrGearRatio` | 72.291 | CONFIG |
| `_bladeRadius` | 7.315 m | CONFIG |
| `_bladeChord` | 0.533 m | CONFIG |
| `_bladeMass` | 72.108 kg | CONFIG |
| `_bladeHingeOffset` | 0.038 | CONFIG |
| `_bladePitch_max` | 19.0 deg | CONFIG |
| `_baseThrust` | 102306 N | CONFIG |
| `_vrsScalarExponent` | 0.3 | CORE (VRS model shape) |
| `_profile_min` / `_profile_max` | 0.180 / 0.407 | CONFIG |
| `_kFlapLat` | 10.0 | CHECK — flapback gain |
| ETL band | 8.23 / 12.35 m/s | CONFIG |
| High-speed shake bands | 66.87 / 72.02 / 77.16 / 82.30 m/s | CONFIG |
| `_velXY` hover gate | 2.6 | CORE (hover/forward-flight boundary in the thrust model) |
| `_mainRtrDamage` | 0.99 | CORE |

`fn_simpleRotorTail.sqf`

| Value | Current | Verdict |
|---|---|---|
| `_rtrDesignRPM` | 1403.0 | CONFIG |
| `_rtrRPMTrimVal` | 1.01 | CONFIG |
| `_rtrGearRatio` | 14.90 | CONFIG |
| `_bladeRadius` | 1.402 m | CONFIG |
| `_bladeChord` | 0.253 m | CONFIG |
| `_baseThrust` | 10230 N | CONFIG |
| `_rtrAirspeedVelocityMod` | 0.4 | CORE |
| `_tailRtrDamage` | 0.85 | CORE |

Plus the tuning scalars the earlier survey found (`_pitchTorqueScalar 2.50*1.3`,
`_rollTorqueScalar 0.75*1.3`, `_inducedVelocityScalar`) which this pattern misses
because they are products, not plain literals.

---

## Systems / drivetrain — 40 values

Per gearbox (NGB1 and NGB2 identical):

| Value | Current | Verdict |
|---|---|---|
| Continuous torque limit | 1.10 | CONFIG |
| Transient limits | 1.22, 1.25 | CONFIG |
| Continuous timer | 150 s | CONFIG |
| Transient timer | 6 s | CONFIG |
| Damage tiers | 0.25 / 0.50 / 0.75 | CORE |
| Damage divisors | /600, /500, /400, /10, /20, /40 | CORE |

Transmission:

| Value | Current | Verdict |
|---|---|---|
| Continuous limit | 2.00 | CONFIG |
| Transient limit | 2.30 | CONFIG |
| Transient timer | 6 s | CONFIG |
| Torque jitter | ±0.10 | CORE |

Electrical: battery power floor `0.25` — CONFIG.

**Note:** `fn_drivetrainNoseGearbox1/2.sqf:58` compares `_engPctTQ > 125`, where
every other comparison in the file is a fraction (1.10, 1.22, 1.25). Looks like a
missing decimal point — worth checking against intent before externalising.

---

## Engine — 21 values

| Value | Current | Verdict |
|---|---|---|
| `_continuosPower` | 1066.0 | CONFIG |
| `_contingencyPower` | 1447.0 | CONFIG |
| `_designRpm` | 20900 | CONFIG |
| `_npIdleRef` / `_npFlyRef` | 0.57 / 1.01 | CONFIG |
| `_ngIdleRef` / `_ngFlyRef` | 0.674 / 0.856 | CONFIG |
| NP overspeed | 1.196 | CONFIG |
| TGT limits | 867 / 896 | CONFIG |
| NG start threshold | 0.52 | CONFIG |
| `_govGain` | 6.0 | CONFIG |
| `_engFriction` | 0.0 | CONFIG |

The UH-60 config has direct equivalents for most of these
(`engDesignRpm`, `engNpTrimRef[]`, `engNgRef[]`, `engTgtRef[]`).

---

## Mass — 11 values

| Value | Current | Verdict |
|---|---|---|
| `_fs0` datum | 6.4 | CONFIG |
| `_fwdCg` / `_aftCg` | 1.117 / 0.964 | CONFIG |
| Crew mass | 113.4 kg x2 | CONFIG |
| 30mm round mass | 0.35 kg | MOD |
| M299 launcher | 64.9 kg | MOD |
| AGM-114 | 46.71 kg | MOD |
| M261 launcher | 39.4 kg | MOD |
| Hydra | 10.4 kg | MOD |
| Aux tank empty | 63.5 kg | MOD |

Station arms are already config. Weapons and launchers are the mod's business -
Core should take a station mass, not know what an AGM-114 weighs. Crew mass, the
CG references and the fuselage datum are Core's.

## FMC — 10 values

| Value | Current | Verdict |
|---|---|---|
| Alt-hold torque cutout | 0.98 | CONFIG |
| Alt-hold engage AGL | 50 | CONFIG |
| Climb rate gate | ±1.016 m/s | CONFIG |
| Bob-up ceiling / speed | 435.254 / 20.577 | CONFIG |
| Bank angle gates | 3.0 / 7.0 deg | CONFIG |
| `_posIkp` / `_posIclamp` | 0.0050 / 0.0200 | CONFIG — PID terms |

Plus the **15 PID gain sets** in `fn_coreConfig.sqf`, all CONFIG.

---

## Smaller groups

| Group | Values | Notes |
|---|---|---|
| Root (`fn_getInput`) | 10 | Keyboard input deadbands ±0.1 — CORE |
| Damage | 3 | NR/TQ thresholds 0.50/0.30/0.9 — CORE |
| Environment | 1 | Altimeter 29.92 inHg — CORE (ISA constant) |
| Transmission | 1 | Gear ratio 72.29 — CONFIG, duplicates the rotor value |
| Rotor (BET) | 1 | `_cgR` 5.0 — CORE (debug sphere radius, not a model value) |
| Fuel | 1 | `_eps` 0.0001 — CORE (epsilon) |
| PrestonAI | 2 | Yaw breakout +-0.01 — CORE |
| Core/util | 4 | Frame guards, UI epsilon — CORE |

---

## Totals

| Verdict | Count |
|---|---|
| CONFIG | ~76 |
| CORE | ~44 |
| MOD | 6 |
| CHECK | 1 |

One CHECK item left: `_kFlapLat` (flapback gain, 10.0) in the simple rotor.
Decide it in context when the rotor is externalised.

## Notable findings

**`_rtrGearRatio` is duplicated.** 72.291 in `fn_simpleRotorMain.sqf`, 72.29 in
`fn_transmission.sqf` — same number, two places, slightly different precision.
Should be one config value.

**Store masses move to the mod.** M299, AGM-114, M261, Hydra, aux tank and the
30mm round mass are AH-64 ordnance sitting in `fn_massUpdateStation.sqf` and
`fn_massUpdateMagazine.sqf`. Core should be handed a station mass rather than
knowing what an AGM-114 weighs. Needs an API: the mod computes its station
masses and feeds them in.

**The `> 125` comparison** in both nose gearbox files, noted above.

**Products are invisible to this sweep.** `2.50 * 1.3` and similar are missed by
a plain-literal pattern. The rotor tuning scalars need reading rather than
grepping.
