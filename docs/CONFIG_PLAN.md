# HeliSim Config Externalisation Plan

**Branch:** `BMKHS-HeliSim-Core-Refactor`
**Goal:** Every value that defines how an aircraft flies lives in its config, not in Core's SQF or headers. Core ships fixed; the aircraft supplies the numbers.

---

## Why

Core is standalone but not yet *authorable*. A second aircraft cannot be built today because the values that define its behaviour are hardcoded in Core:

| Home | Count | Examples |
|---|---|---|
| `headers/core.hpp` macros | 94 | rotor limits, VRS thresholds, auto-pedal constants |
| `headers/systems.hpp` macros | 27 | damage thresholds, hydraulic pressures, timers |
| `headers/fuelConstants.hpp` macros | 14 | low-level thresholds, XFER rate, AUTO triggers |
| PIDs in `fn_coreConfig.sqf` | 15 | every flight control loop |
| Inline SQF constants | ~150 | rotor geometry, mass arms, FMC gains |

The PIDs are the clearest case: one set of gains cannot serve a 9000 kg AH-64 and a 5900 kg UH-60, let alone a coaxial or tandem. Same for rotor geometry — blade radius, chord, mass and hinge offset are the aircraft, not the model.

---

## Where it stands

Most of this is done. The flight control PIDs, mass, engine, fuel, fuselage,
wings, airfoils and the simple rotor all read from config today, and the
systems half went further than externalising - electrical, hydraulics, the APU
and the drivetrain became declared components, recorded in
`SYSTEMS_REDESIGN.md`.

What is still hardcoded in Core:

| subsystem | what is left |
|---|---|
| BET rotor | blade geometry and tuning, in `fn_rotorVariables` |
| Preston AI | auto-pedal gains and deadbands, no config reads at all |
| environment | atmosphere constants; no variables function exists |

Until those move, a second airframe can be declared but not flown differently -
its rotor is the AH-64's.

---

## Config file split

Along system/component lines, `#include`d from `bmkhs_ah64_config.hpp`.

| File | Contents | Source today |
|---|---|---|
| `bmkhs_ah64_config.hpp` | Identity, empty mass/CG, subsystem gates; includes the rest | config |
| `helisim_flightControls.hpp` | Authority, blade pitch ranges, **13 flight PIDs** | config + `fn_coreConfig` |
| `helisim_rotor.hpp` | BET rotor geometry and tuning | **still hardcoded SQF** |
| `helisim_simpleRotor.hpp` | Simple rotor tables and scalars | done |
| `helisim_engine.hpp` | Engine constants, `engPerfTable0-4[]`, governor PID | done |
| `helisim_fuselage.hpp` | Top/Side/Front panels, drag tables | config |
| `helisim_wings.hpp` | Wing geometry, control surfaces | config |
| `helisim_stabilator.hpp` | Stabilator geometry + schedule table | folded into `helisim_flightControls.hpp` |
| `helisim_airfoils.hpp` | `airfoilTable00-02` | config |
| `helisim_mass.hpp` | Station arms, crew mass | done |
| `helisim_fuel.hpp` | Tank masses, capacities, thresholds, XFER rates | config + `fuelConstants.hpp` |
| `helisim_components.hpp` | APU, electrical, hydraulics and drivetrain, as declared components | done — see `SYSTEMS_REDESIGN.md` |
| `helisim_damage.hpp` | Damage thresholds, `DMG_PER_SEC` | stays in Core - thresholds are the model, not the airframe |
| `helisim_environment.hpp` | Atmosphere constants | **still hardcoded SQF** |
| `helisim_prestonAi.hpp` | Auto-pedal gains, deadbands | **still hardcoded SQF** |
| `helisim_misc.hpp` | PERF page: `perfTable`, `hoverTable`, `cruiseTable` | config |
| `hitPointValues.hpp` | Already split | done |

---

## Two decisions already reached

**perfTable split.** `perfTable0-4[]` mixes engine capability (MC TQ, MTA DE, MTA SE) with PERF-page output (max gross weights, go/no-go ratios). Only `fn_perfData.sqf` reads it, and only for the page. The UH-60 config already separates these — its `engPerfTable0-4[]` carries just PA + the three torque columns under Engine Data, with no gross-weight columns at all.

So: engine columns duplicated into `helisim_engine.hpp` as `engPerfTable#[]`, aircraft-performance columns stay in `helisim_misc.hpp`. Duplicate rather than cross-reference — it is 5×7×3 shared numbers against Core resolving a reference across config sections at load.

**Damage thresholds.** 15 `SYS_*_DMG_THRESH` macros all set to 0.85. That is one value, not fifteen knobs. Recommend a single `damageThreshold` in `helisim_damage.hpp` unless per-system tuning is actually wanted.

---

## Risk

**The extraction must be provably lossless.** Every value moved is one the AH-64 currently flies on. The discipline used for the hitpoints applies:

1. Extract the exact literals — no rounding, no "tidying"
2. Verify the built config matches the pre-move values
3. Fly it before moving on
4. Never combine extraction with retuning

The PIDs and rotor constants are the highest-risk items — they are tuned values arrived at over months. Once extracted and verified identical, per-aircraft tuning becomes a feature. Retuning during extraction makes any regression impossible to attribute.

**Ordering question.** This plan splits the *current* flat config. The `bmkhs_config` rewrite (indexed arrays, `numRotors`/`rotorBladeRadius[]`, `aircraftType`) is a different job. Doing the mechanical split first means touching every file twice; doing the rewrite first means a bigger single step. Worth settling before starting.

---

## Suggested sequence

**1. Mechanical split first.** Move existing config into the new files unchanged. Zero risk, immediate readability gain, and it establishes the include structure.

**2. Then externalise by subsystem, easiest first:**
   - `damage` — macro constants, no flight-model risk. Electrical, hydraulics
     and the APU went further than externalising and became declared
     components; see `SYSTEMS_REDESIGN.md`
   - `environment`, `mass`, `fuel` — inline constants, low risk
   - `prestonAi`, `flightControls` PIDs — tuned values, verify handling
   - `rotor`, `simpleRotor` — highest risk, own test cycle

**3. Config validation last.** Once the surface is complete, add defaults and a startup pass reporting missing or invalid entries by name. Without it a modder's typo reads as a mysterious handling bug.
