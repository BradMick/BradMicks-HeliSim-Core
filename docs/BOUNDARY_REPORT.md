# SFM+ → BMKHS (BradMick's HeliSim)

**Date:** 2026-08-29
**Branch:** `BMKHS-HeliSim-Core-Refactor` (created off `HeliSimTuner`)
**Module:** BradMick's HeliSim — prefix `bmkhs`
**Goal:** Retire SFM+ and replace it with **BMKHS Core** (`bmkhs_helisim`) — a standalone, airframe-agnostic flight model PBO carrying everything it needs. Aircraft are then built by shipping a companion `XXX_helisim` pack holding that airframe's config, model bindings, and entry point.

**Rename is scoped to this module only.** `addons/fza_ah64_sfmplus` → `addons/bmkhs_helisim`, and its `fza_sfmplus_*` symbols → `bmkhs_*`. The rest of the repo keeps `fza_ah64_` — no build-system change (`.hemtt/project.toml` `prefix = "fza_ah64"` stays), no stringtable sweep, no renaming the other 16 addons.

> **Status note.** The config migration described here is done, and so is the
> systems redesign it turned out to need - electrical, hydraulic, drivetrain and
> the APU are now declared components rather than functions Core wrote for the
> AH-64. That work is recorded in `SYSTEMS_REDESIGN.md`, which supersedes the
> subsystem-gating detail in §0.6 and §3.3 below: gating survives as a concept,
> but a subsystem is absent because no component declares it, not because a flag
> switched it off. Controls are the remaining piece, and are specified but not
> built.

**End state — two HeliSim modules in the mod:**

| PBO | Role | Lifetime |
|---|---|---|
| `bmkhs_helisim` | **Core** — the flight model engine, airframe-agnostic | Ships standalone; reusable by any aircraft |
| `fza_ah64_helisim` | **Pack** — AH-64 entry point: config class, model bindings, opt-in | Permanent; the AH-64's HeliSim adapter |

Both exist during and after the refactor. `fza_ah64_helisim` is not scaffolding — it is the reference implementation of the pack contract (§0.7) and stays once the work is complete.

**Branching.** All of this work happens on `BMKHS-HeliSim-Core-Refactor`, branched from `HeliSimTuner` (18 commits ahead of `master`). It is deliberately isolated: the refactor renames 1453 symbols, moves ~2142 lines between addons and splits one PBO into two, none of which should collide with flight-model tuning. See §10 for the branch strategy.

**Governing principles:**

1. *Anything that modifies mass or forces belongs in the core of SFM+.* SFM+ **is** the FM; anything able to affect its physics must be central to it, not an integration point (§0).
2. *SFM+ is a framework, not just an addon.* A third-party modder must be able to author a complete aircraft — engines, rotors, wings, fuselage — **through config alone**, without editing flight-model source, and read FM state for their displays through a documented output API (§0.5).
3. *BMKHS Core is one standalone PBO with zero mod dependencies.* Everything the flight model needs ships inside it — no `requiredAddons` beyond vanilla + CBA. Capability tiers (drivetrain, hydraulics, electrical) are **config-gated subsystems inside Core**, not separate addons; switched off, vanilla behaviour applies (§0.6).
4. *Aircraft are packs, not forks.* A designer ships `XXX_helisim` containing config, model bindings and entry point. Core never names an airframe; the pack never contains flight-model code (§0.7).

Together these define the deliverable: **Core is the engine, the pack is the aircraft — config in, outputs out, source untouched.**

---

## 0. The physics-ownership rule

Mass and CG are not data the flight model consumes — they are *state the flight model owns*. `fn_massUpdate.sqf` computes gross weight, longitudinal CG and lateral CG every frame and pushes them to the engine via `setMass`/`setCenterOfMass`. Every term in that sum is a physics input.

This rule sorts the boundary cleanly:

| Category | Test | Disposition |
|---|---|---|
| **Core** | Can it change mass, CG, or forces? | **Inside SFM+.** Not optional, not injectable. |
| **Instrument** | Does it only *display* FM state? | Consumer of the output API. |
| **Peripheral** | Neither? | Fully independent. |

Applied, this moves three things I would otherwise have left outside:

1. **Fuel burn and transfer are core.** Fuel is the largest time-varying mass term, and transfer between fwd/aft cells moves longitudinal CG in flight. `fza_ah64_fuel` (609 lines) is not an optional integration — its mass-bearing logic is flight-model code that happens to live in another addon.
2. **Stores and ordnance are core.** `fnc_massUpdateStation` and `fnc_massUpdateMagazine` already live in SFM+ and are called from `fn_massUpdate.sqf:77-96`. But the *mutators* — jettison, magazine swap, loadout apply — sit outside and call in.
3. **The external mass mutators must become API calls, not writes.** Three external entry points can move mass today (§3.4).

### Current mass model — every physics input

From `fn_massUpdate.sqf`:

| Term | Source | Line |
|---|---|---|
| Empty mass/moment (FCR vs non-FCR) | Config class + `animationPhase "fcr_enable"` | 45-50 |
| Crew (CPG + PLT, 113.4 kg each) | Hardcoded | 56-59 |
| Fwd/ctr/aft fuel cells | **Written by `fza_ah64_fuel`** | 62-64 |
| Stations 1-4 fuel (aux tanks) | **Written by `fza_ah64_fuel`** | 65-68 |
| 30mm ammo (0.35 kg/round) | `fnc_massUpdateMagazine` — **is** called at :77 | 77-81 |
| Stations 1-4 stores | `fnc_massUpdateStation` ×4 | 84-98 |
| → `setCenterOfMass` (lat, long) | Computed | 118-122 |
| → `setMass` | Computed | 137 |

Mass is modelled in the lateral and longitudinal axes only. Arm constants are 2-element `[lat, long]` (`fn_massUpdate.sqf:30-35`) and vertical CG is deliberately pinned to the model datum — **by design, not an omission.** Treat the Z term at `fn_massUpdate.sqf:121` as intentional and leave it alone.

One incidental note: crew mass is hardcoded at 2×113.4 kg regardless of occupancy (`(count (fullcrew _heli))` commented out at `:59`). Pre-existing, independent of encapsulation, and listed only for completeness.

---

## 0.5 Framework requirement — config as the authoring surface

The module must let a modder build an aircraft declaratively. Two halves: **inputs via config**, **outputs via a documented API**.

### Where the config surface stands today

The good news: the pattern already exists and is well shaped in places. `class Fza_SfmPlus` authors fuselage panels as quad geometry with per-face rotation and drag-coefficient tables, and carries full airfoil tables (`airfoilTable01` NACA 0012 at :459, `airfoilTable02` NACA 4418 at :481). That is the *hardest* data to externalise and it is already done.

The bad news is coverage is uneven, and the most important subsystem is the worst:

| Subsystem | Config reads | Hardcoded constants | Authorable today? |
|---|---|---|---|
| Performance (`fn_perfData.sqf`) | 37 | — | **Yes** |
| Fuselage (`fn_fuselageVariables.sqf` + faces) | 15 | — | **Yes** |
| Engine (`fn_engine.sqf`, `fn_engineBET.sqf`) | 14 | — | **Yes** |
| Core (`fn_coreConfig.sqf`) | 11 | — | Partial |
| Wing/stabilator (`fn_wing.sqf`) | 3 | 5 | Partial |
| **Main rotor** (`fn_simpleRotorMain.sqf`) | **0** | **~20** | **No** |
| **Tail rotor** (`fn_simpleRotorTail.sqf`) | **0** | **10** | **No** |
| **BET rotor** (`fn_rotor.sqf`, `fn_rotorUpdate.sqf`) | 2 | 3 | **No** |
| Transmission (`fn_transmission.sqf`) | 0 | 3 | **No** |

**The rotor — the single most defining component of a helicopter's handling — is entirely hardcoded.** From `fn_simpleRotorMain.sqf:54-63`: blade radius 7.315 m, chord 0.533 m, blade mass 72.108 kg, design RPM 289, gear ratio 72.291, hinge offset 0.038, 4 blades, rotor height AGL 3.606 m. Plus tuning scalars — `_pitchTorqueScalar`, `_rollTorqueScalar`, `_baseThrust`, `_inducedVelocityScalar`, `_vrsScalarExponent`, flapback gains `_kFlapLon`/`_kFlapLat`.

> **Since resolved for the SIMPLE rotor.** Its geometry and scalars read from
> `helisim_simpleRotor.hpp` now. The BET rotor still holds its geometry in SQF,
> so the blocker stands for that path - see `HELISIM_CONFIG_PLAN.md`.

A modder dropping SFM+ onto a UH-60 or Mi-8 today would get AH-64 rotor physics and no config path to change it. **This is the framework blocker.**

### Second gap: no config validation or defaults

`fn_coreConfig.sqf` performs **zero** `isNumber`/`isArray`/default checks across its reads. A missing or misspelled config entry yields `0` or `[]` silently — the aircraft flies wrongly rather than failing loudly. For first-party use that is survivable; for a framework consumed by modders it is the difference between "works in ten minutes" and "unusable". Every config read needs a documented default and a startup validation pass that reports missing/invalid entries by name.

Minor: SFM+ reads its own class inconsistently cased — `>> "fza_sfmplus"` (`fn_coreConfig.sqf:21`, `fn_fuselageVariables.sqf:21`) vs `>> "Fza_SfmPlus"` (`fn_engineController.sqf:23`). Harmless today, but normalise it before publishing a spec.

### Target structure

The flat class works but does not scale to authoring. Move to nested subclasses with inheritance, so a modder overrides only what differs:

```cpp
class Fza_SfmPlus {
    class Mass       { /* empty mass, moments, station arms, crew */ };
    class Rotors {
        class Main   { bladeRadius; bladeChord; bladeMass; numBlades;
                       hingeOffset; designRPM; gearRatio; direction;
                       airfoil; /* + tuning scalars */ };
        class Tail   { /* same shape */ };
    };
    class Engines    { class Engine1 {...}; class Engine2 {...}; };
    class Fuselage   { class Top {...}; class Side {...}; class Front {...}; };
    class Wings      { class Stabilator {...}; };
    class Fuel       { /* cells, capacities, arms, transfer rates */ };
    class Airfoils   { class NACA0012 {...}; class NACA4418 {...}; };
    class Model      { /* hitpoint + animation source names — see §4 */ };
};
```

Nesting also solves §4: hitpoint and animation-source names become config-driven, making option (b) — airframe-agnostic — the natural outcome rather than extra work.

### Output side

Modders need FM state for their own displays. The §5 getters are that surface. What matters for a framework: they must be **documented and stable**, since third-party gauges will bind to them. The 241 raw variables stay as internals; the getters are the contract.

### Documentation deliverable

A framework is only as good as its docs. Minimum:
- **Config reference** — every property: units, coordinate frame, sign convention, default, valid range.
- **Output API reference** — every getter's returned keys and units.
- **Worked example** — one complete non-AH-64 aircraft, start to finish.
- **Model contract** — required hitpoints and animation sources (§4).

Units and sign conventions matter more than usual here: the config mixes metres, kilograms, degrees and dimensionless ratios, and this project has already burned time on sign errors in lateral/yaw coupling. Every axis needs its positive direction stated explicitly.

---

## 0.6 One PBO — config-gated subsystems

Everything ships in `addons/fza_ah64_sfmplus`. Tiering is achieved by **config switches inside the single folder**, not by separate installable addons. A user who wants only the flight model sets the subsystem flags off and Arma's vanilla systems apply; a user who wants a study-sim turns them on.

### Absorption budget

| Source | Lines | Disposition |
|---|---|---|
| SFM+ today | 6871 | Base |
| `fza_ah64_systems` | 1203 | Absorb as gated subsystems |
| `fza_ah64_fuel` | 609 | Absorb (mass-bearing core per §0) |
| `fza_ah64_common` (only what SFM+ uses) | 330 | Vendor |
| **Total** | **~9013** | **+31% on the current base** |

A 31% growth for total self-containment is a good trade. Note this absorbs *only* what SFM+ uses from `common`, not the whole addon.

### Subsystem model

| Subsystem | Contents | Gate | Off ⇒ |
|---|---|---|---|
| **Flight model** | Rotors, engines, fuselage, wings, mass/CG/fuel, FMC/SAS, input | Always on | — |
| **Drivetrain** | NGB ×2, transmission, gearbox wear → torque | `enableDrivetrain` | No torque perturbation |
| **Hydraulics** | Primary/utility/emergency, accumulator, leaks | `enableHydraulics` | Full control authority |
| **Electrical** | Battery, generators, rectifiers, buses, APU | `enableElectrical` | Buses powered, APU available |
| **Instruments** | Control indicator, slip ball | `enableInstruments` | No FM gauges drawn |

Each gate's "off" state is the corresponding neutral default from §3.3 — so the switches and the drop-in defaults are the *same mechanism*, not two.

### `fza_ah64base` — resolved by the pack model

SFM+ binds to a vehicle base class defined in **`controls/config/cfgVehicles.hpp:9`**:

| File | Line | Binding |
|---|---|---|
| `extendedEventHandlers.hpp` | 8 | `class fza_ah64base` — GetIn EH registration |
| `fn_analogHandler.sqf` | 2 | `isKindOf "fza_ah64base"` guard |
| `fn_nonAnalogHandler.sqf` | 2 | `isKindOf "fza_ah64base"` guard |

Under §0.7 this is not a blocker: Core binds to vanilla `Helicopter_Base_F` and the **pack** supplies the opt-in (config property + a Core-owned variable set at init). The `isKindOf` guards become checks on that variable, so Core never names an airframe class. The problem only existed because Core and aircraft shared one addon.

### Remaining external symbols — all small

| Symbol | Owner | Lines | Action |
|---|---|---|---|
| `fza_fnc_animSetValue` | controls | 47 | Vendor |
| `fza_audio_fnc_flightTone` | audio | 27 | Vendor or make an overridable hook |
| `fza_fuel_fnc_*` | fuel | 609 | Absorbed (§3.4) |
| `fza_fnc_*` helpers | common | 330 | Vendor (§3.1) |
| `SYS_*` macros | systems header | 7 of 38 | Copy as config properties |

**Dependencies that remain and are acceptable:** vanilla `BIS_fnc_clamp` (89 uses), `BIS_fnc_lerp` (18), `BIS_fnc_getPitchBank` (10), `BIS_fnc_inString` (4) — all base-game. CBA is one function (`CBA_fnc_simplifyAngle`) plus the settings framework; if even CBA must go, that function is trivial to inline and settings would move to config.

### Systems config surface: currently zero

All 1203 lines of `systems` contain **no** `getNumber`/`getArray` calls. Every tunable is a literal or a `#define` in `headers/systems.hpp` — 15 damage thresholds (all `0.85`), hydraulic pressures (`SYS_MIN_HYD_PSI 1260`, `SYS_MIN_ACC_PSI 1650`), timers (battery 12 min, accumulator 1.5 min, leak 2 min), `DMG_PER_SEC 0.003`. On absorption these become config properties under their subsystem classes.

### How separable is `fza_ah64_systems`?

Better than expected — of 1203 lines, only two subsystems touch SFM+ at all:

| Subsystem | Lines | SFM+ refs | Note |
|---|---|---|---|
| `drivetrain` | 376 | **14** | Reads `engPctTQ`/`isSingleEng`; **writes `randomTq`** into FM torque |
| `electrical` | 327 | 1 | Reads `fnc_getRtrRPM` for generator-online threshold |
| `hydraulics` | 255 | 0 | Publishes pressures/level; no SFM+ dependency |
| `apu` | 55 | 0 | Publishes `apuOn`/`apuRPM_pct` |
| `core` | 99 | 0 | Scheduler/state — folds into SFM+'s own loop |
| `repair` | 49 | 0 | Independent |
| `interact` | 42 | 0 | Independent |

The reverse is only **3 SFM+ files** reading systems state — `fn_getInput.sqf` (hydraulic pressure → control authority), `fn_engineController.sqf` (APU → start), `fn_wing.sqf` (DC bus → stabilator). Those are exactly the 8 optional inputs in §3.3.

**So the internal seams already exist.** Absorption is mostly moving files and renaming `fza_systems_*` → `fza_sfmplus_*`, not untangling logic.

### The one cross-subsystem physics coupling

`drivetrain` writes `fza_sfmplus_randomTq` (`fn_drivetrainNoseGearbox1/2.sqf:106`, `fn_drivetrainTransmission.sqf:87-88`), which SFM+ adds directly to engine torque at `fn_engine2.sqf:88` and `fn_engineBET.sqf:138` — gearbox wear changes delivered torque.

By principle 1 the FM core owns a torque-perturbation input defaulting to 0; the wear model that drives it sits behind `enableDrivetrain`. Pattern for every subsystem→FM effect: **core owns the input and its neutral default; the subsystem owns the model.**

### Vanilla fallback is a supported configuration

With all gates off, SFM+ must fly correctly on vanilla systems: full hydraulic pressure, buses powered, APU available, fuel available. §3.3's defaults already deliver this — the work is making it explicit and giving it its own regression pass. "FM only" is a product, not a degraded mode.

### How separable is `fza_ah64_systems` today?

Better than expected. Of 1203 lines, only two subsystems touch SFM+ at all:

| Subsystem | Lines | SFM+ refs | Assessment |
|---|---|---|---|
| `drivetrain` | 376 | **14** | Reads `engPctTQ`/`isSingleEng`; **writes `randomTq`** back into FM torque |
| `electrical` | 327 | 1 | Reads `fnc_getRtrRPM` for generator-online threshold |
| `hydraulics` | 255 | 0 | Publishes `fza_systems_*Psi`/`utilLevel_pct`; **no SFM+ dependency** |
| `apu` | 55 | 0 | Publishes `apuOn`/`apuRPM_pct`; fully separable |
| `core` | 99 | 0 | Scheduler/state — likely folds into the layer framework |
| `repair` | 49 | 0 | Fully separable |
| `interact` | 42 | 0 | Fully separable |

And the reverse direction is only **3 SFM+ files** reading systems state — `fn_getInput.sqf` (hydraulic pressure → control authority), `fn_engineController.sqf` (APU → start availability), `fn_wing.sqf` (DC bus → stabilator power). Those are exactly the 8 optional inputs in §3.3, which already have safe defaults.

**This means the layering essentially already exists** — it is just undeclared and hardcoded. The optional-input shim (Phase 5) *is* the layer interface.

### The one genuine cross-layer physics coupling

`drivetrain` writes `fza_sfmplus_randomTq` (`fn_drivetrainNoseGearbox1/2.sqf:106`, `fn_drivetrainTransmission.sqf:87-88`), which SFM+ adds directly to engine torque output at `fn_engine2.sqf:88` and `fn_engineBET.sqf:138`. Gearbox wear therefore changes delivered torque — a physics effect.

By principle 1 the **coupling** is core: SFM+ owns a torque-perturbation input (default 0, so no layer means no perturbation) exposed as `fza_sfmplus_fnc_setTorqueNoise`. The **wear model** that decides the value stays in the optional drivetrain layer. This is the pattern for every layer→core physics effect: *core owns the input and its neutral default; the layer owns the model that drives it.*

### Systems config surface: currently zero

All 1203 lines contain **no** `getNumber`/`getArray` calls. Every tunable is a literal or a `#define` in `headers/systems.hpp` — 15 damage thresholds (all `0.85`), hydraulic pressures (`SYS_MIN_HYD_PSI 1260`, `SYS_MIN_ACC_PSI 1650`), timers (battery 12 min, accumulator 1.5 min, leak 2 min), `DMG_PER_SEC 0.003`.

For a modder-facing suite these must become config properties. Note SFM+ itself consumes 7 of these macros (§3.2), so the core needs its own copies regardless — a shared header is the wrong mechanism across a layer boundary.

### Vanilla fallback

The core must fly correctly with **no** layers present. §3.3's defaults already deliver this: full hydraulic pressure, DC bus on, APU available, fuel available. The addition is making it explicit and tested — a "core only" configuration is a supported product, not a degraded one, and needs its own regression pass.

---

## 0.7 The two-PBO contract

**SFM+ is retired.** The name, the `fza_ah64_` prefix and the AH-64 coupling all go. What replaces it:

| | **`bmkhs_helisim`** (Core) | **`XXX_helisim`** (pack) |
|---|---|---|
| Ships | Once, by you | Once per airframe, by the designer |
| Contains | Flight model, subsystems, math, input, output API | Config class, model bindings, entry point, aircraft-specific glue |
| Depends on | Vanilla + CBA only | `bmkhs_helisim` |
| Knows about | Nothing airframe-specific | Its own aircraft only |
| Prefix | `bmkhs_` | `XXX_` |

`fza_ah64_helisim` becomes simply the first pack — the AH-64 loses its privileged status and is validated by the same contract as any third-party aircraft. **That is the real test of the framework:** if building the AH-64 pack requires touching Core, the boundary is wrong.

### Core is a passive library; the pack is the executable

**Core owns no lifecycle.** It exports functions and nothing else — no event handlers, no update loop, no `isKindOf` guards, no vehicle discovery. Core is told *"here is the aircraft configuration"* and it crunches the numbers. It has no notion of which aircraft, or that a particular aircraft exists at all.

**The pack drives everything.** It owns the scheduler and calls Core each tick. This inverts the earlier draft of Phase 2, which had Core registering its own update loop — wrong under this model. Core never schedules itself.

**Why not make Core the runner?** A runner needs an opinion about lifecycle: how to discover vehicles, when to guard, what tick rate to impose. The moment Core holds that opinion it stops being airframe-agnostic. The evidence is in the current code — the only things resisting portability today are precisely the lifecycle bits (`fza_ah64base` guards, the GetIn EH). Making Core the runner means adding more of them.

The cost is per-pack boilerplate. That is solvable without giving Core a lifecycle: ship a template pack, or have Core export an optional scheduling helper a pack *may* call. Opt-in convenience, never mandatory lifecycle.

### Noted: Core as a DLL

The ideal expression of "this is a function library" is a native extension — a DLL cannot hold a lifecycle, cannot reach for `configOf`, and cannot accumulate global state, so the boundary is enforced by the linker instead of by discipline. ACE ships extensions for comparable numeric work, so there is precedent.

Not now, for five reasons:

1. **Per-call overhead.** `callExtension` marshals strings. Core runs many force calculations per frame (blade elements, fuselage panels, PID loops), so it would mean either many crossings per frame or one large serialize/deserialize round trip.
2. **No engine access.** A DLL cannot call `setMass`, `setCenterOfMass`, `addForce`, or read `getHitPointDamage`/`animationSourcePhase`. SQF stays in the loop regardless — gather in SQF, cross, cross back, apply in SQF.
3. **Config is engine-side.** The authoring model is CfgVehicles data, which a DLL cannot read. All cached values would have to be marshalled across at setup, and the config surface kept in sync with the DLL surface by hand.
4. **Distribution.** x64 DLL plus a Linux `.so` for servers, antivirus friction, extension whitelisting — real cost for a modder-facing product.
5. **Debuggability.** It forfeits file patching. The `recompile = 1` work in `cfgFunctions.hpp` exists precisely so flight-model code can be iterated live; a compile-link-restart cycle while the FM is still being tuned is a significant regression.

**The architecture survives without it.** What a DLL would enforce — no lifecycle, no global state, inputs in / numbers out — is enforced here by the §9 verification gate instead: no `isKindOf`, no airframe symbols, no self-registration in Core.

**Write Core as if it were a DLL.** Functions take explicit inputs and return values; nothing reaches for `configOf` or vehicle variables mid-computation. That is the "told the config" fix generalised. Kept to, a future port becomes mechanical rather than a rewrite. Functions can be converted to this style incrementally as they are touched — it does not need to happen up front.

One shape a pack can take — illustrative, not a required layout:

```
xxx_helisim/
  config/       aircraft BMKHS_HeliSim data; cfgVehicles binding; EH registration
  functions/    setup / shutdown; tick drivers; aircraft-specific update logic
  XEH_*         lifecycle entry
```

The pack decides its own tick granularity — per-frame, fixed-step, slow, or any mix. That is a scheduling choice Core has no opinion on, which bears on decision #3: rather than Core mandating per-frame or fixed-step, it exposes entry points and the pack calls them how it likes.

The only parts that are actually contract are: the pack supplies the config, the pack opts the vehicle in, and the pack calls Core. Directory names and file splits are the designer's business.

### "Told the config" — where Core falls short today

Core does not currently get *handed* a config; it goes and finds one. `configOf _heli` appears in **15 places** across `fn_coreConfig`, `fn_coreUpdate`, `fn_getInput`, the engine, fuselage, rotor, wing and performance functions. That hardcodes an assumption: the config lives on the aircraft's own vehicle class, under `BMKHS_HeliSim`.

The caching pattern is already correct, though — `fn_coreConfig.sqf` reads config **once** and caches 110 values into vehicle variables, and the per-frame functions read those variables rather than config. So the expensive part of "told the config" is done; only the lookup leaks.

The fix is small and belongs with the pack boundary: `bmkhs_fnc_coreConfig` takes the config path as a parameter, the pack passes it, and the remaining 14 direct `configOf` reads either receive it or read the already-cached variables. That lets a pack keep its config wherever it likes — the aircraft class, a separate class, or several classes composed — instead of Core dictating the location.

### Core's entry points already fit this shape

**DONE.** Core exports the right four functions and the pack now calls all of
them from its own lifecycle - nothing in `fza_ah64_controls` reaches into
HeliSim any more:

| Core function | Called from | Via |
|---|---|---|
| `bmkhs_fnc_coreInit` + `bmkhs_fnc_coreConfig` | pack's `Extended_Init_EventHandlers` | `fnc_setup` |
| `bmkhs_fnc_coreUpdate` + `bmkhs_fnc_coreUpdateFlightModel` | pack's `XEH_preInit` EachFrame | `fnc_perFrame` |
| `bmkhs_fnc_eventGetIn` | pack's `Extended_GetIn_EventHandlers` | pack EH |

**Core is a pure library.** The GetIn registration moved to the pack and the two
`fza_ah64base` guards are gone - the input handlers gate on `bmkhs_initialised`
instead, which is Core's own flag and names no airframe.

### What the pack provides

1. **The config class** — `class BMKHS { class Rotors {...}; class Engines {...}; ... }` per §0.5, on the aircraft's vehicle class.
2. **Model bindings** — hitpoint and animation-source names (§4), declared rather than hardcoded in Core.
3. **The entry point** — how the airframe declares itself HeliSim-driven. This is what replaces the `fza_ah64base` binding (§0.6): rather than Core naming a base class it cannot own, the pack opts its vehicle in.
4. **Aircraft-specific extras** — panels, displays, weapons. Consumers of Core's output API, never contributors to its physics.

### Two things this settles

**The `fza_ah64base` blocker dissolves.** §0.6 framed it as choosing among three workarounds for Core binding to a class in another PBO. Under the pack model there is nothing to work around: Core binds to `Helicopter_Base_F` (vanilla) and the pack supplies the opt-in. The dependency was an artifact of Core and aircraft living in one addon.

**Headers become a public interface.** Eight external files already `#include "\fza_ah64_sfmplus\headers\core.hpp"` — `controls` ×4, `mpd` ×2, `fuel`, `ihadss`. That compile-time coupling is a *precedent*, not a problem: packs will legitimately include Core headers for shared constants. It does mean Core's `headers/` directory is published API and needs the same stability discipline as the output getters.

### The rename — module-scoped

`fza_ah64_sfmplus` → `bmkhs`; `fza_sfmplus_*` → `bmkhs_*`. Nothing outside the module is renamed.

| Target | Count | Note |
|---|---|---|
| `fza_sfmplus_*` inside the module | **1146** | Direct renames |
| `fza_sfmplus_*` in consumer addons | **307** across **48 files** | Must update in lockstep |
| **Total symbol occurrences** | **1453** | |
| Folder + `$PBOPREFIX$` | 1 each | `bmkhs` |
| CfgFunctions tag (`cfgFunctions.hpp:27`) | 1 | `FZA_sfmplus` → `bmkhs` |
| `#include "\fza_ah64_sfmplus\headers\..."` in consumers | 8 files | Path changes with the folder |

**Not renamed:** `.hemtt/project.toml`, stringtables, the other 16 addons, and the `fza_ah64_*` / `fza_fnc_*` symbols the module currently *reads* (291 and 335 occurrences) — those disappear through absorption and vendoring (§3.1, §3.4), not renaming. Distinct mechanisms; don't conflate them.

Do it **once**, as an isolated commit with no behavioural change, and land it *before* the restructuring phases — otherwise every later phase is written against names that change again. The MP-desync caveat (§7) applies at full force: network-synced variables change identity, so clients and server must update together.

The 48 consumer files are the coordination cost, and they are being migrated or retired anyway (decision #8) — sequence the rename with that decision rather than touching them twice.

---

## 1. Executive summary

SFM+ is currently a **shared blackboard**, not a module. State crosses the boundary through `setVariable`/`getVariable` on the helicopter object; nothing is enforced and no contract is documented.

The headline numbers look daunting, but the standalone gap is far smaller than they suggest:

| Metric | Count | Drop-in impact |
|---|---|---|
| Variables SFM+ publishes | 241 | Becomes the output API (§5) |
| External files reading SFM+ | 49 | **Not blockers** — these are consumers, not dependencies |
| Consuming addons | 15 | Optional; module works without them |
| SFM+ outbound call sites | ~330 | **≈330 lines of helper code** — vendorable |
| External *state* SFM+ reads | **8 variables** | Small shim with safe defaults (§3.3) |
| **External mass mutators** | **3 call sites + 24 writes** | **Must move inside — §3.4** |
| **Fuel mass logic to absorb** | **609 lines** | **Core, not optional — §3.4** |
| Macros needed from `systems.hpp` | **7** (of a 38-line header) | Copy constants, drop include |
| Model contracts (hitpoints + anim sources) | **15** | Config-driven under §0.5; see §4 |
| **Rotor constants hardcoded in SQF** | **~33** | **Framework blocker — §0.5** |
| **Config reads with defaults/validation** | **0** | **Framework blocker — §0.5** |
| `systems` lines with any config read | **0 of 1203** | Must be config'd on absorption — §0.6 |
| `systems` subsystems coupled to SFM+ | **2 of 7** | Internal seams already exist — §0.6 |
| **Total lines to absorb/vendor** | **~2142** | +31% on a 6871-line base — §0.6 |
| External config-class bindings | 3 (`fza_ah64base`) | Resolved by the pack model — §0.7 |
| **`fza_sfmplus_*` → `bmkhs_*` rename** | **1453** (1146 in-module + 307 in 48 consumers) | **Do once, early, isolated — §0.7** |
| External files `#include`-ing SFM+ headers | 8 | Headers are published API — §0.7 |

**The critical distinction for a drop-in module:** the 49 consuming files and 241 published variables are *not* obstacles. Consumers reading SFM+ can simply be absent. What blocks portability is what SFM+ *needs* to run — ~330 lines of vendorable helpers, 8 input variables, 7 macros, the model contract — **plus everything that can move mass, which must come inside rather than be defaulted away.**

### Findings that shape the work

1. **SFM+'s own tuning data lives in another addon.** `class Fza_SfmPlus` — 502 lines of masses, moments, fuselage geometry, drag and engine tables — sits at `addons/fza_ah64_controls/config/cfgVehicles/sfmplus.hpp`, included from `cfgVehicles.hpp:27`. **A drop-in module shipping without its own flight characteristics is a non-starter. This is blocker #1.**

2. **SFM+ does not own its update loop or its input path.** Registration is at `controls/XEH_preInit.sqf:245`; key bindings are in `controls/config/CfgUserActions.hpp:22,29,30`. Dropped into a bare project, SFM+ never ticks and never receives input. **Blocker #2.**

3. **15 CBA settings that only SFM+ reads are registered in controls** (`XEH_preInit.sqf:44-177`) — including `fza_ah64_sfmPlusRotorModel`, which selects BET vs simple rotor. Without controls these are `nil` in the hot path. **Blocker #3.**

4. **`requiredAddons[]` is under-declared by five addons** — `config.cpp:9` claims only `fza_ah64_controls`, but there are hard runtime/compile dependencies on `common`, `systems`, `fuel`, `audio`, and `model`.

5. **Two referenced functions do not exist** (verified: no file, no CfgFunctions entry):
   - `fza_sfmplus_fnc_coreFixedUpdate` — called at `controls/.../fn_coreFixedUpdateScheduler.sqf:35`. The entire scheduler is dead: declaration commented at `controls/config/CfgFunctions.hpp:31`, invocation commented at `controls/XEH_preInit.sqf:263`. **The live path is per-frame via `fza_ah64_eachFrameArray`, not fixed-timestep.**
   - `fza_sfmplus_fnc_fuelSet` — called ×6 from `missionplanner/fn_applyConfig.sqf` (826, 836, 903, 919, 948, 958). Real function is `fza_fuel_fnc_fuelSet`.

6. **Circular dependency:** `common/fn_rotateVector.sqf:5-7` calls `fza_sfmplus_fnc_quaternion` — the shared math library depends on the flight model.

---

## 2. What "drop-in" requires

Four properties, in dependency order:

1. **Self-contained** — no `requiredAddons` beyond CBA and the model.
2. **Self-starting** — brings its own update loop and input handling.
3. **Degrading** — runs with sane defaults when optional systems (hydraulics, fuel, electrical) are absent.
4. **Publishing** — a documented, stable output API instead of 241 ad-hoc variables.

Property 3 is the design decision that makes this tractable: rather than requiring `systems` and `fuel`, SFM+ **reads 8 optional inputs with safe defaults** and flies correctly when nothing supplies them.

---

## 3. The actual dependency surface

### 3.1 Vendorable helpers — `fza_ah64_common`, ~330 call sites, **~330 lines**

Uses the bare `fza_fnc_*` prefix, which is why an `fza_<addon>_fnc_` search pattern misses it entirely.

| Symbol | Uses | Lines | Purpose |
|---|---|---|---|
| `fza_fnc_linearInterp` | 93 | 73 | Every airfoil, drag, engine and performance table lookup — core of the aero model |
| `fza_fnc_debugDrawLine` | 70 | 41 | Force-vector debug |
| `fza_fnc_setArrayVariable` | 46 | 27 | Per-index engine/rotor state writes |
| `fza_fnc_pidReset` | 28 | 5 | Clears PID integrators on mode change |
| `fza_fnc_pidRun` | 25 | 30 | Every FMC/SAS/hold loop + engine governor |
| `fza_fnc_pidCreate` | 22 | 10 | PID construction (`fn_coreConfig.sqf:128-176`) |
| `fza_fnc_updateNetworkGlobal` | 21 | 27 | MP propagation of hold-mode state |
| `fza_fnc_rotateVector` | 9 | 9 | Body-frame panel normals (**circular** — see §1.6) |
| `fza_fnc_setMultiArrayVariable` | 6 | 32 | Per-blade-element induced flow |
| `fza_fnc_getArea` | 5 | 32 | Projected drag area |
| `fza_fnc_debugDrawCircle` | 5 | 44 | Rotor disc debug |
| **Total** | **~330** | **~330** | |

Also `fza_fnc_animSetValue` (`fn_interactPowerLever.sqf:29,41,50`) — bare prefix but owned by **controls**, not common. Easy to mis-attribute.

**Verdict:** vendor all of it into `fza_sfmplus_fnc_*`. 330 lines is a rounding error against a flight model, and it eliminates two addon dependencies plus the circular reference.

### 3.2 Compile-time — `fza_ah64_systems` header

`#include "\fza_ah64_systems\headers\systems.hpp"` in 8 files (`fn_actuator.sqf:20`, `fn_getInput.sqf:21`, `fn_fmc.sqf:3`, `fn_engineBET.sqf:2`, `fn_engineController.sqf:21`, `fn_rotor.sqf:2`, `fn_simpleRotorTail.sqf:23`, `fn_wing.sqf:2`). **The build breaks without it.**

Only 7 of the header's macros are used: `SYS_MIN_HYD_PSI`, `SYS_HYD_MIN_LVL`, `SYS_HYD_DMG_THRESH`, `SYS_ENG_DMG_THRESH`, `SYS_STAB_DMG_THRESH`, `SYS_IGB_DMG_THRESH`, `SYS_TGB_DMG_THRESH`.

**Verdict:** copy the 7 constants into an SFM+ header. Dependency gone.

### 3.3 Runtime inputs — the complete list is **8 variables**

This is the entire inbound state surface. Each needs a documented default so the module flies standalone.

| Variable | Owner | Read at | Standalone default |
|---|---|---|---|
| `fza_systems_priHydPsi` | systems | `fn_getInput.sqf:55` | Nominal pressure (full authority) |
| `fza_systems_utilHydPsi` | systems | `fn_getInput.sqf:58` | Nominal pressure |
| `fza_systems_utilLevel_pct` | systems | `fn_getInput.sqf:59` | 100% |
| `fza_systems_apuOn` | systems | `fn_getInput.sqf:62`, `fn_engineController.sqf:26` | `true` (start available) |
| `fza_systems_dcBusOn` | systems | `fn_wing.sqf:36` | `true` (stabilator powered) |
| `fza_fuel_eng1FuelAvail` | fuel | `fn_engineController.sqf:44` | `true` |
| `fza_fuel_eng2FuelAvail` | fuel | `fn_engineController.sqf:45` | `true` |
| `fza_mplanner_applying` | missionplanner | `fn_getInput.sqf:31` | `false` — **already safely defaulted** |

Plus two controls-owned timing globals SFM+ currently *writes*: `fza_ah64_previousTime` (`fn_coreUpdate.sqf:23`) and `fza_ah64_lastFrameGetIn` (`fn_getInput.sqf:33,245,246`). Both become SFM+-internal once it owns its loop.

Also `fza_ah64_emerHydOn` and `fza_ah64_engineOverspeed` (systems-owned, `fn_getInput.sqf:61`, `fn_engine2.sqf:24`, `fn_engineBET.sqf:28`) — same treatment, default to off/false.

**Verdict:** a small optional-input shim. Defaults documented; optional setters (`fza_sfmplus_fnc_setHydraulics`, `setFuelAvailable`, `setElectrical`) let a host model failures if it wants. This converts `systems` and `fuel` from hard dependencies into **optional integrations**.

### 3.4 Mass mutators — core by the §0 rule

Everything here can change mass or CG, so by the governing principle none of it is an integration point. All of it moves inside.

**(a) Fuel — 609 lines, bidirectional**

- SFM+ calls `fza_fuel_fnc_fuelVariables`/`fuelMgmtVariables`/`fuelSet` at `fn_coreConfig.sqf:172-174`, and `fuelUpdate`/`fuelMgmtUpdate` **every frame** from `fn_coreUpdate.sqf:52-53` — *SFM+ is already the fuel system's scheduler.*
- `fza_ah64_fuel` **writes** the SFM+-prefixed `fza_sfmplus_{fwd,ctr,aft,stn1..4,tot}FuelMass` in **24 places** (`fn_fuelVariables.sqf:21-30`, `fn_fuelSet.sqf:81-92`, `fn_fuelUpdate.sqf:346-353`), which SFM+ reads at `fn_massUpdate.sqf:62-68`.

Fuel is the largest time-varying mass term, and fwd/aft transfer moves longitudinal CG in flight. SFM+ already schedules it and already owns the variables' namespace.

| File | Lines | Disposition |
|---|---|---|
| `fn_fuelUpdate.sqf` | 369 | **Absorb** — burn/transfer/gravity-feed; pure mass logic |
| `fn_fuelSet.sqf` | 93 | **Absorb** — replaces the broken `fza_sfmplus_fnc_fuelSet` (§1.5) |
| `fn_fuelVariables.sqf` | 31 | **Absorb** — already initializes SFM+-prefixed vars |
| `fn_fuelMgmtUpdate.sqf` | 60 | **Split** — pump/crossfeed *state* is systems; resulting transfer is core |
| `fn_fuelMgmtVariables.sqf` | 56 | **Split** — same |

**Verdict:** absorb the mass-bearing core (~493 lines) into `functions/mass/` or a new `functions/fuel/`. What remains for an optional `fza_ah64_fuel` is the *panel* — pump switches, crossfeed selection, MPD FUEL page plumbing — which sets intent that SFM+ acts on. Remove the per-frame calls from `fn_coreUpdate.sqf:52-53` once absorbed.

**(b) The three external mass mutators**

| Caller | Line | Calls | Fix |
|---|---|---|---|
| `weapons/fn_jettisonAll.sqf` | 31 | `fnc_massUpdateStation` | Notify via `fnc_onStoresChanged`; SFM+ recomputes |
| `missionplanner/fn_applyConfig.sqf` | 1018 | `fnc_massUpdate` | Same |
| `controls/fn_weaponSwapM230Mag.sqf` | 5-15 | `add/removeMagazineTurret` + `IAFSInstalled` | Same — swaps 1200rd mag for IAFS tank, a large ammo-bay mass change |

Note `fn_jettisonAll.sqf:32` also does `_pylonDummyMain setMass _pylonmass` — the only external `setMass` in the project. It targets a dummy object, not the heli, so it does not fight SFM+; worth confirming during the move.

**Verdict:** external code declares *intent* ("stores changed", "loadout applied"); SFM+ owns recomputation. No external caller drives `fn_massUpdate` directly.

**(c) Already correct** — `fnc_massUpdateStation` and `fnc_massUpdateMagazine` live in SFM+ and are called from `fn_massUpdate.sqf:77,84-96`. Ammo mass at 0.35 kg/round feeds both `_curMass` and `_ammoBayMom`. No change needed.

### 3.5 Audio — trivial

`fza_audio_fnc_flightTone` at `fn_fmcAltitudeHold.sqf:34`, `fn_fmcAltitudeHoldEnable.sqf:38`, `fn_fmcAttitudeHoldEnable.sqf:34`, `fn_fmcHoldModesDisable.sqf:5`. Guard with `isNil` or route through an overridable hook.

Dead code: commented `playSound "fza_ah64_flt_control"` references a CfgSounds entry that no longer exists anywhere in the repo.

### 3.6 Config data — blocker #1

`class Fza_SfmPlus`, 502 lines, at `controls/config/cfgVehicles/sfmplus.hpp`. Read back by SFM+ at `fn_coreConfig.sqf:21,104-118`, `fn_engineController.sqf:23`, `fn_engine.sqf:23-131`, `fn_engineBET.sqf:6-121`, `fn_getInput.sqf:35-37`, `fn_fuselage*.sqf`, and by three external addons.

Note SFM+ reads it inconsistently cased — `>> "fza_sfmplus"` at `fn_coreConfig.sqf:21` vs `>> "Fza_SfmPlus"` at `fn_engineController.sqf:23`. Works today (config lookup is case-insensitive) but worth normalising.

---

## 4. The irreducible dependency: the model

This cannot be vendored or defaulted away. SFM+ has hard contracts with the p3d.

**Hitpoints (10):** `hit_hyd_pripump`, `hit_hyd_utilpump`, `hit_stabilator`, `hitengine1`, `hitengine2`, `hithrotor` (**miscased `HitHRotor`** at `fn_getRtrRPM.sqf:25`, `fn_simpleRotorMain.sqf:423`), `hitvrotor`, `hit_drives_intermediategearbox`, `hit_drives_tailrotorgearbox`.

**Animation sources (5):** `"Hstab"` (`fn_wing.sqf:66` — SFM+ is the sole writer), `"rotorH"`/`"rotorV"` (`fn_rotor.sqf:37`), `"fcr_enable"` (`fn_massUpdate.sqf:45`), `plt_eng1/2_start` (`fn_interactStartSwitch.sqf:26`), `fza_ah64_powerLever1/2` (`fn_interactPowerLever.sqf:26`). Plus a commented `"mainRotorRPM"` read at `fn_engineController.sqf:58`.

No memory-point reads — all geometry comes from the config class, which helps.

**Verdict:** two honest options.
- **(a) Document the contract.** Ship a "required hitpoints and animation sources" spec; the module is drop-in *for any airframe meeting it*. Simplest, and appropriate if SFM+ stays an AH-64 flight model.
- **(b) Make it configurable.** Move hitpoint and animation-source names into the config class with defaults, and guard every read. Fully airframe-agnostic, more work, and each guarded read is a place damage modelling can silently no-op.

Recommend **(a)** unless you intend SFM+ for other airframes — in which case (b) is the whole point and should be scoped in from the start.

---

## 5. Output API — turning 241 variables into a contract

The 49 consuming files do not block portability, but they define what the published surface must cover. Grouped by domain:

| Group | Representative outputs | Consumers |
|---|---|---|
| **Flight state** | `vel2D`, `vel3D`, `gndSpeed`, `velWorldSpace(NoWind)`, `velModelSpace(NoWind)`, `accelX/Y`, `fnc_getAltitude`, `aero_beta_g`, `aero_beta_deg` | ihadss, mpd, controls |
| **Engine** | `engState`, `engPctNG`, `engPctNP`, `engPctTQ`, `engTGT`, `engOilPSI`, `engFF`, `engPowerLeverState`, `isSingleEng`, `fnc_getRtrRPM` | mpd, controls, systems, fire, ufd |
| **Controls** | `cyclicFwdAft`, `cyclicLeftRight`, `collectiveOutput`, `pedalLeftRight`, `fnc_getInterpInput` | **ctrlVis**, animation |
| **FMC/SAS** | `fmcSasPitchOut`, `fmcSasRollOut`, `fmcHdgHoldPedalYawOut`, `fmcAttHoldCycPitchOut/RollOut`, `fmcAltHoldCollOut`, hold-mode flags | **ctrlVis**, mpd, controls |
| **Mass/fuel** | `{fwd,ctr,aft,stn1..4,tot}FuelMass`, `max*FuelMass`, `GWT`, `CG` | fuel, mpd, ufd, auxtank, weapons, missionplanner |
| **Performance** | `hvrTQ_IGE/OGE`, `goNoGoTQ_*`, `maxGWT_*`, `maxTQ_*`, `TAS_*`, `FAT`, wind | mpd PERF page |
| **Utility** | `fnc_onGround` | wca, fcr, controls, mpd, weapons — **most widely used export** |

Proposed: `fza_sfmplus_fnc_getFlightState` / `getEngineState` / `getControlState` / `getFmcState` / `getMassState` / `getPerfState`, each returning a documented hashmap. Keep raw variables as a deprecated shim for one release.

**~25 SFM+-owned variables are misprefixed `fza_ah64_*`** (initialized at `fn_init.sqf:26-57`): `forceTrimPos{Pitch,Roll,Yaw}`, `forceTrimInterupted`, `att/alt/hdgHold*`, `fmc{Pitch,Roll,Yaw,Coll,Trim}On`, `stabilatorPosition`, `IAFSInstalled/On`. These are a rename, not a re-architecture.

**Zero-risk quick wins** — misprefixed with *no* external consumers: `fza_ah64_stabilatorPosition`, `fza_ah64_aircraftEngineInitialised`, `fza_ah64_sfmPlusInitialised`.

### Write-holes to close

External code writing SFM+ internals — each needs an explicit API:

| Writer | Target | Replacement |
|---|---|---|
| `controls/fn_coreControlHandle.sqf:302,367,375-381` | `kbStickyInterupt`, `cyclic*Value`, `pedalYawValue`, `prev*Value` | `fnc_resetInputState` |
| `controls/fn_eventGetIn.sqf:39` | `fza_sfmplus_previousTime` | `fnc_resetClock` |
| `fire/fn_update.sqf:35,38` | `engState` | `fnc_setEngineState` |
| `systems` drivetrain ×3 (`fn_drivetrainNoseGearbox1/2.sqf:106`, `fn_drivetrainTransmission.sqf:87-88`) | `randomTq` | `fnc_setTorqueNoise` |
| `mpd/fn_pageACUtilHandleControl.sqf:19-35` | `fza_ah64_fmc*On` | `fnc_setFmcChannel` |
| `fuel` ×3 files, 24 writes | `fza_sfmplus_*FuelMass` | **Absorbed** (§3.4) — no setter; SFM+ owns it outright |
| `weapons:31`, `missionplanner:1018`, `controls` mag swap | mass recompute | `fnc_onStoresChanged` / `fnc_onLoadoutChanged` |

Note the mass rows differ in kind from the others. Elsewhere an external owner keeps its state and pushes through a setter; for mass there is no external owner left — SFM+ holds the state and outsiders only signal that something changed.

---

## 6. Plan

Phases 1–5 are what "drop-in" actually means; everything after is API quality.

### Phase 0 — Dead code & declarations *(low risk, do first)*
Delete the dead fixed-update scheduler, or implement `fnc_coreFixedUpdate` if fixed-timestep is wanted (§8 decision 1). Fix `missionplanner` ×6 → `fza_fuel_fnc_fuelSet`. Correct `requiredAddons[]`. Fix `HitHRotor`/`hithrotor` and `debugDrawline`/`debugDrawLine` casing; resolve the `fza_ah64_sfmplusSpringlessPedals` vs `...sfmPlusSpringlessPedals` duplicate; normalise `Fza_SfmPlus` config casing. Remove dead `playSound` lines.

### Phase 0.5 — The rename *(do once, isolated, no behaviour change)*
`addons/fza_ah64_sfmplus` → `addons/bmkhs_helisim`; `fza_sfmplus_*` → `bmkhs_*` across 1453 occurrences (1146 in-module, 307 across 48 consumer files); CfgFunctions tag `FZA_sfmplus` → `bmkhs`; `$PBOPREFIX$`; the 8 consumer `#include` paths. **Module-scoped only** — `.hemtt/project.toml`, stringtables and the other 16 addons are untouched. Land before the structural phases so later work does not churn names twice. Coordinate with decision #8. **MP caveat (§7) applies at full force.**

### Phase 1 — Repatriate config and settings *(blocker #1 and #3)*
Move `sfmplus.hpp` into `addons/fza_ah64_sfmplus/config/`; remove the include from `controls/config/cfgVehicles.hpp:27`. Keep the `Fza_SfmPlus` class name so mpd/missionplanner/customise are unaffected. Move the 15 CBA settings into an SFM+ `XEH_preInit.sqf`.

### Phase 2 — Own the loop, input, and base class *(blocker #2)*
Register `coreUpdate`/`coreUpdateFlightModel` from SFM+'s own preInit/EH instead of `controls/XEH_preInit.sqf:245`. Move the `CfgUserActions` `onAnalog`/`onActivate`/`onDeactivate` bindings into SFM+. Make timing internal. **Replace the three `fza_ah64base` bindings** (`extendedEventHandlers.hpp:8`, `fn_analogHandler.sqf:2`, `fn_nonAnalogHandler.sqf:2`) with `Helicopter_Base_F` + an opt-in config property (§0.6) — this is the only external dependency that cannot be vendored. Normalise `fn_prestonPedal.sqf:43`, which reads vanilla `inputAction` directly and bypasses the action layer.

### Phase 3 — Vendor helpers, cut hard dependencies
Vendor the ~330 lines from `common` into `fza_sfmplus_fnc_*`, plus `fza_fnc_animSetValue` (47) and `fza_audio_fnc_flightTone` (27). Copy the 7 `SYS_*` macros. Move quaternion math in and repoint `common/fn_rotateVector.sqf` — breaks the circular. Leaves only vanilla `BIS_fnc_*` and CBA.

### Phase 4 — Absorb the mass model *(the §0 rule)*
Move the mass-bearing fuel logic (~493 lines: `fuelUpdate`, `fuelSet`, `fuelVariables`) into SFM+; split pump/crossfeed *state* out to the optional panel addon. Remove the per-frame fuel calls from `fn_coreUpdate.sqf:52-53`. Replace the three external mass mutators with `fnc_onStoresChanged`/`fnc_onLoadoutChanged` notifications. This also fixes the broken `fza_sfmplus_fnc_fuelSet` calls by making the function real.

### Phase 5 — Layer interface / optional-input shim *(makes systems optional)*
Implement defaults for the 8 inputs in §3.3 plus `emerHydOn`/`engineOverspeed`, with optional setters. Add `fnc_setTorqueNoise` (default 0) so the drivetrain coupling becomes a core-owned input. **This is the layer interface** (§0.6) and the point at which SFM+ is drop-in with vanilla systems. Add a core-only regression pass.

### Phase 6 — Externalise the rotor and remaining hardcoded physics *(framework blocker)*
Move the ~33 hardcoded rotor constants (`fn_simpleRotorMain.sqf:54-63` geometry + tuning scalars, `fn_simpleRotorTail.sqf`, BET, transmission) into the config class. Restructure to nested subclasses (§0.5). This is what makes SFM+ usable on a non-AH-64 airframe.

> **Partly done.** The simple rotor, engine, mass, fuel, fuselage, wings and the
> flight control PIDs all read from config. The BET rotor's geometry, Preston
> AI's gains and the atmosphere constants are what remain; see
> `HELISIM_CONFIG_PLAN.md` for the current state.

### Phase 7 — Config validation and defaults *(framework blocker)*
Give every config read a documented default and add a startup validation pass reporting missing/invalid entries by name. Normalise the `fza_sfmplus` / `Fza_SfmPlus` casing. Without this, modder config errors surface as mysterious handling bugs.

### Phase 8 — Model contract
Hitpoint and animation-source names into `class Model` (§0.5 makes option (b) the natural path); guard every read so a missing source degrades cleanly.

### Phase 9 — Publish the output API
Implement the getters in §5; migrate pure readers (mpd, ihadss, ufd, animation, auxtank, wca, fcr) onto them. Keep raw variables as a deprecated shim. **These become the third-party display contract — stability matters more than elegance.**

### Phase 10 — Move flight-model instruments in
`controls/functions/ui/fn_ctrlVisUpdate.sqf` + `uiConfig/RscCtrlVis.hpp` → SFM+ (**the control indicator** — reads 13 SFM+ vars and calls `fnc_getInterpInput`). `controls/functions/avionics/fn_avionicsSlipIndicator.sqf` → SFM+ (pure SFM+ aero). Bring the `fza_ah64_ctrlVisToggle` handler and colour-scheme setting along. Both keep draw3D registration, from the SFM+ side. Doubles as the reference implementation of a display built on the output API.

### Phase 11 — Close remaining write-holes and rename
Implement the non-mass setters in §5. Rename quick wins first, then the ~22 shared FMC/trim vars as an isolated mechanical commit.

### Phase 12 — Absorb systems as gated subsystems *(§0.6)*
Move all 1203 lines of `fza_ah64_systems` into `fza_ah64_sfmplus` as config-gated subsystems: drivetrain, hydraulics, electrical (+APU), repair. Rename `fza_systems_*` → `fza_sfmplus_*`. Convert the `headers/systems.hpp` `#define`s into config properties under each subsystem class. Wire each gate's "off" path to the §3.3 neutral defaults so the switches and the drop-in fallbacks are one mechanism. Zero-coupling subsystems first (hydraulics, APU, repair, interact); drivetrain last since it needs Phase 5's `setTorqueNoise`.

### Phase 12.5 — Split Core from the pack *(§0.7)*
Separate the codebase into **BMKHS Core** (airframe-agnostic engine) and **`fza_ah64_helisim`** (the AH-64 pack: config class, model bindings, entry point). Core binds `Helicopter_Base_F`; the pack opts the aircraft in. Mark `headers/` as published API. **Acceptance test: building the AH-64 pack requires zero changes to Core** — if it doesn't, the boundary is wrong and needs another pass before shipping.

### Phase 13 — Documentation
Config reference (units, frames, sign conventions, defaults, ranges) for Core **and each subsystem**, output API reference, published header reference, model contract, subsystem-gating guide covering the FM-only case, and — most importantly — **a "build your first `XXX_helisim` pack" guide**, since the pack is the designer's actual entry point. The AH-64 pack doubles as the worked example. Per §0.5, not shippable without this.

---

## 7. Risks

- **Config move (Phase 1) touches every flight characteristic.** A typo silently changes handling rather than erroring. Diff the parsed config before/after and re-verify against the tuned-table baseline.
- **Absorbing fuel (Phase 4) changes mass every frame.** `fn_fuelUpdate.sqf` is 369 lines of burn, transfer and gravity-feed logic feeding straight into `setMass`/`setCenterOfMass`. A subtle error shifts CG in flight and reads as a handling regression, not a fuel bug. Verify gross weight and longitudinal CG against the pre-move build across a full burn-down, not just at spawn.
- **Externalising the rotor (Phase 6) is the highest-risk change in this plan.** Those ~33 constants are the *tuned* values behind the current handling, arrived at over months. Moving them to config must be provably lossless: extract the exact literals as the AH-64 config values, then verify the aircraft flies identically before touching structure. Do not combine externalisation with retuning.
- **Config move (Phase 1) touches every flight characteristic.** A typo silently changes handling rather than erroring. Diff the parsed config before/after and re-verify against the tuned-table baseline.
- **Absorbing fuel (Phase 4) changes mass every frame.** `fn_fuelUpdate.sqf` is 369 lines of burn, transfer and gravity-feed logic feeding straight into `setMass`/`setCenterOfMass`. A subtle error shifts CG in flight and reads as a handling regression, not a fuel bug. Verify gross weight and longitudinal CG against the pre-move build across a full burn-down, not just at spawn.
- **MP desync on rename (Phase 11).** Many shared vars are network-synced (`setVariable [..., true]`). A rename splits compatibility across a version boundary — clients and server must update together. Never fold this into behavior changes.
- **Published APIs are hard to change.** Once third-party displays bind to the §5 getters and modders ship configs, both become compatibility surfaces. Version them from day one and treat the schema as public.
- **`fn_getEffInput` duplication.** `animation/fn_getEffInput.sqf:5` reimplements `fnc_getInterpInput` rather than calling it. Until Phase 9, interp changes must be mirrored by hand or animations silently desync.
- **`onGround` is the most widely used export.** Signature changes have a wide blast radius.
- **Vendoring forks the helpers.** SFM+ and the rest of the project would carry separate copies of `linearInterp`/PID; bug fixes must be applied twice. Acceptable for encapsulation, but a real maintenance cost worth naming.
- **HEMTT rebuild discipline** throughout — recompile with no stale `.sqfc`, close Arma before rebuilding.

---

## 8. Decisions needed

1. **How far does rotor configurability go?** Externalising geometry (radius, chord, mass, blade count, RPM, hinge offset) is clearly required. The question is the *tuning scalars* — `_pitchTorqueScalar`, `_baseThrust`, `_inducedVelocityScalar`, flapback gains. Exposing them makes the framework genuinely general but hands modders knobs that can produce nonsense; keeping them internal means SFM+ only really models AH-64-like helicopters. Recommend exposing with documented ranges and validation warnings.
2. **Which rotor model is the framework target?** Two exist — BET (`functions/rotor/`) and simple (`functions/simpleRotor/`), selected by `fza_ah64_sfmPlusRotorModel`. Per prior work BET is the intended path. Supporting config authoring for *both* roughly doubles the Phase 6 surface. Recommend making BET the documented framework path and treating simple as legacy.
3. **Fixed-timestep or per-frame?** The fixed-step scheduler is dead code; the model runs per-frame today. On a flight-model tuning branch this is a physics question, not cleanup: reinstate a fixed step (better determinism, matches the `*deltaTime` impulse convention) or commit to per-frame and delete the corpse. **Framework-relevant:** third-party aircraft will have different mass/inertia, and per-frame integration is less forgiving across that range.
4. **Where exactly is the fuel cut line?** §0 settles that mass-bearing fuel logic is core. What remains negotiable is `fuelMgmt` (116 lines): pump switch and crossfeed *state* is arguably systems/panel, but the transfer it causes is core. Recommend absorbing the transfer and leaving switch state outside.
5. **Vendor `common`, or accept it as a shipped base dependency?** Recommend vendoring (~330 lines) — a framework with a hidden second dependency is not drop-in.

6. **What is the pack's opt-in mechanism?** A config property on the vehicle class (`bmkhsEnabled = 1`), an inherited marker class, or an init call? Recommend the config property — declarative, greppable, and works before any code runs.
7. **Does CBA stay?** Currently one function (`CBA_fnc_simplifyAngle`) plus the settings framework. If "zero dependencies" is literal, inline the function and move the 15 settings to config. Recommend keeping CBA — near-universal in Arma modding, and the settings UI is real value — but it is your call.
8. **What happens to the donor addons?** Absorbing `systems` and `fuel` leaves the existing AH-64 addons either deleted or reduced to thin panels. This is now urgent because it gates the Phase 0.5 rename: the AH-64 project is the biggest consumer of `fza_sfmplus_*` symbols, so its fate determines whether the rename is one commit or two.
~~9. Does Core keep an `fza_` prefix?~~ **Settled:** Core is `bmkhs_helisim` (BradMick's HeliSim), the AH-64 pack is `fza_ah64_helisim`. Both persist after the refactor.

Note §4's old question — airframe-agnostic vs AH-64-specific — is now **settled by principle 2**. A config-authoring framework implies configurable model contracts.

---

## 9. Suggested order

**Milestone 0 — identity:** Phase 0 (dead code) → 0.5 (rename). Cheap, mechanical, and everything downstream is cheaper once names are final.

**Milestone 1 — self-contained core:** Phase 1 → 2 → 3 → 4 → 5. The FM loads and flies in a bare project with only CBA and a conforming model, owning every term in its own mass budget, running on vanilla systems.

**Milestone 2 — framework:** Phase 6 (externalise rotor) → 7 (validation) → 8 (model contract) → 9 (output API). A modder can now author a new aircraft through config and read FM state for their displays. **This is where the deliverable becomes what you actually want.**

**Milestone 3 — full containment:** Phase 12 (absorb systems) → 12.5 (split Core from pack). **BMKHS Core exists as a product**, with the AH-64 as its first pack.

**Milestone 4 — polish:** Phase 10 (instruments) → 13 (documentation). Phase 13 is not optional; an undocumented framework is unusable by its intended audience.

**Defer until flight-model tuning settles:** Phase 11 bulk renames.

### Sequencing notes

Phase 12 sits late deliberately, but §0.6 shows it could move earlier — hydraulics, APU, repair and interact have **zero** SFM+ coupling and can be absorbed at almost any point. Only drivetrain depends on Phase 5's `setTorqueNoise`.

Phase 0.5 (rename) is placed first on purpose. It is pure churn with no payoff of its own, which makes it tempting to defer — but every phase after it would then be written against names that later change, and the 1453-occurrence sweep would run twice.

### Verification gate

Make containment testable rather than aspirational. After Phase 12.5:

1. **Core's `requiredAddons[]` lists nothing but vanilla + CBA.**
2. **No file under Core references `fza_ah64_*`, `fza_systems_*`, `fza_fuel_*` or `fza_audio_*` symbols, or any `\fza_ah64_<other>\` include path.**
3. **No `isKindOf` against an airframe-specific class anywhere in Core.**
4. **Building the AH-64 pack requires zero Core changes** (§0.7).

The first three are greps; run them in CI so containment cannot silently regress. The fourth is the real acceptance test and can only be judged by doing it — which is why the AH-64 pack should be built as a genuine pack, not grandfathered in.

---

## 10. Branch strategy

**Base:** `BMKHS-HeliSim-Core-Refactor`, cut from `HeliSimTuner`.

This refactor is long-running and touches nearly every file in the module, so it must not share a branch with tuning work. Three concrete reasons:

1. **The rename poisons diffs.** Phase 0.5 rewrites 1453 symbols. Any tuning commit landing after it conflicts textually with every pre-rename change, and `git blame` on the flight model becomes near-useless for the affected lines.
2. **Physics changes must stay separable.** Phases 4 (fuel absorption) and 6 (rotor externalisation) can silently alter handling. If they interleave with deliberate tuning, a handling regression cannot be attributed to either.
3. **MP compatibility breaks mid-way.** The rename changes network-synced variable identities. The branch is unshippable between Phase 0.5 and a completed migration of all consumers.

### Working shape

Land each phase as its own commit (or short-lived sub-branch off this one) with a subject naming the phase — `Phase 0.5: rename fza_sfmplus_* -> bmkhs_*`. The phases were written to be individually revertible; keeping that property is worth the discipline, because Phases 4 and 6 are the two most likely to need backing out.

Two commits should stay strictly mechanical, with **no** behavioural change mixed in:
- **Phase 0.5** (rename) — pure symbol substitution.
- **Phase 11** (bulk variable renames) — same reason.

Both are trivially reviewable when isolated and near-impossible to review when combined with logic edits.

### Merging back

Do **not** rebase onto `HeliSimTuner` after Phase 0.5 — replaying tuning commits across the rename resolves conflicts by hand, hundreds of times. Merge instead, and prefer merging *tuning into the refactor* periodically to keep drift small, rather than the reverse.

The natural merge point back to `master` is after Milestone 3 (Phase 12.5), when Core and pack both exist and the §9 verification gate passes. Merging earlier ships a half-renamed module with broken MP compatibility.

### Baseline for regression testing

Tag or note the pre-refactor commit — currently `4ef8c998e` on `HeliSimTuner`. Phases 1, 4 and 6 all require verifying that handling is unchanged, and that needs a fixed reference build to compare against. Per §7 the specific checks are: parsed-config diff (Phase 1), gross weight and longitudinal CG across a full fuel burn-down (Phase 4), and identical handling from the extracted rotor literals (Phase 6).
