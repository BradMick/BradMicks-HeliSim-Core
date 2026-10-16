# Flight controls refactor - plan

Status: IN PROGRESS.

## Where we are (2026-10-15, end of session)

Nothing below is committed. Repos with uncommitted work: Core, H-60, MH-47G, AH-64D, Bo105, EC665.
All pass `hemtt check`.

**Step 1 - trim layer: DONE, pushed** (Core caf7301, MH-47G 2fbf3fe). AH-64D force trim off untested.

**Step 2 and today's work - built, not committed. User flew the UH-60M, Bo105 and EC665: all
operate and fly as expected. MH-47G and AH-64D not flown on this build yet:**
- Beep with a hold on moves the trim and the hold reference (trimGain deg). Holds never move the
  trim - only the pilot does (beep, or force trim and position the stick).
- Fix 1: every sub-mode change captures the new sub-mode's reference. Fix 2: derivative cleared on
  a sub-mode change, integrals kept. Laws run on this frame's sub-mode.
- `AttitudeHold >> subModes[]` required (pos/vel/att; undeclared falls to nearest declared).
  AH-64D, Bo105, EC665 all three; UH-60M, MH-47G `{"att"}`. FD HVR may use pos regardless.
- `AttitudeHold >> alwaysOn = 1` on UH-60M and MH-47G (MH-47G not flown yet).
- `HeadingHold >> subModes[]` required (hdg/yaw/trn; ntt is auto pedal only). AH-64D, Bo105, EC665
  all three, unchanged. UH-60M, MH-47G `{"hdg","trn"}`: hdg at every speed, trn in turns (7/3 deg),
  heading re-captured after. No blending change - it would change the Apache.
- FD beep: fore/aft moves the IAS target (iasAccelKts/s), lateral the HDG target (turnRateDps),
  unsnapped (fmcFdTarget _snap). NAV ignores it. On an FD axis the beep doesn't move the cyclic or
  the attitude hold reference. Generic - any aircraft with a flight director. Flown, works.
- FD speed floors: optional `class Floors`, {engage, drop} KTAS per mode. UH-60M {50, 48} on alt,
  altp, ias, hdg, nav (not ralt, hvr).
- Config errors go to the RPT and on screen once (bmkhs_fnc_utilConfigError, all 13 sites). A
  missing required field switches off only that feature. Config versioning/migration was built and
  removed (user: bloat).

**Watch:** the Workshop copies of the Bo105 and EC665 lack subModes[] - attitude and heading hold
off until republished (the local builds are fine).
UH-60M/MH-47G pedal breakout is picked by attitude hold sub-mode, so always 0.20 (att) - may be stiff
in the hover. Heading hold "gets confused" (2026-10-12) not seen since.

**Next:**
1. Fly the MH-47G (alwaysOn, att only, hdg/trn) and the AH-64D (must be unchanged).
2. Collective beep on ALT/RALT - two new input commands, 4 fps then 16 fps. Agreed, not built.
3. Commit when the user validates; then step 3.

**Agreed, not built:** UH-60M trimGain {50, 60} (TM 5 deg/s pitch, 6 deg/s roll; MH-47G numbers
open). 1 deg/s coordinated turn from lateral beep - user said useful, no design yet.

**Future:** IAS (EAS = TAS x sqrt(bmkhs_rho / ISA sea level), CAS ~ EAS, optional per-aircraft
position error table, own variable; everything else stays KTAS). Stabilator manual mode. FD VS and
GO AROUND. SAS channels.

## 1. What the aircraft need

| Aircraft | Control path | Trim | Beep | Holds | Hold reference | Flight director |
|---|---|---|---|---|---|---|
| Huey, Bell 206, R-22 | boost or mechanical | force trim (206, Huey) or none (R-22) | no | none | - | no |
| AH-64D | SAS | force trim | **no** | attitude, altitude, heading - key engaged | captured on engage and force trim release (Core today) | no |
| UH-60M | SAS | force trim + trim actuators | yes | FPS attitude hold **always on**, altitude, heading | captured; beep moves the stick and the reference | yes |
| CH-47 / MH-47G | AFCS / DAFCS | centering device (magnetic brakes) | yes | pitch attitude, airspeed (above 40 kt), bank angle, heading, altitude | pitch and airspeed: **the stick position is the reference**; bank and heading: **re-captured when the rate settles** | heading select |
| Bo 105, EC665 | SAS | force trim | no | as the AH-64D - key engaged | captured on engage and force trim release | no |

CH-47D, from the Army AFCS course (Fort Rucker, Oct 2006):
- Fore/aft AFCS trim drives the pitch CCDA trim motor; **the stick moves**; it works **with the AFCS off**.
- Airspeed hold: "maintain a constant airspeed for any given stick position at airspeeds above 40 knots".
- Left/right AFCS trim acts on the roll ILCA; **the stick does not move**. Bank angle hold drops while it is
  used and resumes when roll rate < 1.5 deg/s. Above 40 kt it also drops heading hold, which resumes
  when yaw rate < 1.5 deg/s and bank < 1.5 deg. Lateral trim is off while heading select is engaged.

General (Navy AFCS, BK 117 C-2, SpinningWing, FAA AIM):
- Beep is held, not stepped: the stick or reference moves at a **trim rate** (Navy: ~5 deg/s pitch,
  ~6 deg/s roll attitude; above 50 kt the airspeed reference at 6 kt/s), after a short trim lag.
- With attitude hold, beep moves the attitude reference; with an upper mode (heading, airspeed)
  engaged it moves that mode's reference.

## 2. The problem in Core today

- One trim state: `bmkhs_forceTrimPos{Pitch,Roll,Yaw}` - the stick's zero-force position. Written by
  force trim release (`fn_fmcForceTrimSet`), force trim reset, casual auto attitude, and now beep.
- One reference state per hold (`bmkhs_attHoldDesired{Pos,Vel,Att}`, `bmkhs_altHoldDesiredAlt`,
  `bmkhs_hdgHoldDesiredHdg`), captured only on engage and force trim release.
- The two are unrelated. Moving the trim (beep) leaves the reference behind, so the hold fights the
  stick with its authority. That is right for the AH-64D (no beep) and wrong for the UH-60M and CH-47.
- Engage/release rules live in each hold's own file (key toggle, force trim, collective band, pedal
  breakout, ground), so per-aircraft behaviour can only be added by more special cases.

## 3. Target design

Three layers, each declared per aircraft, each optional:

1. **Trim** - where the controls sit with no force on them. Force trim, beep, centering device.
2. **Holds** - what each hold flies to (its **reference**) and when it is engaged.
3. **Coupling** - how a hold's reference follows the trim and the pilot, per axis.

### 3.1 Trim layer - `class Trim` (flight controls, beside `FMC`)

```
class Trim {
    forceTrim = 1;              //force trim / centering device present (0: the stick has no trim hold)
    class Beep {                //undeclared = no beep (AH-64D, Huey, R-22)
        gate[]   = {};          //what must hold for it (trim actuators powered)
        rate[]   = {0.10, 0.10};//{pitch, roll} stick trim travel per second, held
        lag      = 0.1;         //s, press to movement
        moves[]  = {"stick", "stick"};  //{pitch, roll}: "stick" moves the trim position;
                                        //"reference" moves the hold reference, stick still (CH-47 roll)
    };
};
```

- Beep becomes **rate while held**, not a step per press (replaces the built `beepTrimStep`).
- Works with no FMC at all (Bell 407 style retrofit) and with the FMC off (CH-47 fore/aft).
- `bmkhs_fmcTrimOn` is the force trim switch - the AH-64D sets it from its A/C UTIL page through
  `fmcSetChannel`. Core never reads it today, so turning it off does nothing. Core will honour it:
  off = no force trim hold and no beep (the controls float, the trim position stops being written).
  The UH-60M TRIM switch uses the same variable.
- Force trim hold / release / reset stay as they are, moved under the trim layer's functions.

### 3.2 Holds - reference source per axis

Each hold class gains how its reference is set, per axis:

```
class AttitudeHold {
    alwaysOn  = 0;                       //built (1.4.0, uncommitted)
    reference = "capture";               //"capture" (AH-64D, UH-60M) or "trim" (CH-47 pitch)
    trimGain[] = {10.0, 10.0};           //"trim" only: deg of attitude per unit of stick trim, {pitch, roll}
    recapture[] = {0, 0};                //deg/s: re-capture when the rate settles below (0 = never). CH-47 roll 1.5
    ...
};
class HeadingHold {
    recapture = 0;                       //deg/s yaw rate (and bank) below which heading re-captures. CH-47 1.5
    ...
};
```

- `capture` (Core today): the reference is captured on engage and force trim release. Beep with
  `moves = "stick"` moves the trim and **also** the reference, by the same attitude the trim travel
  equates to (`trimGain`) - UH-60M.
- `trim`: the reference is computed every frame from the trim position (`trimGain`), so anything that
  moves the trim - beep, force trim release - moves the reference with it.
  Above an airspeed threshold the same coupling can drive the airspeed reference instead (CH-47
  airspeed hold) - see 3.3.
- `recapture`: after a pilot input on that axis (lateral stick, lateral beep, pedal), the hold stays
  released until the rate settles, then captures where the aircraft is.

### 3.3 Airspeed hold (CH-47)

Today `vel` holds ground velocity. The CH-47 holds **airspeed for a given stick position** above 40 kt.
Add an `airspeed` sub-mode to AttitudeHold, selected above `airspeedAboveKts` (declared only by
aircraft that have it), whose reference is `trimGain`-coupled airspeed (kt per unit of stick trim).

### 3.4 One engage/release state machine per hold

Replace the scattered rules with one function per hold that evaluates, every frame, in order:
`declared -> gate -> channel on -> alwaysOn / key state -> releases -> recapture -> reference`.
Releases are declared, not coded per aircraft:

```
releases[] = {"forceTrim", "ground", "pedalBreakout", "collBand", "tqLimit", "lateralStick", "lateralBeep"};
```

Each name maps to one condition function in Core. An aircraft lists the ones its hold has.

### 3.5 Flight director

Unchanged in principle: a coupled FD axis replaces that axis's hold output (`fn_fmc.sqf`). New: a beep
on an FD-coupled axis moves the FD target for that axis, otherwise the trim.

## 4. Work, in order

Each step is built, tested in game on the aircraft named, and committed before the next.

| # | Step | Files | Test aircraft |
|---|---|---|---|
| 1 | **Built.** Trim layer: `class Trim`, beep as rate while held (key down/up), lag, gate, `moves[]`; Core honours `bmkhs_fmcTrimOn` (force trim off). Remove `beepTrimStep`. | new `trim/fn_trimVariables`, `fn_trimBeep` (per frame), `fn_trimForceHold/Release/Set/Reset` (moved from fmc), `fn_inputControlHandle`, `CfgUserActions` (beep keys stay), `fn_coreUpdate` | MH-47G (beep), AH-64D (no beep; A/C UTIL force trim off now takes effect) |
| 2 | Reference source `capture` + beep moving the reference (`trimGain`) | `fmc/fn_fmcAttitudeHold`, `fmc/fn_fmcVariables` | UH-60M |
| 3 | Reference source `trim` for pitch | `fmc/fn_fmcAttitudeHold` | MH-47G |
| 4 | Airspeed sub-mode | `fmc/fn_fmcAttitudeHold`, `fmc/fn_fmcVariables` | MH-47G |
| 5 | `recapture` for roll and heading; lateral beep with `moves = "reference"` | `fmc/fn_fmcAttitudeHold`, `fmc/fn_fmcHeadingHold` | MH-47G |
| 6 | Hold state machine + declared `releases[]`; fold in `alwaysOn` (built) | `fmc/fn_fmcAttitudeHold`, `fn_fmcAltitudeHold`, `fn_fmcHeadingHold`, `*Enable`, `fn_fmcHoldModesDisable` | AH-64D, UH-60M, MH-47G regression |
| 7 | FD + beep | `fmc/fn_fmcFlightDirector`, `fn_trimBeep` | UH-60M |
| 8 | Casual mode: auto attitude keeps writing the trim; check it against `trim` references | `input/fn_inputAutoAttitude` | MH-47G, Bo 105 |
| 9 | Every pack declares every field (1.4.0 list in TODO.md); docs (`fmc.hpp`, AIRCRAFT_GUIDE) | packs, docs | all |

Defaults keep every existing pack flying as now: no `Trim` class = force trim only, no beep;
`reference = "capture"`, `recapture = 0`, `releases[]` = today's per-hold rules.

## 5. Already built this session (1.4.0, uncommitted) and where it lands

- `alwaysOn` for the holds - kept; becomes one input to the step 6 state machine.
- `class HydraulicBoost` - kept; independent of this plan (control path, not trim).
- Beep trim keys + `fn_fmcBeepTrim` + `beepTrimStep` - the keys stay; the step model is replaced in step 1.

## 6. Decisions (user, 2026-10-12)

1. Beep is a **rate while held**.
2. UH-60M beep and the FPS reference: **unknown** - no UH-60 source found. Default until a source turns
   up: beep moves the stick trim and the attitude reference together (`capture` + `trimGain`), as the
   general sources describe (BK 117 C-2, H145, Navy AFCS). Verify against TM 1-1520-237-10.
3. CH-47 / MH-47G: build it as the CH-47D AFCS course says - fore/aft beep moves the stick, pitch and
   airspeed referenced to the stick (`trim`); lateral beep reference-only, bank re-captures below
   1.5 deg/s; heading re-captures below 1.5 deg/s yaw rate and 1.5 deg bank above 40 kt.
4. With the FD coupled on an axis, beep **moves the FD target** on that axis.
5. Bo 105 and EC665: **no beep**; hold modes behave as the AH-64D's - key engaged, `capture`
   references, force trim release re-captures (as today).
