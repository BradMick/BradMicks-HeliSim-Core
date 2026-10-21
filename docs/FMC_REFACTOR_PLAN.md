# Flight controls refactor - plan

Status: IN PROGRESS.

## 2026-10-20 session (uncommitted, all six built)

- H-60 YawToRoll at 0.25x the rig table (full rolled the 60 over). Flown: rolls 2-4 deg AWAY from
  the pedal now - next try 0.5x.
- Auto pedal rebuilt as a pilot's foot (Core fn_inputAutoPedal.sqf). An older feet model (lag +
  rate) was deleted 2026-09-23 in 3475b82 - check history before rebuilding anything.
  - Foot: critically damped, AUTOPEDAL_FOOT_TIME 1.5 s to 95%. Sits on the pilot's pedal while on
    it; on release starts there at rest.
  - Hover release: eases to trim - AUTOPEDAL_FEEL x yaw rate (settles where the yaw stops);
    heading captured at 2 deg/s or foot within AUTOPEDAL_FOOT_ARRIVED of that target.
  - Slowing into the hover captures heading at once. Every hand-off bumpless: incoming PID's
    integral preset so its first output = foot position.
  - Aero (ball) only above 50 ft AND 50 kt ground speed; back below 45 ft OR 45 kt. All auto pedal
    gates are ground speed (user).
  - Analog spring-back faster than a foot = release, same as a key.
  - Removed: dead keyboard fade (params-scoping bug threw its result away), 5 write-only vars,
    stale feet comment. Flight log gains pedalInL / pedalInR (raw).
  - User: "not bad", active testing. Known: a key tap at cruise = ~1 s of full pedal (key is 0/1).
    Open: keys-only ramp for the pilot's own pedal; nose-to-tail/aero gains never retuned for the foot.
- Analog right pedal capped at 0.6: hardware stop on the controller - not Core.
- Flight log: new "type" column (typeOf) after id.
- After save point 6e88ca0: LCT goes to GND (0) on the ground (Core, tandem, stateOnGround);
  MH-47G LCT flat to 40 kt (40 kt row = hover row, per the Chinook LCTA chart); MH-47G top
  fuselage panels moved onto the front panel's plane (1.5 deg tilt kept) - pitch settles now.
- Pack tuning, flown, user happy ("where we need to be"):
  - UH-60M: lift coefs roll 0.19 -> 0.14, pitch 0.72 -> 0.60; NTT kp 0.03 -> 0.015. Mast tilt
    went to 0 (rig: -2 deg at 90 kt) then back to -3 (3 deg forward, user) - rig: +0.6 deg at 90 kt.
  - EC665: NTT kp 0.015, hover hdg {0.05, 0.0025, 0.025, 0.15} (was the AH-64D's).
  - Bo105: lift coefs roll 0.166 -> 0.116, pitch 0.315 -> 0.188; NTT kp 0.0075; hover hdg
    halved; Sas / pos / att pitch x1.7, roll x1.4 to restore their effect after the cuts.
  - Lesson: the AH-64D's auto pedal and hold gains were copied into the 60, Tiger and Bo105
    untuned - tune per pack, not in Core.

## Where we are (2026-10-16, end of session)

**How every step runs:** I propose the exact change (behaviour, TM line, files, any config value);
the user says "go" for that step - nothing else is a go; I build only that step, terse comments,
hemtt check, stop; the user flies it; next step only once it's right. No new config value without
the user's yes. No aircraft-specific branches - one generic system, the AH-64D the reference.

**Rules, 2026-10-16:** config stays minimal - a field only for what an aircraft has to tell Core
(what it has, gates, TM facts, gains). Every default and Core constant is a #define in fmc.hpp,
never a literal in an sqf. Gains are in pilot units, converted at load, required only for what
the aircraft runs. Force trim is not optional. Sources: the AH-64D, CH-47F (TM 1-1520-271-10)
and UH-60M (TM 1-1520-280-10) pages in the Claude memory folder (tm/); not the CH-47D course.

**Hold terms (AH-64D):** engaged = the mode is selected; active = it is doing work. A control past
its breakout makes that axis not active - no output, no fight. Back inside, it is active again and
returns the aircraft to its previous reference. Only force trim re-references. Breakouts are
fractions of control throw; the pedals' vary with speed (by attitude hold sub-mode), the cyclic and
collective do not.

**Plan, agreed 2026-10-16:**
1. DONE - tree safe: director override back to force trim only, fmc.hpp text corrected.
2. DONE - every FMC and trim default a #define in fmc.hpp; trim.hpp folded in and deleted; no
   value changed.
3. DONE, flown 2026-10-16 (AH-64D, UH-60M), works - cyclic breakout fly-through, per axis, every
   aircraft: AttitudeHold breakout 0.10 of cyclic throw (FMC_ATT_BREAKOUT). No sub-mode change
   while flying through; back inside, the axis returns to its previous reference (no re-anchor).
   Director cyclic axes the same, HVR included.
   That flight, UH-60M: spd entered at 51 kt in a -16 deg dive, captured it, ran to 119 kt (step
   4). Beep moved the roll trim 0.09 while the reference moved 0.9 deg - the hold resisted every
   beep, and sat at -20 deg bank saturated in cruise (step 6). HVR sank at -1500 fpm into a hard
   landing with no vertical mode engaged - open question: should HVR hold height on its own?
4. BUILT 2026-10-18, not flown - airspeed hold reference = the 60 TM's "airspeed at which the
   trim is established": captured at once on spd entry, engage, force trim release. After a beep,
   captured once settled or FMC_ATT_SPD_DELAY_S (0.1 s; 0.5 always timed out in flight).
   No pitch-target steps (they kicked the att loop's derivative, the felt hitch): beep start folds
   the speed loop into the attitude reference; capture keeps the attitude reference. spd pitch command, beeped reference
   included, within FMC_ATT_SPD_MAX_PITCH_DEG (10) of level - was +-10 about the capture attitude,
   which could not stop a dive entry. Watch: is 10 deg enough nose-down at top speed? UH-60M.
   FLOWN 2026-10-19 (UH-60M): felt good, no derivative kicks on 12 beeps.
4a. BUILT 2026-10-19, not flown - integrals that work. Every hold drifted: the 4th PID value
   (raw error-seconds clamp) was ~ki in every pack, so ki x clamp ~ 0 (pos 0.00005 cyclic).
   pidCreate's 4th value is now the LIMIT - most the integral can push, output units, whatever
   ki is (ki_clamp = limit / ki, stored as before; pidRun, governor, Preston code unchanged).
   Packs, all six: att, pos, FD hvr/pitch/yaw, hdg, auto attitude 0.1; rad 0.05; spdPitch 10 deg.
   Every other PID (engine 3.0, Preston, auto pedal, Sas yaw, trn, FD vs/ias/roll) restated to
   its exact old effect (ki x old clamp). bar still has ki 0. Position hold: closing speed
   FMC_ATT_POS_GAIN 0.2 /s x distance, max FMC_ATT_POS_MAX_KTS 3, through the velocity loop -
   replaces the 0.005 / 0.02 literal integral. Test on the UH-60M only for now.
   4a FLOWN 2026-10-19 (UH-60M): HVR now holds the spot (0.1 kt for minutes, was 1.9 kt drift).
   FD IAS/HDG/ALT/ALTP "dead" = the 60's floorKts 50 refusing them at a hover (silent). Open: keep
   the 60's floor? pack comment says KIAS, Core uses KTAS.
5. BUILT 2026-10-19, not flown - HVR in the hover. While HVR is on the attitude hold is pos at
   any speed (no att at 5 kt - that ran away backwards at 13 kt); while the director flies a
   velocity (slowing, beeped) pos stands by, loops clear, spot following. Hand-off at ref 0 and
   ground speed <= posBelowKts. HVR slow-down is IAS-style: velocity -> wanted pitch / bank
   (hvrPitch / hvrRoll now kt to deg, 60: {0.5, 0.05, 0, 5}), capped maxPitchDeg / maxBankDeg,
   eased at pitchRateDps / rollRateDps, flown by FD pitch[] / roll[] (was velocity straight to
   cyclic: +10 to -17 deg in 2 s). IAS / HDG no longer reset those loops while HVR runs. UH-60M.
   Flown 2026-10-19: good overall; slow-down weak and slow (2 kt/s ramp, 0.5 deg/kt asked <= 8
   deg), hand-off at 4.8 kt then pos slammed full aft (14 deg, overshot 6 kt back). Tuned, not
   flown: 60 hvrDecelKts 4, hvrPitch / hvrRoll kp 1.0; hand-off only once stopped
   (FMC_FD_HVR_HOLD_KTS 0.5 kt). Next knob if still restricted: pitchRateDps (3, shared with IAS).
   Flown: slow-down good (32 -> 2 kt in 12 s on the 15 deg cap) but crawled 2 -> 0.8 kt for 27 s,
   no hand-off; 60 hvrPitch / hvrRoll ki 0.05 -> 0.2. Tiger "oscillation" = Workshop EC665 1.1.3.2
   (rad/s SAS gains, x57 under this Core) - RELEASE: Core 1.4.0 ships with every pack, never alone.
   All six flown on local builds 2026-10-19: PIDs good, SAS never at its limit (47 att hold at its
   pitch limit 38% - step 7). 60 HVR: crawled at 2 kt 15 s (wanted attitude was from level, the
   integral had to build the whole +4.5 deg hover attitude), then bobbed 15 s after hand-off (pos
   started from zero, dropping the director's 0.07 cyclic). Built, not flown: HVR wanted attitude =
   base + velocity loop, base = attitude at start corrected for its acceleration (pitch + atan(a/g),
   FMC_FD_ACCEL_FILTER_S 0.5 s); hand-off seeds pos's integrals with the director's cyclic.
   Flown: director 9.3 -> 0.5 kt in 5.5 s (fixed), but after a fly-through pos went back to the
   old spot at 11 kt: +26 deg, swung to 19 kt backwards, pinned 10 s. NOT DONE. Pos (velocity
   straight to cyclic, no attitude cap) saturates above ~2 kt error - wrong tool above a crawl.
   Built, not flown: the director flies ALL of HVR - slows (ref eased to 0), stops at 0.5 kt,
   captures hvrSpot, then flies back to it (FMC_ATT_POS_GAIN x distance, max
   FMC_ATT_POS_MAX_KTS) through the same capped attitude cascade. Fly-through returns to the
   spot; force trim re-references. Attitude hold stays off under HVR (Enable refuses); step 5's
   pos-forcing / standby / seeding removed - attitude hold HVR lines back to the save point.
   Flown: better, but went back to the OLD anchor after a fly-through (user wants a new one) and
   overshot 6 kt back (velocity integral built on the 15 deg cap). User: pos held perfectly before.
   REVERTED to the hand-off version (director slows on the capped cascade, seeded hand-off to pos
   at 0.5 kt, pos holds) plus: fly-through in HVR drops the anchor like force trim - the director
   slows from there and hands over on a new spot; HVR velocity integral held while on the cap.
   Flown: much better, still aggressive at the hand-off - handed over at 0.5 kt mid-flare (+8.6
   deg, integral wound up in the slow-down), and on a one-frame engage from the hover pos was
   seeded from the director's raw output while it was only ~10% blended in (pos pinned both axes).
   Built, not flown: hand-off only stopped AND settled (ground acceleration <= FMC_FD_HVR_HOLD_KTS
   per s); velocity integral waits for the stop (replaces held-on-the-cap); pos seeded from the
   cyclic applied last frame (bmkhs_fmcAttHoldCycPitchOut / RollOut). No new constants.
6. CLOSED 2026-10-20 - UH-60M trimGain stays {10, 10}; user: the trim gains are fine.
7. Hold saturated after force trim release - re-check after 3 and 6, fix only if still there.
8. Docs tidy, user validates, commit all six.
9. MH-47G flight director, from the 47F TM.
10. Build order below.
11. Once all the above is done: tune the auto pedal in every attitude hold regime - pos, vel and att.

**Save point committed 2026-10-15, not pushed:** Core fbcd7f8, MH-47G 6d6497a, Bo105 0ccd631,
EC665 1e1543e, H-60 5fcdf2f, AH-64D a58b5baf7 (H-60 and AH-64D on local branch helisim-fmc-subModes).

**Built 2026-10-16, uncommitted, hemtt check clean in all six, last version not flown:**
- Gains per pilot unit (Sas deg/s; pos, spd, FD ias, hvr kt; rad/bar ft; FD vs fpm), packs rounded
  to 2 significant figures (0.1-4.5 pct off the old values, AH-64D included).
- Airspeed hold: AttitudeHold sub-mode "spd" above spdAboveKts KTAS (47 40, 60 50), out 10 kt below.
  Captures the airspeed once it settles (0.5 kt/s) - on entry, after a beep, after force trim.
  Pitch within 10 deg of the settled attitude. Needs spdPitch[].
- FD: RALT/ALT/IAS/HDG engage on the present value, ALTP on its preselect. Beep in HVR slides the
  held ground velocity at 1 kt/s; back to nothing holds the new spot. HVR only at or below 40 kt GS.
  Force trim pressed: the director stands aside on the cyclic, resumes on release (HVR from where
  the aircraft is). Axes blend in and out over 1 s.
  floorKts replaces class Floors.
- Beep: a press belongs to what it started on (director target or stick) until released.
- Pruned: forceTrim, Beep lag / moves[], trimGainKts, AltitudeHold / HeadingHold alwaysOn, the
  spd / hvr tuning fields; subModes[] optional (defaults = the old behaviour); packs lost every line
  restating a default and gains for sub-modes they do not run.
- Flights 2026-10-16 (UH-60M) found the problems plan steps 3-7 address: the hold fighting the
  stick, spd running away from a captured dive, HVR flipping pos/att at 5 kt and banking 22 deg,
  beep moving the stick far more than the reference (trimGain 10), two departures.

**Watch:** the attitude hold still captures whatever attitude it has when a director pitch axis
drops (+8.6 deg decelerating in that flight). MH-47G has no class FlightDirector yet (the 47F has
one - modes from the TM).

**Next, in the user's order** - minimal fields, Apache unchanged:
3. Heading hold `trn` engage/exit. 47/60: steps aside on pilot action (lateral beep, force
   trim, stick or pedal displaced), resumes on bank and yaw rate below threshold (47 1.5 deg,
   1.5 deg/s; 60 2 deg, 2 deg/s for 2 s). Apache declares today's 7 deg engage / 3 deg exit.
4. Collective beep on ALT/RALT - two new input commands; fields for step per press, held rate, and
   a faster rate after a time (47 1 ft, 3 ft/s; 60 4 then 16 fps).
2. `armed[]` - sub-modes that run only once the pilot arms them with a key, inside their speed
   bands (47 TRC = vel below 9 kt, PH = pos below 1 kt). Apache doesn't list it - automatic.
5. FD coupled / command-cue switch (CPLR button, FD CPLR REL): uncoupled still computes commands
   and cues but doesn't move the controls.
6. Altitude hold source chosen by the pilot (47 IN/RA) - the armed[] idea on AltitudeHold.
Then fly the MH-47G and AH-64D, commit when the user validates.

**Agreed, not built:** 1 deg/s coordinated turn from lateral beep - user said useful, no design yet.

**Future:** IAS (EAS = TAS x sqrt(bmkhs_rho / ISA sea level), CAS ~ EAS, optional per-aircraft
position error table, own variable; everything else stays KTAS). Stabilator manual mode. FD VS,
GO AROUND, HOV/THOV. SAS channels (47 SYSTEM SEL). LCT manual mode. Torque limiting by backdrive
(47: 96% or 10-min limit, 14% floor, no downward backdrive below 100 ft AGL).

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
