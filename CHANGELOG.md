# Changelog

## 1.4.4.0

### Engine
- **Ground idle holds on hot days.** Below FLY the fuel control now meters in corrected fuel
  flow on corrected Ng, with the air the engine passes (`airflowTable`). In 1.4.3.0 fuel
  followed only the inlet's pressure, so on a hot day idle Ng sagged - as low as 51% on the
  Bo 105 in the Middle East - and the start dragged. Every aircraft now idles at its corrected
  idle Ng in every environment, a little higher on a hot day as a real engine does, and a
  start takes the same time on any day. Standard day and FLY are unchanged, so power
  available is too.
- **The start fuel schedule reads corrected Ng.**

## 1.4.3.0

### Engine
- **Starts no longer overtemp off sea level.** Below FLY the fuel control meters as Wf/P3 on
  corrected Ng - fuel follows the inlet's pressure - so a start or ground idle keeps its
  fuel-air ratio in thin air. Before, the same fuel met less air at altitude, and a start in
  the Asia or Central Asia environments ran past `startTgt` and destroyed the engine. A
  standard day meters exactly as before; FLY is unchanged, so power available is too.

### Tools
- **The rig carries Core's environments** - `ENVIRONMENTS` holds each setting's base altitude
  and FAT, as fn_environment, so a test runs where the game does.

## 1.4.2.0

**Update every HeliSim aircraft with this release.** The starter's `runawayNg` is replaced by
`motoringNg`; a pack that still declares `runawayNg` has no starter power. Use the matching
pack versions.

### Engine
- **Starts come out of the physics.** The spool's torque balance takes the turbines' power
  over speed, so the turbine, the compressor, the starter and the drag all act as torques. A
  start now takes the time a real engine does; idle and every steady state are unchanged.
- **The starter.** Full torque until its power limit, then power over speed - it keeps
  pushing, ever less, until self-sustaining. `motoringNg` is where a crank with no fuel
  settles; Core solves the starter's power limit from it.
- **The rotor turns as Ng rises.** The default T700 compressor map below idle had the
  compressor turbine taking more expansion than the compressor made, so the power turbine
  turned nothing until idle.
- **Engine display**: an engine with no Np overspeed trip (`npOvrspdProt = 0`) no longer
  stops the display with a zero divisor.

### Tools
- **The rig runs the engines a pack declares** - single-engine aircraft no longer need a
  second engine to fit.
- **`compressor_map.py`**: rows below the first rating keep the compressor turbine's share
  of each row's own pressure ratio, so the power turbine turns during a start.
- **The bench's dry-crank target is 25%** (it was an unsourced 42%).

## 1.4.1.0

### Flight controls
- **Nose-to-tail auto pedal holds the flight path, not the airflow.** In wind it kept the
  nose into the air instead of on the track, crabbing the aircraft over the ground. The
  tail now follows the nose, and an aircraft that settles in meets the ground straight.

### Flight log
- **Auto pedal columns**: on, mode (hdg / ntt / aero), hover heading captured, the held
  heading and the pedal it is holding.

## 1.4.0.0

**Update every HeliSim aircraft with this release.** Packs built for 1.3 misread 1.4's
flight-control gains and rotor config; use the matching pack versions (EC665 1.2.0,
Bo 105 1.1.0).

### Flight model
- **The rotor disc tilts as one.** Cyclic tilts the whole disc, and every blade's lift
  with it.
- **Flap back from physics.** The disc blows back with speed and leans with coning from
  the rotor's own advance ratio, inflow, coning and blade pitch - no tuned value.
- **Per-blade inflow.** Each blade's lift reads the air through its own part of the
  tilted disc: the forward blade lifts more than the aft, and holding the nose takes
  forward cyclic as speed builds.
- **Inflow costs angle of attack.** Air coming down through the disc in a climb or at
  speed takes lift away the way it does on a real blade, and collective brings it back.
  The VRS behaviour in a descent is unchanged.
- **Every aircraft refit** to its torque and attitude data on the new rotor.
- **Drivetrain drag** and free-turbine torque at the actual Np; the Ng limit follows each
  engine's own maximum, with optional overspeed trips and no hunting at the limit.

### Tandem rotors
- Fore/aft stick moves lift between the rotors; pedal tilts the discs opposite ways.
- **LCT**: longitudinal cyclic on a schedule of airspeed and altitude, at the actuator's
  rate, back to GND on the ground.

### Flight controls
- **Holds rebuilt.** Attitude hold with position, velocity, attitude and airspeed modes;
  heading hold that holds heading in the hover and centres the ball at speed, giving way
  to the pilot and coming back once settled; flight director RALT / ALT / IAS / HDG / HVR
  on aircraft that have one.
- **Cyclic fly-through**: move past the breakout and the hold steps aside on that axis;
  come back inside it and it resumes.
- **Trim**: beep trim moves at a rate while held; the force trim switch is honoured.
- **Altitude beep**: new keys Altitude Beep Up / Down - a tap steps, held climbs or
  descends the altitude hold.
- **Auto pedal** works like a pilot's foot, with smooth hand-offs between regimes.
- **Barometric altitude hold** now has the same authority as radar altitude hold
  (aircraft packs).

### Fixes
- **Repairs restore systems.** Fluids, engine failures and gearbox damage now reset on
  any repair - script, Zeus or ACE.
- **Getting out of a starting or running aircraft** no longer leaves the rotor behind:
  it keeps spooling with the engines, so getting back in no longer overtorques it.
- **Aircraft without systems** keep running when everyone gets out, and Engine off from
  the action menu now actually shuts the engines down.
- The control input visualiser closes when you leave the aircraft.
- Rotor sound controllers moved to 32 / 31.
- Rotor Model setting: BET is marked INOP; Simple is the default.

### Flight log
- Every force generator's force and moment, the stabilator angle, LCT position, mass,
  centre of mass and bounding centre, aircraft type, and raw pedal inputs.

### For pack authors
- `flapBackRollMax` / `flapBackPitchMax` are gone - remove them.
- FMC and flight-control PID gains are declared per pilot unit (deg/s, kt, ft, fpm); the
  4th value is the integral limit in output units. Defaults live in `fmc.hpp`.
- `lctSchedule[]` is degrees of longitudinal cyclic, - forward, + aft.
- `rotortables.py` raises drag for any short column, skips unreachable ones, and stops
  when the cruise attitude is out of reach; `python/dev/fits/` keeps every aircraft's fit.

### Tools
- **The aerosurface generator places the model.** `fm_generateAeroSurfacePoints.py`
  takes `--turn` (180 deg about vertical) and `--forward` / `--down` / `--right`
  for an fm.p3d not built over the aircraft. The placement is written on line 2
  of both files, and `--add` adds a shift to it.
- **A triangle is a quad.** The generator takes a three-sided face as a quad with
  one corner repeated.
- **New in-game tools, in `tools/`** - paste into the debug console, LOCAL EXEC:
  - `fm_nudge.sqf` moves the aerosurfaces, each rotor hub and the centre of mass
    live, and copies the config line or generator command for each.
  - `seats.sqf` measures every seat's and pylon's arm in the model frame.
  - `dump.sqf` lists every variant of the aircraft's base class: turrets,
    weapons, pylons and animation sources.
- **New `python/dev/airflow_trim.py`**: trims an engine's `airflowTable` per FAT
  against an OGE hover chart, through the rig.

All of these were the Bo 105's own tools, now Core's for every aircraft. They ship
with the release.

## 1.3.0.0

### Multiplayer
- **Taking the controls now works.** When the other crew member took the
  controls, the engines started over from cold and the aircraft fell. The new
  pilot now carries on from exactly where the aircraft was: engines, governor,
  rotor rpm, trims and holds.
- **The other crew station shows everything.** Engine readouts, pressure
  altitude, temperature, wind, gross weight and CG, the slip ball and force-trim
  positions now reach the machine that is not flying the aircraft. That fixes
  false cautions on the deck (APU, PRI/UTIL hydraulic pressure, low rotor rpm)
  and blank pages on that station.
- **Less network traffic.** The running state goes out as one packed message
  ten times a second, instead of more than fifteen separate values. Several
  values that used to be sent every frame are not sent separately any more.
- A player who gets into an aircraft that is already running builds its
  flight model on their own machine, without changing anything for the pilot.

### Flight model
- **A destroyed rotor makes no force.** A main or tail rotor that was shot off
  still produced full lift and yaw. It now produces nothing, and with the load
  gone the engines surge into what is left.

### Flight log
- Two new columns: pressure altitude (`baroAltFt`) and free air temperature
  (`fatC`).
- Long headers and rows are split across continuation lines, so the RPT no
  longer cuts them off and every row parses.

### Tools (`python/dev`)
- New `rotortables.py`: fits a main rotor's lift and drag tables in the rig,
  from the hover and power-curve targets. Options for the aircraft's own power
  curve, max range, the cruise pitch attitude and hover collective. Reports a
  failure at once and exits with an error code. The guide's "Fitting the main
  rotor tables" section is rewritten around it.
- `rotortables.py` shapes the main rotor's drag at collective 0: idle torque in
  the hover, falling to 1/36 of it by 60 kt. Full-down collective now unloads
  the engines in forward flight, so a power-on autorotation overruns them and
  the needles split. Tail rotors keep their own rows.
- `python/dev/fits/` keeps every aircraft's rotor table fit - the AH-64D, UH-60,
  EC665 Tiger and Bo 105 - with its command and full output, so a fit can be
  rerun and its targets are never lost.
- The rig flies the stabilator, and its replay measures the stick from the
  force trim, as the rotor sees it.
- `airframe.py`, `forces.py` and `engine.py` ship with the release.

### For aircraft authors
- **Add `damageRole` to each simple rotor**, or that rotor can never be
  destroyed:
  ```cpp
  class SimpleRotor01 { damageRole = "mainRotor"; ... };
  class SimpleRotor02 { damageRole = "tailRotor"; ... };
  ```
- **New, optional: `netStateVars[]`** in `BMKHS_HeliSim`. List the variables
  your pack computes on the machine that owns the aircraft and the other crew
  station displays. Core carries them in the packed state.
- **Nothing else changes in your pack.** Do not tick, set up or unpack an
  aircraft your machine does not own. Core does that itself.
