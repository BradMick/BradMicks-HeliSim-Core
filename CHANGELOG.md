# Changelog

## 1.4.0.0

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

All of these were the Bo 105's own tools, now Core's for every aircraft.

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
