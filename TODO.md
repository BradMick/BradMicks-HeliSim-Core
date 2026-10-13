# To do

## Engine start can't reach its TGT peak with a late light-off
`fn_engineGovernor` caps the start fuel at idle fuel below idle Ng:
`(_base + ((1.0 - _base) * _ng / _idleNg)) min 1.0`. Lighting at 15% Ng, idle fuel was rich
enough to peak about 653 C; at the AH-64D's real light-off, 23% Ng, the compressor moves more air
and the start peaks around 490 C in game (482 C in the rig). `startFuelBase` can't fix it: even at
1.0 the rig peaks at 558 C.

- Lift the `min 1.0` so a `startFuelBase` above 1 meters acceleration fuel richer than idle at
  light-off, tapering to idle fuel at idle Ng. A base of 1 or less is unchanged, so no pack moves.
- Port it to `python/dev/engine.py`, then fit `startFuelBase` to about 653 C lighting at 23% Ng
  in the AH-64D and the UH-60 (both moved to 23%).
- The bench's `EXPECTED` light-off time (2.6 s) belongs to the old 15% light-off; it moves to
  about 5.7 s.

## Bring every aircraft up to 1.4.0
Each aircraft, in sequence, is updated with all of the variables added in this Core update - Bo 105,
EC665 (Tiger), AH-64D, H-60. Where a variable doesn't apply, declare it at its "none" value so
the config says so.

- Simple rotor config, aircraft level: `controlScheme`, `drivetrainDrag`; `type` may be main, tail,
  propeller or proprotor.
- Simple rotor config, per rotor: `position`; tandem only - `tandemPitchLiftCoef`,
  `tandemYawLiftCoef`, `lctSchedule[]`, `lctTime`.
- `ControlMixing`: the `collective` target.
- Flight controls: `alwaysOn` in each FMC hold class; `class HydraulicBoost` on aircraft with no FMC;
  `beepTrimStep`.
- Engine: `ngOvrspdProt`, `npOvrspdProt`; `maxNg` is now the spool's real ceiling (no fixed 1.1).
- Torque is power over shaft speed: idle Np/Nr rises on every pack (Bo 105 37.8 -> 52.5%,
  EC665 48.8 -> 62.1%, AH-64D 54.9 -> 67.1%, H-60 52.7 -> 65.4%). Fit `drivetrainDrag` and the
  rotor's flat-pitch drag to each aircraft's real idle Ng / torque / Np / Nr, then refit its tables.
- `compressor_map.py` now scales the low end on the engine's own pressure ratio and takes an idle
  rating with its own efficiency - rebuild each map with its real idle point.
- Standardize every config file to the MH-47G's layout (`helisim_simpleRotor.hpp`,
  `helisim_engine.hpp`): fields grouped under section headers in the same order, a header listing
  each field by group, and comments pruned to one short description per term - no sourcing,
  history, rescale notes, TODOs or explanations.
