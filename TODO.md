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
