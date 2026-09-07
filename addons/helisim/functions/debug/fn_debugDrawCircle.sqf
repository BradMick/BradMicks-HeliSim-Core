/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_debugDrawCircle

Description:
  Draws a debug circle in model space. The circle is built in the XY plane and
  then rotated by a pitch/roll/yaw triplet, so it follows the orientation of
  whatever it is representing (rotor disc, etc).

Parameters:
  _heli     - The apache helicopter to get information from [Unit].
  _numSides - Number of segments used to approximate the circle [Number].
  _pos      - Model space center of the circle [Array].
  _rot      - [pitch, roll, yaw] setting the orientation of the circle [Array].
  _radius   - Radius of the circle [Number].
  _col      - Line color [String].

Returns:

Examples:
  ...

Author:
  BradMick
---------------------------------------------------------------------------- */
params["_heli", "_numSides", "_pos", "_rot", "_radius", "_col"];

private _p = _rot select 0;
private _r = _rot select 1;
private _y = _rot select 2;

private _incr = 360 / _numSides;

for "_i" from 1 to _numSides do {
  private _angA = _incr * _i;
  private _angB = _incr * (_i + 1);

  private _a = [_radius * cos _angA, _radius * sin _angA, 0.0];
  private _b = [_radius * cos _angB, _radius * sin _angB, 0.0];

  _a = _pos vectorAdd ([_a, _p, _r, _y] call bmkhs_fnc_mathVectorRotate);
  _b = _pos vectorAdd ([_b, _p, _r, _y] call bmkhs_fnc_mathVectorRotate);

  [_heli, _a, _b, _col] call bmkhs_fnc_debugDrawLine;
};
