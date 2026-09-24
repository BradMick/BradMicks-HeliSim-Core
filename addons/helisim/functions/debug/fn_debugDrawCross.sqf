/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_debugDrawCross

Description:
  Draws a three axis debug cross in model space, marking a position. The arms
  run along the model X, Y and Z axes.

Parameters:
  _heli - The helicopter to get information from [Unit].
  _pos  - Model space center of the cross [Array].
  _size - Half length of each arm of the cross [Number].
  _col  - Line color [String].

Returns:

Examples:
  ...

Author:
  BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_pos", "_size", "_col"];

private _axes = [[_size, 0.0, 0.0], [0.0, _size, 0.0], [0.0, 0.0, _size]];

{
    private _a = _pos vectorAdd (_x vectorMultiply -1.0);
    private _b = _pos vectorAdd _x;

    [_heli, _a, _b, _col] call bmkhs_fnc_debugDrawLine;
} forEach _axes;
