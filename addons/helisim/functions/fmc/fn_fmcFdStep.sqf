/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcFdStep

Description:
    Steps a flight director target by its declared step.

Parameters:
    _heli   - The helicopter [Object]
    _target - "ralt", "alt", "altp", "ias" or "hdg" [String]
    _dir    - 1 up, -1 down [Number]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_target", "_dir"];

private _fd = (_heli getVariable "bmkhs_fmc") getOrDefault ["FlightDirector", createHashMap];
if (count _fd == 0) exitWith {};
private _t = (_fd get "targets") getOrDefault [_target, []];
if (_t isEqualTo []) exitWith {};

[_heli, _target, (_heli getVariable ("bmkhs_fdTgt_" + _target)) + (_dir * (_t select 2))] call bmkhs_fnc_fmcFdTarget;
