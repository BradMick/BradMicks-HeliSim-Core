/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcFdTarget

Description:
    Sets a flight director target - clamped to its declared range, or wrapped
    where it wraps, and snapped to its step.

Parameters:
    _heli   - The helicopter [Object]
    _target - "ralt", "alt", "altp", "ias" or "hdg" [String]
    _value  - In the target's units: ft, kt or deg [Number]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_target", "_value"];

private _fd = (_heli getVariable "bmkhs_fmc") getOrDefault ["FlightDirector", createHashMap];
if (count _fd == 0) exitWith {};
private _t = (_fd get "targets") getOrDefault [_target, []];
if (_t isEqualTo []) exitWith {};
_t params ["_min", "_max", "_step", "_wraps"];

_value = (round (_value / _step)) * _step;
_value = if (_wraps) then {
    private _span = _max - _min;
    _min + ((((_value - _min) mod _span) + _span) mod _span)
} else {
    (_value max _min) min _max
};

[_heli, "bmkhs_fdTgt_" + _target, _value] call bmkhs_fnc_utilUpdateNetworkGlobal;
