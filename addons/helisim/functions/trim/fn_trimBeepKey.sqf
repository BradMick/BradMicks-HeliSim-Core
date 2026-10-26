/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_trimBeepKey

Description:
    A beep trim switch pressed or released. fn_trimUpdate moves the trim while
    it is held. A press belongs to what it started on - a flight director
    target or the stick - until it is released.

Parameters:
    _heli  - The helicopter [Object]
    _name  - The action [String]
    _axis  - 0 pitch, 1 roll [Number]
    _sign  - 1 forward / left, -1 backward / right [Number]
    _state - Pressed [Bool]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_name", "_axis", "_sign", "_state"];

private _held = _heli getVariable "bmkhs_trimBeepHeld";
if (!_state) exitWith { _held deleteAt _name };
if (currentPilot _heli != player || !local _heli) exitWith {};

private _fd = ((([["ias", "hvr"], ["hdg", "nav", "hvr"]] select _axis) findIf {_heli getVariable ["bmkhs_fd_" + _x, false]}) >= 0);
_held set [_name, [_axis, _sign, CBA_missionTime, _fd]];
