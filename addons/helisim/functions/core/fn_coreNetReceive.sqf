/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_coreNetReceive

Description:
    Unpacks the owner's published state into this machine's copy of the
    aircraft, so the crew station displays it and a change of controls
    carries on from it. Runs every frame on a machine that does not own the
    aircraft; only a packet it has not applied yet is unpacked.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

//The frame clock stays current, so the first frame as owner is one frame long.
_heli setVariable ["bmkhs_previousTime", diag_tickTime];

(_heli getVariable ["bmkhs_netState", []]) params [["_sent", -1], ["_values", []], ["_pids", []]];
if (_sent < 0 || {_sent == (_heli getVariable ["bmkhs_netStateApplied", -1])}) exitWith {};
_heli setVariable ["bmkhs_netStateApplied", _sent];

{ _heli setVariable [_x, _values select _forEachIndex] } forEach (_heli getVariable "bmkhs_netStateVars");

private _localPids = _heli getVariable ["bmkhs_pid_engine", []];
{
    _x params ["_integral", "_prevError"];
    private _pid = _localPids param [_forEachIndex, createHashMap];
    _pid set ["integral",  _integral];
    _pid set ["prevError", _prevError];
} forEach _pids;
