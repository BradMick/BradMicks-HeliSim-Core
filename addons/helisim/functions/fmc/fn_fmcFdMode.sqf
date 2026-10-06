/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcFdMode

Description:
    Toggles a flight director mode. Engaging one cancels the others on its axis -
    vertical {ralt, alt, altp}, longitudinal {ias, hvr}, lateral {hdg, nav}. HVR slows
    to a stop and holds it on the attitude hold, releasing that when it disengages.

    Refused while the flight director's gate is shut, off the ground for anything
    but the vertical modes, and for NAV with no waypoint.

Parameters:
    _heli - The helicopter [Object]
    _mode - "ralt", "alt", "altp", "ias", "hdg", "nav" or "hvr" [String]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_mode"];

private _fd = (_heli getVariable "bmkhs_fmc") getOrDefault ["FlightDirector", createHashMap];
if (count _fd == 0 || {!(_mode in (_fd get "modes"))}) exitWith {};

//Off, releasing what the mode holds
private _off = {
    if (_heli getVariable ["bmkhs_fd_" + _this, false]) then {
        [_heli, "bmkhs_fd_" + _this, false] call bmkhs_fnc_utilUpdateNetworkGlobal;
        if (_this == "hvr" && {_heli getVariable "bmkhs_attHoldActive"}) then {
            [_heli] call bmkhs_fnc_fmcAttitudeHoldEnable;
        };
        [_heli, "fdModeChanged", [_this, false]] call bmkhs_fnc_utilNotify;
    };
};

if (_heli getVariable ["bmkhs_fd_" + _mode, false]) exitWith { _mode call _off };

if !(_heli getVariable ["bmkhs_fmcFdAvail", false]) exitWith {};
if (!(_mode in ["ralt", "alt", "altp"]) && {[_heli] call bmkhs_fnc_stateOnGround}) exitWith {};
if (_mode == "nav" && {(_heli getVariable ["bmkhs_fdWaypoint", []]) isEqualTo []}) exitWith {};
if (_mode == "hvr" && {!(_heli getVariable ["bmkhs_fmcAttHoldAvail", false])}) exitWith {};

private _axis = [["ralt", "alt", "altp"], ["ias", "hvr"], ["hdg", "nav"]] select {_mode in _x} select 0;
{ if (_x != _mode) then { _x call _off } } forEach _axis;

//HVR slows the aircraft itself, then engages the attitude hold over the spot it stops on
if (_mode == "hvr") then {
    if (_heli getVariable "bmkhs_attHoldActive") then { [_heli] call bmkhs_fnc_fmcAttitudeHoldEnable };
};

[_heli, "bmkhs_fd_" + _mode, true] call bmkhs_fnc_utilUpdateNetworkGlobal;
[_heli, "fdModeChanged", [_mode, true]] call bmkhs_fnc_utilNotify;
