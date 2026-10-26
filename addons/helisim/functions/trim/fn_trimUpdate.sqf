/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_trimUpdate

Description:
    The trim, each frame. Force trim off (bmkhs_fmcTrimOn) holds the trim
    position at 0; turning it back on captures the stick where it is. With it
    on, a held beep switch moves the trim position at its rate - or, a press
    started on a flight director axis, that director target. bmkhs_trimBeepDelta
    carries this frame's stick travel to the holds, bmkhs_trimBeepFd a target's.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\fmc.hpp"

private _trim = _heli getVariable "bmkhs_trim";
private _on   = _heli getVariable "bmkhs_fmcTrimOn";

//The axes a casual assist owns - it writes the trim position itself (fn_trimForceSet)
private _casual    = bmkhs_helisimRealismSetting != REALISTIC;
private _autoPitch = bmkhs_autoPitch && _casual;
private _autoRoll  = bmkhs_autoRoll  && _casual;

//The beep's travel this frame, {pitch, roll} - the stick's moves the holds' references, a
//director target's the flight director
_heli setVariable ["bmkhs_trimBeepDelta", [0.0, 0.0]];
_heli setVariable ["bmkhs_trimBeepFd",    [0.0, 0.0]];

if (_on != (_heli getVariable "bmkhs_trimOn")) then {
    _heli setVariable ["bmkhs_trimOn", _on];
    if (_on) then {
        [_heli] call bmkhs_fnc_trimForceSet;
    } else {
        //The force trim switch is dead with it - a hold interrupt it left set would never clear
        [_heli, "bmkhs_forceTrimInterupted", false] call bmkhs_fnc_utilUpdateNetworkGlobal;
    };
};

if (!_on) exitWith {
    if (!_autoPitch)       then { _heli setVariable ["bmkhs_forceTrimPosPitch", 0.0] };
    if (!_autoRoll)        then { _heli setVariable ["bmkhs_forceTrimPosRoll",  0.0] };
    if (!bmkhs_autoPedal)  then { _heli setVariable ["bmkhs_forceTrimPosYaw",   0.0] };
};

//Beep
private _beep = _trim getOrDefault ["beep", createHashMap];
private _held = _heli getVariable "bmkhs_trimBeepHeld";
if (count _beep == 0 || {count _held == 0} || {!([_heli, _beep get "gate"] call bmkhs_fnc_utilGateOpen)}) exitWith {};

//A press started on a director target stays on it - dead if that mode drops - and one started
//on the stick stays on the stick
private _fdOn = [["ias", "hvr"], ["hdg", "nav", "hvr"]] apply { (_x findIf {_heli getVariable ["bmkhs_fd_" + _x, false]}) >= 0 };
private _dirStick = [0, 0];
private _dirFd    = [0, 0];
{
    _y params ["_axis", "_sign", "_pressed", ["_onFd", false]];
    if (CBA_missionTime - _pressed >= TRIM_BEEP_LAG) then {
        if (!_onFd) then {
            _dirStick set [_axis, (_dirStick select _axis) + _sign];
        } else {
            if (_fdOn select _axis) then { _dirFd set [_axis, (_dirFd select _axis) + _sign] };
        };
    };
} forEach _held;

private _deltaTime = _heli getVariable "bmkhs_deltaTime";
private _moved     = [0.0, 0.0];
private _fd        = [0.0, 0.0];
{
    _x params ["_var", "_blocked"];
    private _rate = ((_beep get "rate") select _forEachIndex) * _deltaTime;
    _fd set [_forEachIndex, (_dirFd select _forEachIndex) * _rate];
    private _d = _dirStick select _forEachIndex;
    if (_d != 0 && {!_blocked}) then {
        private _was = _heli getVariable _var;
        private _pos = ((_was + _d * _rate) max -1.0) min 1.0;
        _heli setVariable [_var, _pos];
        _moved set [_forEachIndex, _pos - _was];
    };
} forEach [
    ["bmkhs_forceTrimPosPitch", _autoPitch || bmkhs_springlessCyclic || bmkhs_keyboardStickyPitch],
    ["bmkhs_forceTrimPosRoll",  _autoRoll  || bmkhs_springlessCyclic || bmkhs_keyboardStickyRoll]
];
_heli setVariable ["bmkhs_trimBeepDelta", _moved];
_heli setVariable ["bmkhs_trimBeepFd",    _fd];
