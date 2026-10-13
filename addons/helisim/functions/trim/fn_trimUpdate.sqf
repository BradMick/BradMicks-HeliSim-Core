/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_trimUpdate

Description:
    The trim, each frame. Force trim off (bmkhs_fmcTrimOn, or forceTrim = 0)
    holds the trim position at 0; turning it back on captures the stick where
    it is. With it on, a held beep switch moves the trim position at its rate.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

private _trim = _heli getVariable "bmkhs_trim";
private _on   = (_trim get "forceTrim") > 0 && {_heli getVariable "bmkhs_fmcTrimOn"};

//The axes a casual assist owns - it writes the trim position itself (fn_trimForceSet)
private _casual    = bmkhs_helisimRealismSetting != REALISTIC;
private _autoPitch = bmkhs_autoPitch && _casual;
private _autoRoll  = bmkhs_autoRoll  && _casual;

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

private _dir = [0, 0];
{
    _y params ["_axis", "_sign", "_pressed"];
    if (CBA_missionTime - _pressed >= (_beep get "lag")) then { _dir set [_axis, (_dir select _axis) + _sign] };
} forEach _held;

private _deltaTime = _heli getVariable "bmkhs_deltaTime";
{
    _x params ["_var", "_blocked"];
    private _d = _dir select _forEachIndex;
    //"reference" moves the hold reference with the stick still - not built yet (refactor step 5)
    if (_d != 0 && {!_blocked} && {((_beep get "moves") select _forEachIndex) == "stick"}) then {
        private _pos = (_heli getVariable _var) + _d * ((_beep get "rate") select _forEachIndex) * _deltaTime;
        _heli setVariable [_var, (_pos max -1.0) min 1.0];
    };
} forEach [
    ["bmkhs_forceTrimPosPitch", _autoPitch || bmkhs_springlessCyclic || bmkhs_keyboardStickyPitch],
    ["bmkhs_forceTrimPosRoll",  _autoRoll  || bmkhs_springlessCyclic || bmkhs_keyboardStickyRoll]
];
