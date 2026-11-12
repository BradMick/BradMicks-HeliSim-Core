/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_trimForceSet

Description:
    Sets the trim position to where the controls are - force trim release, and
    force trim switched back on.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

if (currentPilot _heli != player || !local _heli) exitWith {};

//A keyboard or mouse axis is not trimmed from the stick. The rate command keeps its own trim; sticky has none.
private _noTrimPitch = bmkhs_pitchScheme in ["kb", "mouseGamepad"];
private _noTrimRoll  = bmkhs_rollScheme  in ["kb", "mouseGamepad"];

//Cyclic pitch trim
private _curCyclicFwdAft  = (_heli getVariable "bmkhs_cyclicFwdAft");
private _prevCyclicFwdAft = _heli getVariable "bmkhs_forceTrimPosPitch";
private _pitchTrimVal     = [_curCyclicFwdAft, _prevCyclicFwdAft] call bmkhs_fnc_inputGetInterp;
if !(_heli getVariable "bmkhs_rateCommandPitchOn") then {
    if (bmkhs_springlessCyclic || _noTrimPitch) then {
        _heli setVariable ["bmkhs_forceTrimPosPitch", 0.0];
    } else {
        _heli setVariable ["bmkhs_forceTrimPosPitch", _pitchTrimVal];
    };
};
//Cyclic roll trim
private _curCyclicLeftRight  = (_heli getVariable "bmkhs_cyclicLeftRight");
private _prevCyclicLeftRight = _heli getVariable "bmkhs_forceTrimPosRoll";
private _rollTrimVal         = [_curCyclicLeftRight, _prevCyclicLeftRight] call bmkhs_fnc_inputGetInterp;
if !(_heli getVariable "bmkhs_rateCommandRollOn") then {
    if (bmkhs_springlessCyclic || _noTrimRoll) then {
        _heli setVariable ["bmkhs_forceTrimPosRoll",  0.0];
    } else {
        _heli setVariable ["bmkhs_forceTrimPosRoll", _rollTrimVal];
    };
};
//Pedal trim - the feet keep their own (auto pedal, unless HOTAS pedals are in use)
if !(bmkhs_autoPedal && {bmkhs_pedalScheme != "hotas"}) then {
    private _curPedalLeftRight  = (_heli getVariable "bmkhs_pedalLeftRight");
    private _prevPedalLeftRight = _heli getVariable "bmkhs_forceTrimPosYaw";
    private _pedalTrimVal       = [_curPedalLeftRight, _prevPedalLeftRight] call bmkhs_fnc_inputGetInterp;
    if (bmkhs_springlessPedals || {bmkhs_keyboardStickyYaw && {bmkhs_pedalScheme == "kb"}}) then {
        _heli setVariable ["bmkhs_forceTrimPosYaw", 0.0];
    } else {
        _heli setVariable ["bmkhs_forceTrimPosYaw", _pedalTrimVal];
    };
};
