/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcBeepTrim

Description:
    Beep trim - one bump of the cyclic force trim position, beepTrimStep per press.

Parameters:
    _heli - The helicopter [Object]
    _axis - "bmkhs_forceTrimPosPitch" or "bmkhs_forceTrimPosRoll" [String]
    _sign - 1 forward / right, -1 backward / left [Number]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_axis", "_sign"];

if (currentPilot _heli != player || !local _heli) exitWith {};

private _trim = (_heli getVariable _axis) + (_sign * (_heli getVariable ["bmkhs_beepTrimStep", 0.01]));
_heli setVariable [_axis, (_trim max -1.0) min 1.0];
