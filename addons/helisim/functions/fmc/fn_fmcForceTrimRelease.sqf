/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcForceTrimRelease

Description:
    Force trim switch released. Captures the current attitude, velocity and
    heading as the new hold references and re-arms the hold modes.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli"];

if (currentPilot _heli != player || !local _heli) exitWith {};

//Velocity Hold Velocities
private _curVel   = velocityModelSpace _heli;
private _curVelX  = (_curVel # 0) * -1.0;
private _curVelY  = _curVel # 1;
//Attitude Hold Pitch & Roll
private _curAtt   = _heli call BIS_fnc_getPitchBank;
private _curPitch = _curAtt # 0;
private _curRoll  = _curAtt # 1;

_heli setVariable ["bmkhs_forceTrimInterupted",    false,                 true];
_heli setVariable ["bmkhs_attHoldDesiredPos",      getPos _heli,          true];
_heli setVariable ["bmkhs_attHoldDesiredVel",      [_curVelX, _curVelY],  true];
_heli setVariable ["bmkhs_attHoldDesiredAtt",      [_curPitch, _curRoll], true];
_heli setVariable ["bmkhs_hdgHoldDesiredHdg",      getDir _heli,          true];
//Sideslip setpoint is ZERO - a centred ball IS aerodynamic trim, which is what the
//heading hold's yaw/trn sub-modes are for. Do not capture the gauge signal here: it is
//clamped deflection, not lateral g, so it would not be comparable with the measurement.
_heli setVariable ["bmkhs_hdgHoldDesiredSideslip", 0.0,                   true];

[_heli] call bmkhs_fnc_fmcForceTrimSet;
[_heli] call bmkhs_fnc_inputCenterTrimMode;
