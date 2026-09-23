/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputAutoAttitude

Description:
    Auto attitude - an input assist that drives the aircraft to wings level.

    CASUAL + KEYBOARD only. A keyboard has no proportional axis, so a player
    cannot trim out the pitch-up the airframe develops with forward airspeed.
    This holds the attitude there for them.

    The setpoint is always ZERO - there is no captured target and no capture
    event, so there is nothing to go stale and nothing to snap on release.

    It never fights an input: authority fades out in proportion to stick
    deflection and returns as the stick centres.

Parameters:
    _heli            - The helicopter [Object].
    _deltaTime       - Frame time [Number].
    _cyclicFwdAft    - Pilot pitch input [Number].
    _cyclicLeftRight - Pilot roll input [Number].

Returns:
    Nothing. Writes bmkhs_forceTrimPosPitch and bmkhs_forceTrimPosRoll.

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli", "_deltaTime", "_cyclicFwdAft", "_cyclicLeftRight"];

private _pidPitch   = _heli getVariable "bmkhs_pid_autoAttPitch";
private _pidRoll    = _heli getVariable "bmkhs_pid_autoAttRoll";
private _levelPitch = _heli getVariable "bmkhs_autoAttLevelPitch";
private _rollLimit  = _heli getVariable "bmkhs_autoAttRollLimit";

private _gndSpeed = (_heli getVariable "bmkhs_gndSpeed") * KNOTS_TO_MPS;
(_heli call BIS_fnc_getPitchBank) params ["_curPitch", "_curRoll"];

//Authority recedes in proportion to stick deflection, so the assist never fights an input
private _pitchKey = [0.0, abs _cyclicFwdAft]    select ((abs _cyclicFwdAft)    > AUTOATT_KEY_DEADBAND);
private _rollKey  = [0.0, abs _cyclicLeftRight] select ((abs _cyclicLeftRight) > AUTOATT_KEY_DEADBAND);
private _breakout = (_pitchKey > 0.0) || (_rollKey > 0.0);

//A hover has no correct attitude, so the assist fades in with airspeed
private _wSpeed = linearConversion [AUTOATT_SPD_LO, AUTOATT_SPD_HI, _gndSpeed, 0.0, 1.0, true];

//The roll key commands a TARGET, not the stick - held it sweeps to the bank limit,
//released it returns to level. Lerped so the roll in and out is not a step.
private _rollSet = _heli getVariable ["bmkhs_autoAttRollTarget", 0.0];
private _rollWant = 0.0;
if (_rollKey > 0.0) then {
    _rollWant = _rollLimit * ([-1, 1] select (_cyclicLeftRight < 0.0));
};
private _rollFade = [AUTOATT_ROLL_OUT_TIME, AUTOATT_ROLL_IN_TIME] select (_rollKey > 0.0);
_rollSet = [_rollSet, _rollWant, (1.0 / _rollFade) * _deltaTime] call BIS_fnc_lerp;
//Snap the last fraction - an asymptotic target never settles, so the PID chases the creep
if ((abs (_rollWant - _rollSet)) < 0.5) then { _rollSet = _rollWant; };
_heli setVariable ["bmkhs_autoAttRollTarget", _rollSet];

private _pitchOut = 0.0;
private _rollOut  = 0.0;

if (_wSpeed > 0.0) then {
    //pidRun gives (setpoint - measurement), so the output is negated
    _pitchOut = [_pidPitch, _deltaTime, _levelPitch, _curPitch] call bmkhs_fnc_pidRun;
    _pitchOut = -([_pitchOut, -1.0, 1.0] call BIS_fnc_clamp);
    private _rollError = [_curRoll - _rollSet] call CBA_fnc_simplifyAngle180;
    _rollOut  = [_pidRoll,  _deltaTime, 0.0, _rollError] call bmkhs_fnc_pidRun;
    _rollOut  = -([_rollOut,  -1.0, 1.0] call BIS_fnc_clamp);

    _pitchOut = _pitchOut * _wSpeed * (1.0 - _pitchKey);
    _rollOut  = _rollOut  * _wSpeed;
} else {
    [_pidPitch] call bmkhs_fnc_pidReset;
    [_pidRoll]  call bmkhs_fnc_pidReset;
};

//On the stick pitch yields, so its integral is meaningless
if (_pitchKey > 0.0) then { [_pidPitch] call bmkhs_fnc_pidReset; };
//A disabled axis must not wind up, or enabling it mid-flight dumps the accumulation at once
if (!bmkhs_autoPitch) then { [_pidPitch] call bmkhs_fnc_pidReset; };
if (!bmkhs_autoRoll)  then { [_pidRoll]  call bmkhs_fnc_pidReset; };

_pitchOut = [_pitchOut, -AUTOATT_PITCH_OUT_CLAMP, AUTOATT_PITCH_OUT_CLAMP] call BIS_fnc_clamp;
_rollOut  = [_rollOut,  -AUTOATT_ROLL_OUT_CLAMP,  AUTOATT_ROLL_OUT_CLAMP]  call BIS_fnc_clamp;

if (bmkhs_autoPitch) then { _heli setVariable ["bmkhs_forceTrimPosPitch", _pitchOut, true]; };
//Written either way - skipping it would leave a stale roll command summed at the rotor
_heli setVariable ["bmkhs_autoAttCycRollOut", [0.0, _rollOut] select bmkhs_autoRoll, true];

//The PID takes the roll axis as it washes in, so the raw key hands it over in step
[_cyclicLeftRight, _cyclicLeftRight * (1.0 - _wSpeed)] select bmkhs_autoRoll
