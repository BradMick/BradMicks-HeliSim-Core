/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputRateCommand

Description:
    Keyboard or mouse cyclic on one axis, or the pedal while auto pedal has it held. The control
    commands a rate and releasing it holds the attitude. The control is driven from the error
    between the commanded and the measured rate.

Parameters:
    _heli      - The helicopter [Object].
    _deltaTime - Frame time [Number].
    _axis      - "Pitch", "Roll" or "Yaw" [String].
    _key       - The key or mouse, in the control's sense: + forward, + left, + right pedal, -1..1 [Number].
    _isKeyboard - The control is a key, so a short press is a tap at the tap rate [Bool].
    _measured  - Measured rate in the control's sense: + nose down, + roll left, + yaw right, deg/s [Number].
    _attitude  - Pitch (+ nose up), bank (+ right) or minus the heading, deg [Number].
    _onGround  - Weight on the ground [Bool].
    _share     - Share of its rate the forward key asks for; the rest holds _level. Under 1 climbing out of a rolling takeoff [Number].
    _level     - The attitude the rest holds, deg [Number].

Returns:
    The control, in its own sense [Number].

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli", "_deltaTime", "_axis", "_key", "_isKeyboard", "_measured", "_attitude", "_onGround", ["_share", 1.0], ["_level", 0.0]];

private _pid  = _heli getVariable ("bmkhs_pid_rateCommand" + _axis);
private _ref  = _heli getVariable ("bmkhs_rateCommand" + _axis + "Ref");
private _held = _heli getVariable ("bmkhs_rateCommand" + _axis + "Held");
private _rateHold = [RATECMD_RATE_HOLD_PITCH, RATECMD_RATE_HOLD_ROLL, RATECMD_RATE_HOLD_YAW] select (["Pitch", "Roll", "Yaw"] find _axis);

//A mouse at rest is not exactly zero
if (abs _key < RATECMD_DEADBAND) then { _key = 0.0 };

//Taking the axis, or resting on the ground: hold the attitude the aircraft has
if (!(_heli getVariable ("bmkhs_rateCommand" + _axis + "On")) || {_onGround && {_key == 0}}) then { _ref = _attitude };
//Nothing rotates on the ground for the loop to wind up against
if (!(_heli getVariable ("bmkhs_rateCommand" + _axis + "On")) || _onGround) then { [_pid] call bmkhs_fnc_pidReset };

private _released = _held > 0 && {_key == 0};
_held = [0.0, _held + _deltaTime] select (_key != 0);

//The rate asked for, in the cyclic's sense
private _command = 0.0;
if (_key != 0) then {
    //A key's tap is slow, its hold quick. The mouse has no tap - its travel is the rate.
    _command = _key * ([_rateHold, RATECMD_RATE_TAP] select (_isKeyboard && {_held <= RATECMD_TAP_TIME}));
    //Climbing out of a rolling takeoff the forward key is still partly the hold on the level-flight attitude
    if (_share < 1.0 && {_key > 0.0}) then {
        _command = _share * _command + (1.0 - _share) * RATECMD_HOLD_GAIN * ([_attitude - _level] call CBA_fnc_simplifyAngle180);
    };
} else {
    //Hold where the rotation will have stopped - forward and left both lower the attitude
    if (_released) then { _ref = _attitude - _measured * RATECMD_COAST };
    _command = RATECMD_HOLD_GAIN * ([_attitude - _ref] call CBA_fnc_simplifyAngle180);
    _command = [_command, -_rateHold, _rateHold] call BIS_fnc_clamp;
};

_heli setVariable ["bmkhs_rateCommand" + _axis + "Ref",  _ref];
_heli setVariable ["bmkhs_rateCommand" + _axis + "Held", _held];
_heli setVariable ["bmkhs_rateCommand" + _axis + "Rate", _command];
_heli setVariable ["bmkhs_rateCommand" + _axis + "On",   true];

//Cyclic from the rate error; its derivative is the angular acceleration
private _out = [[_pid, _deltaTime, _command, _measured] call bmkhs_fnc_pidRun, -1.0, 1.0] call BIS_fnc_clamp;

//Trim - where a centred stick rests, so another crew member takes the controls without a jump
private _trimVar = "bmkhs_forceTrimPos" + _axis;
private _settled = _heli getVariable ("bmkhs_rateCommand" + _axis + "Settled");
_settled = [0.0, _settled + _deltaTime] select (_key == 0 && {!_onGround} && {abs _measured < RATECMD_TRIM_RATE} && {abs _command < RATECMD_TRIM_RATE});
if (_heli getVariable "bmkhs_fmcTrimOn") then {
    //Neutral on the ground
    if (_onGround) then {
        _heli setVariable [_trimVar, [_heli getVariable _trimVar, 0.0, _deltaTime, RATECMD_TRIM_TIME] call bmkhs_fnc_utilSmooth];
    };
    //Settled with the control released: the stick's position becomes the trim and the stick centres
    if (_settled > RATECMD_TRIM_TIME && {abs _out > RATECMD_DEADBAND}) then {
        _heli setVariable [_trimVar, [_out, _heli getVariable _trimVar] call bmkhs_fnc_inputGetInterp];
        [_pid] call bmkhs_fnc_pidReset;
        _out     = 0.0;
        _settled = 0.0;
    };
};
_heli setVariable ["bmkhs_rateCommand" + _axis + "Settled", _settled];

_out
