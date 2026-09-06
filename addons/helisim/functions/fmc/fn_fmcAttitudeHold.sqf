params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

//pos/vel use pid_roll/pid_pitch (posRoll/posPitch gains); att uses pid_roll_att/pid_pitch_att (attRoll/attPitch).
//Roll
private _pidRoll      = _heli getVariable "bmkhs_pid_roll";
private _pidRoll_att  = _heli getVariable "bmkhs_pid_roll_att";

//Pitch
private _pidPitch     = _heli getVariable "bmkhs_pid_pitch";
private _pidPitch_att = _heli getVariable "bmkhs_pid_pitch_att";

//Position & Velocity hold
private _subMode  = _heli getVariable "bmkhs_attHoldSubMode";

((_heli getVariable "bmkhs_velModelSpaceNoWind"))
    params [
             "_velX"
           , "_velY"
           , "_velZ"
           ];

((_heli getVariable "bmkhs_angVelModelSpace"))
    params [
             "_angVelX"
           , "_angVelY"
           , "_angVelZ"
           ];

private _deltaTime = _heli getVariable "bmkhs_deltaTime";
private _gndSpeed  = (_heli getVariable "bmkhs_gndSpeed") * KNOTS_TO_MPS;

//Attitude hold
private _curAtt   = _heli call BIS_fnc_getPitchBank;
private _curPitch = _curAtt # 0;
private _curRoll  = _curAtt # 1;

private _attHoldCycPitchOut = 0.0;
private _attHoldCycRollOut  = 0.0;

//Submode selection: speed-driven.
//Position hold
if (_gndSpeed <= POS_HOLD_SPEED_SWITCH) then {
    [_heli, "bmkhs_attHoldSubMode", "pos"] call bmkhs_fnc_utilUpdateNetworkGlobal;
};
//Velocity hold
//This needs to check if accelerating or decelerating...really it's
//5 to 40 knots accelerating, 30 to 5 knots decelerating
if (_gndSpeed > POS_HOLD_SPEED_SWITCH && _gndSpeed <= VEL_HOLD_SPEED_SWITCH_ACCEL) then {
    [_heli, "bmkhs_attHoldSubMode", "vel"] call bmkhs_fnc_utilUpdateNetworkGlobal;
};
//Attitude hold
if (_gndSpeed > VEL_HOLD_SPEED_SWITCH_ACCEL) then {
    [_heli, "bmkhs_attHoldSubMode", "att"] call bmkhs_fnc_utilUpdateNetworkGlobal;
};

if (_heli getVariable "bmkhs_attHoldActive" && !(_heli getVariable "bmkhs_forceTrimInterupted")) then {
    //Position hold = velocity-null loop + a SLOW, TIGHTLY-CLAMPED position-error integral that biases
    //the velocity SETPOINT (not the output) to trim out the standing drift a pure velocity-null loop
    //leaves (type-0 -> type-1: zero steady-state position error). The bias works THROUGH the velocity
    //loop, so velocity does all the actuation and the integral only re-aims it - it can't fight the loop.
    //Anti-windup: the accumulator is clamped to a tiny ceiling (posIntClamp) near the loop's real working
    //range so it physically cannot rail the +-0.1 servo (the old 0.6 ceiling was 60x too big -> railed).
    if (_subMode == "pos") then {
        private _desiredPos = _heli getVariable ["bmkhs_attHoldDesiredPos", getPos _heli];
        private _dPos = _desiredPos vectorDiff (getPos _heli);
        private _hdg  = direction _heli;
        //model-space position error: X = right+, Y = fwd+
        private _posErrX = ((_dPos # 0) * cos _hdg) - ((_dPos # 1) * sin _hdg);
        private _posErrY = ((_dPos # 0) * sin _hdg) + ((_dPos # 1) * cos _hdg);

        private _posIkp    = 0.0050;
        private _posIclamp = 0.0200;
        private _iX = (_heli getVariable ["bmkhs_posIntX", 0.0]) + (_posErrX * _deltaTime * _posIkp);
        private _iY = (_heli getVariable ["bmkhs_posIntY", 0.0]) + (_posErrY * _deltaTime * _posIkp);
        _iX = [_iX, -_posIclamp, _posIclamp] call BIS_fnc_clamp;
        _iY = [_iY, -_posIclamp, _posIclamp] call BIS_fnc_clamp;
        _heli setVariable ["bmkhs_posIntX", _iX];
        _heli setVariable ["bmkhs_posIntY", _iY];

        //Bias is a velocity SETPOINT (m/s toward datum). roll measures -velX so its setpoint = -_iX;
        //pitch measures +velY so its setpoint = +_iY (matches the un-negated pos convention).
        private _roll  = [_pidRoll,  _deltaTime, (-_iX), -_velX] call bmkhs_fnc_pidRun;
        _roll          = [_roll,  -1.0, 1.0] call BIS_fnc_clamp;
        private _pitch = [_pidPitch, _deltaTime, ( _iY),  _velY] call bmkhs_fnc_pidRun;
        _pitch         = [_pitch, -1.0, 1.0] call BIS_fnc_clamp;

        _attHoldCycPitchOut = _pitch;
        _attHoldCycRollOut  = _roll;

        //Publish pos-branch internals for the hold-chain logger.
    };
    //Velocity hold
    if (_subMode == "vel") then {
        (_heli getVariable "bmkhs_attHoldDesiredVel")
            params ["_setVelX", "_setVelY"];
        private _roll  = [_pidRoll,  _deltaTime, _setVelX, -_velX] call bmkhs_fnc_pidRun;
        _roll          = [_roll,  -1.0, 1.0] call BIS_fnc_clamp;
        private _pitch = [_pidPitch, _deltaTime, _setVelY, _velY] call bmkhs_fnc_pidRun;
        _pitch         = [_pitch, -1.0, 1.0] call BIS_fnc_clamp;

        _attHoldCycPitchOut = _pitch;
        _attHoldCycRollOut  = _roll;
    };
    //Attitude hold
    if (_subMode == "att") then {
       (_heli getVariable "bmkhs_attHoldDesiredAtt")
              params ["_setPitch", "_setRoll"];
        private _pitchError = [_curPitch - _setPitch] call CBA_fnc_simplifyAngle180;
        private _rollError  = [_curRoll  - _setRoll]  call CBA_fnc_simplifyAngle180;

        private _roll  = [_pidRoll_att,  _deltaTime, 0.0, _rollError] call bmkhs_fnc_pidRun;
        _roll          = [_roll,  -1.0, 1.0] call BIS_fnc_clamp;
        private _pitch = [_pidPitch_att, _deltaTime, 0.0, _pitchError] call bmkhs_fnc_pidRun;
        _pitch         = [_pitch, -1.0, 1.0] call BIS_fnc_clamp;

        _attHoldCycPitchOut = _pitch * -1.0;
        _attHoldCycRollOut  = _roll  * -1.0;
    };
} else {
    //Position & Velocity hold
    [_pidRoll]  call bmkhs_fnc_pidReset;
    [_pidPitch] call bmkhs_fnc_pidReset;

    //Attitude hold
    [_pidRoll_att]  call bmkhs_fnc_pidReset;
    [_pidPitch_att] call bmkhs_fnc_pidReset;

    //Clear the position integral so re-engaging pos hold starts clean (no stale bias on engage).
    _heli setVariable ["bmkhs_posIntX", 0.0];
    _heli setVariable ["bmkhs_posIntY", 0.0];
};

//systemChat format ["Dist = %4 -- DistX = %1 -- DistY = %2 -- Dir = %3", _distX toFixed 2, _distY toFixed 2, _dir toFixed 2, _dist toFixed 2];
//systemChat format ["VelX = %1 -- VelY = %2 -- Pitch Out = %3 -- Roll Out = %4", _curVelX toFixed 2, _curVelY toFixed 2, _attHoldCycPitchOut toFixed 2, _attHoldCycRollOut toFixed 2];

_attHoldCycPitchOut = [_attHoldCycPitchOut, -0.1, 0.1] call BIS_fnc_clamp;
_attHoldCycRollOut  = [_attHoldCycRollOut, -0.1, 0.1] call BIS_fnc_clamp;

[_attHoldCycPitchOut, _attHoldCycRollOut]
