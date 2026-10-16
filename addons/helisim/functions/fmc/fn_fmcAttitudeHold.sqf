params ["_heli", "_att", "_on"];
#include "\bmkhs_helisim\functions\core\core.hpp"

if (count _att == 0) exitWith {[0.0, 0.0]};

//pos/vel use the posRoll/posPitch gains; att uses attRoll/attPitch.
//Roll
private _pidRoll      = _att get "posRoll";
private _pidRoll_att  = _att get "attRoll";

//Pitch
private _pidPitch     = _att get "posPitch";
private _pidPitch_att = _att get "attPitch";

private _posBelow  = (_att get "posBelowKts") * KNOTS_TO_MPS;
private _velBelow  = (_att get "velBelowKts") * KNOTS_TO_MPS;
private _attBelow  = (_att get "attBelowKts") * KNOTS_TO_MPS;
private _authority = _att get "authority";

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
private _gndSpeed  = _heli getVariable "bmkhs_gndSpeed";

//Attitude hold
private _curAtt   = _heli call BIS_fnc_getPitchBank;
private _curPitch = _curAtt # 0;
private _curRoll  = _curAtt # 1;

private _attHoldCycPitchOut = 0.0;
private _attHoldCycRollOut  = 0.0;

//Beep
private _beepDelta  = +(_heli getVariable ["bmkhs_trimBeepDelta", [0.0, 0.0]]);
//Not on the FD's axes
if ((["ias", "hvr"] findIf {_heli getVariable ["bmkhs_fd_" + _x, false]}) >= 0) then { _beepDelta set [0, 0.0] };
if ((["hdg", "nav", "hvr"] findIf {_heli getVariable ["bmkhs_fd_" + _x, false]}) >= 0) then { _beepDelta set [1, 0.0] };
private _gainDeg    = _att get "trimGain";
private _gainKts    = _att get "trimGainKts";
private _beeping    = ((_gainDeg + _gainKts) findIf {_x != 0}) >= 0
    && {(_beepDelta findIf {_x != 0}) >= 0}
    && {_heli getVariable "bmkhs_attHoldActive"};
private _wasBeeping = _heli getVariable ["bmkhs_attHoldBeeping", false];

//Submode selection, by ground speed: position hold below posBelowKts; velocity hold up to
//velBelowKts accelerating, and attitude hold above it until back below attBelowKts.
private _subModes = _att get "subModes";
//FD HVR is position hold
if (_heli getVariable ["bmkhs_fd_hvr", false]) then { _subModes = _subModes + ["pos"] };
private _want = switch (true) do {
    case (_beeping && {_gndSpeed <= _posBelow}):             { "vel" };
    case (_gndSpeed <= _posBelow):                           { "pos" };
    case (_subMode == "att" && {_gndSpeed >= _attBelow}):    { "att" };
    case (_gndSpeed > _velBelow):                            { "att" };
    default                                                  { "vel" };
};
if !(_want in _subModes) then {
    //Nearest declared
    _want = (([["vel", "att"], ["att", "pos"], ["vel", "pos"]] select (["pos", "vel", "att"] find _want)) select {_x in _subModes}) select 0;
};
[_heli, "bmkhs_attHoldSubMode", _want] call bmkhs_fnc_utilUpdateNetworkGlobal;

private _newSubMode = _heli getVariable "bmkhs_attHoldSubMode";
if (_newSubMode != _subMode) then {
    //Capture
    if (_heli getVariable "bmkhs_attHoldActive") then {
        switch (_newSubMode) do {
            case "pos": {
                _heli setVariable ["bmkhs_attHoldDesiredPos", getPos _heli, true];
                _heli setVariable ["bmkhs_posIntX", 0.0];
                _heli setVariable ["bmkhs_posIntY", 0.0];
            };
            case "vel": { _heli setVariable ["bmkhs_attHoldDesiredVel", [-_velX, _velY], true] };
            case "att": { _heli setVariable ["bmkhs_attHoldDesiredAtt", [_curPitch, _curRoll], true] };
        };
    };
    //Clear derivative
    {
        _x deleteAt "prevError";
        _x set ["derivFilt", 0];
    } forEach [_pidRoll, _pidPitch, _pidRoll_att, _pidPitch_att];
};
if (_beeping) then {
    //Published on release
    _beepDelta params ["_dPitch", "_dRoll"];
    if (_newSubMode == "att") then {
        (_heli getVariable "bmkhs_attHoldDesiredAtt") params ["_setPitch", "_setRoll"];
        _heli setVariable ["bmkhs_attHoldDesiredAtt", [_setPitch - _dPitch * (_gainDeg # 0), _setRoll - _dRoll * (_gainDeg # 1)]];
    };
    if (_newSubMode == "vel") then {
        (_heli getVariable "bmkhs_attHoldDesiredVel") params ["_setVelX", "_setVelY"];
        _heli setVariable ["bmkhs_attHoldDesiredVel", [_setVelX + _dRoll * (_gainKts # 1) * KNOTS_TO_MPS, _setVelY + _dPitch * (_gainKts # 0) * KNOTS_TO_MPS]];
    };
};
if (_wasBeeping && {!_beeping}) then {
    { _heli setVariable [_x, _heli getVariable _x, true] } forEach ["bmkhs_attHoldDesiredPos", "bmkhs_attHoldDesiredVel", "bmkhs_attHoldDesiredAtt"];
};
_heli setVariable ["bmkhs_attHoldBeeping", _beeping];

//Always on - engaged whenever it can be, released by force trim, the ground or its gate, and
//re-engaged on the new reference once they clear. The flight director's hover owns it while active.
if ((_att get "alwaysOn") == 1) then {
    private _can = _on && {!(_heli getVariable "bmkhs_forceTrimInterupted")} && {!([_heli] call bmkhs_fnc_stateOnGround)};
    if (!_can && {_heli getVariable "bmkhs_attHoldActive"}) then {
        _heli setVariable ["bmkhs_attHoldActive", false, true];
    };
    if (_can && {!(_heli getVariable "bmkhs_attHoldActive")} && {!(_heli getVariable ["bmkhs_fd_hvr", false])}) then {
        [_heli] call bmkhs_fnc_fmcAttitudeHoldEnable;
    };
};

if (_on && {_heli getVariable "bmkhs_attHoldActive"} && {!(_heli getVariable "bmkhs_forceTrimInterupted")}) then {
    //Position hold = velocity-null loop + a SLOW, TIGHTLY-CLAMPED position-error integral that biases
    //the velocity SETPOINT (not the output) to trim out the standing drift a pure velocity-null loop
    //leaves (type-0 -> type-1: zero steady-state position error). The bias works THROUGH the velocity
    //loop, so velocity does all the actuation and the integral only re-aims it - it can't fight the loop.
    //Anti-windup: the accumulator is clamped to a tiny ceiling (posIntClamp) near the loop's real working
    //range so it physically cannot rail the +-0.1 servo (the old 0.6 ceiling was 60x too big -> railed).
    if (_newSubMode == "pos") then {
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
    if (_newSubMode == "vel") then {
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
    if (_newSubMode == "att") then {
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

_attHoldCycPitchOut = [_attHoldCycPitchOut, -_authority, _authority] call BIS_fnc_clamp;
_attHoldCycRollOut  = [_attHoldCycRollOut, -_authority, _authority] call BIS_fnc_clamp;

[_attHoldCycPitchOut, _attHoldCycRollOut]
