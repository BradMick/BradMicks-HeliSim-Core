params ["_heli", "_att", "_on"];
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\fmc.hpp"

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

//Fly-through - a cyclic axis past the breakout is not active
private _boPitch   = abs (_heli getVariable "bmkhs_cyclicFwdAft")    > (_att get "breakout");
private _boRoll    = abs (_heli getVariable "bmkhs_cyclicLeftRight") > (_att get "breakout");

//Airspeed hold, KTAS
private _hasSpd    = "spd" in (_att get "subModes");
private _pidSpd    = _att get "spdPitch";
private _spdAbove  = (_att get "spdAboveKts") * KNOTS_TO_MPS;
private _spdBelow  = _spdAbove - FMC_ATT_SPD_BAND_KTS * KNOTS_TO_MPS;
private _maxPitch  = FMC_ATT_SPD_MAX_PITCH_DEG;
private _settle    = FMC_ATT_SPD_SETTLE_KTS * KNOTS_TO_MPS;
private _airspeed  = (_heli getVariable "bmkhs_velModelSpace") # 1;
//Airspeed rate, filtered - m/s per s
private _dtSpd     = (_heli getVariable "bmkhs_deltaTime") max 0.001;
private _spdRate   = _heli getVariable ["bmkhs_attHoldSpdRate", 0.0];
_spdRate = _spdRate + ((_dtSpd / (0.5 + _dtSpd)) * (((_airspeed - (_heli getVariable ["bmkhs_attHoldSpdPrev", _airspeed])) / _dtSpd) - _spdRate));
_heli setVariable ["bmkhs_attHoldSpdPrev", _airspeed];
_heli setVariable ["bmkhs_attHoldSpdRate", _spdRate];

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

//Beep - the stick's travel; a director target's goes to the director
private _beepDelta  = +(_heli getVariable ["bmkhs_trimBeepDelta", [0.0, 0.0]]);
private _gainDeg    = _att get "trimGain";
private _beeping    = (_gainDeg findIf {_x != 0}) >= 0
    && {(_beepDelta findIf {_x != 0}) >= 0}
    && {_heli getVariable "bmkhs_attHoldActive"};
private _wasBeeping = _heli getVariable ["bmkhs_attHoldBeeping", false];

//Submode selection, by ground speed: position hold below posBelowKts; velocity hold up to
//velBelowKts accelerating, and attitude hold above it until back below attBelowKts. Airspeed
//hold above spdAboveKts KTAS until 10 kt below it, pitch only.
private _subModes = _att get "subModes";
//FD HVR is position hold
if (_heli getVariable ["bmkhs_fd_hvr", false]) then { _subModes = _subModes + ["pos"]; _hasSpd = false };
private _want = switch (true) do {
    case (_hasSpd && {_airspeed > _spdAbove || {_subMode == "spd" && {_airspeed >= _spdBelow}}}): { "spd" };
    case (_beeping && {_gndSpeed <= _posBelow}):             { "vel" };
    case (_gndSpeed <= _posBelow):                           { "pos" };
    case (_subMode == "att" && {_gndSpeed >= _attBelow}):    { "att" };
    case (_gndSpeed > _velBelow):                            { "att" };
    default                                                  { "vel" };
};
if !(_want in _subModes) then {
    //Nearest declared
    _want = (([["vel", "att", "spd"], ["att", "pos", "spd"], ["vel", "pos", "spd"]] select (["pos", "vel", "att"] find _want)) select {_x in _subModes}) select 0;
};
//No change while flying through
if (_boPitch || _boRoll) then { _want = _subMode };
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
            //The speed once it settles
            case "spd": { _heli setVariable ["bmkhs_attHoldDesiredAtt", [_curPitch, _curRoll], true] };
        };
    };
    //Clear derivative
    {
        _x deleteAt "prevError";
        _x set ["derivFilt", 0];
    } forEach ([_pidRoll, _pidPitch, _pidRoll_att, _pidPitch_att] + ([[], [_pidSpd]] select _hasSpd));
    //Only spd runs it
    if (_newSubMode == "spd") then { [_pidSpd] call bmkhs_fnc_pidReset };
    _heli setVariable ["bmkhs_attHoldSpdPending", _newSubMode == "spd"];
};
if (_beeping) then {
    //Published on release
    _beepDelta params ["_dPitch", "_dRoll"];
    if (_newSubMode in ["att", "spd"]) then {
        (_heli getVariable "bmkhs_attHoldDesiredAtt") params ["_setPitch", "_setRoll"];
        _heli setVariable ["bmkhs_attHoldDesiredAtt", [_setPitch - _dPitch * (_gainDeg # 0), _setRoll - _dRoll * (_gainDeg # 1)]];
    };
    //Airspeed re-captured once it settles after the beep
    if (_newSubMode == "spd" && {_dPitch != 0}) then {
        _heli setVariable ["bmkhs_attHoldSpdPending", true];
    };
};
if (_wasBeeping && {!_beeping}) then {
    { _heli setVariable [_x, _heli getVariable _x, true] } forEach ["bmkhs_attHoldDesiredPos", "bmkhs_attHoldDesiredVel", "bmkhs_attHoldDesiredAtt", "bmkhs_attHoldDesiredSpd"];
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
    //Attitude hold; airspeed hold flies pitch to the speed
    if (_newSubMode in ["att", "spd"]) then {
       (_heli getVariable "bmkhs_attHoldDesiredAtt")
              params ["_setPitch", "_setRoll"];
        if (_newSubMode == "spd") then {
            if (_heli getVariable ["bmkhs_attHoldSpdPending", false]) then {
                //Holding the beeped attitude until the speed settles, then that speed is the reference
                if (!_beeping && {abs _spdRate < _settle}) then {
                    _heli setVariable ["bmkhs_attHoldSpdPending", false];
                    _heli setVariable ["bmkhs_attHoldDesiredAtt", [_curPitch, _setRoll], true];
                    _heli setVariable ["bmkhs_attHoldDesiredSpd", _airspeed, true];
                    _setPitch = _curPitch;
                };
                [_pidSpd] call bmkhs_fnc_pidReset;
            } else {
                //Slow is nose down
                private _dSpd = -([_pidSpd, _deltaTime, _heli getVariable "bmkhs_attHoldDesiredSpd", _airspeed] call bmkhs_fnc_pidRun);
                _setPitch = _setPitch + ((_dSpd max -_maxPitch) min _maxPitch);
            };
        };
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
    if (_hasSpd) then { [_pidSpd] call bmkhs_fnc_pidReset };

    //Clear the position integral so re-engaging pos hold starts clean (no stale bias on engage).
    _heli setVariable ["bmkhs_posIntX", 0.0];
    _heli setVariable ["bmkhs_posIntY", 0.0];
};

//Not active axes - no output, loops cleared, reference kept
if (_boPitch) then {
    _attHoldCycPitchOut = 0.0;
    { [_x] call bmkhs_fnc_pidReset } forEach [_pidPitch, _pidPitch_att, _pidSpd];
    _heli setVariable ["bmkhs_posIntY", 0.0];
};
if (_boRoll) then {
    _attHoldCycRollOut = 0.0;
    { [_x] call bmkhs_fnc_pidReset } forEach [_pidRoll, _pidRoll_att];
    _heli setVariable ["bmkhs_posIntX", 0.0];
};

//systemChat format ["Dist = %4 -- DistX = %1 -- DistY = %2 -- Dir = %3", _distX toFixed 2, _distY toFixed 2, _dir toFixed 2, _dist toFixed 2];
//systemChat format ["VelX = %1 -- VelY = %2 -- Pitch Out = %3 -- Roll Out = %4", _curVelX toFixed 2, _curVelY toFixed 2, _attHoldCycPitchOut toFixed 2, _attHoldCycRollOut toFixed 2];

_attHoldCycPitchOut = [_attHoldCycPitchOut, -_authority, _authority] call BIS_fnc_clamp;
_attHoldCycRollOut  = [_attHoldCycRollOut, -_authority, _authority] call BIS_fnc_clamp;

[_attHoldCycPitchOut, _attHoldCycRollOut]
