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
//FD HVR is position hold, at any speed it runs at
private _hvr = _heli getVariable ["bmkhs_fd_hvr", false];
if (_hvr) then { _hasSpd = false };
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
if (_hvr) then { _want = "pos" };
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
            };
            case "vel": { _heli setVariable ["bmkhs_attHoldDesiredVel", [-_velX, _velY], true] };
            case "att": { _heli setVariable ["bmkhs_attHoldDesiredAtt", [_curPitch, _curRoll], true] };
            //The speed now
            case "spd": {
                _heli setVariable ["bmkhs_attHoldDesiredAtt", [_curPitch, _curRoll], true];
                _heli setVariable ["bmkhs_attHoldDesiredSpd", _airspeed, true];
            };
        };
    };
    //Clear derivative
    {
        _x deleteAt "prevError";
        _x set ["derivFilt", 0];
    } forEach ([_pidRoll, _pidPitch, _pidRoll_att, _pidPitch_att] + ([[], [_pidSpd]] select _hasSpd));
    //Only spd runs it
    if (_newSubMode == "spd") then { [_pidSpd] call bmkhs_fnc_pidReset };
    _heli setVariable ["bmkhs_attHoldSpdPending", false];
    _heli setVariable ["bmkhs_attHoldSpdSetPitch", nil];
};
if (_beeping) then {
    //Published on release
    _beepDelta params ["_dPitch", "_dRoll"];
    if (_newSubMode in ["att", "spd"]) then {
        (_heli getVariable "bmkhs_attHoldDesiredAtt") params ["_setPitch", "_setRoll"];
        //Speed loop folded in - no step
        if (_newSubMode == "spd" && {_dPitch != 0} && {!(_heli getVariable ["bmkhs_attHoldSpdPending", false])}) then {
            _setPitch = _heli getVariable ["bmkhs_attHoldSpdSetPitch", _setPitch];
        };
        _setPitch = _setPitch - _dPitch * (_gainDeg # 0);
        if (_newSubMode == "spd") then { _setPitch = (_setPitch max -_maxPitch) min _maxPitch };
        _heli setVariable ["bmkhs_attHoldDesiredAtt", [_setPitch, _setRoll - _dRoll * (_gainDeg # 1)]];
    };
    //Airspeed re-captured after the beep
    if (_newSubMode == "spd" && {_dPitch != 0}) then {
        _heli setVariable ["bmkhs_attHoldSpdPending", true];
        _heli setVariable ["bmkhs_attHoldSpdWait", 0.0];
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
    //FD HVR flying a velocity - position hold stands by on the present spot
    private _standBy = _hvr && {!((((_heli getVariable "bmkhs_fmc") getOrDefault ["FlightDirector", createHashMap]) getOrDefault ["hvrHeld", false]))};
    if (_newSubMode == "pos" && _standBy) then {
        { [_x] call bmkhs_fnc_pidReset } forEach [_pidRoll, _pidPitch];
        _heli setVariable ["bmkhs_attHoldDesiredPos", getPos _heli];
    };
    //Position hold - fly back to the spot at a speed set by the distance, through the velocity loop
    if (_newSubMode == "pos" && !_standBy) then {
        private _desiredPos = _heli getVariable ["bmkhs_attHoldDesiredPos", getPos _heli];
        private _dPos = _desiredPos vectorDiff (getPos _heli);
        private _hdg  = direction _heli;
        //Model space, m: X right, Y fwd
        private _posErrX = ((_dPos # 0) * cos _hdg) - ((_dPos # 1) * sin _hdg);
        private _posErrY = ((_dPos # 0) * sin _hdg) + ((_dPos # 1) * cos _hdg);

        //Closing speed, m/s
        private _vSet = [_posErrX * FMC_ATT_POS_GAIN, _posErrY * FMC_ATT_POS_GAIN, 0.0];
        private _vMax = FMC_ATT_POS_MAX_KTS * KNOTS_TO_MPS;
        if (vectorMagnitude _vSet > _vMax) then { _vSet = (vectorNormalized _vSet) vectorMultiply _vMax };

        //Roll measures -velX
        private _roll  = [_pidRoll,  _deltaTime, -(_vSet # 0), -_velX] call bmkhs_fnc_pidRun;
        _roll          = [_roll,  -1.0, 1.0] call BIS_fnc_clamp;
        private _pitch = [_pidPitch, _deltaTime,  (_vSet # 1),  _velY] call bmkhs_fnc_pidRun;
        _pitch         = [_pitch, -1.0, 1.0] call BIS_fnc_clamp;

        _attHoldCycPitchOut = _pitch;
        _attHoldCycRollOut  = _roll;
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
                //Beeped attitude held until the speed settles or the delay ends
                private _wait = 0.0;
                if (!_beeping) then { _wait = (_heli getVariable ["bmkhs_attHoldSpdWait", 0.0]) + _deltaTime };
                _heli setVariable ["bmkhs_attHoldSpdWait", _wait];
                //Attitude reference kept - no step
                if (!_beeping && {abs _spdRate < _settle || {_wait >= FMC_ATT_SPD_DELAY_S}}) then {
                    _heli setVariable ["bmkhs_attHoldSpdPending", false];
                    _heli setVariable ["bmkhs_attHoldDesiredSpd", _airspeed, true];
                };
                [_pidSpd] call bmkhs_fnc_pidReset;
            } else {
                //Slow is nose down
                _setPitch = _setPitch - ([_pidSpd, _deltaTime, _heli getVariable "bmkhs_attHoldDesiredSpd", _airspeed] call bmkhs_fnc_pidRun);
            };
            _setPitch = (_setPitch max -_maxPitch) min _maxPitch;
            _heli setVariable ["bmkhs_attHoldSpdSetPitch", _setPitch];
        };
        //Heading hold's bank
        _setRoll = _setRoll + (_heli getVariable ["bmkhs_hdgHoldBankDeg", 0.0]);
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
    _heli setVariable ["bmkhs_attHoldSpdSetPitch", nil];
};

//Not active axes - no output, loops cleared, reference kept
if (_boPitch) then {
    _attHoldCycPitchOut = 0.0;
    { [_x] call bmkhs_fnc_pidReset } forEach [_pidPitch, _pidPitch_att, _pidSpd];
};
if (_boRoll) then {
    _attHoldCycRollOut = 0.0;
    { [_x] call bmkhs_fnc_pidReset } forEach [_pidRoll, _pidRoll_att];
};

//systemChat format ["Dist = %4 -- DistX = %1 -- DistY = %2 -- Dir = %3", _distX toFixed 2, _distY toFixed 2, _dir toFixed 2, _dist toFixed 2];
//systemChat format ["VelX = %1 -- VelY = %2 -- Pitch Out = %3 -- Roll Out = %4", _curVelX toFixed 2, _curVelY toFixed 2, _attHoldCycPitchOut toFixed 2, _attHoldCycRollOut toFixed 2];

_attHoldCycPitchOut = [_attHoldCycPitchOut, -_authority, _authority] call BIS_fnc_clamp;
_attHoldCycRollOut  = [_attHoldCycRollOut, -_authority, _authority] call BIS_fnc_clamp;

[_attHoldCycPitchOut, _attHoldCycRollOut]
