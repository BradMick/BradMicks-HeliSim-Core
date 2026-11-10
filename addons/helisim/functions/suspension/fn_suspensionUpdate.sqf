/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_suspensionUpdate

Description:
    Runs the wheels each frame. Forces only; the pose is held only while the
    aircraft is not free to move - loading in, under time acceleration, idle with
    nobody aboard, or asleep on all wheels with the brake on and the collective
    down. Never reads and writes angular velocity while it is free: that throws
    away the moments from every addForce.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\suspension\suspension.hpp"

params ["_heli"];

private _wheels = _heli getVariable "bmkhs_wheels";
if (_wheels isEqualTo []) exitWith {};

//The idle gate returns before bmkhs_deltaTime is published, so an aircraft nobody has boarded reads 0.0
private _deltaTime = _heli getVariable "bmkhs_deltaTime";
if (_deltaTime <= 0.0) exitWith {};

if !(_heli getVariable "bmkhs_suspPlaced") then {
    _heli setVariable ["bmkhs_suspPlaced", true];
    [_heli] call bmkhs_fnc_suspensionPlace;
};

//Ahead of the hold, so a parked aircraft still draws its wheels
if (BMKHS_FM_DEBUG) then { [_heli] call bmkhs_fnc_suspensionDebug; };

//Read after placing, which may have just set it
private _settings   = _heli getVariable "bmkhs_wheelSettings";
private _held       = _heli getVariable "bmkhs_suspHeldPose";
private _isHeld     = _held isNotEqualTo [];
private _compressed = _heli getVariable "bmkhs_wheelCompression";
private _timeAccel  = accTime > 1.1;
//The main conditions of the idle gate: nobody aboard and nothing turning
private _idle       = ((crew _heli) findIf {isPlayer _x}) < 0
    && {((_heli getVariable "bmkhs_engState") findIf {_x != "OFF"}) < 0}
    && {(_heli getVariable "bmkhs_rtrRpm") <= 0.01};
private _mass       = getMass _heli;
private _brakeCtl   = _settings get "brakeControl";
private _brake      = _brakeCtl isNotEqualTo "" && {_heli getVariable format ["bmkhs_%1On", _brakeCtl]};
private _collDown   = (_heli getVariable "bmkhs_collectiveOutput") <= SUSP_SLEEP_COLLECTIVE;
//Read before the hold zeroes it, less the frame of free fall a held aircraft gets
private _vel        = velocity _heli;
private _moved      = vectorMagnitude [_vel select 0, _vel select 1, (_vel select 2) + (GRAVITY * _deltaTime)] > SUSP_WAKE_SPEED
    || {abs (_mass - (_heli getVariable "bmkhs_suspSleepMass")) > SUSP_WAKE_MASS_FRAC * _mass};

//Loading: waiting out the long frames of a mission starting up
private _steady = _heli getVariable "bmkhs_suspLoadSteady";
if (_steady >= 0.0) then {
    _steady = [0.0, _steady + _deltaTime] select (_deltaTime < SUSP_LOAD_LONG_FRAME);
    if (_steady >= SUSP_LOAD_STEADY_TIME || {CBA_missionTime > (_heli getVariable "bmkhs_suspLoadUntil")}) then {
        _steady = -1.0;
    };
    _heli setVariable ["bmkhs_suspLoadSteady", _steady];
};

if (_isHeld && {_timeAccel || _idle || _steady >= 0.0 || {_brake && _collDown && !_moved}}) exitWith {
    //The velocities PhysX gave it since the last hold - what is pushing on a held aircraft
    if (bmkhs_flightLog && {time - (_heli getVariable "bmkhs_suspHeldLogged") >= 1.0}) then {
        _heli setVariable ["bmkhs_suspHeldLogged", time];
        diag_log format ["SUSP,HELD,%1,coll=%2,brake=%3,idle=%4,steady=%5,accel=%6,moved=%7,vel=%8,angVel=%9,dt=%10,armaEng=%11,rtrDmg=%12",
            time toFixed 2, (_heli getVariable "bmkhs_collectiveOutput") toFixed 3,
            _brake, _idle, _steady toFixed 2, _timeAccel, _moved,
            (_vel apply {_x toFixed 3}) joinString "|",
            ((_heli vectorWorldToModel (angularVelocity _heli)) apply {(deg _x) toFixed 2}) joinString "|",
            _deltaTime toFixed 3, isEngineOn _heli, ([_heli, "mainRotor"] call bmkhs_fnc_damageGet) toFixed 2];
    };
    _held params ["_posHeld", "_dirHeld", "_upHeld"];
    _heli setPosASL _posHeld;
    _heli setVectorDirAndUp [_dirHeld, _upHeld];
    _heli setVelocity [0.0, 0.0, 0.0];
    _heli setAngularVelocity [0.0, 0.0, 0.0];
};
//Waking: no compression to difference against, so the wheels take their first-contact rate
if (_isHeld) then {
    _compressed = _wheels apply {-1.0};
    _heli setVariable ["bmkhs_suspHeldPose", []];
    _heli setVariable ["bmkhs_suspRestTime", 0.0];
    _heli setVariable ["bmkhs_wheelCompression", _compressed];
};

private _allDown  = (_compressed findIf {_x < 0.0}) < 0;
private _settled  = _allDown && {_idle || {_brake && _collDown}};
private _atRest   = _settled
    && {vectorMagnitude (velocity _heli) < SUSP_REST_SPEED}
    && {vectorMagnitude (angularVelocity _heli) < SUSP_REST_RATE};
private _restTime = [0.0, (_heli getVariable "bmkhs_suspRestTime") + _deltaTime] select _atRest;
_heli setVariable ["bmkhs_suspRestTime", _restTime];

if (_restTime >= SUSP_REST_TIME || {_timeAccel && {(_compressed findIf {_x >= 0.0}) >= 0}}) exitWith {
    _heli setVelocity [0.0, 0.0, 0.0];
    _heli setAngularVelocity [0.0, 0.0, 0.0];
    _heli setVariable ["bmkhs_suspHeldPose", [getPosASL _heli, vectorDir _heli, vectorUp _heli]];
    _heli setVariable ["bmkhs_suspSleepMass", _mass];
};

//Static load on each wheel: carries the mass with no moment about the centre of mass, about the
//wheels' centroid, floored so a wheel the CG has left still carries something
private _cg = getCenterOfMass _heli;
(_heli getVariable "bmkhs_wheelLoadFit") params ["_midX", "_midY", "_ixx", "_ixy", "_iyy"];
private _momX  = _mass * ((_cg select 0) - _midX);
private _momY  = _mass * ((_cg select 1) - _midY);
private _slopeX = (_ixx * _momX) + (_ixy * _momY);
private _slopeY = (_ixy * _momX) + (_iyy * _momY);
private _loads = _wheels apply {
    (_x get "position") params ["_px", "_py"];
    ((_mass / (count _wheels)) + (_slopeX * (_px - _midX)) + (_slopeY * (_py - _midY))) max (SUSP_MIN_LOAD_FRAC * _mass)
};
private _sum = 0.0;
{ _sum = _sum + _x } forEach _loads;
_loads = _loads apply { _x * _mass / _sum };

private _muLon   = [_settings get "muRoll", _settings get "muBrake"] select _brake;
private _results = [];
{
    private _wheel  = _x;
    private _i      = _forEachIndex;
    private _angle  = 0.0;
    private _muSide = _settings get "muSide";
    private _muRoll = [_settings get "muRoll", _muLon] select (_wheel get "braked");
    if (_wheel get "castors") then {
        ([_heli, _i, (_compressed select _i) >= 0.0, _deltaTime] call bmkhs_fnc_suspensionCastor) params ["_castor", "_pinned"];
        _angle  = _castor;
        _muSide = [_settings get "muCastorFree", _settings get "muCastorLocked"] select _pinned;
    };
    _results pushBack ([_heli, _i, _loads select _i, _muSide, _muRoll, _angle, _deltaTime] call bmkhs_fnc_suspensionWheel);
} forEach _wheels;

//One RPT line a frame while a wheel is on the ground, with Core's flight log, on its clock
if (bmkhs_flightLog && {(_results findIf {(_x select 0) >= 0.0}) >= 0}) then {
    diag_log format ["SUSP,%1,%2,%3,%4", time toFixed 2, _deltaTime toFixed 3, _brake,
        (_results apply {
            format ["%1,%2,%3,%4,%5", (_x select 0) toFixed 3, (_x select 1) toFixed 0,
                (_x select 2) toFixed 3, (_x select 3) toFixed 3, (_x select 4) toFixed 1]
        }) joinString ","];
};
