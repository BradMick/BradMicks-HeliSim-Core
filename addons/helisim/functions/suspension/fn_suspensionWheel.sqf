/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_suspensionWheel

Description:
    One wheel: a spring and damper on its compression, and friction from the load
    that puts on the ground. Forces only - rotor lift unloads the spring.

Parameters:
    _heli      - The helicopter [Object]
    _index     - Wheel index into bmkhs_wheels [Number]
    _load      - Static load on this wheel, kg [Number]
    _muSide    - Friction coefficient across the wheel [Number]
    _muRoll    - Friction coefficient along the wheel [Number]
    _angle     - Wheel heading from the aircraft's long axis, deg clockwise [Number]
    _deltaTime - Frame time, s [Number]

Returns:
    [compression (-1 off the ground), normal force N, across m/s, along m/s, angle deg]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\suspension\suspension.hpp"

params ["_heli", "_index", "_load", "_muSide", "_muRoll", "_angle", "_deltaTime"];

private _wheel  = (_heli getVariable "bmkhs_wheels") select _index;
private _pos    = _wheel get "position";
private _sag    = _wheel get "sag";
private _bottom = _heli modelToWorldWorld (_pos vectorAdd [0.0, 0.0, -(_wheel get "radius")]);
private _hits   = lineIntersectsSurfaces [
    _bottom vectorAdd [0.0, 0.0, SUSP_RAY_ABOVE],
    _bottom vectorAdd [0.0, 0.0, -_sag],
    _heli, objNull, true, 1, "GEOM"
];

private _bends = _heli getVariable "bmkhs_wheelGripBend";

//Zero at full extension, _sag where the model draws it, -1 off the ground
private _compression = -1.0;
if (_hits isNotEqualTo []) then {
    _compression = _sag - ((_bottom select 2) - ((_hits select 0 select 0) select 2));
};

//The gear animation: the stroke as a fraction of sag, 0 at full extension, 1 at static. Only when
//it has visibly moved: animateSource goes to every machine.
private _source = _wheel get "compressionSource";
if (_source isNotEqualTo "") then {
    private _stroke = (_compression max 0.0) / _sag;
    if (abs (_stroke - ((_heli getVariable "bmkhs_wheelStrokeAnimated") select _index)) > SUSP_STROKE_ANIM_STEP) then {
        [_heli, "bmkhs_wheelStrokeAnimated", _index, _stroke] call bmkhs_fnc_utilSetArrayVariable;
        _heli animateSource [_source, _stroke, true];
    };
};

if (_hits isEqualTo []) exitWith {
    [_heli, "bmkhs_wheelCompression", _index, -1.0] call bmkhs_fnc_utilSetArrayVariable;
    _bends set [_index, [0.0, 0.0]];
    [-1.0, 0.0, 0.0, 0.0, _angle]
};

//First contact has nothing to difference, so the rate is the wheel's own descent, capped at what
//this frame's compression accounts for. Arma's angular velocity is the inverse of the right hand rule.
private _prev = (_heli getVariable "bmkhs_wheelCompression") select _index;
private _rate = 0.0;
if (_prev >= 0.0) then {
    _rate = (_compression - _prev) / _deltaTime;
} else {
    private _arm     = (_heli modelToWorldWorld _pos) vectorDiff (_heli modelToWorldWorld (getCenterOfMass _heli));
    private _descent = -(((velocity _heli) vectorDiff ((angularVelocity _heli) vectorCrossProduct _arm)) select 2);
    _rate = ((_descent max -(_compression / _deltaTime)) min (_compression / _deltaTime));
};
_rate = (_rate max -SUSP_MAX_RATE) min SUSP_MAX_RATE;
[_heli, "bmkhs_wheelCompression", _index, _compression] call bmkhs_fnc_utilSetArrayVariable;

//A gas strut, per unit of the wheel's load: soft near full extension, stiffening as it compresses.
//A low frame rate eases it about the static load from both sides, so its slope there falls with
//the frame time; the shortfall below static fades out toward full extension, so a touching wheel
//does not kick.
private _omegaSq = SUSP_SPRING_EXP * GRAVITY / _sag;
private _spring  = GRAVITY * ((_compression / _sag) ^ SUSP_SPRING_EXP);
private _ease    = (SUSP_SPRING_DT_LIMIT / (_omegaSq * _deltaTime * _deltaTime)) min 1.0;
if (_spring > GRAVITY) then {
    _spring = GRAVITY + ((_spring - GRAVITY) * _ease);
} else {
    _spring = _spring + ((GRAVITY - _spring) * (1.0 - _ease) * (_compression / _sag));
};
private _zeta    = [SUSP_ZETA_REBOUND, SUSP_ZETA_COMPRESS] select (_rate > 0.0);
private _damping = (2.0 * _zeta * (sqrt _omegaSq)) min (SUSP_DAMP_DT_LIMIT / _deltaTime);
private _normal  = _load * (_spring + (_damping * _rate));
_normal          = (_normal max 0.0) min (_load * GRAVITY * SUSP_MAX_LOAD);

_heli addForce [[0.0, 0.0, _normal * _deltaTime], _pos];

//Velocity of the contact point; positive yaw is clockwise from above
private _velModel = _heli vectorWorldToModel (velocity _heli);
private _yawRate  = (_heli vectorWorldToModel (angularVelocity _heli)) select 2;
private _velX     = (_velModel select 0) + (_yawRate * (_pos select 1));
private _velY     = (_velModel select 1) - (_yawRate * (_pos select 0));

//Along and across the way the wheel is pointing
private _sin      = sin _angle;
private _cos      = cos _angle;
private _velRoll  = (_velX * _sin) + (_velY * _cos);
private _velSide  = (_velX * _cos) - (_velY * _sin);

//Grip: the tyre bends like a stiff spring until it reaches the friction limit, then slips
private _gripStiff = (SUSP_GRIP_OMEGA * SUSP_GRIP_OMEGA) min (SUSP_SPRING_DT_LIMIT / (_deltaTime * _deltaTime));
private _gripDamp  = (2.0 * SUSP_GRIP_ZETA * SUSP_GRIP_OMEGA) min (SUSP_DAMP_DT_LIMIT / _deltaTime);
private _fnc_grip  = {
    params ["_bend", "_vel", "_mu"];
    private _limit = _mu * _normal;
    private _reach = _limit / (_load * _gripStiff);
    _bend = ((_bend + (_vel * _deltaTime)) max -_reach) min _reach;
    [_bend, ((-_load * ((_gripStiff * _bend) + (_gripDamp * _vel))) max -_limit) min _limit]
};
(_bends select _index) params ["_bendSide", "_bendRoll"];
([_bendSide, _velSide, _muSide] call _fnc_grip) params ["_newSide", "_forceSide"];
([_bendRoll, _velRoll, _muRoll] call _fnc_grip) params ["_newRoll", "_forceRoll"];
_bends set [_index, [_newSide, _newRoll]];

//Applied near the height of the centre of mass, so grip does not tip the aircraft over its feet
private _cgZ     = (getCenterOfMass _heli) select 2;
private _gripPos = [_pos select 0, _pos select 1, _cgZ + (SUSP_ROLL_INFLUENCE * ((_pos select 2) - _cgZ))];
_heli addForce [
    _heli vectorModelToWorld [
        ((_forceRoll * _sin) + (_forceSide * _cos)) * _deltaTime,
        ((_forceRoll * _cos) - (_forceSide * _sin)) * _deltaTime,
        0.0
    ],
    _gripPos
];

[_compression, _normal, _velSide, _velRoll, _angle]
