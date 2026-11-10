/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_suspensionPlace

Description:
    Sits the aircraft on its wheels at static compression, on the first frame the
    suspension runs: the plane through the wheel bottoms onto the plane of the
    ground under them, keeping the heading. Leaves an airborne or rolling aircraft alone.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\suspension\suspension.hpp"

params ["_heli"];

private _wheels = _heli getVariable "bmkhs_wheels";

//The engine's drop at spawn already has it sliding at about 0.6 m/s
(velocity _heli) params ["_velX", "_velY"];
if (vectorMagnitude [_velX, _velY, 0.0] > SUSP_PLACE_MAX_SPEED) exitWith {};

private _fnc_bottoms = {
    _wheels apply { _heli modelToWorldWorld ((_x get "position") vectorAdd [0.0, 0.0, -(_x get "radius")]) }
};
private _fnc_grounds = {
    (call _fnc_bottoms) apply {
        private _hits = lineIntersectsSurfaces [
            _x vectorAdd [0.0, 0.0, SUSP_PLACE_RANGE],
            _x vectorAdd [0.0, 0.0, -SUSP_PLACE_RANGE],
            _heli, objNull, true, 1, "GEOM"
        ];
        [(_hits select 0) select 0, []] select (_hits isEqualTo [])
    }
};

private _grounds = call _fnc_grounds;
if ((_grounds findIf {_x isEqualTo []}) >= 0) exitWith {};

//Twice, because turning the aircraft moves its wheels over different ground
private _normalModel = _heli getVariable "bmkhs_wheelPlaneNormal";
for "_pass" from 1 to 2 do {
    private _groundUp = [_grounds] call bmkhs_fnc_suspensionPlane;
    if (_groundUp isEqualTo []) exitWith {};

    //Level at the current heading, then tilted so the wheel plane lies on the ground
    private _dir   = vectorNormalized [(vectorDir _heli) select 0, (vectorDir _heli) select 1, 0.0];
    private _up    = [0.0, 0.0, 1.0];
    private _right = _dir vectorCrossProduct _up;
    private _n0    = ((_right vectorMultiply (_normalModel select 0))
                        vectorAdd (_dir vectorMultiply (_normalModel select 1)))
                        vectorAdd (_up vectorMultiply (_normalModel select 2));
    //mathVectorRotateAroundAxis turns the other way round, so the axis is reversed
    private _axis = _groundUp vectorCrossProduct _n0;
    private _sinA = vectorMagnitude _axis;
    if (_sinA > 0.0001) then {
        private _tilt = _sinA atan2 (_n0 vectorDotProduct _groundUp);
        _axis = _axis vectorMultiply (1.0 / _sinA);
        _dir  = [_dir, _axis, _tilt] call bmkhs_fnc_mathVectorRotateAroundAxis;
        _up   = [_up,  _axis, _tilt] call bmkhs_fnc_mathVectorRotateAroundAxis;
    };
    _heli setVectorDirAndUp [_dir, _up];

    private _fresh = call _fnc_grounds;
    if ((_fresh findIf {_x isEqualTo []}) < 0) then { _grounds = _fresh; };
};

//Down or up, so the wheels rest on the ground
private _bottoms = call _fnc_bottoms;
private _rise    = 0.0;
{ _rise = _rise + ((_x select 2) - ((_bottoms select _forEachIndex) select 2)) } forEach _grounds;
_heli setPosASL ((getPosASL _heli) vectorAdd [0.0, 0.0, _rise / (count _grounds)]);

_heli setVelocity [0.0, 0.0, 0.0];
_heli setAngularVelocity [0.0, 0.0, 0.0];

//At rest and at static compression, so the wheels take over with nothing to settle
_heli setVariable ["bmkhs_wheelCompression", _wheels apply {_x get "sag"}];

//Held here until the mission's long frames have passed, then straight into the rest hold with
//the brake on: bmkhs_fnc_suspensionUpdate
_heli setVariable ["bmkhs_suspHeldPose", [getPosASL _heli, vectorDir _heli, vectorUp _heli]];
_heli setVariable ["bmkhs_suspSleepMass", getMass _heli];
_heli setVariable ["bmkhs_suspLoadSteady", 0.0];
_heli setVariable ["bmkhs_suspLoadUntil", CBA_missionTime + SUSP_LOAD_MAX_TIME];
