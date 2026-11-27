/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_suspensionParkable

Description:
    Whether the aircraft can be held parked with the brake off: every wheel over
    ground that will not move away from under it, and that ground no steeper
    along the way the wheels roll than rolling friction holds.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Parkable [Boolean]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\suspension\suspension.hpp"

params ["_heli"];

private _hits = (_heli getVariable "bmkhs_wheels") apply {
    private _bottom = _heli modelToWorldWorld ((_x get "position") vectorAdd [0.0, 0.0, -(_x get "radius")]);
    lineIntersectsSurfaces [
        _bottom vectorAdd [0.0, 0.0, SUSP_RAY_ABOVE],
        _bottom vectorAdd [0.0, 0.0, -SUSP_PLACE_RANGE],
        _heli, objNull, true, 1, "GEOM"
    ]
};
if ((_hits findIf {_x isEqualTo []}) >= 0) exitWith {false};
if ((_hits findIf {private _obj = (_x select 0) select 2; !isNull _obj && {_obj isKindOf "AllVehicles"}}) >= 0) exitWith {false};

private _up = [_hits apply {(_x select 0) select 0}] call bmkhs_fnc_suspensionPlane;
if (_up isEqualTo []) exitWith {false};

private _fwd = vectorNormalized [(vectorDir _heli) select 0, (vectorDir _heli) select 1, 0.0];
(abs ((_up vectorDotProduct _fwd) / (_up select 2))) <= ((_heli getVariable "bmkhs_wheelSettings") get "muRoll")
