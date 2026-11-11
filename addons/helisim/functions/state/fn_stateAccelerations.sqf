/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stateAccelerations

Description:


Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:


Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

private _deltaTime  = _heli getVariable "bmkhs_deltaTime";
if (_deltaTime < 0.0001) exitWith {};

private _worldVel_prev = _heli getVariable "bmkhs_velWorldSpaceNoWind_prev";
private _worldVel      = _heli getVariable "bmkhs_velWorldSpaceNoWind";
private _worldAccel    = (_worldVel vectorDiff _worldVel_prev) vectorMultiply (1 / _deltaTime);

_heli setVariable ["bmkhs_worldAccel", _worldAccel];
_heli setVariable ["bmkhs_velWorldSpaceNoWind_prev", _worldVel];

private _worldAccelF = [_heli getVariable "bmkhs_worldAccelFiltered", _worldAccel, _deltaTime, ACCEL_SMOOTH_TIME] call bmkhs_fnc_utilSmooth;
_heli setVariable ["bmkhs_worldAccelFiltered", _worldAccelF];

private _velX_prev  = _heli getVariable "bmkhs_velX_prev";
private _accelX     = _heli getVariable "bmkhs_accelX";

private _velY_prev  = _heli getVariable "bmkhs_velY_prev";
private _accelY     = _heli getVariable "bmkhs_accelY";

private _velZ_prev  = _heli getVariable "bmkhs_velZ_prev";
private _accelZ     = _heli getVariable "bmkhs_accelZ";

//X Axis Acceleration
private _velX = (_heli getVariable "bmkhs_velModelSpaceNoWind") select 0;
_accelX       = [_accelX, (_velX - _velX_prev) / _deltaTime, _deltaTime, ACCEL_SMOOTH_TIME] call bmkhs_fnc_utilSmooth;
_velX_prev    = _velX;

//Y Axis Acceleration
private _velY = (_heli getVariable "bmkhs_velModelSpaceNoWind") select 1;
_accelY       = [_accelY, (_velY - _velY_prev) / _deltaTime, _deltaTime, ACCEL_SMOOTH_TIME] call bmkhs_fnc_utilSmooth;
_velY_prev    = _velY;

//Z Axis Acceleration
private _velZ = (_heli getVariable "bmkhs_velModelSpaceNoWind") select 2;
_accelZ       = [_accelZ, (_velZ - _velZ_prev) / _deltaTime, _deltaTime, ACCEL_SMOOTH_TIME] call bmkhs_fnc_utilSmooth;
_velZ_prev    = _velZ;

_heli setVariable ["bmkhs_velX_prev", _velX_prev];
_heli setVariable ["bmkhs_accelX",    _accelX];

_heli setVariable ["bmkhs_velY_prev", _velY_prev];
_heli setVariable ["bmkhs_accelY",    _accelY];

_heli setVariable ["bmkhs_velZ_prev", _velZ_prev];
_heli setVariable ["bmkhs_accelZ",    _accelZ];

private _aWorld = _worldAccelF vectorDiff [0.0, 0.0, -GRAVITY];
private _dirB   = vectorDir _heli;
private _upB    = vectorUp  _heli;
private _rightB = _dirB vectorCrossProduct _upB;

private _kLat = _worldAccelF vectorDotProduct _rightB;
private _gLat = [0.0, 0.0, GRAVITY] vectorDotProduct _rightB;
_heli setVariable ["bmkhs_ballTerms", [_kLat, _gLat, _kLat + _gLat]];

private _bodyAccel = [
    _aWorld vectorDotProduct _rightB,
    _aWorld vectorDotProduct _dirB,
    _aWorld vectorDotProduct _upB
];
_heli setVariable ["bmkhs_bodyAccel", _bodyAccel];
//Load factor at the CoM, g
_heli setVariable ["bmkhs_loadFactor", _bodyAccel vectorMultiply (1.0 / GRAVITY)];
