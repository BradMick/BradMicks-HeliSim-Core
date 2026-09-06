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

private _wAccX_avg = _heli getVariable "bmkhs_worldAccelX_avg";
private _wAccY_avg = _heli getVariable "bmkhs_worldAccelY_avg";
private _wAccZ_avg = _heli getVariable "bmkhs_worldAccelZ_avg";
private _worldAccelF = [
    [_wAccX_avg, _worldAccel # 0] call bmkhs_fnc_utilSmoothAverage,
    [_wAccY_avg, _worldAccel # 1] call bmkhs_fnc_utilSmoothAverage,
    [_wAccZ_avg, _worldAccel # 2] call bmkhs_fnc_utilSmoothAverage
];
_heli setVariable ["bmkhs_worldAccelFiltered", _worldAccelF];

private _velX_prev  = _heli getVariable "bmkhs_velX_prev";
private _accelX     = _heli getVariable "bmkhs_accelX";
private _accelX_avg = _heli getVariable "bmkhs_accelX_avg";

private _velY_prev  = _heli getVariable "bmkhs_velY_prev";
private _accelY     = _heli getVariable "bmkhs_accelY";
private _accelY_avg = _heli getVariable "bmkhs_accelY_avg";

private _velZ_prev  = _heli getVariable "bmkhs_velZ_prev";
private _accelZ     = _heli getVariable "bmkhs_accelZ";
private _accelZ_avg = _heli getVariable "bmkhs_accelZ_avg";

//X Axis Acceleration
private _velX = (_heli getVariable "bmkhs_velModelSpaceNoWind") select 0;
_accelX       = [_accelX_avg, (_velX - _velX_prev) / _deltaTime] call bmkhs_fnc_utilSmoothAverage;
_velX_prev    = _velX;

//Y Axis Acceleration
private _velY = (_heli getVariable "bmkhs_velModelSpaceNoWind") select 1;
_accelY       = [_accelY_avg, (_velY - _velY_prev) / _deltaTime] call bmkhs_fnc_utilSmoothAverage;
_velY_prev    = _velY;

//Z Axis Acceleration
private _velZ = (_heli getVariable "bmkhs_velModelSpaceNoWind") select 2;
_accelZ       = [_accelZ_avg, (_velZ - _velZ_prev) / _deltaTime] call bmkhs_fnc_utilSmoothAverage;
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

_heli setVariable ["bmkhs_bodyAccel", [
    _aWorld vectorDotProduct _rightB,
    _aWorld vectorDotProduct _dirB,
    _aWorld vectorDotProduct _upB
]];
