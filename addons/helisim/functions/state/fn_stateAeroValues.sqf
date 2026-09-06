/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stateAeroValues

Description:
    Calculates and returns _alpha (angle of attack) and _beta_g (sideslip) for the
    helicopter.

    Reference:
    https://www.mathworks.com/help/aeroblks/incidencesideslipairspeed.html
    https://trace.tennessee.edu/cgi/viewcontent.cgi?referer=&httpsredir=1&article=5851&context=utk_gradthes

Parameters:
    _heli - The apache helicopter to check.

Returns:
    _alpha (angle of attack) in degrees
    _beta_g (sideslip) in degrees

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

#include "\bmkhs_helisim\functions\core\core.hpp"

private _totVel   = _heli getVariable "bmkhs_velModelSpace";
private _totVelX  = _totVel # 0;
private _totVelY  = _totVel # 1;
private _totVelZ  = _totVel # 2;

//Alpha (angle of attack): airflow angle in the pitch plane (vertical vs forward velocity).
private _alpha_deg   = if (_totVelY == 0) then { 0.0; } else { atan (_totVelZ / _totVelY); };
//Beta (sideslip): airflow angle in the yaw plane (lateral vs total velocity).
private _beta_deg    = if ((vectorMagnitude _totVel) == 0.0) then { 0.0; } else { asin (_totVelX / (vectorMagnitude _totVel)); };
//Beta (sideslip): lateral specific force in G - the trim ball.
private _bodyAccel   = _heli getVariable ["bmkhs_bodyAccel", [0.0, 0.0, 0.0]];
private _accel_x     = _bodyAccel # 0;
private _beta_g_raw  = _accel_x / GRAVITY;
_beta_g_raw = [_beta_g_raw, -1.0, 1.0] call BIS_fnc_clamp;
private _dt          = _heli getVariable ["bmkhs_deltaTime", 0.03];
private _alpha       = if (_dt > 0.0 && {_dt < 1.0}) then { 1.0 - (exp (-_dt / BETA_G_TAU)) } else { 1.0 };
private _beta_g_prev = _heli getVariable "bmkhs_aero_beta_g_prev";
private _beta_g      = _beta_g_prev + ((_beta_g_raw - _beta_g_prev) * _alpha);

_heli setVariable ["bmkhs_aero_beta_deg",    _beta_deg,  true];
_heli setVariable ["bmkhs_aero_beta_g",      _beta_g,    true];
_heli setVariable ["bmkhs_aero_beta_g_prev", _beta_g,    true];
