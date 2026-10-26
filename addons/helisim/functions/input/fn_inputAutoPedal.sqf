/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputAutoPedal

Description:
    Auto pedal - an input assist that works the pedals for the pilot, like a pilot's feet.

    Three regimes, selected by hysteresis:
        HDG  (hover)        - hold HEADING (deg); the ball is meaningless without airflow
        NTT  (accel, <50ft) - NOSE-TO-TAIL: drive the ground track's angle off the nose to zero
        AERO (>50ft AGL)    - AERODYNAMIC: drive lateral accel (beta_g) to zero, ball centred

    The foot is critically damped - it eases off, eases in, and never overshoots on its own.
    On release it starts from the pilot's pedal and, in the hover, eases to the pedal that
    stops the yaw, then holds the heading. An analog pedal springing back faster than a foot
    moves is a release, the same as a key.

Parameters:
    _heli           - The helicopter [Object].
    _deltaTime      - Frame time [Number].
    _pedalLeftRight - Pilot's pedal [Number].

Returns:
    The pilot's pedal to pass downstream - zero once released [Number].

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli", "_deltaTime", "_pedalLeftRight"];

private _pidAutoPedalHdg  = _heli getVariable "bmkhs_pid_autoPedalHdg";
private _pidAutoPedalNtt  = _heli getVariable "bmkhs_pid_autoPedalNtt";
private _pidAutoPedalAero = _heli getVariable "bmkhs_pid_autoPedalAero";

private _hdgOut     = 0.0;
private _nttOut     = 0.0;
private _aeroOut    = 0.0;
private _curHdg     = getDir _heli;
private _desiredHdg = _heli getVariable "bmkhs_autoPedalHdg";

//Use the RAW radar altitude - the displayed one is rounded to 10ft above 50ft.
private _radAltRaw = (_heli getVariable "bmkhs_radAlt") * METERS_TO_FEET;

//FORWARD speed, not ground speed - a sideways hover drift is not the aircraft leaving the hover.
private _velFwd = (_heli getVariable "bmkhs_velModelSpaceNoWind") select 1;

//Regime, with hysteresis so the boundary cannot chatter. Only the active PID runs;
//the idle ones are held reset so nothing winds up while contributing nothing.
private _regime = _heli getVariable "bmkhs_autoPedalRegime";
//The hover gate and the regime switch are the SAME threshold, or nothing works in between.
if (_regime == "hdg" && _velFwd >= AUTOPEDAL_NTT_SPD) then {
    _regime = "ntt";
    [_pidAutoPedalHdg] call bmkhs_fnc_pidReset;
};
if (_regime != "hdg" && _velFwd < AUTOPEDAL_NTT_SPD) then {
    _regime = "hdg";
    [_pidAutoPedalNtt]  call bmkhs_fnc_pidReset;
    [_pidAutoPedalAero] call bmkhs_fnc_pidReset;
};
//NTT <-> AERO - ball only when high AND fast
private _gsKts = (_heli getVariable "bmkhs_gndSpeed") * MPS_TO_KNOTS;
if (_regime == "ntt" && {_radAltRaw > AUTOPEDAL_AERO_IN_FT && _gsKts > AUTOPEDAL_AERO_IN_KTS}) then {
    _regime = "aero";
    [_pidAutoPedalNtt] call bmkhs_fnc_pidReset;
};
if (_regime == "aero" && {_radAltRaw < AUTOPEDAL_AERO_OUT_FT || _gsKts < AUTOPEDAL_AERO_OUT_KTS}) then {
    _regime = "ntt";
    [_pidAutoPedalAero] call bmkhs_fnc_pidReset;
};
_heli setVariable ["bmkhs_autoPedalRegime", _regime];

private _yawRate = deg ((_heli getVariable "bmkhs_angVelModelSpace") select 2);
private _ftYaw   = _heli getVariable "bmkhs_forceTrimPosYaw";
private _pos     = _heli getVariable "bmkhs_autoPedalPos";
private _trim    = _heli getVariable "bmkhs_autoPedalTrim";
private _w       = AUTOPEDAL_FOOT_95PCT / AUTOPEDAL_FOOT_TIME;

//On the pedals - unless the pedal is springing back faster than a foot moves
private _raw      = abs ((_heli getVariable "bmkhs_heliRudderRightOut") - (_heli getVariable "bmkhs_heliRudderLeftOut"));
private _rawPrev  = _heli getVariable "bmkhs_autoPedalRawPrev";
private _onPedals = _raw > 0.01;
private _pilot    = [_pedalLeftRight, _ftYaw] call bmkhs_fnc_inputGetInterp;
private _sprung   = _onPedals && {_heli getVariable "bmkhs_autoPedalSprung"};
if (_onPedals) then {
    if (_raw > _rawPrev) then { _sprung = false };                          //pushing again
    if (_rawPrev - _raw > _w * _deltaTime) then { _sprung = true };         //spring, not a foot
};
_heli setVariable ["bmkhs_autoPedalRawPrev", _raw];
_heli setVariable ["bmkhs_autoPedalSprung", _sprung];
private _engaged = _onPedals && !_sprung;

//The setpoint tracks the aircraft while the pilot is on the pedals AND at any speed above
//the hover gate - above it the slip channels own the axis. Below it, the hover heading holds.
if (_engaged || _velFwd > AUTOPEDAL_NTT_SPD) then {
    _desiredHdg = getDir _heli;
    _heli setVariable ["bmkhs_autoPedalHdg", _desiredHdg];
    _heli setVariable ["bmkhs_autoPedalHdgSet", !_engaged];   //slowing into the hover holds at once
    if (!_sprung) then {
        _trim = _ftYaw;     //pedal before the turn
        _heli setVariable ["bmkhs_autoPedalTrim", _trim];
    };
} else {
    //Hold where the turn stopped - or where the foot reached the pedal it was easing to
    private _feel = [_trim - AUTOPEDAL_FEEL * _yawRate, -1.0, 1.0] call BIS_fnc_clamp;
    if (!(_heli getVariable "bmkhs_autoPedalHdgSet") && {abs _yawRate < AUTOPEDAL_HDG_SETTLE_DPS || {abs (_pos - _feel) < AUTOPEDAL_FOOT_ARRIVED}}) then {
        _desiredHdg = getDir _heli;
        _heli setVariable ["bmkhs_autoPedalHdg", _desiredHdg];
        _heli setVariable ["bmkhs_autoPedalHdgSet", true];
    };
};

//ERROR SIGNS ARE VETTED - do not unify them. Heading is (actual - desired); the slip channels
//are negated, because a heading error and a lateral acceleration need opposite pedal sense.
private _betaG   = _heli getVariable "bmkhs_aero_beta_g";     // g,   + = accel right

//Nose-to-tail is the FLIGHT PATH on the nose - the ground track, not the airflow. The tail
//follows the nose past obstacles, and an aircraft that settles in moves straight onto its gear
//instead of sideways into a dynamic rollover. Air sideslip would crab it into a crosswind.
//Same sense as beta_deg (+ = moving right of the nose), so calm air is unchanged.
(_heli getVariable "bmkhs_velModelSpaceNoWind") params ["_gndVelX", "_gndVelY"];
private _gndVelXY = vectorMagnitude [_gndVelX, _gndVelY, 0.0];
private _trackDeg = if (_gndVelXY > 0.0) then { asin (_gndVelX / _gndVelXY) } else { 0.0 };

private _hdgError  = [_curHdg - _desiredHdg] call CBA_fnc_simplifyAngle180;
private _nttError  = 0.0 - _trackDeg;
private _aeroError = 0.0 - _betaG;

//PILOT DEADBAND. Subtracting the band rather than zeroing inside it keeps the response
//continuous - the correction just starts from zero once the error is worth correcting.
if (abs _aeroError <= AUTOPEDAL_AERO_DEADBAND_G) then {
    _aeroError = 0.0;
} else {
    _aeroError = _aeroError - (AUTOPEDAL_AERO_DEADBAND_G * ([1, -1] select (_aeroError < 0)));
};
if (abs _nttError <= AUTOPEDAL_NTT_DEADBAND_DEG) then {
    _nttError = 0.0;
} else {
    _nttError = _nttError - (AUTOPEDAL_NTT_DEADBAND_DEG * ([1, -1] select (_nttError < 0)));
};

//Hand-off: a loop fresh from reset starts from the pedal the foot holds - nothing moves at the switch
private _takeOver = {
    params ["_pid", "_err"];
    if !("prevError" in _pid) then {
        private _ki = _pid get "ki";
        private _i  = if (_ki == 0) then { 0.0 } else { (_pos - (_pid get "kp") * _err) / _ki };
        _pid set ["integral", [_i, -(_pid get "ki_clamp"), _pid get "ki_clamp"] call BIS_fnc_clamp];
        _pid set ["prevError", _err];
    };
};

//Only the active regime's PID runs.
switch (_regime) do {
    case "hdg": {
        if (_heli getVariable "bmkhs_autoPedalHdgSet") then {
            [_pidAutoPedalHdg, -_hdgError] call _takeOver;
            _hdgOut = [_pidAutoPedalHdg, _deltaTime, 0.0, _hdgError] call bmkhs_fnc_pidRun;
            _hdgOut = [_hdgOut, -1.0, 1.0] call BIS_fnc_clamp;
        } else {
            //Ease toward the pedal that stops the yaw - settles where the yaw stops
            _hdgOut = [_trim - AUTOPEDAL_FEEL * _yawRate, -1.0, 1.0] call BIS_fnc_clamp;
        };
    };
    case "ntt": {
        [_pidAutoPedalNtt, -_nttError] call _takeOver;
        _nttOut = [_pidAutoPedalNtt, _deltaTime, 0.0, _nttError] call bmkhs_fnc_pidRun;
        _nttOut = [_nttOut, -1.0, 1.0] call BIS_fnc_clamp;
    };
    case "aero": {
        [_pidAutoPedalAero, -_aeroError] call _takeOver;
        _aeroOut = [_pidAutoPedalAero, _deltaTime, 0.0, _aeroError] call bmkhs_fnc_pidRun;
        _aeroOut = [_aeroOut, -1.0, 1.0] call BIS_fnc_clamp;
    };
};

//Only one regime is ever active, so its output IS the pedal.
private _yawOutput = _hdgOut + _nttOut + _aeroOut;

//The foot - critically damped: eases off, eases in, no overshoot of its own
if (_engaged) then {
    [_pidAutoPedalHdg]  call bmkhs_fnc_pidReset;
    [_pidAutoPedalNtt]  call bmkhs_fnc_pidReset;
    [_pidAutoPedalAero] call bmkhs_fnc_pidReset;
    //Sits on the pilot's pedal, at rest - release starts from there
    _heli setVariable ["bmkhs_autoPedalPos", _pilot];
    _heli setVariable ["bmkhs_autoPedalVel", 0.0];
} else {
    private _vel = _heli getVariable "bmkhs_autoPedalVel";
    _vel = _vel + (_w * _w * (_yawOutput - _pos) - 2.0 * _w * _vel) * _deltaTime;
    _pos = [_pos + _vel * _deltaTime, -1.0, 1.0] call BIS_fnc_clamp;
    _heli setVariable ["bmkhs_autoPedalPos", _pos];
    _heli setVariable ["bmkhs_autoPedalVel", _vel];
    _heli setVariable ["bmkhs_forceTrimPosYaw", _pos];
};

[_pedalLeftRight, 0.0] select _sprung
