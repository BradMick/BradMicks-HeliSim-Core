/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputAutoPedal

Description:
    Auto pedal - an input assist that works the pedals for the pilot.

    Three regimes, selected by hysteresis and blended across the transition band:
        HDG  (hover)        - hold HEADING (deg); the ball is meaningless without airflow
        NTT  (accel, <50ft) - NOSE-TO-TAIL: drive kinematic sideslip (beta_deg) to zero
        AERO (>50ft AGL)    - AERODYNAMIC: drive lateral accel (beta_g) to zero, ball centred

Parameters:
    _heli             - The helicopter [Object].
    _deltaTime        - Frame time [Number].
    _pedalLeftRight   - Current pedal input [Number].
    _kbPedalLeftRight - Keyboard pedal input [Number].
    _kbYawSwitchVel   - Hover -> nose-to-tail handover speed [Number].

Returns:
    [_pedalLeftRight, _yawBreakout] - the pedal to pass downstream, and whether the
    pilot is on the pedals this frame.

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli", "_deltaTime", "_pedalLeftRight", "_kbPedalLeftRight", "_kbYawSwitchVel"];

private _yawBreakout = false;
private _gndSpeed    = (_heli getVariable "bmkhs_gndSpeed") * KNOTS_TO_MPS;

private _yawBreakoutVal = (inputAction "HeliRudderRight") - (inputAction "HeliRudderLeft");
if (_yawBreakoutVal < -0.01 || _yawBreakoutVal > 0.01) then {
    _yawBreakout = true;
};

private _pidAutoPedalHdg  = _heli getVariable "bmkhs_pid_autoPedalHdg";
private _pidAutoPedalNtt  = _heli getVariable "bmkhs_pid_autoPedalNtt";
private _pidAutoPedalAero = _heli getVariable "bmkhs_pid_autoPedalAero";

private _hdgOut     = 0.0;
private _nttOut     = 0.0;
private _aeroOut    = 0.0;
private _yawOutput  = 0.0;
private _curHdg     = getDir _heli;
private _desiredHdg = _heli getVariable "bmkhs_autoPedalHdg";

//Use the RAW radar altitude - the displayed one is rounded to 10ft above 50ft.
private _radAltRaw = (_heli getVariable "bmkhs_radAltRaw") * METERS_TO_FEET;

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
//NTT <-> AERO handover on height, once out of the hover regime.
if (_regime == "ntt" && _radAltRaw >= AUTOPEDAL_AERO_AGL_FT) then {
    _regime = "aero";
    [_pidAutoPedalNtt] call bmkhs_fnc_pidReset;
};
if (_regime == "aero" && _radAltRaw < AUTOPEDAL_NTT_AGL_FT) then {
    _regime = "ntt";
    [_pidAutoPedalAero] call bmkhs_fnc_pidReset;
};
_heli setVariable ["bmkhs_autoPedalRegime", _regime];

//The setpoint tracks the aircraft while the pilot is on the pedals AND at any speed above
//the hover gate - above it the slip channels own the axis, so a frozen heading would have
//the heading PID fighting them. Below it, in a hover, the setpoint holds and heading hold works.
//The keyboard pedal is lerped BOTH ways - in toward the key the pilot is holding, and back
//to centre when they let go. A key is on or off; the pedal it drives is not.
private _kbTarget = [_pedalLeftRight, 0.0] select (!_yawBreakout);
_kbPedalLeftRight = [_kbPedalLeftRight, _kbTarget, (1.0 / AUTOPEDAL_KB_FADE_TIME) * _deltaTime] call BIS_fnc_lerp;
_kbPedalLeftRight = [_kbPedalLeftRight, -1.0, 1.0] call BIS_fnc_clamp;
_heli setVariable ["bmkhs_kbPedalLeftRight", _kbPedalLeftRight];

if (_yawBreakout || _velFwd > AUTOPEDAL_NTT_SPD) then {
    _desiredHdg     = getDir _heli;
    _pedalLeftRight = _kbPedalLeftRight;
    _heli setVariable ["bmkhs_autoPedalHdg", _desiredHdg, true];
};

//ERROR SIGNS ARE VETTED - do not unify them. Heading is (actual - desired); the slip channels
//are negated, because a heading error and a lateral acceleration need opposite pedal sense.
private _betaG   = _heli getVariable "bmkhs_aero_beta_g";     // g,   + = accel right
private _betaDeg = _heli getVariable "bmkhs_aero_beta_deg";   // deg, + = flow from right

private _hdgError  = [_curHdg - _desiredHdg] call CBA_fnc_simplifyAngle180;
private _nttError  = 0.0 - _betaDeg;
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

//Only the active regime's PID runs.
switch (_regime) do {
    case "hdg": {
        _hdgOut = [_pidAutoPedalHdg, _deltaTime, 0.0, _hdgError] call bmkhs_fnc_pidRun;
        _hdgOut = [_hdgOut, -1.0, 1.0] call BIS_fnc_clamp;
    };
    case "ntt": {
        _nttOut = [_pidAutoPedalNtt, _deltaTime, 0.0, _nttError] call bmkhs_fnc_pidRun;
        _nttOut = [_nttOut, -1.0, 1.0] call BIS_fnc_clamp;
    };
    case "aero": {
        _aeroOut = [_pidAutoPedalAero, _deltaTime, 0.0, _aeroError] call bmkhs_fnc_pidRun;
        _aeroOut = [_aeroOut, -1.0, 1.0] call BIS_fnc_clamp;
    };
};

//Only one regime is ever active, so its output IS the pedal - blending it against the
//idle channels would hand back a fraction of the authority the active one asked for.
//The hysteresis band is what keeps the handover from chattering.
_yawOutput = _hdgOut + _nttOut + _aeroOut;

//Normalise the clamp by deltaTime so the angular impulse stays constant regardless of
//step size - prevents Euler instability during large time steps.
private _apClamp = [0.033 / (_deltaTime max 0.001), 0.0, 1.0] call BIS_fnc_clamp;
_yawOutput = [_yawOutput, -_apClamp, _apClamp] call BIS_fnc_clamp;

_heli setVariable ["bmkhs_autoPedalRegimeWgt", 1.0];
_heli setVariable ["bmkhs_autoPedalHdgErr",    _hdgError];
_heli setVariable ["bmkhs_autoPedalNttErr",    _nttError];
_heli setVariable ["bmkhs_autoPedalAeroErr",   _aeroError];
_heli setVariable ["bmkhs_autoPedalOut",       _yawOutput];

if (_yawBreakout) then {
    [_pidAutoPedalHdg]  call bmkhs_fnc_pidReset;
    [_pidAutoPedalNtt]  call bmkhs_fnc_pidReset;
    [_pidAutoPedalAero] call bmkhs_fnc_pidReset;
} else {
    _heli setVariable ["bmkhs_forceTrimPosYaw", _yawOutput, true];
};

[_pedalLeftRight, _yawBreakout]
