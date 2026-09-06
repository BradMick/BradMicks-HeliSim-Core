/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_prestonPedal

Description:
    Preston Pilot AI - the machine pilot's FEET on the pedals, as fn_prestonPilot
    is its hands.

    Three regimes, blended by weight and never switched:
        HDG  (hover)        - hold HEADING (deg); the ball is meaningless without airflow
        NTT  (accel, <50ft) - NOSE-TO-TAIL: drive kinematic sideslip (beta_deg) to zero
        AERO (>50ft AGL)    - AERODYNAMIC: drive lateral accel (beta_g) to zero, ball centred

    TWO CALLERS, deliberately:
        fn_preston  - when Preston is flying the aircraft (AI / CPG)
        fn_getInput - when a HUMAN pilot has the "auto pedal" option enabled
    The second is a player accommodation and must keep working independently of
    whether Preston is engaged.

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

/////////////////////////////////////////////////////////////////////////////////////////////
// KB Pedal Yaw         /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
private _yawBreakoutVal = (inputAction "HeliRudderRight") - (inputAction "HeliRudderLeft");
if (_yawBreakoutVal < -0.01 || _yawBreakoutVal > 0.01) then {
    _yawBreakout = true;
};

private _pidAutoPedalHdg  = _heli getVariable "bmkhs_pid_autoPedalHdg";
private _pidAutoPedalNtt  = _heli getVariable "bmkhs_pid_autoPedalNtt";
private _pidAutoPedalAero = _heli getVariable "bmkhs_pid_autoPedalAero";

private _hdgOut        = 0.0;
private _yawOutput     = 0.0;
private _curHdg        = getDir _heli;
private _desiredHdg    = _heli getVariable "bmkhs_autoPedalHdg";
private _hdgError      = 0.0;

//Heading capture. The setpoint is re-captured only while the pilot is actually on the pedals
//(yaw breakout) - NOT every frame above 5kts as it used to be. Re-capturing continuously pinned
//_hdgError to exactly zero at any speed above a hover, which both removed heading hold from the
//Above the breakout the pedals are the pilot's; the heading PID re-engages against the heading
//held at release (see the release-capture below).
if (_yawBreakout) then {
    _desiredHdg       = getDir _heli;
    _heli setVariable ["bmkhs_autoPedalHdg",     _desiredHdg, true];
};
if (_yawBreakout || _gndSpeed > POS_HOLD_SPEED_SWITCH) then {
    _kbPedalLeftRight = [_kbPedalLeftRight, _pedalLeftRight, (1.0 / 0.1) * _deltaTime] call BIS_fnc_lerp;
    _kbPedalLeftRight = [_kbPedalLeftRight, -1.0, 1.0] call BIS_fnc_clamp;
    _pedalLeftRight   = _kbPedalLeftRight;

    _heli setVariable ["bmkhs_kbPedalLeftRight", _kbPedalLeftRight];
} else {
    _heli setVariable ["bmkhs_kbPedalLeftRight", 0.0];
};

//THREE REGIMES, each with its own error signal, units and PID:
//  HDG  (hover)        -> hold HEADING (deg); the ball is meaningless without airflow
//  NTT  (accel, <50ft) -> NOSE-TO-TAIL: drive kinematic sideslip (beta_deg) to zero
//  AERO (>50ft AGL)    -> AERODYNAMIC: drive lateral accel (beta_g) to zero, ball centred
//Blended by weight, never switched: hover->NTT over ground speed, NTT->AERO over AGL.
//
//ERROR SIGNS ARE VETTED - do not unify them. Heading is (actual - desired); the slip channels
//are negated, because a heading error and a lateral acceleration need opposite pedal sense.
//
//AERO uses the per-vehicle beta_g, NOT the gauge sideslip global - that one is clamped at
//0.15g (the controller would go blind past it) and is stale for AI aircraft.
private _betaG    = _heli getVariable "bmkhs_aero_beta_g";     // g,   + = accel right
private _betaDeg  = _heli getVariable "bmkhs_aero_beta_deg";   // deg, + = flow from right
//Use the RAW (3rd) return - the displayed radalt is rounded to 10ft above 50ft, which would
//turn the blend below into a staircase.
([_heli] call bmkhs_fnc_stateAltitude) params ["", "", "_radAltRaw"];

//Slip setpoints are zero, error formed (desired - actual). Heading keeps (actual - desired).
private _desiredSlip = 0.0;
private _nttError    = _desiredSlip - _betaDeg;
private _aeroError   = _desiredSlip - _betaG;

//PILOT DEADBAND. A real pilot does not chase a ball that is a hair off centre - they accept a
//small standing skid and only correct once it is actually visible. Feeding raw error straight to
//the PID makes the pedals twitch at every tiny fluctuation in lateral g (which is noisy: it is a
//filtered accelerometer, not a clean signal). Subtracting the deadband rather than zeroing inside
//it keeps the response CONTINUOUS - no step at the edge of the band, the correction just starts
//from zero once the error is worth correcting.
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

_hdgError            = [_curHdg - _desiredHdg] call CBA_fnc_simplifyAngle180;
_hdgOut              = [_pidAutoPedalHdg,  _deltaTime, 0.0, _hdgError]  call bmkhs_fnc_pidRun;
private _nttOut      = [_pidAutoPedalNtt,  _deltaTime, 0.0, _nttError]  call bmkhs_fnc_pidRun;
private _aeroOut     = [_pidAutoPedalAero, _deltaTime, 0.0, _aeroError] call bmkhs_fnc_pidRun;

//Blend WEIGHTS.
//
//  wHover : 1 at a hover -> 0 by the hover handover speed. Owns the HEADING channel.
//  wFast  : 0 below 50kt -> 1 above it.   "fast enough for aerodynamic trim"
//  wHigh  : 0 below 50ft -> 1 above it.   "high enough for aerodynamic trim"
//
//AERODYNAMIC trim requires BOTH fast AND high - hence min(). LOW **OR** SLOW means
//nose-to-tail trim: NOE flight can sit above 50ft and still be nose-to-tail, as is a fast run
//down low. Airspeed alone gates the HEADING channel - a 1kt hover at 200ft is still a hover.
private _wHover = 1.0 - (linearConversion [0.0, _kbYawSwitchVel, _gndSpeed, 0.0, 1.0, true]);
private _wFast  = linearConversion [AUTOPEDAL_AERO_SPD_LO, AUTOPEDAL_AERO_SPD_HI, _gndSpeed,  0.0, 1.0, true];
private _wHigh  = linearConversion [AUTOPEDAL_NTT_AGL_FT,  AUTOPEDAL_AERO_AGL_FT, _radAltRaw, 0.0, 1.0, true];
//Aero share of the NON-hover authority: both gates must be open (min = logical AND).
private _wCruise = _wFast min _wHigh;

private _wHdg   = _wHover;
private _wAero  = (1.0 - _wHover) * _wCruise;
private _wNtt   = (1.0 - _wHover) * (1.0 - _wCruise);

_yawOutput      = (_hdgOut * _wHdg) + (_nttOut * _wNtt) + (_aeroOut * _wAero);
_yawOutput      = [_yawOutput, -1.0, 1.0] call BIS_fnc_clamp;

//PILOT FEET MODEL - a rate limit (feet have mass, pedals have breakout) and a first-order lag
//(a press, not micro-jabs) applied to the BLENDED output, so handovers are smoothed too. The
//PIDs still see true error; this only shapes delivery.
//
//ORDER MATTERS: lag FIRST, then rate-limit. The other way round takes a fraction of an
//already-limited step, so the output converges short of the commanded pedal - and the
//shortfall worsens at higher frame rates.
private _pedalPrev = _heli getVariable "bmkhs_autoPedalPrevOut";
private _lagCoef   = [(_deltaTime / AUTOPEDAL_PEDAL_TAU), 0.0, 1.0] call BIS_fnc_clamp;
private _pedalLag  = _pedalPrev + ((_yawOutput - _pedalPrev) * _lagCoef);
//Rate limit: cap the per-frame CHANGE at what a foot could actually move in this timestep.
private _maxStep   = AUTOPEDAL_PEDAL_RATE * _deltaTime;
private _step      = [_pedalLag - _pedalPrev, -_maxStep, _maxStep] call BIS_fnc_clamp;
_yawOutput         = [_pedalPrev + _step, -1.0, 1.0] call BIS_fnc_clamp;
_heli setVariable ["bmkhs_autoPedalPrevOut", _yawOutput];

//pedals - mid-transition all three contribute and the response is unattributable.
private _regime = "hdg";
private _wDom   = _wHdg;
if (_wNtt  > _wDom) then { _regime = "ntt";  _wDom = _wNtt;  };
if (_wAero > _wDom) then { _regime = "aero"; _wDom = _wAero; };

_heli setVariable ["bmkhs_autoPedalRegime",    _regime];
_heli setVariable ["bmkhs_autoPedalRegimeWgt", _wDom];
//what the loop is acting on.
_heli setVariable ["bmkhs_autoPedalHdgErr",    _hdgError];
_heli setVariable ["bmkhs_autoPedalNttErr",    _nttError];
_heli setVariable ["bmkhs_autoPedalAeroErr",   _aeroError];
_heli setVariable ["bmkhs_autoPedalOut",       _yawOutput];

private _hdgHoldBreakout     = (_pedalLeftRight <= -HDG_HOLD_BREAKOUT_VALUE && _pedalLeftRight < 0.0) || (_pedalLeftRight >= HDG_HOLD_BREAKOUT_VALUE && _pedalLeftRight > 0.0);
private _prevHdgHoldBreakout = _heli getVariable ["bmkhs_prevAutoPedalHdgBreakout", false];
if (_yawBreakout) then {
    [_pidAutoPedalHdg]  call bmkhs_fnc_pidReset;
    [_pidAutoPedalNtt]  call bmkhs_fnc_pidReset;
    [_pidAutoPedalAero] call bmkhs_fnc_pidReset;
    //Re-seed the pilot-feet filter to where the PILOT'S pedal actually is, so when they release
    //the auto-pedal picks up from that position instead of rate-limiting back from a stale one.
    _heli setVariable ["bmkhs_autoPedalPrevOut", _pedalLeftRight];
} else {
    //Release capture: on the falling edge of the pedal breakout, hold the heading the pilot let
    //go at. No longer gated to sub-5kt - the heading channel is live at every speed now, so the
    //setpoint has to be re-captured on release at every speed too, or the PID would fight to
    //recover a heading from before the pilot's pedal input.
    if (_prevHdgHoldBreakout && !_hdgHoldBreakout) then {
        _heli setVariable ["bmkhs_autoPedalHdg", getDir _heli, true];
    };
    _heli setVariable ["bmkhs_forceTrimPosYaw", _yawOutput, true];
};
_heli setVariable ["bmkhs_prevAutoPedalHdgBreakout", _hdgHoldBreakout];

[_pedalLeftRight, _yawBreakout]
