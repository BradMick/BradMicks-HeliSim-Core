/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_prestonVariables

Description:
    Defines the Preston Pilot AI state - the PIDs it flies with, its targets and
    its filter state. Called once from fn_coreConfig.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

//ATT regime pitch. Error is in DEGREES. Runs with no rate damping underneath it, so raise kp in
//small steps (0.02) and only alongside kd. kd is deliberately large relative to kp - it is what
//stops the overshoot - and dCoef is lifted off the 0.3 default so the filter keeps that lead.
_heli setVariable ["bmkhs_pid_prestonPitch",      [0.1200, 0.0200, 0.1000, 5.0000] call bmkhs_fnc_pidCreate];
(_heli getVariable "bmkhs_pid_prestonPitch") set ["dCoef", 0.5];
_heli setVariable ["bmkhs_prestonPitchActive",    false];
_heli setVariable ["bmkhs_prestonPitchTarget",    -6.0];

//ATT regime roll. ki is the term that matters - it absorbs the airframe's standing right-roll
//offset, which a rate-damping SAS cannot do. Roll is the LOW-INERTIA axis so it needs less gain
//than pitch, not more: a quicker axis reaches the same rate on less input.
_heli setVariable ["bmkhs_pid_prestonRoll",       [0.0700, 0.0200, 0.0900, 5.0000] call bmkhs_fnc_pidCreate];
(_heli getVariable "bmkhs_pid_prestonRoll") set ["dCoef", 0.5];
_heli setVariable ["bmkhs_prestonRollActive",     false];
_heli setVariable ["bmkhs_prestonRollTarget",     0.0];

//POS regime (hover). Error is GROUND VELOCITY in m/s, not attitude. ki carries the standing
//cyclic offset that holds the hover. Pitch needs a larger offset than roll (CG sits aft), so the
//asymmetric ki_clamp is deliberate; 1.00 rails the integral.
_heli setVariable ["bmkhs_pid_prestonHoverX",     [0.0600, 0.2200, 0.0500, 0.2500] call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_prestonHoverY",     [0.0700, 0.2500, 0.0600, 0.5500] call bmkhs_fnc_pidCreate];
(_heli getVariable "bmkhs_pid_prestonHoverX") set ["dCoef", 0.5];
(_heli getVariable "bmkhs_pid_prestonHoverY") set ["dCoef", 0.5];

//VEL regime (transition): ROLL nulls lateral drift, PITCH holds forward velocity. Same error
//units as the hover PIDs. The lateral channel stops the hands and the auto-pedal's feet fighting
//over a drifting velocity vector.
_heli setVariable ["bmkhs_pid_prestonVelX",       [0.0500, 0.0400, 0.1000, 1.0000] call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_prestonVelY",       [0.1000, 0.0200, 0.1200, 1.0000] call bmkhs_fnc_pidCreate];
(_heli getVariable "bmkhs_pid_prestonVelX") set ["dCoef", 0.5];
(_heli getVariable "bmkhs_pid_prestonVelY") set ["dCoef", 0.5];
_heli setVariable ["bmkhs_prestonVelCmdFwd",      0.0];   //commanded forward velocity (m/s)

//HANDS filter state (lag + rate limit on the blended cyclic output). Re-seeded to the live trim
//position on breakout so the machine pilot resumes from where the stick actually is.
_heli setVariable ["bmkhs_prestonPrevPitch",      0.0];
_heli setVariable ["bmkhs_prestonPrevRoll",       0.0];
_heli setVariable ["bmkhs_prestonBreakout",       false];
_heli setVariable ["bmkhs_prestonHoverDatum",     getPos _heli];  //world-space position datum
_heli setVariable ["bmkhs_prestonHoverIntX",      0.0];           //position-error integral (right)
_heli setVariable ["bmkhs_prestonHoverIntY",      0.0];           //position-error integral (fwd)

//PRE-SEEDED hover integrals so the first pickup already carries the standing forward cyclic the
//aft CG needs; starting from zero it pitches back and drifts aft while the loop catches up.
//Overwritten by the learned values once the aircraft holds a steady hover.
_heli setVariable ["bmkhs_prestonLearnedIntX",    0.0];
_heli setVariable ["bmkhs_prestonLearnedIntY",    0.5500];

//Live regime weights - POS (hover) / VEL (transition) / ATT (cruise). Published for the readouts.
_heli setVariable ["bmkhs_prestonWPos",           0.0];
_heli setVariable ["bmkhs_prestonWVel",           0.0];
_heli setVariable ["bmkhs_prestonWAtt",           0.0];
