/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcVariables

Description:
    Loads the flight management computer gains - hold modes, SAS and
    auto-pedal PIDs.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

//Position Hold
_heli setVariable ["bmkhs_pid_roll", (getArray (_config >> "pidRoll")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_pitch", (getArray (_config >> "pidPitch")) call bmkhs_fnc_pidCreate];
//Attitude Hold
_heli setVariable ["bmkhs_pid_roll_att", (getArray (_config >> "pidRollAtt")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_pitch_att", (getArray (_config >> "pidPitchAtt")) call bmkhs_fnc_pidCreate];
//Altitude Hold
_heli setVariable ["bmkhs_pid_radHold", (getArray (_config >> "pidRadAlt")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_barHold", (getArray (_config >> "pidBarAlt")) call bmkhs_fnc_pidCreate];
//Heading Hold
_heli setVariable ["bmkhs_pid_hdgHold", (getArray (_config >> "pidHdgHold")) call bmkhs_fnc_pidCreate];
//Turn coordination / yaw slip loop. Error is LATERAL G (bmkhs_aero_beta_g); output is
//clamped to +-0.1 in fn_fmcHeadingHold, so size the gains against that, not the +-1 gauge.
_heli setVariable ["bmkhs_pid_trnCoord", (getArray (_config >> "pidTrnCoord")) call bmkhs_fnc_pidCreate];
//SAS - proportional rate DAMPING (output = -kp*rate).
_heli setVariable ["bmkhs_pid_sas_pitch", (getArray (_config >> "pidSasPitch")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_sas_roll", (getArray (_config >> "pidSasRoll")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_sas_yaw", (getArray (_config >> "pidSasYaw")) call bmkhs_fnc_pidCreate];

_heli setVariable ["bmkhs_pid_autoPedalHdg", (getArray (_config >> "pidAutoPedalHdg")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_autoPedalNtt", (getArray (_config >> "pidAutoPedalNtt")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_autoPedalAero", (getArray (_config >> "pidAutoPedalAero")) call bmkhs_fnc_pidCreate];

_heli setVariable ["bmkhs_posIntKp",    0.0200];
_heli setVariable ["bmkhs_posIntClamp", 0.2500];
_heli setVariable ["bmkhs_posIntX",     0.0];
_heli setVariable ["bmkhs_posIntY",     0.0];

_heli setVariable ["bmkhs_autoPedalHdg",       getDir _heli];
_heli setVariable ["bmkhs_autoPedalRegime",    "hdg"];   //hdg | ntt | aero (live regime)
_heli setVariable ["bmkhs_autoPedalRegimeWgt", 1.0];     //0-1, share of the pedal that regime owns
_heli setVariable ["bmkhs_autoPedalHdgErr",    0.0];     //deg, heading error
_heli setVariable ["bmkhs_autoPedalNttErr",    0.0];     //deg, kinematic sideslip
_heli setVariable ["bmkhs_autoPedalAeroErr",   0.0];     //g,   lateral accel
_heli setVariable ["bmkhs_autoPedalOut",       0.0];     //blended pedal output
_heli setVariable ["bmkhs_autoPedalPrevOut",   0.0];     //pilot-feet filter state (rate limit + lag)

//FMC output state
_heli setVariable ["bmkhs_fmcAttHoldCycPitchOut", 0.0];
_heli setVariable ["bmkhs_fmcSasPitchOut",        0.0];
_heli setVariable ["bmkhs_fmcSasRollOut",         0.0];
_heli setVariable ["bmkhs_fmcHdgHoldPedalYawOut", 0.0];
_heli setVariable ["bmkhs_fmcSasYawOut",          0.0];
_heli setVariable ["bmkhs_fmcAltHoldCollOut",     0.0];
