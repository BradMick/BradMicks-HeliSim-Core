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

_heli setVariable ["bmkhs_autoAttLevelPitch", getNumber (_config >> "autoAttLevelPitch")];
_heli setVariable ["bmkhs_autoAttRollLimit", getNumber (_config >> "autoAttRollLimit")];
_heli setVariable ["bmkhs_pid_autoAttPitch", (getArray (_config >> "pidAutoAttPitch")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_autoAttRoll", (getArray (_config >> "pidAutoAttRoll")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_autoPedalHdg", (getArray (_config >> "pidAutoPedalHdg")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_autoPedalNtt", (getArray (_config >> "pidAutoPedalNtt")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_autoPedalAero", (getArray (_config >> "pidAutoPedalAero")) call bmkhs_fnc_pidCreate];

_heli setVariable ["bmkhs_posIntX",     0.0];
_heli setVariable ["bmkhs_posIntY",     0.0];

//FMC output state
_heli setVariable ["bmkhs_fmcAttHoldCycPitchOut", 0.0];
_heli setVariable ["bmkhs_fmcSasPitchOut",        0.0];
_heli setVariable ["bmkhs_fmcSasRollOut",         0.0];
_heli setVariable ["bmkhs_fmcHdgHoldPedalYawOut", 0.0];
_heli setVariable ["bmkhs_fmcSasYawOut",          0.0];
_heli setVariable ["bmkhs_fmcAltHoldCollOut",     0.0];
_heli setVariable ["bmkhs_fmcAttHoldCycRollOut",  0.0];
_heli setVariable ["bmkhs_fmcCollectiveToPitch",  0.0];
_heli setVariable ["bmkhs_fmcCollectiveToRoll",   0.0];
_heli setVariable ["bmkhs_fmcYawToPitch",         0.0];
_heli setVariable ["bmkhs_fmcYawToRoll",          0.0];

//Modes and holds - networked, so only the machine the aircraft is local to sets them.
if (local _heli) then {
    //FMC
    _heli setVariable ["bmkhs_fmcPitchOn",                true,  true];
    _heli setVariable ["bmkhs_fmcRollOn",                 true,  true];
    _heli setVariable ["bmkhs_fmcYawOn",                  true,  true];
    _heli setVariable ["bmkhs_fmcCollOn",                 true,  true];
    _heli setVariable ["bmkhs_fmcTrimOn",                 true,  true];
    //Force Trim
    _heli setVariable ["bmkhs_forceTrimInterupted",       false, true];
    _heli setVariable ["bmkhs_forceTrimPosPitch",         0.0,   true];
    _heli setVariable ["bmkhs_forceTrimPosRoll",          0.0,   true];
    _heli setVariable ["bmkhs_forceTrimPosYaw",           0.0,   true];
    //Attitude Hold
    _heli setVariable ["bmkhs_attHoldActive",             false, true];
    _heli setVariable ["bmkhs_attHoldDesiredPos",         getPos _heli, true];
    _heli setVariable ["bmkhs_attHoldDesiredVel",         [0.0, 0.0], true];
    _heli setVariable ["bmkhs_attHoldDesiredAtt",         [0.0, 0.0], true];
    _heli setVariable ["bmkhs_attHoldSubMode",            "pos", true];   //pos, vel, att
    //Altitude Hold
    _heli setVariable ["bmkhs_altHoldActive",             false, true];
    _heli setVariable ["bmkhs_altHoldDesiredAlt",         0.0,   true];
    _heli setVariable ["bmkhs_altHoldSubMode",            "rad", true];   //rad, bar
    _heli setVariable ["bmkhs_altHoldCollRef",            0.0,   true];
    //Heading Hold
    _heli setVariable ["bmkhs_hdgHoldActive",             false, true];
    _heli setVariable ["bmkhs_hdgHoldDesiredHdg",         0.0,   true];
    _heli setVariable ["bmkhs_hdgHoldDesiredSideslip",    0.0,   true];
    _heli setVariable ["bmkhs_hdgHoldSubMode",            "hdg", true];    //hdg, trn, yaw, aut
};
