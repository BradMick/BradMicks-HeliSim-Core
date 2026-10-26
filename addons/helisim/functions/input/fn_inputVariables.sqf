/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputVariables

Description:
    Initialises the pilot input state - cyclic, pedal and collective.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

//Cyclic input
_heli setVariable ["bmkhs_heliCyclicForwardOut",   0.0];
_heli setVariable ["bmkhs_heliCyclicBackwardOut",  0.0];
_heli setVariable ["bmkhs_heliCyclicLeftOut",      0.0];
_heli setVariable ["bmkhs_heliCyclicRightOut",     0.0];
//Pedal input
_heli setVariable ["bmkhs_heliRudderLeftOut",      0.0];
_heli setVariable ["bmkhs_heliRudderRightOut",     0.0];
//Collective input
_heli setVariable ["bmkhs_heliCollectiveRaiseOut", 0.0];
_heli setVariable ["bmkhs_heliCollectiveLowerOut", 0.0];

_heli setVariable ["bmkhs_cyclicFwdAft",          0.0];
_heli setVariable ["bmkhs_cyclicPitchValue",      0.0];
_heli setVariable ["bmkhs_prevCyclicPitchValue",  0.0];

_heli setVariable ["bmkhs_cyclicLeftRight",       0.0];
_heli setVariable ["bmkhs_cyclicRollValue",       0.0];
_heli setVariable ["bmkhs_prevCyclicRollValue",   0.0];

_heli setVariable ["bmkhs_pedalLeftRight",        0.0];
_heli setVariable ["bmkhs_pedalYawValue",         0.0];
_heli setVariable ["bmkhs_prevPedalYawValue",     0.0];

_heli setVariable ["bmkhs_collectiveOutput",      0.0];

//Input state
_heli setVariable ["bmkhs_kbStickyInterupt",     false];
_heli setVariable ["bmkhs_flightControlLockOut", false];
_heli setVariable ["bmkhs_kbHeliCollectiveRaiseOut", 0.0];
_heli setVariable ["bmkhs_kbHeliCollectiveLowerOut", 0.0];

//Auto attitude and auto pedal
_heli setVariable ["bmkhs_autoAttCycRollOut",  0.0];
_heli setVariable ["bmkhs_autoAttRollTarget",  0.0];
_heli setVariable ["bmkhs_autoPedalHdg",       getDir _heli];
_heli setVariable ["bmkhs_autoPedalRegime",    "hdg"];   //hdg | ntt | aero (live regime)
_heli setVariable ["bmkhs_autoPedalPos",       0.0];     //where the feet have the pedals
_heli setVariable ["bmkhs_autoPedalVel",       0.0];     //foot speed, per s
_heli setVariable ["bmkhs_autoPedalTrim",      0.0];     //pedal before the pilot's turn
_heli setVariable ["bmkhs_autoPedalHdgSet",    true];    //hover heading captured
_heli setVariable ["bmkhs_autoPedalRawPrev",   0.0];     //pilot's raw pedal, last frame
_heli setVariable ["bmkhs_autoPedalSprung",    false];   //analog pedal springing back - released
