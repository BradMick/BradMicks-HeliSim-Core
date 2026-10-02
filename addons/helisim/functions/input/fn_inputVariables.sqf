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
_heli setVariable ["bmkhs_kbPedalLeftRight",      0.0];
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
_heli setVariable ["bmkhs_autoPedalRegimeWgt", 1.0];     //0-1, share of the pedal that regime owns
_heli setVariable ["bmkhs_autoPedalHdgErr",    0.0];     //deg, heading error
_heli setVariable ["bmkhs_autoPedalNttErr",    0.0];     //deg, kinematic sideslip
_heli setVariable ["bmkhs_autoPedalAeroErr",   0.0];     //g,   lateral accel
_heli setVariable ["bmkhs_autoPedalOut",       0.0];     //blended pedal output
