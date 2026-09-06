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
_heli getVariable ["bmkhs_prevCyclicPitchValue",  0.0];

_heli setVariable ["bmkhs_cyclicLeftRight",       0.0];
_heli setVariable ["bmkhs_cyclicRollValue",       0.0];
_heli setVariable ["bmkhs_prevCyclicRollValue",   0.0];

_heli setVariable ["bmkhs_pedalLeftRight",        0.0];
_heli setVariable ["bmkhs_kbPedalLeftRight",      0.0];
_heli setVariable ["bmkhs_pedalYawValue",         0.0];
_heli setVariable ["bmkhs_prevPedalYawValue",     0.0];

_heli setVariable ["bmkhs_collectiveOutput",      0.0];

_heli setVariable ["bmkhs_previousTime",        0.0];
_heli setVariable ["bmkhs_deltaTime",           0.0];
_heli setVariable ["bmkhs_deltaTime_avg",       [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];

_heli setVariable ["bmkhs_gndSpeed",            0.0];
_heli setVariable ["bmkhs_vel2D",               0.0];
_heli setVariable ["bmkhs_vel3D",               0.0];
_heli setVariable ["bmkhs_velWindWorldSpace",   [0.0,0.0,0.0]];
_heli setVariable ["bmkhs_velWindModelSpace",   [0.0,0.0,0.0]];
_heli setVariable ["bmkhs_windDirFrom",         0];
_heli setVariable ["bmkhs_windSpeedKts",        0];
_heli setVariable ["bmkhs_velModelSpace",       [0.0,0.0,0.0]];
_heli setVariable ["bmkhs_velModelSpaceNoWind", [0.0,0.0,0.0]];
_heli setVariable ["bmkhs_velModelSpaceX_avg",  [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];
_heli setVariable ["bmkhs_velModelSpaceY_avg",  [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];
_heli setVariable ["bmkhs_velModelSpaceZ_avg",  [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];
_heli setVariable ["bmkhs_velWorldSpace",       [0.0,0.0,0.0]];
_heli setVariable ["bmkhs_velWorldSpaceNoWind", [0.0,0.0,0.0]];
_heli setVariable ["bmkhs_velWorldSpaceNoWind_prev", [0.0,0.0,0.0]];
_heli setVariable ["bmkhs_velWorldSpaceX_avg",  [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];
_heli setVariable ["bmkhs_velWorldSpaceY_avg",  [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];
_heli setVariable ["bmkhs_velWorldSpaceZ_avg",  [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];
_heli setVariable ["bmkhs_velClimb",            0.0];
_heli setVariable ["bmkhs_angVelModelSpace",    [0.0,0.0,0.0]];
_heli setVariable ["bmkhs_angVelModelSpaceX_avg", [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];
_heli setVariable ["bmkhs_angVelModelSpaceY_avg", [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];
_heli setVariable ["bmkhs_angVelModelSpaceZ_avg", [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];

_heli setVariable ["bmkhs_worldAccel",        [0.0,0.0,0.0]];

//Input state
_heli setVariable ["bmkhs_kbStickyInterupt",     false];
_heli setVariable ["bmkhs_flightControlLockOut", false];
_heli setVariable ["bmkhs_cyclicPitchValue",   0.0];
_heli setVariable ["bmkhs_cyclicRollValue",    0.0];
_heli setVariable ["bmkhs_pedalYawValue",      0.0];
