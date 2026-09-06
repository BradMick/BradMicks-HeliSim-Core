/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stateVariables

Description:
    Initialises the derived aircraft state - velocities, accelerations and the
    aerodynamic values computed from them.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

//Smoothed worldAccel - slip ball only.
_heli setVariable ["bmkhs_worldAccelFiltered", [0.0,0.0,0.0]];
_heli setVariable ["bmkhs_worldAccelX_avg",   [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];
_heli setVariable ["bmkhs_worldAccelY_avg",   [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];
_heli setVariable ["bmkhs_worldAccelZ_avg",   [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];

_heli setVariable ["bmkhs_velX_prev",         0.0];
_heli setVariable ["bmkhs_accelX",            0.0];
_heli setVariable ["bmkhs_accelX_avg",        [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];

_heli setVariable ["bmkhs_velY_prev",         0.0];
_heli setVariable ["bmkhs_accelY",            0.0];
_heli setVariable ["bmkhs_accelY_avg",        [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];

_heli setVariable ["bmkhs_velZ_prev",         0.0];
_heli setVariable ["bmkhs_accelZ",            0.0];
_heli setVariable ["bmkhs_accelZ_avg",        [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];

//Aerodynamic state
_heli setVariable ["bmkhs_aero_beta_deg",      0.0];
_heli setVariable ["bmkhs_aero_beta_g",        0.0];
_heli setVariable ["bmkhs_aero_beta_g_prev",   0.0];   //EGI low-pass filter state for the skid/slip (beta_g)
