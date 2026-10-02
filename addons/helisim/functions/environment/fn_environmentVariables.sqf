/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_environmentVariables

Description:
    Seeds the atmosphere and wind state, then solves it once so an init that
    reads the conditions gets the real values.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

_heli setVariable ["bmkhs_pa",                  0.0];
_heli setVariable ["bmkhs_fat",                 0.0];
_heli setVariable ["bmkhs_rho",                 ISA_STD_DAY_AIR_DENSITY];

_heli setVariable ["bmkhs_windSpeedKts",        0];
_heli setVariable ["bmkhs_windDirFrom",         0];
_heli setVariable ["bmkhs_velWindWorldSpace",   [0.0, 0.0, 0.0]];

[_heli] call bmkhs_fnc_environment;
