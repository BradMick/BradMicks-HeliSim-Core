/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_environmentVariables

Description:
    Seeds the atmosphere, wind and gust state, then solves it once so an init
    that reads the conditions gets the real values.

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

_heli setVariable ["bmkhs_barAlt",              0.0];
_heli setVariable ["bmkhs_fat",                 0.0];
_heli setVariable ["bmkhs_pressure",            ISA_PRESSURE_HPA];
_heli setVariable ["bmkhs_dewPoint",            -DEG_C_TO_KELVIN];
_heli setVariable ["bmkhs_rho",                 ISA_STD_DAY_AIR_DENSITY];
_heli setVariable ["bmkhs_rainCooling",         0.0];

//What the environment eases, started where it is now
private _posASL = getPosASL _heli;
_heli setVariable ["bmkhs_envAgl",              ((_posASL select 2) - ((getTerrainHeightASL _posASL) max 0.0)) max 0.0];
_heli setVariable ["bmkhs_windSteady",          [wind select 0, wind select 1, 0.0]];
_heli setVariable ["bmkhs_gustState",           [0.0, 0.0, 0.0]];

_heli setVariable ["bmkhs_windSpeed",           0];
_heli setVariable ["bmkhs_windDirFrom",         0];
_heli setVariable ["bmkhs_velWindWorldSpace",   [0.0, 0.0, 0.0]];
_heli setVariable ["bmkhs_velGustWorldSpace",   [0.0, 0.0, 0.0]];

[_heli] call bmkhs_fnc_environment;
