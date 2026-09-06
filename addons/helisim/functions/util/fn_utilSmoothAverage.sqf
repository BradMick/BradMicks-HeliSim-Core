/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_getVelocitiesSmoothAverage
Description:

Parameters:

Returns:

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_array", "_values"];

[_array, _values] call bmkhs_fnc_smoothAverageAdd;

[_array] call bmkhs_fnc_smoothAverageGet;
