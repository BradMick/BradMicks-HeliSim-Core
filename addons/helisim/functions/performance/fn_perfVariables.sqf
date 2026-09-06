/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_perfVariables

Description:
    Defines the initial performance page variables and initializes them.

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

_heli setVariable ["bmkhs_PA",            0.0];
_heli setVariable ["bmkhs_FAT",           0.0];
_heli setVariable ["bmkhs_GWT",           0.0];

_heli setVariable ["bmkhs_maxTQ_CONT",    0.0];
_heli setVariable ["bmkhs_maxTQ_DE",      0.0];
_heli setVariable ["bmkhs_maxTQ_SE",      0.0];

_heli setVariable ["bmkhs_maxGWT_DE_IGE", 0.0];
_heli setVariable ["bmkhs_maxGWT_DE_OGE", 0.0];
_heli setVariable ["bmkhs_maxGWT_SE_IGE", 0.0];
_heli setVariable ["bmkhs_maxGWT_SE_OGE", 0.0];

_heli setVariable ["bmkhs_goNoGoTQ_IGE",  0.0];
_heli setVariable ["bmkhs_goNoGoTQ_OGE",  0.0];

_heli setVariable ["bmkhs_hvrTQ_IGE",     0.0];
_heli setVariable ["bmkhs_hvrTQ_OGE",     0.0];

_heli setVariable ["bmkhs_TAS_vne",       0.0];
_heli setVariable ["bmkhs_TAS_vsse",      0.0];

_heli setVariable ["bmkhs_TAS_rngTAS",    0.0];
_heli setVariable ["bmkhs_TAS_rngTQ",     0.0];
_heli setVariable ["bmkhs_TAS_rngFF",     0.0];

_heli setVariable ["bmkhs_TAS_endTAS",    0.0];
_heli setVariable ["bmkhs_TAS_endTQ",     0.0];
_heli setVariable ["bmkhs_TAS_endFF",     0.0];
