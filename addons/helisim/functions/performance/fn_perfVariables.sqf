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

_heli setVariable ["bmkhs_perfDataChange",  ""];

_heli setVariable ["bmkhs_maxTq_cont",    0.0];
_heli setVariable ["bmkhs_maxTq_de",      0.0];
_heli setVariable ["bmkhs_maxTq_se",      0.0];

_heli setVariable ["bmkhs_maxGwt_de_ige", 0.0];
_heli setVariable ["bmkhs_maxGwt_de_oge", 0.0];
_heli setVariable ["bmkhs_maxGwt_se_ige", 0.0];
_heli setVariable ["bmkhs_maxGwt_se_oge", 0.0];

_heli setVariable ["bmkhs_goNoGoTq_ige",  0.0];
_heli setVariable ["bmkhs_goNoGoTq_oge",  0.0];

_heli setVariable ["bmkhs_hvrTq_ige",     0.0];
_heli setVariable ["bmkhs_hvrTq_oge",     0.0];

_heli setVariable ["bmkhs_tas_vne",       0.0];
_heli setVariable ["bmkhs_tas_vsse",      0.0];

_heli setVariable ["bmkhs_tas_rngTas",    0.0];
_heli setVariable ["bmkhs_tas_rngTq",     0.0];
_heli setVariable ["bmkhs_tas_rngFf",     0.0];

_heli setVariable ["bmkhs_tas_endTas",    0.0];
_heli setVariable ["bmkhs_tas_endTq",     0.0];
_heli setVariable ["bmkhs_tas_endFf",     0.0];
