/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fuselageVariables

Description:
    Defines core fuselage variables.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

private _config = configOf _heli >> "BMKHS_HeliSim";

_heli setVariable ["bmkhs_fuselagePosition",            getArray  (_config >> "fuselagePosition")];

_heli setVariable ["bmkhs_fuselageTopRotation",         getArray  (_config >> "fuselageTopRotation")];
_heli setVariable ["bmkhs_fuselageTopDragCoefTable",    getArray  (_config >> "fuselageTopDragCoefTable")];
_heli setVariable ["bmkhs_fuselageTopCount",     	    getNumber (_config >> "fuselageTopCount")];
_heli setVariable ["bmkhs_fuselageTop",	 		        getArray  (_config >> "fuselageTop")];

_heli setVariable ["bmkhs_fuselageSideRotation",        getArray  (_config >> "fuselageSideRotation")];
_heli setVariable ["bmkhs_fuselageSideDragCoefTable",   getArray  (_config >> "fuselageSideDragCoefTable")];
_heli setVariable ["bmkhs_fuselageSideCount",           getNumber (_config >> "fuselageSideCount")];
_heli setVariable ["bmkhs_fuselageSide",			    getArray  (_config >> "fuselageSide")];

_heli setVariable ["bmkhs_fuselageFrontRotation",       getArray  (_config >> "fuselageFrontRotation")];
_heli setVariable ["bmkhs_fuselageFrontDragCoefTable",  getArray  (_config >> "fuselageFrontDragCoefTable")];
_heli setVariable ["bmkhs_fuselageFrontCount",          getNumber (_config >> "fuselageFrontCount")];
_heli setVariable ["bmkhs_fuselageFront",			    getArray  (_config >> "fuselageFront")];

_heli setVariable ["bmkhs_fuselageAirfoil",             getText (_config >> "fuselageAirfoil")];

//Aerodynamic centre
_heli setVariable ["bmkhs_aerodynamicCenter",           getArray  (_config >> "aerodynamicCenter")];   //m
