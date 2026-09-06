/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_wingVariables

Description:
    Loads the per-wing configuration. Core loops over numWings, so an aircraft
    declares as many lifting surfaces as it has - wings, fins, stabilators - or
    none at all.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

_heli setVariable ["bmkhs_numWings",           getNumber (_config >> "numWings")];
_heli setVariable ["bmkhs_wingIsStabilator",   getArray  (_config >> "wingIsStabilator")];
_heli setVariable ["bmkhs_wingPos",            getArray  (_config >> "wingPos")];
_heli setVariable ["bmkhs_wingPitch",          getArray  (_config >> "wingPitch")];
_heli setVariable ["bmkhs_wingRoll",           getArray  (_config >> "wingRoll")];
_heli setVariable ["bmkhs_wingSpan",           getArray  (_config >> "wingSpan")];
_heli setVariable ["bmkhs_wingChord",          getArray  (_config >> "wingChord")];
_heli setVariable ["bmkhs_wingSweep",          getArray  (_config >> "wingSweep")];
_heli setVariable ["bmkhs_wingTwist",          getArray  (_config >> "wingTwist")];
_heli setVariable ["bmkhs_wingTipWidthScalar", getArray  (_config >> "wingTipWidthScalar")];
_heli setVariable ["bmkhs_wingNumElements",    getArray  (_config >> "wingNumElements")];
_heli setVariable ["bmkhs_wingAirfoil",        getArray (_config >> "wingAirfoil")];
_heli setVariable ["bmkhs_wingChordLinePos",   getArray  (_config >> "wingChordLinePos")];
