/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fuselageVariables

Description:
    Loads the fuselage panel configuration. Core loops over numFuselagePanels, so
    an aircraft declares as many panel sets as its shape needs.

    Each set becomes one hashmap keyed by the config's own property names, and
    names which way it faces rather than being found by position.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

private _numPanelSets = getNumber (_config >> "numFuselagePanels");
private _panelSets    = createHashMap;

for "_i" from 1 to _numPanelSets do {
    private _p = (_config >> "FuselagePanels") >> format ["FuselagePanel%1%2", ["0", ""] select (_i > 9), _i];

    private _facing = toLower getText (_p >> "facing");
    private _panels = getArray (_p >> "panels");

    //Keyed by what the set IS, so nothing downstream assumes set 0 is the top.
    _panelSets set [_facing, createHashMapFromArray [
        ["facing",        _facing],
        ["dragCoefTable", getArray (_p >> "dragCoefTable")],
        ["panels",        _panels],
        ["count",         count _panels]
    ]];
};

_heli setVariable ["bmkhs_fuselagePosition",     getArray (_config >> "fuselagePosition")];
_heli setVariable ["bmkhs_fuselageRotation",     getArray (_config >> "fuselageRotation")];
_heli setVariable ["bmkhs_fuselageAirfoil",      getText  (_config >> "fuselageAirfoil")];
_heli setVariable ["bmkhs_numFuselagePanels",    _numPanelSets];
_heli setVariable ["bmkhs_fuselagePanels",       _panelSets];
