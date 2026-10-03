/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fuselageVariables

Description:
    Loads the fuselage panel configuration: the top, side and front sets.

    Each set becomes one hashmap keyed by the config's own property names, and
    is found by its name rather than by position.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

private _panelSets = createHashMap;

{
    private _p = _x;

    private _name   = getText (_p >> "name");
    private _facing = toLower getText (_p >> "facing");
    private _panels = getArray (_p >> "panels");

    //Keyed by what the set IS, so nothing downstream assumes set 0 is the top.
    _panelSets set [_name, createHashMapFromArray [
        ["name",          _name],
        ["facing",        _facing],
        ["dragCoefTable", getArray (_p >> "dragCoefTable")],
        ["panels",        _panels],
        ["count",         count _panels]
    ]];
} forEach ("true" configClasses (_config >> "FuselagePanels"));

_heli setVariable ["bmkhs_fuselagePosition",     getArray (_config >> "fuselagePosition")];
_heli setVariable ["bmkhs_fuselageRotation",     getArray (_config >> "fuselageRotation")];
_heli setVariable ["bmkhs_fuselageAirfoil",      getText  (_config >> "fuselageAirfoil")];
_heli setVariable ["bmkhs_fuselagePanels",       _panelSets];
