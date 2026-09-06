/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_airfoilVariables

Description:
    Loads the aircraft's airfoil sections into a hashmap keyed by name, so a
    rotor or wing names the section it uses rather than pointing at a position
    in a list.

    Reading these once at load matters: the lift and drag lookups run per blade
    element per frame, and the tables were previously fetched from config on
    every one of those calls.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

private _airfoils = createHashMap;

for "_i" from 1 to (getNumber (_config >> "numAirfoils")) do {
    private _a    = (_config >> "Airfoils") >> format ["Airfoil%1%2", ["0", ""] select (_i > 9), _i];
    private _name = getText (_a >> "name");

    if (_name == "") then {
        diag_log text format [
            "[BMKHS] AIRFOIL CONFIG ERROR: Airfoil%1 has no name and cannot be referenced.", _i
        ];
    } else {
        if (_name in _airfoils) then {
            diag_log text format [
                "[BMKHS] AIRFOIL CONFIG ERROR: '%1' is declared more than once; the later one wins.", _name
            ];
        };
        _airfoils set [_name, getArray (_a >> "table")];
    };
};

_heli setVariable ["bmkhs_airfoils", _airfoils];
