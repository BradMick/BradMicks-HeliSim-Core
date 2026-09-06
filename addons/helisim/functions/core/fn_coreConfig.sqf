/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_coreConfig

Description:
    Defines key values for the simulation.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", ["_configIn", configNull]];

//Caller supplies the config; fall back to the vehicle class for legacy callers
private _config = if (isNull _configIn) then { configOf _heli >> "BMKHS_HeliSim" } else { _configIn };
bmkhs_movingAverageSize = 10;

//Systems gate - all or nothing
_heli setVariable ["bmkhs_useSystems",          getNumber (_config >> "useSystems")          > 0];

//Damage map and airfoils first - everything downstream resolves names against them.
[_heli, _config] call bmkhs_fnc_damageVariables;
[_heli, _config] call bmkhs_fnc_airfoilVariables;
[_heli, _config] call bmkhs_fnc_stateVariables;
[_heli, _config] call bmkhs_fnc_inputVariables;
[_heli, _config] call bmkhs_fnc_fmcVariables;
[_heli, _config] call bmkhs_fnc_systemsVariables;
//Before the graph - a component gates on the variable a control publishes.
[_heli, _config] call bmkhs_fnc_controlsVariables;
//After systemsVariables - components resolve against the damage map and the tuning
//values it seeds.
[_heli, _config] call bmkhs_fnc_systemsComponents;
[_heli, _config] call bmkhs_fnc_fuelVariables;
[_heli, _config] call bmkhs_fnc_massVariables;
[_heli] call bmkhs_fnc_fuelSet;
[_heli, _config] call bmkhs_fnc_engineVariables;
[_heli, _config] call bmkhs_fnc_fuselageVariables;
[_heli, _config] call bmkhs_fnc_wingVariables;
[_heli] call bmkhs_fnc_transmissionVariables;
[_heli, _config] call bmkhs_fnc_simpleRotorVariables;
[_heli, _config] call bmkhs_fnc_rotorVariables;
[_heli] call bmkhs_fnc_perfVariables;
[_heli] call bmkhs_fnc_actuatorVariables;
[_heli] call bmkhs_fnc_prestonVariables;
