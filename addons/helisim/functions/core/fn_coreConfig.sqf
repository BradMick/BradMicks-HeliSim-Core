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
#include "\bmkhs_helisim\functions\core\netState.hpp"

//Caller supplies the config; fall back to the vehicle class for legacy callers
private _config = if (isNull _configIn) then { configOf _heli >> "BMKHS_HeliSim" } else { _configIn };
bmkhs_movingAverageSize = 10;

//Built on this machine - every machine builds its own copy, the config is never sent.
_heli setVariable ["bmkhs_configured", true];

//What the packed running state carries: Core's own list, then any values the aircraft computes
//itself and declares in netStateVars[]. Built from config on every machine, so the sender and
//every receiver agree on the order.
_heli setVariable ["bmkhs_netStateVars", NET_STATE_VARS + getArray (_config >> "netStateVars")];

//On a machine that does not own the aircraft, what has already arrived from the owner stands:
//the build makes this machine's copy of the config and seeds only what is missing, then puts
//back everything that was already here. Nothing below publishes from a machine that is not
//the owner.
private _arrived = [];
if (!local _heli) then {
    _arrived = (allVariables _heli) apply {
        private _v = _heli getVariable _x;
        [_x, if (_v isEqualType []) then { +_v } else { _v }]
    };
};

//Systems gate - all or nothing
_heli setVariable ["bmkhs_useSystems",          getNumber (_config >> "useSystems")          > 0];

[_heli] call bmkhs_fnc_environmentVariables;
[_heli, _config] call bmkhs_fnc_damageVariables;
[_heli, _config] call bmkhs_fnc_airfoilVariables;
[_heli, _config] call bmkhs_fnc_stateVariables;
[_heli, _config] call bmkhs_fnc_inputVariables;
[_heli, _config] call bmkhs_fnc_fmcVariables;
[_heli, _config] call bmkhs_fnc_systemsVariables;
[_heli] call bmkhs_fnc_apuVariables;
[_heli, _config] call bmkhs_fnc_controlsVariables;
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
[_heli] call bmkhs_fnc_actuatorVariables;
[_heli] call bmkhs_fnc_prestonVariables;
[_heli] call bmkhs_fnc_fmDebugVariables;

{ _heli setVariable _x } forEach _arrived;
