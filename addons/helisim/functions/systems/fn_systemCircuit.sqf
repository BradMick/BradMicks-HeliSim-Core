/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemCircuit

Description:
    What a circuit is carrying, in whatever unit its domain uses - PSI, volts,
    Nr as a fraction. Highest feeder wins rather than summing, so two pumps
    give 3000 psi and not 6000. Capacity and load are not modelled.

Parameters:
    _heli    - The helicopter [Object]
    _circuit - Circuit name, as declared by whatever feeds or reads it [String]

Returns:
    The circuit's value, or 0 if nothing feeds it [Number]

Examples:
    [_heli, "UTIL_HYD"] call bmkhs_fnc_systemCircuit

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_circuit"];

if (_circuit == "") exitWith {0};

(_heli getVariable ["bmkhs_sysCircuits", createHashMap]) getOrDefault [_circuit, 0]
