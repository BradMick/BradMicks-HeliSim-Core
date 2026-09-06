/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemCircuitState

Description:
    Publishes the state of any circuit the aircraft named. A bus being up is a
    fact about the node rather than something drawing from it, so this is not a
    consumer - it is Core reporting what a circuit is carrying.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

private _named = _heli getVariable ["bmkhs_sysNamed", []];
if (_named isEqualTo []) exitWith {};


{
    private _up = ([_heli, _x get "circuit"] call bmkhs_fnc_systemCircuit) >= (_x get "minValue");
    if (_x get "networked") then {
        [_heli, _x get "varName", _up] call bmkhs_fnc_utilUpdateNetworkGlobal;
    } else {
        _heli setVariable [_x get "varName", _up];
    };
} forEach _named;
