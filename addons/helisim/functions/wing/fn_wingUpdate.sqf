/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_wingUpdate

Description:
    Applies the aerodynamic forces of every lifting surface the aircraft
    declares - wings, fins and the stabilator.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    Nothing
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

private _numWings = _heli getVariable "bmkhs_numWings";
private _wings    = _heli getVariable "bmkhs_wings";

for "_i" from 0 to (_numWings - 1) do {
    //TESTING - vertical fin skipped to see the yaw balance without its side force.
    //if (_i == 2) then { continue };
    [_heli, _i, _wings select _i] call bmkhs_fnc_wing;
};
