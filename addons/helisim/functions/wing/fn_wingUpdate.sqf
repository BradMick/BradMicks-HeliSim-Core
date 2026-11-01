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
private _lost     = _heli getVariable "bmkhs_partsLost";

for "_i" from 0 to (_numWings - 1) do {
    private _wing = _wings select _i;
    if ((_wing get "name") in _lost) then { continue };
    [_heli, _i, _wing] call bmkhs_fnc_wing;
};
