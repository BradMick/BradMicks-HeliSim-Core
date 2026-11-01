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
#include "\bmkhs_helisim\functions\damage\gStress.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

private _numWings = _heli getVariable "bmkhs_numWings";
private _wings    = _heli getVariable "bmkhs_wings";

//Surfaces gone with the tail structure (fn_damageGStress)
private _lost = _heli getVariable "bmkhs_partsLost";
private _gone = [];
if ("stabilator"  in _lost) then { _gone append GSTRESS_WINGS_STABILATOR };
if ("verticalFin" in _lost) then { _gone append GSTRESS_WINGS_FIN };

for "_i" from 0 to (_numWings - 1) do {
    private _wing = _wings select _i;
    if ((toLower (_wing get "name")) in _gone) then { continue };
    [_heli, _i, _wing] call bmkhs_fnc_wing;
};
