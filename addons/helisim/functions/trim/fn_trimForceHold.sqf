/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_trimForceHold

Description:
    Force trim switch held. Interrupts the hold modes so the pilot can fly the
    aircraft to a new reference without the FMC fighting the input.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli"];

if (currentPilot _heli != player || !local _heli) exitWith {};
//Force trim off - the switch is unpowered
if !(_heli getVariable "bmkhs_trimOn") exitWith {};

_heli setVariable ["bmkhs_forceTrimInterupted", true, true];
