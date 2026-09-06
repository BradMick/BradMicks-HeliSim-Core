/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcForceTrimHold

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

_heli setVariable ["bmkhs_forceTrimInterupted", true, true];
