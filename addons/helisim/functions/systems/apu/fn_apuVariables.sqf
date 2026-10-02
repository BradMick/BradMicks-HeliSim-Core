/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_apuVariables

Description:
    Initialises the APU's state.

Parameters:
    _heli - The helicopter [Unit].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli"];

//The opposite of the APU's cold start, so the first frame reports its state.
_heli setVariable ["bmkhs_apuOnLast", true];
