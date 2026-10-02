/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmDebugVariables

Description:
    Initialises the flight model debug overlay's state.

Parameters:
    _heli - The helicopter [Unit].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli"];

//The forces each module reports this frame, for the overlay to draw.
_heli setVariable ["bmkhs_dbgForces", []];
