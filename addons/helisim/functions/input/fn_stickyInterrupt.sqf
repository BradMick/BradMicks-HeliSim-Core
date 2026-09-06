/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stickyInterrupt

Description:
    Sticky-control interrupt. While held, keyboard cyclic/pedal input stops
    accumulating so the pilot can recentre.

Parameters:
    _heli  - The helicopter [Object]
    _state - Held [Bool]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_state"];

_heli setVariable ["bmkhs_kbStickyInterupt", _state];
