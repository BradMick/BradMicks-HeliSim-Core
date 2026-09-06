/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_damageApply <-- rename me to perfLimit, move to engine

Description:
    Applies damage within a defined period of time after exceeding aircraft
    operating limits.

    STUB - not implemented. The droop detection and the bmkhs_liftLossTimer
    ramp it fed were removed: nothing ever read the timer, so the whole
    function was a per-frame no-op. Rebuild the detection here when the
    overlimit penalty is actually wired into the rotor.

Parameters:
    _heli      - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

if (!local _heli) exitWith {};
