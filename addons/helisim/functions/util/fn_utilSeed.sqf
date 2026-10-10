/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_utilSeed

Description:
    Seeds one of the aircraft's variables at init. This is the ONE way the
    init path writes state, and the only place it decides locality.

    The owner sets it, publishing it when it is networked. Any other machine
    sets its own copy only and never publishes - building its copy of the
    aircraft must not change it for anyone else. What had already arrived
    from the owner is put back by bmkhs_fnc_coreConfig.

    Never wrap a call in "if (local _heli)": that leaves a machine that does
    not own the aircraft with nothing seeded at all.

Parameters:
    _heli      - The helicopter [Object]
    _var       - Variable name [String]
    _value     - Seed value [Any]
    _networked - Published by the owner [Bool, optional, default false]

Returns:
    Nothing

Examples:
    [_heli, "bmkhs_engFailed", [false, false], true] call bmkhs_fnc_utilSeed;

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_var", "_value", ["_networked", false]];

_heli setVariable [_var, _value, _networked && {local _heli}];
