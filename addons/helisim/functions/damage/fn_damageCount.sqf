/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_damageCount

Description:
    How many members a damage role has on this aircraft - two engines, one
    battery, no pylons. Returns 0 for a role the aircraft did not declare, so
    a loop over it simply does not run.

Parameters:
    _heli - The helicopter [Object]
    _role - Role name, as declared in DamagePoints [String]

Returns:
    Member count [Number]

Examples:
    for "_i" from 0 to ([_heli, "engines"] call bmkhs_fnc_damageCount) - 1 do {...}

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_role"];

count ((_heli getVariable ["bmkhs_damagePoints", createHashMap]) getOrDefault [_role, []])
