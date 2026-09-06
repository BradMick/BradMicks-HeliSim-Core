/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_damageSet

Description:
    Sets the damage on a system Core models.

    A role the aircraft did not declare is a no-op: there is no hitpoint to
    write to, and that is not an error.

Parameters:
    _heli   - The helicopter [Object]
    _role   - Role name, as declared in DamagePoints [String]
    _damage - Damage to apply, 0 to 1 [Number]
    _index  - Which member, or -1 for all of them [Number, optional]

Returns:
    Nothing

Examples:
    [_heli, "priPump", 1.0] call bmkhs_fnc_damageSet
    [_heli, "engines", 0.0, 1] call bmkhs_fnc_damageSet   //second engine only

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_role", "_damage", ["_index", -1]];

private _points = (_heli getVariable ["bmkhs_damagePoints", createHashMap]) getOrDefault [_role, []];
if (_points isEqualTo []) exitWith {};

if (_index >= 0) exitWith {
    private _hp = _points param [_index, ""];
    if (_hp != "") then { _heli setHitPointDamage [_hp, _damage] };
};

{
    _heli setHitPointDamage [_x, _damage];
} forEach _points;
