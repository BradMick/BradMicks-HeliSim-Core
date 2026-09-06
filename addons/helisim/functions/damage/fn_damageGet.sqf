/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_damageGet

Description:
    Returns the damage on a system Core models, 0 to 1.

    A role the aircraft did not declare returns 0 - undamaged - because no
    hitpoint means nothing can break it. An aircraft with no pylons is not an
    aircraft with broken pylons.

    Arma returns -1 for a hitpoint name the vehicle does not have, which sums
    and comparisons downstream would treat as a real value. That is clamped
    here so no caller has to know about it.

    With several members - engines, generators, pylons - the WORST is returned
    by default, since that is what "is this system damaged" nearly always
    means. Pass an index for one specific member.

Parameters:
    _heli  - The helicopter [Object]
    _role  - Role name, as declared in DamagePoints [String]
    _index - Which member, or -1 for the worst of them [Number, optional]

Returns:
    Damage 0..1 [Number]

Examples:
    [_heli, "mainRotor"] call bmkhs_fnc_damageGet
    [_heli, "engines", 0] call bmkhs_fnc_damageGet
    [_heli, "generators"] call bmkhs_fnc_damageGet   //worst of however many

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_role", ["_index", -1]];

private _points = (_heli getVariable ["bmkhs_damagePoints", createHashMap]) getOrDefault [_role, []];
if (_points isEqualTo []) exitWith { 0 };

if (_index >= 0) exitWith {
    private _hp = _points param [_index, ""];
    if (_hp == "") exitWith { 0 };
    (_heli getHitPointDamage _hp) max 0        //-1 when the vehicle has no such hitpoint
};

private _worst = 0;
{
    _worst = _worst max ((_heli getHitPointDamage _x) max 0);
} forEach _points;

_worst
