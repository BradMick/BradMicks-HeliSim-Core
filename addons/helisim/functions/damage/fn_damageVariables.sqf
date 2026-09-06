/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_damageVariables

Description:
    Builds the damage map by reading the aircraft's own hitpoint definitions.

    Each hitpoint declares which HeliSim system it damages, so the mapping and
    the hitpoint live together in one place - there is no second list of names
    to keep in step. Core never learns what the aircraft calls anything; it
    asks by role.

    A role no hitpoint claims has no damage source, so that system runs
    undamaged. An aircraft with no pylons, no APU or one generator is a config
    with fewer entries, not a special case in the code.

    Required roles are the ones without which the aircraft is not a helicopter.
    A missing one is reported at load; the aircraft still flies, with that
    component permanently undamaged.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config]. Unused - hitpoints live
              on the vehicle class itself, not under BMKHS_HeliSim.

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

//Without these the aircraft is not a helicopter.
#define BMKHS_DAMAGE_REQUIRED ["mainRotor", "transmission"]

private _damage = createHashMap;

//Collect (roleIndex, hitpointName) per role, so multi-member roles keep the order the
//aircraft declared rather than the order the config happens to enumerate.
{
    private _role = getText (_x >> "bmkhsRole");
    if (_role != "") then {
        private _name = getText (_x >> "name");
        if (_name != "") then {
            private _list = _damage getOrDefault [_role, []];
            _list pushBack [getNumber (_x >> "bmkhsRoleIndex"), _name];
            _damage set [_role, _list];
        };
    };
} forEach ("true" configClasses ((configOf _heli) >> "HitPoints"));

//Sort by roleIndex and drop it - consumers index into a plain list of names.
{
    private _sorted = +(_damage get _x);
    _sorted sort true;
    _damage set [_x, _sorted apply {_x select 1}];
} forEach (keys _damage);

{
    if ((_damage getOrDefault [_x, []]) isEqualTo []) then {
        diag_log text format [
            "[BMKHS] DAMAGE CONFIG ERROR: no hitpoint declares role '%1'. That component cannot be damaged. Add bmkhsRole = ""%1"" to the hitpoint that represents it.",
            _x
        ];
    };
} forEach BMKHS_DAMAGE_REQUIRED;

_heli setVariable ["bmkhs_damagePoints", _damage];
