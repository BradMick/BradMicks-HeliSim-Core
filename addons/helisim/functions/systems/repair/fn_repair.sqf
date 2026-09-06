/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_repair

Description:
    Restores the state that goes with a repaired component.

    Runs when a HandleDamage event saw a hitpoint go DOWN - a repair announces
    itself, so there is nothing to poll for.

    Walks the components the aircraft declared rather than naming any, so an
    airframe with three generators or no accumulator gets what it actually has.
    A repaired store comes back FULL, since charge is a quantity a repair
    replaces; pressure is not restored, because that depends on something
    turning the pumps and the solve will produce it on the next frame.

    Each component is re-seeded to 0.000001 once handled, so it reads as
    not-exactly-zero and does not trigger again.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

if !(_heli getVariable ["bmkhs_repairPending", false]) exitWith {};
_heli setVariable ["bmkhs_repairPending", false];

//Stores come back full - fluid and charge are what a repair replaces. A store with no
//damage role has no hitpoint to read, so it simply refills: it could not have been the
//thing that broke, but a serviced aircraft has it full either way.
{
    private _role  = _x get "damageRole";
    private _index = _x get "index";
    private _fill  = _role == "" || {([_heli, _role, _index] call bmkhs_fnc_damageGet) == 0};
    if (_fill) then {
        _heli setVariable [(_x get "varName") + "Charge", 1.0, true];
        if (_role != "") then {
            [_heli, _role, 0.000001, _index] call bmkhs_fnc_damageSet;
        };
    };
} forEach (_heli getVariable ["bmkhs_sysStorage", []]);

//Producers and converters hold no state of their own - the solve recomputes what they
//make from their own damage, so they only need the marker.
{
    private _role = _x get "damageRole";
    if (_role != "" && {([_heli, _role, _x get "index"] call bmkhs_fnc_damageGet) == 0}) then {
        [_heli, _role, 0.000001, _x get "index"] call bmkhs_fnc_damageSet;
    };
} forEach ((_heli getVariable ["bmkhs_sysProducers", []]) + (_heli getVariable ["bmkhs_sysConverters", []]));

//Engines are not components, and their overspeed latch is what a repair clears.
private _engines = [_heli, "engines"] call bmkhs_fnc_damageCount;
for "_i" from 0 to (_engines - 1) do {
    if (([_heli, "engines", _i] call bmkhs_fnc_damageGet) == 0) then {
        [_heli, "bmkhs_engineOverspeed", _i, false, true] call bmkhs_fnc_utilSetArrayVariable;
        [_heli, "engines", 0.000001, _i] call bmkhs_fnc_damageSet;
    };
};
