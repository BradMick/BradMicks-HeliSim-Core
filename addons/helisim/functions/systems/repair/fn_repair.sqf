/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_repair

Description:
    Restores the state that goes with a repaired component.

    Runs when a hitpoint has gone DOWN since the last read, about once a second
    on the owning machine. Scripted damage (setHitPointDamage, setDamage, Zeus,
    ACE) fires no event, so the hitpoints are read rather than waited on.

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
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\damage\gStress.hpp"

//The main rotor's hitpoint doubles as the rotor lock (fn_engineUpdate), held at 0.9 while
//bmkhs_shiftLocked. Its drop is a repair only when the lock was not holding it at the last read.
private _rotorHp = ((_heli getVariable "bmkhs_damagePoints") getOrDefault ["mainRotor", []]) apply { toLower _x };
private _locked  = _heli getVariable ["bmkhs_shiftLocked", false];

//Any hitpoint lower than at the last read is a repair
if (CBA_missionTime >= (_heli getVariable ["bmkhs_repairReadT", -1]) + 1) then {
    _heli setVariable ["bmkhs_repairReadT", CBA_missionTime];
    (getAllHitPointsDamage _heli) params [["_names", []], "", ["_now", []]];
    private _last       = _heli getVariable ["bmkhs_repairLastDmg", _now];
    private _lastLocked = _heli getVariable ["bmkhs_repairLastLocked", _locked];
    if (count _last == count _now) then {
        {
            private _heldByLock = _lastLocked && {(toLower (_names select _forEachIndex)) in _rotorHp};
            if (!_heldByLock && {_x < ((_last select _forEachIndex) - 0.0001)}) exitWith {
                _heli setVariable ["bmkhs_repairPending", true];
                diag_log text format ["[BMKHS] repair seen on %1: %2 %3 -> %4", typeOf _heli, _names select _forEachIndex, _last select _forEachIndex, _x];
            };
        } forEach _now;
    };
    _heli setVariable ["bmkhs_repairLastDmg", _now];
    _heli setVariable ["bmkhs_repairLastLocked", _locked];

    //The repair marker - a repair zeroes it, which is the drop above; it goes straight back
    if (([_heli, "tailRotor"] call bmkhs_fnc_damageGet) == 0) then {
        [_heli, "tailRotor", GSTRESS_REPAIR_MARK] call bmkhs_fnc_damageSet;
    };
};

if !(_heli getVariable "bmkhs_repairPending") exitWith {};
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
            [_heli, _role, EPSILON, _index] call bmkhs_fnc_damageSet;
        };
    };
} forEach (_heli getVariable ["bmkhs_sysStorage", []]);

//Producers and converters hold no state of their own - the solve recomputes what they
//make from their own damage, so they only need the marker.
{
    private _role = _x get "damageRole";
    if (_role != "" && {([_heli, _role, _x get "index"] call bmkhs_fnc_damageGet) == 0}) then {
        [_heli, _role, EPSILON, _x get "index"] call bmkhs_fnc_damageSet;
    };
} forEach ((_heli getVariable ["bmkhs_sysProducers", []]) + (_heli getVariable ["bmkhs_sysConverters", []]));

//Drive parts whose damage Core keeps have no hitpoint to repair, so a repair replaces them -
//as a store with no damage role comes back full either way.
{
    private _var = _x getOrDefault ["damageVar", ""];
    if (_var != "") then { _heli setVariable [_var, 0] };
} forEach (_heli getVariable ["bmkhs_sysTorqued", []]);

//Engines are not components - a repair clears their latches.
private _engines = _heli getVariable ["bmkhs_numEngines", 0];
for "_i" from 0 to (_engines - 1) do {
    if (([_heli, "engines", _i] call bmkhs_fnc_damageGet) == 0) then {
        [_heli, "bmkhs_engineOverspeed",  _i, false, true] call bmkhs_fnc_utilSetArrayVariable;
        [_heli, "bmkhs_engChips",         _i, false, true] call bmkhs_fnc_utilSetArrayVariable;
        [_heli, "bmkhs_engFailed",        _i, false, true] call bmkhs_fnc_utilSetArrayVariable;
        [_heli, "bmkhs_lowOilPsiFailure", _i, false, true] call bmkhs_fnc_utilSetArrayVariable;
        [_heli, "bmkhs_engOilPsiLow",     _i, false, true] call bmkhs_fnc_utilSetArrayVariable;
        [_heli, "bmkhs_engOilHealth",     _i, 1.0] call bmkhs_fnc_utilSetArrayVariable;
        if ((_heli getVariable "bmkhs_engFailureResult") == _i) then { _heli setVariable ["bmkhs_engFailureResult", -1] };
        [_heli, "engines", EPSILON, _i] call bmkhs_fnc_damageSet;
    };
};

//G-stress - only a full repair replaces the structure
(getAllHitPointsDamage _heli) params [["_names", []], "", ["_dmg", []]];
private _full = true;
{
    private _heldByLock = _locked && {(toLower (_names select _forEachIndex)) in _rotorHp};
    if (!_heldByLock && {_x > GSTRESS_REPAIR_MARK}) exitWith { _full = false };
} forEach _dmg;
diag_log text format ["[BMKHS] repair on %1: full %2, fatigue %3", typeOf _heli, _full, _heli getVariable "bmkhs_gFatigue"];
if (_full) then {
    private _fn   = _heli getVariable "bmkhs_partFunction";
    private _sels = _heli getVariable "bmkhs_partSelections";
    private _parts = +(_heli getVariable "bmkhs_partsLost");
    {
        private _rotorIndex = _forEachIndex;
        { if ((_x select 0) == BLADE_SEPARATED) then { _parts pushBack format ["rotor%1_blade%2", _rotorIndex, _forEachIndex + 1] } } forEach _x;
    } forEach (_heli getVariable "bmkhs_bladeState");
    if (_fn != "") then {
        {
            private _sel = _sels getOrDefault [toLower _x, ""];
            if (_sel != "") then { [_heli, _sel, true] call (missionNamespace getVariable [_fn, {}]) };
        } forEach _parts;
    };
    _heli setVariable ["bmkhs_gFatigue",  0.0];
    _heli setVariable ["bmkhs_partsLost", []];
    _heli setVariable ["bmkhs_bladeState", (_heli getVariable "bmkhs_bladeState") apply { _x apply { [0, 0.0] } }];
};
