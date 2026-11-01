/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_damageGStress

Description:
    Accrues g-stress fatigue and rolls the failures it causes - main rotor blades
    and the tail structure.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\rotor\rotor.hpp"
#include "\bmkhs_helisim\functions\damage\gStress.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

private _deltaTime = _heli getVariable "bmkhs_deltaTime";
private _fatigue   = _heli getVariable "bmkhs_gFatigue";
private _g         = ((_heli getVariable "bmkhs_loadFactor") select 2) min GSTRESS_G_CAP;

private _show = {
    params ["_part"];
    private _fn  = _heli getVariable "bmkhs_partFunction";
    private _sel = (_heli getVariable "bmkhs_partSelections") getOrDefault [toLower _part, ""];
    if (_fn != "" && {_sel != ""}) then { [_heli, _sel, false] call (missionNamespace getVariable [_fn, {}]) };
};

//Rotor imbalance tears the transmission off its mounts, and the other blades off the hub
private _imbalance = 0.0;
{ _imbalance = _imbalance + _x } forEach (_heli getVariable "bmkhs_rtrImbalance");
if (_imbalance > GSTRESS_MOUNT_RATING) then {
    private _rate = ((_imbalance - GSTRESS_MOUNT_RATING) / GSTRESS_MOUNT_DIVISOR) * _deltaTime;
    private _xmsn = (_heli getVariable ["bmkhs_sysTorqued", []]) select {(_x get "damageRole") == "transmission" || {(_x get "varName") == "bmkhs_transmission"}};
    if (_xmsn isNotEqualTo []) then {
        _xmsn = _xmsn select 0;
        private _ownVar = _xmsn getOrDefault ["damageVar", ""];
        private _damage = if (_ownVar != "") then { _heli getVariable [_ownVar, 0] } else { [_heli, "transmission", _xmsn get "index"] call bmkhs_fnc_damageGet };
        _damage = (_damage + _rate) min 1.0;
        if (_ownVar != "") then { _heli setVariable [_ownVar, _damage] } else { [_heli, "transmission", _damage, _xmsn get "index"] call bmkhs_fnc_damageSet };
    };
    if (random 1.0 < _rate * GSTRESS_SHED_SCALE) then {
        private _states = _heli getVariable "bmkhs_bladeState";
        {
            private _rotorIndex = _forEachIndex;
            if ((_x findIf {(_x select 0) == BLADE_SEPARATED}) < 0) then { continue };
            private _intact = [];
            { if ((_x select 0) != BLADE_SEPARATED) then { _intact pushBack _forEachIndex } } forEach _x;
            if (_intact isEqualTo []) then { continue };
            private _blade = selectRandom _intact;
            _x set [_blade, [BLADE_SEPARATED, 0.0]];
            [format ["rotor%1_blade%2", _rotorIndex, _blade + 1]] call _show;
        } forEach _states;
        _heli setVariable ["bmkhs_bladeState", _states];
    };
};

if (_fatigue >= 1.0 || {_g <= (GSTRESS_LIMITS select 0)} || {[_heli] call bmkhs_fnc_stateOnGround}) exitWith {};

private _accrue = 0.0;
{
    if (_g > _x) then { _accrue = _accrue + ((_g - _x) / (GSTRESS_DIVISORS select _forEachIndex)) };
} forEach GSTRESS_LIMITS;

private _mult = 1.0;
{
    if (_fatigue > _x) then { _mult = GSTRESS_PROGRESS_MULT select _forEachIndex };
} forEach GSTRESS_PROGRESS_STEPS;

private _gained = (_accrue * _mult * _deltaTime) min (1.0 - _fatigue);
_fatigue        = _fatigue + _gained;
_heli setVariable ["bmkhs_gFatigue", _fatigue];

//Tail structure - the boom takes the fin and stabilator
private _lost = _heli getVariable "bmkhs_partsLost";
{
    _x params ["_part", "_at", "_takes"];
    if (_fatigue >= _at && {!(_part in _lost)}) then {
        {
            if !(_x in _lost) then {
                _lost pushBack _x;
                [_x] call _show;
            };
        } forEach ([_part] + _takes);
    };
} forEach [
    ["stabilator",  GSTRESS_STAB_FAIL, []],
    ["verticalFin", GSTRESS_FIN_FAIL,  []],
    ["tailboom",    GSTRESS_BOOM_FAIL, ["verticalFin", "stabilator"]]
];
_heli setVariable ["bmkhs_partsLost", _lost];

//Main rotor
if (random 1.0 >= _gained * GSTRESS_EVENT_SCALE) exitWith {};

private _rotors = _heli getVariable "bmkhs_simpleRotors";
private _states = _heli getVariable "bmkhs_bladeState";
private _mains  = [];
{ if ((_x get "type") == MAIN) then { _mains pushBack _forEachIndex } } forEach _rotors;
if (_mains isEqualTo []) exitWith {};

private _rotorIndex = selectRandom _mains;
private _blades     = _states select _rotorIndex;
private _intact     = [];
{ if ((_x select 0) != BLADE_SEPARATED) then { _intact pushBack _forEachIndex } } forEach _blades;
if (_intact isEqualTo []) exitWith {};

private _blade = selectRandom _intact;
(_blades select _blade) params ["_mode", "_track"];
private _event = [BLADE_OUT_OF_TRACK, BLADE_TIP_CAP, BLADE_PC_LINK, BLADE_SEPARATED] selectRandomWeighted [
    (1.0 - _fatigue) ^ 2,
    _fatigue * (1.0 - _fatigue),
    _fatigue * (1.0 - _fatigue),
    _fatigue ^ 2
];
private _sign = if (_track == 0.0) then { selectRandom [1.0, -1.0] } else { [-1.0, 1.0] select (_track > 0.0) };
switch (_event) do {
    case BLADE_OUT_OF_TRACK: { _track = _track + (_sign * BLADE_TRACK_STEP) };
    case BLADE_TIP_CAP:      { _track = _track + (_sign * BLADE_TIP_CAP_TRACK) };
};
_mode = _mode max _event;
_blades set [_blade, [_mode, _track]];
_heli setVariable ["bmkhs_bladeState", _states];

if (_mode == BLADE_SEPARATED) then {
    [format ["rotor%1_blade%2", _rotorIndex, _blade + 1]] call _show;
};
