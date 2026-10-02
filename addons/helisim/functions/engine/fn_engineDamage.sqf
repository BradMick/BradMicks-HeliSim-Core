/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engineDamage

Description:
    Engine damage from the Np, Ng and TGT book limits, and the damage ladder.

Parameters:
    _heli      - The helicopter [Object]
    _deltaTime - Frame time [Number]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_deltaTime"];
#include "\bmkhs_helisim\functions\systems\systems.hpp"

private _engines = _heli getVariable "bmkhs_engines";
private _state   = _heli getVariable "bmkhs_engState";

//{timer name, limit set, readings} - the order the countdowns are published in.
private _readings = [
    ["np",  "npLimits",  _heli getVariable "bmkhs_engPctNp"],
    ["ng",  "ngLimits",  _heli getVariable "bmkhs_engPctNg"],
    ["tgt", ["tgtLimits", "tgtLimitsSe"] select (_heli getVariable "bmkhs_isSingleEng"), _heli getVariable "bmkhs_engTgt"]
];

//Self-worsening, as fn_systemTorque.
private _worsen = {
    params ["_damage"];
    private _persistent = _damage / 600.0;
    if (_damage > 0.50) then { _persistent = _damage / 500.0 };
    if (_damage > 0.75) then { _persistent = _damage / 400.0 };
    _persistent
};

// ── Book limits ─────────────────────────────────────────────────────────────
private _running = [];
private _accrues = [];
{
    private _engine = _x;
    private _i      = _forEachIndex;
    private _on     = (_state select _i) in ["STARTING", "ON"];
    private _accrue = 0;
    private _left   = [];

    {
        _x params ["_name", "_key", "_values"];
        private _limits = _engine get _key;
        private _value  = _values select _i;
        private _band   = -1;

        //Tier clocks, as fn_systemTorque.
        private _armed = false;
        {
            _x params ["_limit", "_seconds"];
            private _ceiling = if (_forEachIndex == ((count _limits) - 1)) then {1e10}
                                                 else {(_limits select (_forEachIndex + 1)) select 0};
            private _timerVar = format ["bmkhs_engTimer_%1%2_%3", _name, _i, _forEachIndex];

            if (_on && {_value > _limit} && {_value <= _ceiling}) then {
                if (_seconds <= 0) then {
                    _armed = true;
                    _band  = 0;
                } else {
                    private _held = (_heli getVariable [_timerVar, 0]) + _deltaTime;
                    if (_held >= _seconds) then {
                        _held  = _seconds;
                        _armed = true;
                    };
                    _heli setVariable [_timerVar, _held];
                    _band = _seconds - _held;
                };
            } else {
                _heli setVariable [_timerVar, 0];
            };
        } forEach _limits;

        if (_armed) then {
            {
                _x params ["_limit", "_seconds", ["_divisor", 0]];
                if (_divisor > 0 && {_value > _limit}) then {
                    _accrue = _accrue + ((_value - _limit) / _divisor);
                };
            } forEach _limits;
        };
        _left pushBack _band;
    } forEach _readings;

    //Hot start - above startTgt while starting, no allowance.
    private _tgtNow = (_heli getVariable "bmkhs_engTgt") select _i;
    if ((_state select _i) == "STARTING" && {_tgtNow > (_engine get "startTgt")}) then {
        _accrue = _accrue + ((_tgtNow - (_engine get "startTgt")) / SYS_ENG_HOTSTART_DIVISOR);
    };

    _running pushBack _on;
    _accrues pushBack _accrue;
    [_heli, "bmkhs_engLimitTimers", _i, _left] call bmkhs_fnc_utilSetArrayVariable;
} forEach _engines;

// ── useSystems = 1: a hitpoint per engine ───────────────────────────────────
if (_heli getVariable "bmkhs_useSystems") exitWith {
    {
        private _i      = _forEachIndex;
        private _role   = _x get "damageRole";
        private _index  = _x get "damageRoleIndex";
        private _on     = _running select _i;
        private _damage = [_heli, _role, _index] call bmkhs_fnc_damageGet;
        private _accrue = _accrues select _i;

        if (_on && {_damage > 0.25}) then { _accrue = _accrue + ([_damage] call _worsen) };

        //Oil runs down from 0.65, faster from 0.75, gone at 0.85.
        private _health = (_heli getVariable "bmkhs_engOilHealth") select _i;
        if (_on && {_damage > SYS_ENG_OIL_DROP_DMG}) then {
            private _drain = (_damage - SYS_ENG_OIL_DROP_DMG) / SYS_ENG_OIL_DIVISOR;
            if (_damage > SYS_ENG_OIL_FAST_DMG) then {
                _drain = _drain + ((_damage - SYS_ENG_OIL_FAST_DMG) / (SYS_ENG_OIL_DIVISOR * 2));
            };
            _health = (_health - (_drain * _deltaTime)) max 0;
        };
        if (_damage >= SYS_ENG_OIL_ZERO_DMG) then { _health = 0 };
        [_heli, "bmkhs_engOilHealth", _i, _health] call bmkhs_fnc_utilSetArrayVariable;

        //Oil starvation, scaled by Ng.
        if (_on && {_health <= 0}) then {
            private _ng = (_heli getVariable "bmkhs_engPctNg") select _i;
            _accrue = _accrue + (SYS_ENG_STARVE_RATE * (_ng / SYS_ENG_STARVE_REF_NG));
            if !((_heli getVariable "bmkhs_lowOilPsiFailure") select _i) then {
                [_heli, "bmkhs_lowOilPsiFailure", _i, true, true] call bmkhs_fnc_utilSetArrayVariable;
            };
        };

        if (_accrue > 0) then {
            _damage = (_damage + (_accrue * _deltaTime)) min 1.0;
            [_heli, _role, _damage, _index] call bmkhs_fnc_damageSet;
        };

        if (_damage >= SYS_ENG_CHIPS_DMG && {!((_heli getVariable "bmkhs_engChips") select _i)}) then {
            [_heli, "bmkhs_engChips", _i, true, true] call bmkhs_fnc_utilSetArrayVariable;
        };
        if (_damage >= 1.0 && {!((_heli getVariable "bmkhs_engFailed") select _i)}) then {
            [_heli, "bmkhs_engFailed", _i, true, true] call bmkhs_fnc_utilSetArrayVariable;
        };
    } forEach _engines;
};

// ── useSystems = 0: every engine shares hitengine ───────────────────────────
private _role   = (_engines select 0) get "damageRole";
private _damage = [_heli, _role, 0] call bmkhs_fnc_damageGet;
private _n      = count _engines;

//As a single engine - the worse of them, not the sum.
private _accrue = 0;
{ _accrue = _accrue max _x } forEach _accrues;

if ((true in _running) && {_damage > 0.25}) then { _accrue = _accrue + ([_damage] call _worsen) };

//Oil starvation on any engine turning with no oil.
{
    if ((_running select _forEachIndex) && {_x <= 0}) then {
        _accrue = _accrue + (SYS_ENG_STARVE_RATE * (((_heli getVariable "bmkhs_engPctNg") select _forEachIndex) / SYS_ENG_STARVE_REF_NG));
    };
} forEach (_heli getVariable "bmkhs_engOilHealth");

if (_accrue > 0) then {
    _damage = (_damage + (_accrue * _deltaTime)) min 1.0;
    [_heli, _role, _damage] call bmkhs_fnc_damageSet;
};

//Chips or oil pressure, a coin toss.
private _fault = {
    params ["_e"];
    if (random 1 < 0.5) then {
        [_heli, "bmkhs_engChips", _e, true, true] call bmkhs_fnc_utilSetArrayVariable;
    } else {
        [_heli, "bmkhs_engOilHealth", _e, 0] call bmkhs_fnc_utilSetArrayVariable;
        [_heli, "bmkhs_lowOilPsiFailure", _e, true, true] call bmkhs_fnc_utilSetArrayVariable;
    };
};

//0.50 before 0.25, so a hit past both fails a random engine without a fault.
private _pick = _heli getVariable "bmkhs_engFailureResult";
if (_damage >= SYS_ENG_SHARED_FAIL_DMG && {!(true in (_heli getVariable "bmkhs_engFailed"))}) then {
    [_heli, "bmkhs_engFailed", [_pick, floor random _n] select (_pick < 0), true, true]
        call bmkhs_fnc_utilSetArrayVariable;
};
if (_damage >= SYS_ENG_SHARED_CHIPS_DMG && {_pick < 0} && {!(true in (_heli getVariable "bmkhs_engFailed"))}) then {
    _pick = floor random _n;
    _heli setVariable ["bmkhs_engFailureResult", _pick];
    [_pick] call _fault;
};

//The same on the engine still operating.
if (_damage >= SYS_ENG_SHARED_CHIPS2_DMG) then {
    for "_i" from 0 to (_n - 1) do {
        if (!((_heli getVariable "bmkhs_engFailed") select _i)
            && {!((_heli getVariable "bmkhs_engChips") select _i)}
            && {!((_heli getVariable "bmkhs_lowOilPsiFailure") select _i)}) then {
            [_i] call _fault;
        };
    };
};
if (_damage >= 1.0) then {
    for "_i" from 0 to (_n - 1) do {
        if !((_heli getVariable "bmkhs_engFailed") select _i) then {
            [_heli, "bmkhs_engFailed", _i, true, true] call bmkhs_fnc_utilSetArrayVariable;
        };
    };
};
