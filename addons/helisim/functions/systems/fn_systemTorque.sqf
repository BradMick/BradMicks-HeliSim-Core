/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemTorque

Description:
    Damages components run past their torque limits. A drive component is rated
    for a torque, and for how long it will take more than that - the limits are
    the aircraft's, since a gearbox is rated for what it is rated for, and
    accruing damage past one is Core's.

    Limits are declared low to high as {torque, seconds, divisor}: how much it
    will take and how long before that starts costing it. A zero duration
    damages immediately.

    A component already damaged degrades further on its own, faster the worse
    it is, which is what makes an overtorqued gearbox a problem that grows
    rather than a threshold that trips once.

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

//Damaged once, where the aircraft is local - this runs outside the solve, so it does not
//inherit the solve's guard.
if !(local _heli) exitWith {};

private _torqued = _heli getVariable ["bmkhs_sysTorqued", []];
if (_torqued isEqualTo []) exitWith {};

private _engines  = _heli getVariable "bmkhs_engines";
private _tqTimers = _engines apply {-1};
private _driveDmg = _engines apply {0};

{
    private _comp    = _x;
    private _role    = _x get "damageRole";
    private _index   = _x get "index";

    //The engine's limits - times the engine count where the component carries them all,
    //except single engine, when one engine carries the sum.
    private _fromEngine = {
        params ["_key", "_single"];
        if (_key == "") exitWith {[]};
        if (_comp get "torqueSum") exitWith {
            private _n = [count _engines, 1] select _single;
            ((_engines select 0) get _key) apply {[(_x select 0) * _n, _x select 1, (_x param [2, 0]) * _n]}
        };
        (_engines select (_index min ((count _engines) - 1))) get _key
    };
    private _limits   = [_comp get "tqLimitsFrom", false] call _fromEngine;
    private _seLimits = [_comp get "tqLimitsSeFrom", true] call _fromEngine;
    if (_seLimits isNotEqualTo [] && {_heli getVariable "bmkhs_isSingleEng"}) then {
        _limits = _seLimits;
    };

    //What this component is carrying. Per-member where the source is per-member, so
    //engine 2's torque is what nose gearbox 2 sees.
    private _tqVar = _x get "torqueFrom";
    private _tq    = 0;
    if (_tqVar != "") then {
        private _val = _heli getVariable [_tqVar, 0];
        //Summed where the component carries the lot - a transmission takes both engines'
        //output, while a nose gearbox takes only its own engine's.
        private _sums = _comp get "torqueSum";
        _tq = if !(_val isEqualType []) then {_val} else {
            if (_sums) then {
                private _total = 0;
                { _total = _total + _x } forEach _val;
                _total
            } else {
                _val param [_index, 0]
            };
        };
    };

    //Its damage is its role's hitpoint. Where it has no role, it damages named hitpoints
    //instead and reads the worst of them - or, naming none, Core keeps it (fn_systemsComponents).
    private _direct = _comp get "damages";
    private _ownVar = _comp getOrDefault ["damageVar", ""];
    private _damage = call {
        if (_ownVar != "") exitWith { _heli getVariable [_ownVar, 0] };
        if (_direct isEqualTo []) exitWith { [_heli, _role, _index] call bmkhs_fnc_damageGet };
        private _worst = 0;
        { _worst = _worst max ((_heli getHitPointDamage _x) max 0) } forEach _direct;
        _worst
    };
    private _accrue = 0;

    //Nothing accrues with the engines off - an unpowered drivetrain is not overtorqued.
    private _running = isEngineOn _heli;

    //One clock per tier, running only while the torque is IN that tier and reset the moment
    //it leaves. Time spent higher up does not count toward a lower tier's grace, and a
    //brief excursion is not cumulative. Any tier whose clock has expired arms the damage.
    private _armed = false;
    private _band  = -1;
    {
        _x params ["_limit", "_seconds"];
        //Low to high, so a tier's ceiling is the next tier's limit; the top tier has none.
        private _ceiling = if (_forEachIndex == ((count _limits) - 1)) then {1e10}
                                             else {(_limits select (_forEachIndex + 1)) select 0};
        private _timerVar = format ["bmkhs_tqTimer_%1%2_%3", _role, _index, _forEachIndex];

        if (_running && {_tq > _limit} && {_tq <= _ceiling}) then {
            if (_seconds <= 0) then {
                _armed = true;                     //no grace at all above this
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

    //The countdown each engine shows - the soonest of the parts its torque loads.
    if (_band >= 0) then {
        {
            if ((_comp get "torqueSum") || {_forEachIndex == _index}) then {
                _tqTimers set [_forEachIndex, [_band, _band min _x] select (_x >= 0)];
            };
        } forEach +_tqTimers;
    };

    //Rate scales with HOW FAR past each limit it is, and the tiers stack - pulled harder,
    //it comes apart faster.
    if (_armed) then {
        {
            _x params ["_limit", "_seconds", ["_divisor", 0]];
            if (_divisor > 0 && {_tq > _limit}) then {
                _accrue = _accrue + ((_tq - _limit) / _divisor);
            };
        } forEach _limits;
    };

    //Damage feeds itself while turning - the bands REPLACE each other rather than stacking.
    if (_running && {_damage > 0.25}) then {
        private _persistent = _damage / 600.0;
        if (_damage > 0.50) then { _persistent = _damage / 500.0 };
        if (_damage > 0.75) then { _persistent = _damage / 400.0 };
        _accrue = _accrue + _persistent;
    };

    if (_accrue > 0) then {
        _damage = (_damage + (_accrue * _deltaTime)) min 1.0;
        call {
            if (_ownVar != "") exitWith { _heli setVariable [_ownVar, _damage] };
            if (_direct isEqualTo []) exitWith { [_heli, _role, _damage, _index] call bmkhs_fnc_damageSet };
            { _heli setHitPointDamage [_x, _damage] } forEach _direct;
        };
    };

    //The worst damage among the slipping parts each engine loads.
    if (_comp get "jitters") then {
        {
            if ((_comp get "torqueSum") || {_forEachIndex == _index}) then {
                _driveDmg set [_forEachIndex, _x max _damage];
            };
        } forEach +_driveDmg;
    };

    //What a destroyed component takes with it. An entry naming a damage role destroys that
    //role outright - a transmission is what holds the rotors, the generators and the pumps
    //up, so losing it loses all of them. An entry naming a variable latches it true for the
    //engines this component carries - its own engine for a nose gearbox, every engine for a
    //component that sums them (a transmission). Unloaded, an engine overspeeds and trips.
    //Set on destruction only; repair clears it.
    {
        if ((_x select [0, 6]) == "bmkhs_") then {
            if (_damage >= 1.0) then {
                private _var = _x;
                private _members = [_index];
                if (_comp get "torqueSum") then {
                    _members = [];
                    for "_e" from 0 to ((count _engines) - 1) do { _members pushBack _e };
                };
                {
                    if !((_heli getVariable [_var, []]) param [_x, false]) then {
                        [_heli, _var, _x, true, true] call bmkhs_fnc_utilSetArrayVariable;
                    };
                } forEach _members;
            };
        } else {
            if (_damage >= 1.0) then {
                private _n = [_heli, _x] call bmkhs_fnc_damageCount;
                for "_m" from 0 to ((_n max 1) - 1) do {
                    [_heli, _x, 1.0, _m] call bmkhs_fnc_damageSet;
                };
            };
        };
    } forEach (_comp get "breaksVar");
} forEach _torqued;

_heli setVariable ["bmkhs_engTqTimer", _tqTimers];

//A damaged drive slips like a failing clutch - each engine on its own, the torque it passes
//dropping and grabbing again, more often the worse the damage.
{
    private _i      = _forEachIndex;
    private _slip   = 1.0;
    private _t      = (_heli getVariable "bmkhs_engSlipT") select _i;
    if (isEngineOn _heli && {_x > 0.25}) then {
        if (_t < 0) then {
            private _wait = ((_heli getVariable "bmkhs_engSlipWait") select _i) - _deltaTime;
            if (_wait <= 0) then {
                _t = 0;
                [_heli, "bmkhs_engSlipDepth", _i, _x * SYS_SLIP_DEPTH * (0.5 + random 0.5)] call bmkhs_fnc_utilSetArrayVariable;
                _wait = (linearConversion [0.25, 1.0, _x, SYS_SLIP_WAIT_LOW_DMG, SYS_SLIP_WAIT_HIGH_DMG, true])
                      * (0.8 + random 0.4);
            };
            [_heli, "bmkhs_engSlipWait", _i, _wait] call bmkhs_fnc_utilSetArrayVariable;
        } else {
            _t = _t + _deltaTime;
        };
        if (_t >= 0) then {
            private _d = (_heli getVariable "bmkhs_engSlipDepth") select _i;
            if (_t < SYS_SLIP_DROP_SEC) then {
                _slip = 1.0 - (_d * (_t / SYS_SLIP_DROP_SEC));
            } else {
                if (_t < SYS_SLIP_GRAB_SEC) then {
                    _slip = 1.0 + linearConversion [SYS_SLIP_DROP_SEC, SYS_SLIP_GRAB_SEC, _t, -_d, _d * SYS_SLIP_OVERSHOOT];
                } else {
                    if (_t < SYS_SLIP_END_SEC) then {
                        _slip = 1.0 + linearConversion [SYS_SLIP_GRAB_SEC, SYS_SLIP_END_SEC, _t, _d * SYS_SLIP_OVERSHOOT, 0];
                    } else {
                        _t = -1;
                    };
                };
            };
        };
    } else {
        _t = -1;
    };
    [_heli, "bmkhs_engSlipT", _i, _t] call bmkhs_fnc_utilSetArrayVariable;
    [_heli, "bmkhs_engClutchSlip", _i, _slip] call bmkhs_fnc_utilSetArrayVariable;
} forEach _driveDmg;
