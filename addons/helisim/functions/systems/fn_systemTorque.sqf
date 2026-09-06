/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemTorque

Description:
    Damages components run past their torque limits. A drive component is rated
    for a torque, and for how long it will take more than that - the limits are
    the aircraft's, since a gearbox is rated for what it is rated for, and
    accruing damage past one is Core's.

    Limits are declared worst-first as {torque, seconds}: how much it will take
    and how long before that starts costing it. A zero duration damages
    immediately.

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

{
    private _comp    = _x;
    private _role    = _x get "damageRole";
    private _index   = _x get "index";
    //Single-engine ratings where the aircraft declares them, since one engine doing the
    //work of two is a different case from both sharing it.
    private _limits  = _x get "tqLimits";
    private _seLimits = _x get "tqLimitsSE";
    if ((count _seLimits) > 0 && {_heli getVariable ["bmkhs_isSingleEng", false]}) then {
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

    //Where no role claims this component, it damages named hitpoints instead - and reads
    //its current damage from the worst of them.
    private _direct = _comp get "damages";
    private _damage = if (_direct isEqualTo []) then {
        [_heli, _role, _index] call bmkhs_fnc_damageGet
    } else {
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
    {
        _x params ["_limit", "_seconds"];
        //Worst first, so a tier's ceiling is the limit above it; the top tier has none.
        private _ceiling = if (_forEachIndex == 0) then {1e10}
                                             else {(_limits select (_forEachIndex - 1)) select 0};
        private _timerVar = format ["bmkhs_tqTimer_%1%2_%3", _role, _index, _forEachIndex];

        if (_running && {_tq > _limit} && {_tq <= _ceiling}) then {
            if (_seconds <= 0) then {
                _armed = true;                     //no grace at all above this
            } else {
                private _held = (_heli getVariable [_timerVar, 0]) + _deltaTime;
                if (_held >= _seconds) then {
                    _held  = _seconds;
                    _armed = true;
                };
                _heli setVariable [_timerVar, _held];
            };
        } else {
            _heli setVariable [_timerVar, 0];
        };
    } forEach _limits;

    //Rate scales with HOW FAR past each limit it is, and the tiers stack - pulled harder,
    //it comes apart faster. Each tier declares its own divisor; the deeper ones bite less
    //per unit because they are already being counted by the tiers beneath them.
    if (_armed) then {
        {
            _x params ["_limit", "_seconds", ["_divisor", 0]];
            if (_divisor > 0 && {_tq > _limit}) then {
                _accrue = _accrue + ((_tq - _limit) / _divisor);
            };
        } forEach _limits;
    };

    //Damage feeds itself: the worse it is, the faster it worsens. The bands REPLACE each
    //other rather than stacking, so the rate is the one band it is in.
    if (_damage > 0.25) then {
        private _persistent = _damage / 600.0;
        if (_damage > 0.50) then { _persistent = _damage / 500.0 };
        if (_damage > 0.75) then { _persistent = _damage / 400.0 };
        _accrue = _accrue + _persistent;
    };

    if (_accrue > 0) then {
        _damage = (_damage + (_accrue * _deltaTime)) min 1.0;
        if (_direct isEqualTo []) then {
            [_heli, _role, _damage, _index] call bmkhs_fnc_damageSet;
        } else {
            { _heli setHitPointDamage [_x, _damage] } forEach _direct;
        };
    };

    //A damaged drive makes the torque needle wander, scaled by how bad it is. The
    //component publishes its OWN, under its own variable, and whatever reads torque asks
    //Core for the total - no shared array with a layout baked into it.
    if (_comp get "jitters") then {
        _heli setVariable [(_comp get "varName") + "TqJitter",
            if (_damage > 0.25) then {_damage * (random [-0.10, 0, 0.10])} else {0}];
    };

    //What a destroyed component takes with it. An entry naming a damage role destroys that
    //role outright - a transmission is what holds the rotors, the generators and the pumps
    //up, so losing it loses all of them. An entry naming a variable sets it at this
    //member's index instead, which is how a nose gearbox that has come apart overspeeds
    //the engine driving it.
    {
        if ((_x select [0, 6]) == "bmkhs_") then {
            [_heli, _x, _index, _damage >= 1.0, false] call bmkhs_fnc_utilSetArrayVariable;
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
