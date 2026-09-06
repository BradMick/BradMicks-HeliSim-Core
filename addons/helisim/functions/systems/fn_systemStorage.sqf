/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemStorage

Description:
    Runs every store the aircraft declares - accumulators, batteries.

    Runs TWICE per solve. Charge is state rather than supply, so the first
    pass puts it onto circuits before anything upstream is solved, which is
    what cuts the startup loop. Charge itself can only move once the rest has
    solved, so draining and refilling happen on the settle pass - a store
    reading its own recharge circuit before the producers have run sees zero
    and never refills.

Parameters:
    _heli      - The helicopter [Object]
    _deltaTime - Frame time [Number]
    _settle    - false to put charge onto circuits, true to move charge from the
                 solved result [Bool]

Returns:
    Nothing - circuit values are accumulated into bmkhs_sysCircuits

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_index", "_deltaTime", ["_settle", false]];
#include "\bmkhs_helisim\functions\systems\systems.hpp"

//One component, named by the walk. Returns whether what it PUTS OUT moved; the settle
//pass is where charge is applied, and runs for every store regardless.
private _storage = _heli getVariable ["bmkhs_sysStorage", []];
if (_index >= (count _storage)) exitWith {false};
private _comp = _storage select _index;

private _varName = _comp get "varName";
private _nominal = _comp get "nominal";
private _charge  = _heli getVariable [_varName + "Charge", 1.0];

    private _damage  = [_heli, _comp get "damageRole", _comp get "index"] call bmkhs_fnc_damageGet;
    {
        _damage = _damage + ([_heli, _x] call bmkhs_fnc_damageGet);
    } forEach (_comp get "drainedBy");
    private _damaged = _damage > SYS_COMP_DMG_THRESH;

    //A variable name, or {circuit, threshold} read live.
    private _gateOn = true;
    {
        private _ok = if (_x isEqualType []) then {
            ([_heli, _x select 0] call bmkhs_fnc_systemCircuit) >= (_x select 1)
        } else {
            _heli getVariable [_x, false]
        };
        if (!_ok) exitWith { _gateOn = false };
    } forEach (_comp get "gates");

    //Leaking is separate from discharging - a holed store empties with nothing drawing
    //from it. Rate ramps from the onset threshold to full damage.
    private _leakStart = _comp get "leakStartDmg";
    if (_settle && _leakStart > 0 && _damage > _leakStart) then {
        if (_damage >= 1) then {
            //Destroyed holds nothing at all rather than draining out.
            _charge = 0;
        } else {
            private _frac = ((_damage - _leakStart) / (1 - _leakStart)) min 1;
            private _rate = _comp get "leakRate";
            _charge = (_charge - (_rate * _frac * _deltaTime)) max 0;
        };
    };

    //Below this it is spent - for a gas-charged store this is the precharge, which is
    //not usable pressure.
    private _spentFrac = if (_nominal > 0) then {(_comp get "stopBelow") / _nominal} else {0};

    //Spent once when the start gate rises, not per frame - the thing being cranked only
    //comes up seconds later, so a continuous drain empties the store before it does.
    //StartOk latches whether there was enough, since the draw itself drops the store
    //below startAbove and would cut the start it is paying for.
    private _startedBy = _comp get "startedBy";
    if (_settle && _startedBy != "") then {
        private _latchVar = _varName + "Drawn";
        private _okVar    = _varName + "StartOk";
        if (_heli getVariable [_startedBy, false]) then {
            if !(_heli getVariable [_latchVar, false]) then {
                private _above = _comp get "startAbove";
                private _ok    = _nominal <= 0 || {_charge * _nominal >= _above};
                //A start spends the usable charge, leaving the precharge behind.
                if (_ok && _nominal > 0) then {
                    _charge = _spentFrac;
                };
                _heli setVariable [_okVar,    _ok,  true];
                _heli setVariable [_latchVar, true, true];
            };
        } else {
            _heli setVariable [_latchVar, false, true];
            _heli setVariable [_okVar,    true,  true];
        };
    };

    //Anything else holding its nodes up makes the store a reserve, not a supply. On the
    //settle pass a node already carries this store's own supply, so compare against what
    //the producers put there rather than the total.
    private _outputs  = _comp get "outputs";
    private _elseFeed = 0;
    {
        private _c = _x get "circuit";
        if (_c != "") then {
            //The PRODUCER-only total, never the node total. Contributions persist between
            //frames now, so a store reading the whole node reads its own supply back as
            //somebody else's and shuts itself off - which is a store that can never be the
            //thing holding a circuit up.
            _elseFeed = _elseFeed max (_heli getVariable ["bmkhs_sysProducerFeed_" + _c, 0]);
        };
    } forEach _outputs;

    private _live      = !_damaged && _gateOn && _charge > _spentFrac;

    //Drains while nothing is covering for it. That is its charging source where it has
    //one - a battery runs down whenever the bus that charges it is dead - and otherwise
    //whatever else feeds its output.
    private _rechargedBy = _comp get "rechargedBy";
    private _covered     = if (_rechargedBy != "") then {
        ([_heli, _rechargedBy] call bmkhs_fnc_systemCircuit) > (_comp get "minRecharge")
    } else {
        _elseFeed > 0
    };

    if (_settle && _live && !_covered) then {
        private _drain = _comp get "emerRate";
        if (_drain > 0) then { _charge = (_charge - (_drain * _deltaTime)) max 0 };
    };

    //Refills whenever something is covering for it, and never from the node it supplies.
    if (_settle && _covered && _rechargedBy != "" && _charge < 1.0) then {
        //Over the usable band rather than the whole range, so the configured time is what
        //it actually takes - a store only ever refills from its floor.
        private _rate = (_comp get "rechargeRate") * (1 - _spentFrac);
        if (_rate > 0) then { _charge = (_charge + (_rate * _deltaTime)) min 1.0 };
    };

    //Whether it is up, as a property of the component rather than of any node - the same
    //publish a producer does.
    private _stateVar = _comp get "stateVar";
    if (_stateVar != "") then {
        [_heli, format ["bmkhs_%1", _stateVar], (_charge * _nominal) >= (_comp get "stateAbove")]
            call bmkhs_fnc_utilUpdateNetworkGlobal;
    };

    _heli setVariable [_varName + "Charge", _charge];
    private _published = _charge * _nominal;
    if (_comp get "networked") then {
        [_heli, _varName, _published] call bmkhs_fnc_utilUpdateNetworkGlobal;
    } else {
        _heli setVariable [_varName, _published];
    };

    //Only feeds while it is the one supplying - and when it stops, its stored
    //contribution has to be cleared or the node would hold it forever.
    private _moved = false;
    if !(_live && _elseFeed <= 0) then {
        {
            if ((_x get "circuit") != "") then {
                _heli setVariable [_varName + "Feed_" + (_x get "circuit"), 0];
                if (([_heli, _x get "circuit", _varName, 0, false] call bmkhs_fnc_systemCircuitFeed)) then {
                    _moved = true;
                };
            };
        } forEach _outputs;
    };
    if (_live && _elseFeed <= 0) then {
        {
            private _c = _x get "circuit";
            if (_c != "") then {
                private _fixed = _x get "nominal";
                private _val   = if (_fixed > 0) then {_fixed} else {_charge * _nominal * (_x get "ratio")};
                _heli setVariable [_varName + "Feed_" + _c, _val];
                if (([_heli, _c, _varName, _val, false] call bmkhs_fnc_systemCircuitFeed)) then {
                    _moved = true;
                };
            };
        } forEach _outputs;
    };

//Mid-transition keeps itself awake: a store actively draining or refilling has more to do
//next frame, the same way a spooling producer does.
private _busy = _settle && _live && {(!_covered && {(_comp get "emerRate") > 0})
                                  || {_covered && _rechargedBy != "" && _charge < 1.0}};
_heli setVariable [_varName + "Awake", _busy];

_moved
