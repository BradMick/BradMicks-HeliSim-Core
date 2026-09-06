/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemProducer

Description:
    Runs every producer the aircraft declares - pumps, generators, the APU.
    A producer feeds its circuit while undamaged, gated on, and driven fast
    enough, scaled by whatever it draws from, ramping toward its target.

    Damage is read AT THE MEMBER'S INDEX - the role alone returns the worst
    member, which would fail every generator because one is destroyed.

Parameters:
    _heli      - The helicopter [Object]
    _deltaTime - Frame time [Number]

Returns:
    Nothing - circuit values are accumulated into bmkhs_sysCircuits

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_index", "_deltaTime"];
#include "\bmkhs_helisim\functions\systems\systems.hpp"

//One component, named by the walk. Returns whether its output MOVED, which is what
//decides if the things it feeds need waking.
private _producers = _heli getVariable ["bmkhs_sysProducers", []];
if (_index >= (count _producers)) exitWith {false};
private _comp = _producers select _index;

private _varName = _comp get "varName";
private _nominal = _comp get "nominal";

    private _damaged = ([_heli, _comp get "damageRole", _comp get "index"] call bmkhs_fnc_damageGet)
                            > SYS_COMP_DMG_THRESH;

    //No gate means always armed, and every gate declared has to be on. A gated component
    //that is off is not failed - it just contributes nothing.
    //A variable name, or {circuit, threshold} read live. _comp because the forEach
    //rebinds _x.
    private _gateOn = true;
    private _gateWhy = "";
    {
        private _ok = if (_x isEqualType []) then {
            ([_heli, _x select 0] call bmkhs_fnc_systemCircuit) >= (_x select 1)
        } else {
            _heli getVariable [_x, false]
        };
        if (!_ok) exitWith {
            _gateOn = false;
            _gateWhy = if (_x isEqualType []) then {_x select 0} else {_x select [6]};
        };
    } forEach (_comp get "gates");
    _heli setVariable [_varName + "GateWhy", _gateWhy];

    //Per-component threshold: an autorotating rotor drives hydraulics at 0.45 but not
    //generators at 0.85.
    private _drivenBy = _comp get "drivenBy";
    private _driven   = _drivenBy == ""
                     || {([_heli, _drivenBy] call bmkhs_fnc_systemCircuit) > (_comp get "minDrive")};

    //Scales rather than gates, so a leak shows as falling pressure. Still closes the
    //chain: no fluid is no pressure.
    private _requires = _comp get "requires";
    private _supply   = if (_requires == "") then {1} else {
        //Full output down to the level where it loses prime, zero below that.
        private _level = _heli getVariable [_requires, 1];
        (linearConversion [_comp get "requiresAbove", 1, _level, 0, 1, true])
    };

    //No nominal means it carries whatever drives it - a shaft turns at the speed of the
    //thing turning it rather than producing a level of its own.
    private _out_val = if (_nominal > 0) then {_nominal} else {
        [_heli, _drivenBy] call bmkhs_fnc_systemCircuit
    };

    private _target  = ([0, _out_val] select (!_damaged && _gateOn && _driven)) * _supply;
    private _current = _heli getVariable [_varName, 0];

    //What the producer itself saw, rather than the debug re-deriving it separately.
    _heli setVariable [_varName + "Why", [_damaged, _gateOn, _driven, _supply, _out_val, _deltaTime]];

    //Nothing left to move means no pressure at once - a destroyed reservoir does not
    //bleed its pump down gently.
    private _rate = [_comp get "rampRate", 0] select (_supply <= 0);
    private _out  = if (_rate <= 0) then {
        _target
    } else {
        private _step = _rate * _deltaTime;
        if (_current < _target) then {(_current + _step) min _target} else {(_current - _step) max _target};
    };

    //Gauges read in steps, not single units.
    private _step = _comp get "increment";
    if (_step > 0) then { _out = round (_out / _step) * _step };

    //Mid-transition keeps ITSELF awake: a spooling APU has not reached its target, so it
    //asks for the next frame. Everything else waits to be woken by what it reads.
    private _ramping = _out != _target;
    _heli setVariable [_varName + "Tgt", _target];
    _heli setVariable [_varName + "Awake", _ramping];

    private _moved = _out != (_heli getVariable [_varName, 0]);

    if (_comp get "networked") then {
        [_heli, _varName, _out] call bmkhs_fnc_utilUpdateNetworkGlobal;
    } else {
        _heli setVariable [_varName, _out];
    };

    //Whether it is RUNNING, which is a property of the component rather than of any node -
    //an APU is on once it is turning fast enough to be useful.
    private _stateVar = _comp get "stateVar";
    if (_stateVar != "") then {
        private _running = _out >= (_comp get "stateAbove");
        [_heli, format ["bmkhs_%1", _stateVar], _running] call bmkhs_fnc_utilUpdateNetworkGlobal;
    };

    //Everything it feeds. Highest feeder wins each node, and the contribution is recorded
    //separately so a store can tell whether anything OTHER than itself is supplying it.
    {
        private _circuit = _x get "circuit";
        if (_circuit == "") then { continue };

        //A clutch drops THIS output out - an APU declutches from the accessory section
        //once the rotor is driving it, while its bleed air carries on.
        private _clutch = _x get "disengageOn";
        if (_clutch != "" && {([_heli, _clutch] call bmkhs_fnc_systemCircuit) >= (_x get "disengageAt")}) then {
            continue;
        };

        private _fixed = _x get "nominal";
        private _val   = if (_out <= 0) then {0} else {
            if (_fixed > 0) then {_fixed} else {_out * (_x get "ratio")}
        };

        _heli setVariable [_varName + "Feed_" + _circuit, _val];
        //Whether the NODE moved, not just this feeder - that is what needs propagating.
        if (([_heli, _circuit, _varName, _val, true] call bmkhs_fnc_systemCircuitFeed)) then {
            _moved = true;
        };
    } forEach (_comp get "outputs");

//Moved, or still ramping - either way the walk has more to do with this one.
_moved || _ramping
