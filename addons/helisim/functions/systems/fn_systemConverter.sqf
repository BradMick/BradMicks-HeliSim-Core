/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemConverter

Description:
    Runs every converter the aircraft declares. A converter consumes from one
    circuit and produces onto another - a rectifier taking AC and making DC, an
    inverter doing the reverse, a gearbox taking one shaft speed and giving
    another.

    It CREATES nothing. Without its input it has nothing to pass on, which is
    the whole difference between it and a source.

    Direction is data: swap input and output and a rectifier is an inverter, so
    a DC-generator aircraft needs no code here.

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

//One component, named by the walk. Returns whether its output MOVED.
private _converters = _heli getVariable ["bmkhs_sysConverters", []];
if (_index >= (count _converters)) exitWith {false};
private _comp = _converters select _index;

private _varName = _comp get "varName";

    private _damaged = ([_heli, _comp get "damageRole", _comp get "index"] call bmkhs_fnc_damageGet)
                            > SYS_COMP_DMG_THRESH;

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

    //What it has to work with. Nothing in, nothing out.
    private _inVal = [_heli, _comp get "input"] call bmkhs_fnc_systemCircuit;
    private _hasIn = _inVal > (_comp get "minInput");

    //A fixed nominal converts to a level - a rectifier makes DC or it does not. Without
    //one it scales its input, which is what a gearbox ratio does.
    private _nominal = _comp get "nominal";
    private _out     = 0;
    if (!_damaged && _gateOn && _hasIn) then {
        _out = if (_nominal > 0) then {_nominal} else {_inVal * (_comp get "ratio")};
    };

    private _step = _comp get "increment";
    if (_step > 0) then { _out = round (_out / _step) * _step };

    private _moved = _out != (_heli getVariable [_varName, 0]);

    if (_comp get "networked") then {
        [_heli, _varName, _out] call bmkhs_fnc_utilUpdateNetworkGlobal;
    } else {
        _heli setVariable [_varName, _out];
    };

    private _stateVar = _comp get "stateVar";
    if (_stateVar != "") then {
        [_heli, format ["bmkhs_%1", _stateVar], _out >= (_comp get "stateAbove")]
            call bmkhs_fnc_utilUpdateNetworkGlobal;
    };

    //Everything it feeds, with the same clutch and ratio rules a source has.
    {
        private _circuit = _x get "circuit";
        if (_circuit == "") then { continue };

        private _clutch = _x get "disengageOn";
        if (_clutch != "" && {([_heli, _clutch] call bmkhs_fnc_systemCircuit) >= (_x get "disengageAt")}) then {
            continue;
        };

        private _fixed = _x get "nominal";
        private _val   = if (_out <= 0) then {0} else {
            if (_fixed > 0) then {_fixed} else {_out * (_x get "ratio")}
        };

        _heli setVariable [_varName + "Feed_" + _circuit, _val];
        //Whether the NODE moved, not just this feeder.
        if (([_heli, _circuit, _varName, _val, true] call bmkhs_fnc_systemCircuitFeed)) then {
            _moved = true;
        };
    } forEach (_comp get "outputs");

//A converter creates nothing and holds no state, so it is never mid-transition - it has
//either moved its node or it has not.
_moved
