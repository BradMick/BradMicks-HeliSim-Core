/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_control

Description:
    Moves one control to one position, if its interlocks let it.

    A switch is nothing more than a gate: this moves an index and publishes the
    position's value, and whatever gates on that value reacts. Core learns no
    switch semantics.

    A position that springs back is HELD for one full solve before it is
    released, because the walk's watcher sweep samples values rather than
    seeing transitions - a press and release between two frames would otherwise
    read the same value twice and wake nothing.

Parameters:
    _heli   - The helicopter [Object]
    _index  - Which control, as an index into bmkhs_ctrlList [Number]
    _target - Position to move to, or -1 to re-evaluate where it already is
              [Number]

Returns:
    Whether it moved [Boolean]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_index", "_target"];

private _list = _heli getVariable ["bmkhs_ctrlList", []];
if (_index < 0 || {_index >= (count _list)}) exitWith {false};
private _ctl = _list select _index;

private _varName = _ctl get "varName";
private _poss    = _ctl get "positions";
private _n       = count _poss;
if (_n == 0) exitWith {false};

private _cur = _heli getVariable [_varName + "Idx", _ctl get "rest"];

//-1 is what a woken interlock asks for: re-check where we already are rather than move.
private _want = [_target, _cur] select (_target < 0);

//A rotary steps round from the last position to the first; everything else clamps.
//The extra _n is because SQF's mod keeps the sign, so stepping back off 0 gives -1.
_want = if (_ctl get "wraps") then {((_want mod _n) + _n) mod _n} else {(_want max 0) min (_n - 1)};

//Interlocks are the gate form, so this is the gate loop: a variable name, or
//{circuit, threshold} read live. _ctl because the forEach rebinds _x.
private _free    = true;
private _gateWhy = "";
{
    private _ok = if (_x isEqualType []) then {
        ([_heli, _x select 0] call bmkhs_fnc_systemCircuit) >= (_x select 1)
    } else {
        _heli getVariable [_x, false]
    };
    if (!_ok) exitWith {
        _free = false;
        _gateWhy = if (_x isEqualType []) then {_x select 0} else {_x select [6]};
    };
} forEach ((_ctl get "enabledBy") + ((_poss select _want) get "enabledBy"));

//The same loop with the test inverted - any one of these blocks it.
if (_free) then {
    {
        private _bad = if (_x isEqualType []) then {
            ([_heli, _x select 0] call bmkhs_fnc_systemCircuit) >= (_x select 1)
        } else {
            _heli getVariable [_x, false]
        };
        if (_bad) exitWith {
            _free = false;
            _gateWhy = if (_x isEqualType []) then {_x select 0} else {_x select [6]};
        };
    } forEach ((_ctl get "inhibitedBy") + ((_poss select _want) get "inhibitedBy"));
};
_heli setVariable [_varName + "GateWhy", _gateWhy];

//An inhibited control does not move, and does not spring - a rotor brake set while the
//lever is at FLY leaves it at FLY. The interlock stops the throw, not the state.
if (!_free) exitWith {false};

private _moved = _want != _cur;
if (_moved) then {
    [_heli, _ctl, _want, _cur] call bmkhs_fnc_controlPublish;
};

//HOLD. fn_systemsSolve's watcher sweep compares each variable against its value at the
//PREVIOUS sweep, so it sees differences between samples rather than transitions. Keybinds
//fire asynchronously from the frame, so a tapped switch moving rest -> on -> rest between
//two sweeps is invisible and wakes nothing. Spring-back makes that the NORMAL case, so the
//position is held here and released by fn_controlsRelease AFTER the walk has read it.
//Re-arming on an unmoved repeat is what keeps a HELD key thrown.
private _springs = (_poss select _want) get "springsBack";
_heli setVariable [_varName + "Held", [-1, _want] select _springs];
//Mid-transition asks for the next frame, exactly as a spooling producer does.
_heli setVariable [_varName + "Awake", _springs];

_moved
