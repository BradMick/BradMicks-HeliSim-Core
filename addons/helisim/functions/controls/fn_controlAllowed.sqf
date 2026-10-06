/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_controlAllowed

Description:
    Whether a control may move to a position - its enabledBy[] and inhibitedBy[],
    on the control and on that position. Solved where the aircraft is local;
    elsewhere, the owner's published bmkhs_<control>Allowed.

Parameters:
    _heli    - The helicopter [Object]
    _control - The control's variableName, or its index in bmkhs_ctrlList [String, Number]
    _pos     - Position index [Number]
    _record  - Write GateWhy [Boolean, default true]

Returns:
    Whether it may move there [Boolean]

Examples:
    [_heli, "eng1PwrLvr", 2] call bmkhs_fnc_controlAllowed   //FLY, with the rotor brake on?

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_control", "_pos", ["_record", true]];

private _index = if (_control isEqualType "") then {
    (_heli getVariable ["bmkhs_ctrlIndex", createHashMap]) getOrDefault [_control, -1]
} else {
    _control
};
private _list = _heli getVariable ["bmkhs_ctrlList", []];
if (_index < 0 || {_index >= (count _list)}) exitWith {false};
private _ctl  = _list select _index;
private _poss = _ctl get "positions";
if (_pos < 0 || {_pos >= (count _poss)}) exitWith {false};

private _published = _heli getVariable ((_ctl get "varName") + "Allowed");
if (!(local _heli) && {!isNil "_published"}) exitWith {_published param [_pos, false]};

private _gate = {
    if (_this isEqualType []) then {
        ([_heli, _this select 0] call bmkhs_fnc_systemCircuit) >= (_this select 1)
    } else {
        _heli getVariable [_this, false]
    };
};
private _name = { if (_this isEqualType []) then {_this select 0} else {_this select [6]} };

private _free = true;
private _why  = "";
{
    if !(_x call _gate) exitWith { _free = false; _why = _x call _name; };
} forEach ((_ctl get "enabledBy") + ((_poss select _pos) get "enabledBy"));
if (_free) then {
    {
        if (_x call _gate) exitWith { _free = false; _why = _x call _name; };
    } forEach ((_ctl get "inhibitedBy") + ((_poss select _pos) get "inhibitedBy"));
};
if (_record) then {_heli setVariable [(_ctl get "varName") + "GateWhy", _why]};

_free
