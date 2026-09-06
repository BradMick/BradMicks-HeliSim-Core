/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_controlSet

Description:
    Moves a control by name. This is the keybind target and the public API -
    every generated cockpit bind dispatches here.

    Resolve the name, refuse if an axis owns this control outright, translate a
    step into an index, and hand off to fn_control.

Parameters:
    _name - The control's variableName, without the bmkhs_ prefix [String]
    _pos  - Position index [Number], or "+1" / "-1" to step [String]
    _heli - The helicopter [Object, default the player's]

Returns:
    Whether it moved [Boolean]

Examples:
    ["battSwitch", 1] call bmkhs_fnc_controlSet;
    ["battSwitch", "+1"] call bmkhs_fnc_controlSet;
    ["battSwitch", 1, _heli] call bmkhs_fnc_controlSet;

Author:
    BradMick
---------------------------------------------------------------------------- */
//A keybind fires wherever the player is and has no aircraft to hand, so that is the
//default - but a caller holding one says so rather than being second-guessed.
params ["_name", "_pos", ["_heli", vehicle player]];

//Gate on HeliSim being initialised, not on an airframe class - Core is airframe-agnostic.
if !(_heli getVariable ["bmkhs_initialised", false]) exitWith {false};

private _index = (_heli getVariable ["bmkhs_ctrlIndex", createHashMap]) getOrDefault [_name, -1];
if (_index < 0) exitWith {false};

private _ctl = (_heli getVariable ["bmkhs_ctrlList", []]) select _index;

//A step is relative to where the control is now; an index is absolute. fn_control wraps or
//clamps it, so a step past the end is that control's business rather than handled twice.
//"+1" parses to 1 and "-1" to -1; anything unparseable is 0, which stays put.
private _target = if (_pos isEqualType "") then {
    (_heli getVariable [(_ctl get "varName") + "Idx", _ctl get "rest"]) + (parseNumber _pos)
} else {
    _pos
};

[_heli, _index, _target] call bmkhs_fnc_control
