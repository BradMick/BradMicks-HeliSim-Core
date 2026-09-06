/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_controlPublish

Description:
    The only writer of a control's three variables.

        bmkhs_<name>Idx   the position index - canonical
        bmkhs_<name>Val   the current position's declared value
        bmkhs_<name>On    Val != 0

    Publishes only what it is told to, and only when something moved. Core is
    NOT the sole writer of these - a fire handle forces the APU button off from
    outside HeliSim, every frame while it is armed - so a control that
    republished its own index each frame would fight it continuously.

    Raises controlMoved for animation and audio.

Parameters:
    _heli    - The helicopter [Object]
    _ctl     - The control [HashMap]
    _idx     - Position index to publish [Number]
    _prevIdx - Where it came from, for the event [Number]
    _moved   - The control actually moved. False when seeding at init, where the
               targets do not exist yet [Boolean, default true]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_ctl", "_idx", "_prevIdx", ["_moved", true]];

private _poss = _ctl get "positions";
if (_idx < 0 || {_idx >= (count _poss)}) exitWith {};

private _varName = _ctl get "varName";
private _pos     = _poss select _idx;
private _val     = _pos get "value";

//utilUpdateNetworkGlobal compares before writing, so an unchanged value costs nothing and
//sends nothing - which is what makes reconciling against an external writer free.
if (_ctl get "networked") then {
    [_heli, _varName + "Idx", _idx]      call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, _varName + "Val", _val]      call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, _varName + "On",  _val != 0] call bmkhs_fnc_utilUpdateNetworkGlobal;
} else {
    _heli setVariable [_varName + "Idx", _idx];
    _heli setVariable [_varName + "Val", _val];
    _heli setVariable [_varName + "On",  _val != 0];
};

//The position name is the designer's label, passed through for animation and audio.
if (_moved) then {
    [_heli, "controlMoved",
        [_ctl get "variableName", _idx, _prevIdx, _val, _pos get "name"]] call bmkhs_fnc_utilNotify;
};
