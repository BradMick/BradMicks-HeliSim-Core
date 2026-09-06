/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_controlsRelease

Description:
    Releases spring-back positions, AFTER the walk has read them.

    This runs at the very end of the solve rather than at the start of the next
    one, so the circuit states and consumers published this frame were computed
    with the switch still at its thrown position. Releasing any earlier would
    publish a frame of state where the switch had already sprung.

    Together with the hold set in fn_control this is what makes a momentary
    press durable enough for the watcher sweep to see: the level exists for
    exactly one complete walk.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

{
    //_ctl, not _x: everything below reads this control.
    private _ctl     = _x;
    private _varName = _ctl get "varName";
    private _held    = _heli getVariable [_varName + "Held", -1];
    if (_held < 0) then { continue };

    //The walk has now run with the switch at _held, so the level was durable enough to
    //read. Spring it and let the next frame's sweep see the return.
    private _rest = _ctl get "rest";
    private _cur  = _heli getVariable [_varName + "Idx", _rest];
    _heli setVariable [_varName + "Held",  -1];
    _heli setVariable [_varName + "Awake", false];

    if (_cur != _rest) then {
        [_heli, _ctl, _rest, _cur] call bmkhs_fnc_controlPublish;
    };
} forEach (_heli getVariable ["bmkhs_ctrlList", []]);
