/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_controlsUpdate

Description:
    The per-frame control pass, run BEFORE the walk.

    Two jobs. RECONCILE against anything outside HeliSim that wrote a control's
    variable, so the external writer stays authoritative without Core knowing
    what it is. Then SAMPLE any bound axis and quantise it to the declared
    detents.

    Runs before the watcher sweep because a control feeds a variable and that
    sweep is what turns a changed variable into a wake; running after would
    delay every switch by a frame.

Parameters:
    _heli      - The helicopter [Object]
    _deltaTime - Frame time [Number]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_deltaTime"];

{
    //_ctl, not _x: the position forEach below rebinds it.
    private _ctl     = _x;
    private _index   = _forEachIndex;
    private _varName = _ctl get "varName";
    private _poss    = _ctl get "positions";
    private _idx     = _heli getVariable [_varName + "Idx", _ctl get "rest"];
    private _mine    = (_poss select _idx) get "value";

    //Core is NOT the sole writer. Something outside HeliSim may force a switch - a fire
    //handle pulling the APU button off, every frame while it is armed. So the control does
    //not own its variable, it reconciles with it: if the published state disagrees with
    //the index, something else wrote it, so adopt that and move the index to match.
    //
    //The external writer touches On - a BOOLEAN - so On is what is compared, and that
    //makes it lossy. Prefer rest where rest already satisfies it, so a three-position
    //switch forced off lands at centre rather than at whichever end is declared first.
    //Discrete only: a continuous control's Val is the axis, not the detent's value, so this
    //would find a disagreement every frame and republish over the axis.
    private _extOn = _heli getVariable [_varName + "On", _mine != 0];
    if ((_ctl get "steps") > 0 && {_extOn isNotEqualTo (_mine != 0)}) then {
        private _rest = _ctl get "rest";
        private _want = -1;

        if ((((_poss select _rest) get "value") != 0) isEqualTo _extOn) then {
            _want = _rest;
        } else {
            {
                if (((_x get "value") != 0) isEqualTo _extOn) exitWith { _want = _forEachIndex };
            } forEach _poss;
        };

        if (_want >= 0) then {
            //A forced move is not a throw: it bypasses the interlocks, because a fire
            //handle does not ask the rotor brake for permission, and it does not spring.
            _heli setVariable [_varName + "Held",  -1];
            _heli setVariable [_varName + "Awake", false];
            [_heli, _ctl, _want, _idx] call bmkhs_fnc_controlPublish;
            _idx = _want;
        } else {
            //NO DECLARED POSITION MATCHES, so there is nothing sane to adopt. Leave the
            //index alone and let the external value stand - On is what gates read, so the
            //external actor gets what it asked for, and the next legitimate throw
            //republishes all three and resyncs. Forcing the index somewhere would invent a
            //position the designer never declared, so say so instead of guessing.
            _heli setVariable [_varName + "GateWhy", "extern"];
        };
    };

    //Axes are handled on the input path, not here - see fn_controlsAxisUpdate.
} forEach (_heli getVariable ["bmkhs_ctrlList", []]);
