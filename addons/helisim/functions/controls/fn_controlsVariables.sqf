/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_controlsVariables

Description:
    Reads the aircraft's declared controls out of config once and publishes them
    for the per-frame pass to move.

    The AIRCRAFT declares what it has; Core declares nothing. A control is N
    POSITIONS and the index is canonical - Core tracks an index and publishes
    the position's value, and whatever gates on that value reacts.

    Seeds every control at its rest position and publishes it, so a component
    gate naming a control variable finds it already there when the graph is
    built.

    Registers NO dependency edges. A control's interlocks are read exactly like
    a component's gates, but the maps they live in are created fresh by
    fn_systemsComponents, which runs after this - so the edges are registered
    there, where the helpers are and where they will not be clobbered.

Parameters:
    _heli   - The helicopter [Object]
    _config - The aircraft's BMKHS_HeliSim config [Config]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

//Field reference and the interlock rules: \bmkhs_helisim\controls.hpp
#define CONTROL_FIELDS(cfg) createHashMapFromArray [ \
    ["variableName", getText   (cfg >> "variableName")], \
    ["rest",         getNumber (cfg >> "rest")], \
    ["enabledBy",    (getArray (cfg >> "enabledBy"))   apply {_x}], \
    ["inhibitedBy",  (getArray (cfg >> "inhibitedBy")) apply {_x}], \
    ["wraps",        getNumber (cfg >> "wraps") > 0], \
    ["networked",    getNumber (cfg >> "networked") > 0], \
    ["axis",         (getArray (cfg >> "axis")) param [0, ""]], \
    ["hysteresis",   [0.05, getNumber (cfg >> "hysteresis")] \
                        select (isNumber (cfg >> "hysteresis"))], \
    ["steps",        [1, getNumber (cfg >> "steps")] select (isNumber (cfg >> "steps"))] \
]

//What a position IS. Order is declaration order - index 0 is the first class written, and
//the whole design rests on that, so reordering the classes renumbers every position.
//  displayName  what the player reads in the bindings menu; Core never interprets it
//  value        what this position publishes as Val
//  springsBack  releasing returns the control to rest
private _readPositions = {
    params ["_cfg"];
    private _ps = [];
    {
        //_p, not _x: apply below rebinds it.
        private _p = _x;
        _ps pushBack (createHashMapFromArray [
            ["name",        configName _p],
            ["displayName", getText   (_p >> "displayName")],
            ["value",       getNumber (_p >> "value")],
            ["springsBack", getNumber (_p >> "springsBack") > 0],
            //Interlocks on THIS position, added to the control's own - a rotor brake blocks
            //a power lever reaching fly without blocking idle.
            ["enabledBy",   (getArray (_p >> "enabledBy"))   apply {_x}],
            ["inhibitedBy", (getArray (_p >> "inhibitedBy")) apply {_x}]
        ]);
    } forEach ("true" configClasses (_cfg >> "Positions"));
    _ps
};

private _controls = [];
private _byName   = createHashMap;

{
    //_cCfg, not _x: _readPositions loops below and everything after reads this control.
    private _cCfg = _x;
    private _c    = CONTROL_FIELDS(_cCfg);
    private _poss = [_cCfg] call _readPositions;

    //A control with no positions has nothing to move to, so it is a declaration error
    //rather than something to publish a nil value from.
    if (_poss isEqualTo []) then { continue };

    _c set ["positions", _poss];
    _c set ["varName",   format ["bmkhs_%1", _c get "variableName"]];

    //Rest has to name a real position - it is where the control spawns and what a
    //spring-back returns to, so a bad index would publish nil forever.
    private _rest = ((_c get "rest") max 0) min ((count _poss) - 1);
    _c set ["rest", _rest];

    _byName set [_c get "variableName", count _controls];
    _controls pushBack _c;
} forEach ("true" configClasses (_config >> "Controls"));

_heli setVariable ["bmkhs_ctrlList",  _controls];
_heli setVariable ["bmkhs_ctrlIndex", _byName];

//Seed at rest and publish, so the gates resolve before the component graph is built.
{
    private _ctl  = _x;
    private _v    = _ctl get "varName";
    private _rest = _ctl get "rest";
    private _val  = ((_ctl get "positions") select _rest) get "value";

    _heli setVariable [_v + "Held",    -1];
    _heli setVariable [_v + "Awake",   false];
    _heli setVariable [_v + "GateWhy", ""];

    //Plainly first: utilUpdateNetworkGlobal reads the variable with no default, so it
    //throws on one that has never been set. Everything else it publishes was seeded by
    //systemsVariables; these are ours to seed.
    _heli setVariable [_v + "Idx", _rest];
    _heli setVariable [_v + "Val", _val];
    _heli setVariable [_v + "On",  _val != 0];

    //No notify: seeding is not a movement, and nothing has subscribed yet.
    [_heli, _ctl, _rest, _rest, false] call bmkhs_fnc_controlPublish;
} forEach _controls;
