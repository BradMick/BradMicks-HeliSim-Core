/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemsComponents

Description:
    Reads the aircraft's declared components out of config once and publishes
    them for the per-frame kinds to walk.

    The AIRCRAFT declares what it has; Core declares nothing. Member count
    comes from the damage role, so a role nothing claims means the airframe
    does not have that component. Circuits are collected from what components
    reference - a node exists because something feeds or reads it.

Parameters:
    _heli   - The helicopter [Object]
    _config - The aircraft's BMKHS_HeliSim config [Config]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];
#include "\bmkhs_helisim\functions\systems\systems.hpp"

//Field reference and the networking rules: \bmkhs_helisim\components.hpp
#define COMPONENT_FIELDS(cfg) createHashMapFromArray [ \
    ["damageRole",   getText   (cfg >> "damageRole")], \
    ["variableName", getText   (cfg >> "variableName")], \
    ["gates",        (getArray (cfg >> "gate")) apply {_x}], \
    ["output",       getText   (cfg >> "output")], \
    ["drivenBy",     (getArray (cfg >> "drivenBy")) param [0, ""]], \
    ["disengageOn",  (getArray (cfg >> "disengageAbove")) param [0, ""]], \
    ["disengageAt",  (getArray (cfg >> "disengageAbove")) param [1, 0]], \
    ["minDrive",     (getArray (cfg >> "drivenBy")) param [1, 0]], \
    ["input",        (getArray (cfg >> "input")) param [0, ""]], \
    ["minInput",     (getArray (cfg >> "input")) param [1, 0]], \
    ["ratio",        [1, getNumber (cfg >> "ratio")] select (isNumber (cfg >> "ratio"))], \
    ["requires",     getText   (cfg >> "requires")], \
    ["requiresAbove",getNumber (cfg >> "requiresAbove")], \
    ["nominal",      getNumber (cfg >> "nominal")], \
    ["rampRate",     if ((getNumber (cfg >> "rampSeconds")) > 0) \
                        then {(getNumber (cfg >> "nominal")) / (getNumber (cfg >> "rampSeconds"))} \
                        else {0}], \
    ["increment",    getNumber (cfg >> "increment")], \
    ["networked",    getNumber (cfg >> "networked") > 0], \
    ["stateVar",     getText   (cfg >> "stateName")], \
    ["stateAbove",   getNumber (cfg >> "stateAbove")], \
    ["torqueFrom",   getText   (cfg >> "torqueFrom")], \
    ["tqLimits",     getArray  (cfg >> "tqLimits")], \
    ["breaksVar",    if (isArray (cfg >> "breaksOnFailure")) \
                        then {getArray (cfg >> "breaksOnFailure")} \
                        else {[getText (cfg >> "breaksOnFailure")] select {_x != ""}}], \
    ["tqLimitsSE",   getArray  (cfg >> "tqLimitsSE")], \
    ["torqueSum",    getNumber (cfg >> "torqueSum") > 0], \
    ["jitters",      getNumber (cfg >> "jittersTorque") > 0], \
    ["damages",      getArray  (cfg >> "damagesHitpoints")] \
]

private _circuits = createHashMap;

//What a component puts where. One entry per Outputs class, or the single output field
//for something that only feeds one circuit.
//  circuit         node it feeds
//  ratio           of its own value; 1 passes it straight through, as a shaft does
//  nominal         fixed value instead, for an output that does not scale with the source
//  disengageAbove  circuit and threshold above which THIS output drops out, for a clutch
private _readOutputs = {
    params ["_cfg"];
    private _outs = [];
    {
        _outs pushBack (createHashMapFromArray [
            ["circuit",     getText   (_x >> "circuit")],
            ["ratio",       [1, getNumber (_x >> "ratio")] select (isNumber (_x >> "ratio"))],
            ["nominal",     getNumber (_x >> "nominal")],
            ["disengageOn", (getArray (_x >> "disengageAbove")) param [0, ""]],
            ["disengageAt", (getArray (_x >> "disengageAbove")) param [1, 0]]
        ]);
        _circuits set [getText (_x >> "circuit"), 0];
    } forEach ("true" configClasses (_cfg >> "Outputs"));

    if (_outs isEqualTo [] && {(getText (_cfg >> "output")) != ""}) then {
        _outs pushBack (createHashMapFromArray [
            ["circuit",     getText (_cfg >> "output")],
            ["ratio",       1],
            ["nominal",     getNumber (_cfg >> "nominal")],
            ["disengageOn", ""],
            ["disengageAt", 0]
        ]);
        _circuits set [getText (_cfg >> "output"), 0];
    };
    _outs
};

//Producers - pumps, generators, the APU. Anything that puts a value onto a circuit
//given whatever drives it.
private _producers = [];
{
    private _c    = COMPONENT_FIELDS(_x);
    private _role = _c get "damageRole";

    //No role is not the same as a role nothing claims: it means present but not
    //separately damageable, so one member that never fails.
    private _count = if (_role == "") then {1} else {[_heli, _role] call bmkhs_fnc_damageCount};
    for "_i" from 0 to (_count - 1) do {
        private _m = +_c;
        _m set ["index",   _i];
        //Numbered only when there is more than one: gen1On and gen2On, but priHydPsi.
        _m set ["varName", format ["bmkhs_%1%2", _c get "variableName", [_i + 1, ""] select (_count <= 1)]];
        _m set ["outputs", [_x] call _readOutputs];
        _producers pushBack _m;
    };

    if ((_c get "output") != "") then { _circuits set [_c get "output", 0] };
} forEach ("true" configClasses (_config >> "Producers"));

//Converters - consume from one circuit and produce onto another. They create nothing.
private _converters = [];
{
    private _c    = COMPONENT_FIELDS(_x);
    private _role = _c get "damageRole";

    private _count = if (_role == "") then {1} else {[_heli, _role] call bmkhs_fnc_damageCount};
    for "_i" from 0 to (_count - 1) do {
        private _m = +_c;
        _m set ["index",   _i];
        _m set ["varName", format ["bmkhs_%1%2", _c get "variableName",
                                   [_i + 1, ""] select (_count <= 1)]];
        _m set ["outputs", [_x] call _readOutputs];
        _converters pushBack _m;
    };

    if ((_c get "input") != "") then { _circuits set [_c get "input", 0] };
} forEach ("true" configClasses (_config >> "Converters"));
_heli setVariable ["bmkhs_sysConverters", _converters];

//Storage - a producer holding a charge.
private _storage = [];
{
    private _c    = COMPONENT_FIELDS(_x);
    private _role = _c get "damageRole";

    _c set ["rechargedBy", (getArray (_x >> "rechargedBy")) param [0, ""]];
    _c set ["minRecharge", (getArray (_x >> "rechargedBy")) param [1, 0]];
    _c set ["stopBelow",   getNumber (_x >> "stopBelow")];
    _c set ["startedBy",   getText   (_x >> "startedBy")];
    _c set ["startAbove",  getNumber (_x >> "startAbove")];

    //Charge is a fraction, so a full-to-empty time converts straight to a rate.
    private _drainSecs = getNumber (_x >> "emerDischarge");
    private _leakSecs  = getNumber (_x >> "leakSeconds");
    _c set ["emerRate",  if (_drainSecs > 0) then {1 / _drainSecs} else {0}];
    _c set ["leakRate",  if (_leakSecs  > 0) then {1 / _leakSecs}  else {0}];
    private _rechargeSecs = getNumber (_x >> "startRecharge");
    _c set ["rechargeRate", if (_rechargeSecs > 0) then {1 / _rechargeSecs} else {1 / SYS_START_RECHARGE_SEC}];
    _c set ["leakStartDmg", getNumber (_x >> "leakStartDmg")];
    _c set ["drainedBy",    getArray  (_x >> "drainedBy")];

    //As above: no role means present but not separately damageable, not absent.
    private _count = if (_role == "") then {1} else {[_heli, _role] call bmkhs_fnc_damageCount};
    for "_i" from 0 to (_count - 1) do {
        private _m = +_c;
        _m set ["index",   _i];
        _m set ["varName", format ["bmkhs_%1%2", _c get "variableName", [_i + 1, ""] select (_count <= 1)]];
        _m set ["outputs", [_x] call _readOutputs];
        _storage pushBack _m;
    };

    if ((_c get "output") != "") then { _circuits set [_c get "output", 0] };
} forEach ("true" configClasses (_config >> "Storage"));

//Circuits that publish their own state - a bus being up is a fact about the node, not
//something drawing from it.
private _named = [];
{
    private _c = createHashMapFromArray [
        ["circuit",      getText   (_x >> "circuit")],
        ["minValue",     getNumber (_x >> "minValue")],
        ["networked",    getNumber (_x >> "networked") > 0]
    ];
    _c set ["varName", format ["bmkhs_%1", getText (_x >> "variableName")]];
    _named pushBack _c;
    _circuits set [_c get "circuit", 0];
} forEach ("true" configClasses (_config >> "Circuits"));
_heli setVariable ["bmkhs_sysNamed", _named];

//Consumers - suppliedBy is an OR by default, or an AND with needsAll.
private _consumers = [];
{
    private _c = createHashMapFromArray [
        ["variableName", getText  (_x >> "variableName")],
        ["needsAll",     getNumber (_x >> "needsAll") > 0],
        ["networked",    getNumber (_x >> "networked") > 0]
    ];
    _c set ["circuits", (getArray (_x >> "suppliedBy")) apply {[_x select 0, _x param [1, 0]]}];
    _c set ["varName", format ["bmkhs_%1", _c get "variableName"]];
    _consumers pushBack _c;

    { _circuits set [_x select 0, 0] } forEach (_c get "circuits");
} forEach ("true" configClasses (_config >> "Consumers"));

//Anything with torque limits, gathered from every kind - a gearbox is a converter and
//the transmission is a producer, but both are rated for a torque.
//Either set counts - a component rated only for the single-engine case declares just
//tqLimitsSE, which is a nose gearbox: it carries enough to hurt it only when one engine
//is doing the work of two.
private _torqued = (_producers + _converters + _storage)
                        select {(count (_x get "tqLimits")) > 0 || {(count (_x get "tqLimitsSE")) > 0}};

//An airframe that models no systems still has a drivetrain, and it does not get to ignore
//what that is rated for. The top-level limits are for THAT CASE ONLY - with systems on, a
//component carries its own ratings and these are not read at all.
if !(_heli getVariable ["bmkhs_useSystems", false]) then {
    _torqued = [];
    {
        _x params ["_role", "_torqueVar", "_sums", "_limits", "_limitsSE", "_breaks"];
        private _count = [_heli, _role] call bmkhs_fnc_damageCount;
        for "_i" from 0 to ((_count max 1) - 1) do {
            _torqued pushBack (createHashMapFromArray [
                ["damageRole", _role],
                ["index",      _i],
                ["varName",    format ["bmkhs_%1%2", _role, _i]],
                ["jitters",    false],
                ["torqueFrom", _torqueVar],
                ["torqueSum",  _sums],
                ["tqLimits",   _limits],
                ["tqLimitsSE", _limitsSE],
                ["breaksVar",  _breaks],
                //With no systems modelled the damage lands on the rotors themselves -
                //Arma's own hitpoints, which every helicopter has - rather than on
                //whatever drivetrain parts the airframe happens to declare.
                ["damages",    ["hithrotor", "hitvrotor"]]
            ]);
        };
    } forEach [
        //The transmission carries both engines summed, and has no single-engine case -
        //one engine can never overtorque what is rated for two.
        ["transmission",  "bmkhs_engPctTQ", true,  getArray (_config >> "xmsnTqLimits"),
                          [], []],
        //A nose gearbox carries its own engine, which is only enough to hurt it when that
        //engine is doing the work of two - so it is rated single-engine and no other way.
        //Nothing breaks anything else here: there are no systems to fail.
        ["noseGearboxes", "bmkhs_engPctTQ", false, [],
                          getArray (_config >> "ngbTqLimitsSE"), []]
    ];
    //Only the ones the aircraft actually gave limits for.
    _torqued = _torqued select {(count (_x get "tqLimits")) > 0 || {(count (_x get "tqLimitsSE")) > 0}};
};
_heli setVariable ["bmkhs_sysTorqued", _torqued];

_heli setVariable ["bmkhs_sysProducers", _producers];
_heli setVariable ["bmkhs_sysStorage",   _storage];
_heli setVariable ["bmkhs_sysConsumers", _consumers];
_heli setVariable ["bmkhs_sysCircuits",  _circuits];

//The dependency graph, built once. A component is woken by whatever it READS, so the
//edges come from the fields it already declares rather than a separate dependsOn that
//could drift out of step with what the code actually looks at.
//
//  bmkhs_sysReaders   circuit  -> [[kind, listIndex], ...] that read it
//  bmkhs_sysWatchers  variable -> [[kind, listIndex], ...] gated on it
//
//Kinds are indices into the solve's own tables, so a woken component is dispatched
//without searching for it.
private _readers  = createHashMap;
private _watchers = createHashMap;

//A field a kind does not have reads back nil, not "" - storage has no input, a producer
//has no rechargedBy - so anything that is not a real name is dropped here rather than
//becoming a nil key the walk would choke on.
private _addEdge = {
    params ["_map", "_key", "_ref"];
    if (isNil "_key" || {!(_key isEqualType "")} || {_key == ""}) exitWith {};
    private _list = _map getOrDefault [_key, []];
    if !(_ref in _list) then {
        _list pushBack _ref;
        _map set [_key, _list];
    };
};

//Gates are read by every kind, and are either a variable or a {circuit, threshold}.
private _addGates = {
    params ["_comp", "_ref"];
    {
        if (_x isEqualType []) then {
            [_readers, _x select 0, _ref] call _addEdge;
        } else {
            [_watchers, _x, _ref] call _addEdge;
        };
    } forEach (_comp get "gates");
};

{
    _x params ["_list", "_kind"];
    {
        //_c, not _x: the outputs forEach below rebinds it.
        private _c   = _x;
        private _ref = [_kind, _forEachIndex];
        [_c, _ref] call _addGates;
        //What turns it, what it draws from, and the consumable it needs.
        [_readers,  _c get "drivenBy",    _ref] call _addEdge;
        [_readers,  _c get "input",       _ref] call _addEdge;
        [_watchers, _c get "requires",    _ref] call _addEdge;
        [_readers,  _c get "rechargedBy", _ref] call _addEdge;
        [_watchers, _c get "startedBy",   _ref] call _addEdge;
        //A clutch drops an output out, so the circuit it watches wakes the component.
        { [_readers, _x get "disengageOn", _ref] call _addEdge } forEach (_c get "outputs");
    } forEach _list;
} forEach [
    [_producers,  "producer"],
    [_converters, "converter"],
    [_storage,    "storage"]
];

//A control is not a component and feeds no circuit, but its interlocks are read exactly
//like a gate - so it is woken the same way. Registered here because the helpers are locals
//of this scope and the maps above are created fresh.
{
    private _c   = _x;
    private _ref = ["control", _forEachIndex];
    {
        if (_x isEqualType []) then {
            [_readers,  _x select 0, _ref] call _addEdge;
        } else {
            [_watchers, _x,          _ref] call _addEdge;
        };
    } forEach ((_c get "enabledBy") + (_c get "inhibitedBy"));
} forEach (_heli getVariable ["bmkhs_ctrlList", []]);

//Circuit states and consumers feed nothing, so they are not in the walk - the solve
//publishes them from the settled graph instead, which also stops a node that fell quiet
//keeping its last published state.

//What each component FEEDS, so waking it can mark its own outputs dirty in turn.
private _feedsOf = createHashMap;
{
    _x params ["_list", "_kind"];
    {
        //_c and _i taken before the inner forEach rebinds _x and _forEachIndex.
        private _c    = _x;
        private _i    = _forEachIndex;
        private _outs = [];
        { if ((_x get "circuit") != "") then { _outs pushBackUnique (_x get "circuit") } }
            forEach (_c get "outputs");
        _feedsOf set [_kind + str _i, _outs];
    } forEach _list;
} forEach [
    [_producers,  "producer"],
    [_converters, "converter"],
    [_storage,    "storage"]
];

_heli setVariable ["bmkhs_sysReaders",  _readers];
_heli setVariable ["bmkhs_sysWatchers", _watchers];
_heli setVariable ["bmkhs_sysFeeds_of", _feedsOf];
