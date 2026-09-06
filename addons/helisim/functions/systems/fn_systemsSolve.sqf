/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemsSolve

Description:
    Walks the component graph from whatever changed, and nothing else.

    A component is woken by what it READS - a circuit whose value moved, or a
    variable it is gated on. Waking it recomputes it; if its own output moves,
    everything reading THAT is woken in turn, and so on until the queue empties.
    A component mid-transition - an APU spooling, a store draining - keeps
    itself awake by asking for the next frame.

    So a settled aircraft costs one check of the dirty set, not a pass over
    every component. A change costs its own chain and no more.

    Ordering falls out of the walk rather than needing a topological sort, which
    a cycle could not have anyway: the accumulator starts the APU, which drives
    the accessory section, which turns the pumps, which recharge the
    accumulator. What cuts that cycle is that CHARGE IS STATE, not supply - a
    store delivers what was put there earlier, so it is a root of the walk and
    its recharge edge settles afterwards from the walk's own result.

Parameters:
    _heli      - The helicopter [Object]
    _deltaTime - Frame time [Number]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_deltaTime"];
#include "\bmkhs_helisim\functions\systems\systems.hpp"

//Solved once, where the aircraft is local. Everyone else reads the networked results,
//which is what stops every client computing the same state and fighting over it.
if !(local _heli) exitWith {};

//Nothing is simulated without modelled systems - no hydraulics spooling, no buses coming
//up, no APU. The engines and the transmission still run because they are the flight model,
//and everything else stays at the static value it was seeded with. There is no damage
//model either, so nothing can degrade it.
if !(_heli getVariable ["bmkhs_useSystems", false]) exitWith {};

//Before the sweep: a control feeds a variable, and the sweep is what turns that into a wake.
[_heli, _deltaTime] call bmkhs_fnc_controlsUpdate;

private _producers  = _heli getVariable ["bmkhs_sysProducers",  []];
private _converters = _heli getVariable ["bmkhs_sysConverters", []];
private _storage    = _heli getVariable ["bmkhs_sysStorage",    []];

private _readers  = _heli getVariable ["bmkhs_sysReaders",  createHashMap];
private _watchers = _heli getVariable ["bmkhs_sysWatchers", createHashMap];
private _feedsOf  = _heli getVariable ["bmkhs_sysFeeds_of", createHashMap];

//Anything a component is gated on is a dependency like any other, so a switch being
//thrown or a hitpoint being lost enters the walk the same way a circuit moving does.
private _watched = _heli getVariable ["bmkhs_sysWatchedLast", createHashMap];
private _queue   = [];
private _seen    = createHashMap;

private _wake = {
    params ["_ref"];
    private _key = (_ref select 0) + str (_ref select 1);
    if (_seen getOrDefault [_key, false]) exitWith {};
    _seen set [_key, true];
    _queue pushBack _ref;
};

//A node moving wakes everything that reads it.
private _dirtyCircuit = {
    params ["_circuit"];
    { [_x] call _wake } forEach (_readers getOrDefault [_circuit, []]);
};

{
    //_v, not _x: the inner forEach rebinds it.
    private _v    = _x;
    private _now  = _heli getVariable [_v, false];
    if !(_now isEqualTo (_watched getOrDefault [_v, "unset"])) then {
        _watched set [_v, _now];
        { [_x] call _wake } forEach (_watchers getOrDefault [_v, []]);
    };
} forEach (keys _watchers);
_heli setVariable ["bmkhs_sysWatchedLast", _watched];

//Damage is a dependency too, and it has no variable to watch - so a component is woken
//when the damage it reads has moved since it last ran.
{
    _x params ["_list", "_kind"];
    {
        if ((_x get "damageRole") != "") then {
            private _d = [_heli, _x get "damageRole", _x get "index"] call bmkhs_fnc_damageGet;
            if (_d != (_heli getVariable [(_x get "varName") + "DmgLast", -1])) then {
                _heli setVariable [(_x get "varName") + "DmgLast", _d];
                [[_kind, _forEachIndex]] call _wake;
            };
        };
    } forEach _list;
} forEach [
    [_producers,  "producer"],
    [_converters, "converter"],
    [_storage,    "storage"]
];

//Nr comes from the flight model rather than a component, and it moves constantly, so it
//is fed in first and wakes the drivetrain when it actually changes.
private _nr = [_heli] call bmkhs_fnc_stateRtrRPM;
if (([_heli, "Nr", "rotor", _nr, true] call bmkhs_fnc_systemCircuitFeed)) then {
    ["Nr"] call _dirtyCircuit;
};

//Storage is a root: what it delivers was put there earlier, so it supplies before
//anything has been solved. That is what breaks the start cycle.
{
    //Taken before the inner forEach rebinds _forEachIndex.
    private _i = _forEachIndex;
    if (([_heli, _i, _deltaTime, false] call bmkhs_fnc_systemStorage)) then {
        { [_x] call _dirtyCircuit } forEach (_feedsOf getOrDefault ["storage" + str _i, []]);
    };
} forEach _storage;

//Anything mid-transition asked for the next frame last time it ran - a spooling APU, a
//store still draining. It keeps itself in the walk until it reaches its target.
{
    _x params ["_list", "_kind"];
    {
        if (_heli getVariable [(_x get "varName") + "Awake", false]) then {
            [[_kind, _forEachIndex]] call _wake;
        };
    } forEach _list;
} forEach [
    [_producers,  "producer"],
    [_converters, "converter"],
    [_storage,    "storage"]
];

//Drain the queue. Each component that MOVES dirties what it feeds, which appends to the
//queue - so the walk reaches exactly as far as the change does and then stops.
private _guard = 0;
while {(count _queue) > 0 && {_guard < SYS_WALK_LIMIT}} do {
    _guard = _guard + 1;
    private _ref  = _queue deleteAt 0;
    _ref params ["_kind", "_i"];
    _seen set [_kind + str _i, false];

    private _moved = switch (_kind) do {
        case "producer":  { [_heli, _i, _deltaTime] call bmkhs_fnc_systemProducer };
        case "converter": { [_heli, _i, _deltaTime] call bmkhs_fnc_systemConverter };
        case "storage":   { [_heli, _i, _deltaTime, false] call bmkhs_fnc_systemStorage };
        //A control feeds no circuit, so it dirties nothing - but an interlock that moved
        //reaches it through the same wake. -1 is re-check where it already is.
        case "control":   { [_heli, _i, -1] call bmkhs_fnc_control; false };
        default           { false };
    };

    if (_moved) then {
        { [_x] call _dirtyCircuit } forEach (_feedsOf getOrDefault [_kind + str _i, []]);
    };
};

//What the walk actually cost this frame. The honest measure of the dirty-flag design is
//not frame rate but this: near zero on a settled aircraft, spiking only when something moves.
_heli setVariable ["bmkhs_sysWalkCost", _guard];

//Charge moves last, off the solved result - a store reading its recharge circuit any
//earlier sees zero and never refills. This is the cycle's cut edge.
{ [_heli, _forEachIndex, _deltaTime, true] call bmkhs_fnc_systemStorage } forEach _storage;

//Publishing is cheap and reads the settled graph, so it is not worth waking selectively -
//and it must not be skipped, or a node that fell quiet keeps its last published state.
[_heli] call bmkhs_fnc_systemCircuitState;
[_heli] call bmkhs_fnc_systemConsumer;

//Last: a spring-back is released only once the walk has read it, or a tapped switch moves
//and returns between two sweeps and wakes nothing.
[_heli] call bmkhs_fnc_controlsRelease;
