/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engineVariables

Description:
    Loads the per-engine configuration and initialises engine state. Core loops
    over numEngines, so an aircraft declares as many engines as it has.

    Each engine becomes one hashmap keyed by the config's own property names, so
    adding a knob to the config means naming it in the field list below and
    nothing else.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];
#include "\bmkhs_helisim\functions\systems\systems.hpp"

//Fields grouped by the assembly that owns them, matching the config's own nesting. The key a
//reader asks for IS the config property name, so grepping a knob finds config, list and use.
//Every field lands in ONE flat hashmap - the grouping is the config's, not the reader's.
private _numFields = [
     "designRpm"
   , "npFly"
   , "maxFuelFlow"
   , "powerKw"
     //Hard shutdowns - fly weights and the electrical trip.
   , "maxNg"
   , "maxNp"
];
private _sectionFields = [
     ["ColdSection",  ["compressorInertia", "compressorLoad", "airCoef", "compRunMult", "compRunExp"
                     , "compDragMult", "compDragFloor", "lightOffNg", "selfSustNg", "idleNg"
                     , "ngLimitMax", "ngLimitBase", "ngLimitSlope"]]
   , ["HotSection",   ["massFlowExp", "tgtK", "thermalMassCoef", "coolingCoef", "stillAirFlow"
                     , "ramAirCoef", "maxTgt", "maxTgtSe", "startTgt", "startMinTgt", "residualHeatGain"]]
   , ["PowerTurbine", ["ptEfficiency", "ptInertia", "ptDrag", "ptDragFloor"]]
   , ["Governor",     ["fuelIdle", "fuelFly", "startFuelBase", "ffwdGain", "leverTravelTime", "loadShareGain"]]
];
private _textFields = ["name", "engineType", "damageRole"];

//Hitpoints declare the count when the airframe has them; numEngines is what an aircraft
//declaring none is counted by.
private _numEngines = [_heli, "engines"] call bmkhs_fnc_damageCount;
if (_numEngines == 0) then { _numEngines = getNumber (_config >> "numEngines") };

private _engines = [];

for "_i" from 1 to _numEngines do {
    private _e = (_config >> "Engines") >> format ["Engine%1%2", ["0", ""] select (_i > 9), _i];

    //Hashmap, not a positional array: adding or removing a field cannot silently shift
    //what every reader sees.
    private _engine = createHashMap;

    { _engine set [_x, getNumber (_e >> _x)]; } forEach _numFields;
    { _engine set [_x, getText   (_e >> _x)]; } forEach _textFields;

    {
        _x params ["_section", "_fields"];
        private _sect = _e >> _section;
        { _engine set [_x, getNumber (_sect >> _x)]; } forEach _fields;
    } forEach _sectionFields;

    _engine set ["pid", getArray (_e >> "Governor" >> "pid")];

    _engine set ["damageRoleIndex", getNumber (_e >> "damageRoleIndex")];

    private _name = _engine get "name";
    if (_name isEqualTo "") then { _name = format ["eng %1", _i]; _engine set ["name", _name]; };

    //Starter - air or electric, and what it needs available before the spool will turn.
    private _s = _e >> "Starter";
    _engine set ["starterType",   toLower getText (_s >> "type")];
    _engine set ["starterTorque", getNumber (_s >> "torque")];
    _engine set ["starterGates",  (getArray (_s >> "gate")) apply {_x}];

    //What the ECU needs to keep metering fuel. Declaring none means always powered.
    _engine set ["governorGates", (getArray (_e >> "Governor" >> "gate")) apply {_x}];

    _engine set ["ngMin", getNumber (_e >> "ngMin")];
    //Book limits. Oil {minimum, maximum}; the rest low to high {limit, seconds, divisor}.
    { _engine set [_x, getArray (_e >> _x)]; }
        forEach ["oilPsiLimits", "ngLimits", "npLimits", "tqLimits", "tgtLimits", "tqLimitsSe", "tgtLimitsSe"];

    //refTq is derived, never declared: Q = P / w from maximum continuous power at governed Np.
    _engine set ["refTq", ((_engine get "powerKw") * 1000)
                        / ((_engine get "designRpm") * (_engine get "npFly") * 0.10472)];

    _engines pushBack _engine;
};

_heli setVariable ["bmkhs_numEngines", _numEngines];
_heli setVariable ["bmkhs_engines",    _engines];

private _zeros = _engines apply {0.0};

if (!(_heli getVariable ["bmkhs_engineInitialised", false]) && local _heli) then {
    _heli setVariable ["bmkhs_engineInitialised", true, true];

    //Everything starts OFF, with or without modelled systems. Without them there is no
    //start PROCEDURE - no battery, APU or generators to sequence - but Arma's own startup
    //still runs, and driving it from OFF is what spins the rotor up over that window.
    //The graph then follows: Nr turns the transmission, which drives the accessories,
    //which build pressure. Seeding it already-running skipped all of that and left the
    //values reading as failures until the model caught up.
    //Cold and dark either way: the levers start OFF. With no systems the controller moves
    //them to FLY when the player wakes the aircraft, through the same path a click takes
    //so the animation travels rather than snapping.
    _heli setVariable ["bmkhs_engPowerLeverState", _engines apply {"OFF"}, true]; //OFF, IDLE, FLY
    _heli setVariable ["bmkhs_engState",           _engines apply {"OFF"}, true]; //OFF, STARTING, ON
};

if(isMultiplayer) then {
    _heli setVariable ["bmkhs_lastTimePropagated", 0];
};

//The shaft reference the transmission and Nr read.
if (_numEngines > 0) then {
    _heli setVariable ["bmkhs_engDesignRPM", (_engines # 0) get "designRpm"];
};

//Governor PID - one per engine, from that engine's own gains.
_heli setVariable ["bmkhs_pid_engine", _engines apply {(_x get "pid") call bmkhs_fnc_pidCreate}];

//RUNTIME STATE - what the model carries frame to frame.
_heli setVariable ["bmkhs_shiftLocked",           false];
_heli setVariable ["bmkhs_isSingleEng",           false];

//Latched by either hard trip, cleared by a repair. Seeded here rather than in
//systemsVariables, which runs before the engine count is known.
_heli setVariable ["bmkhs_engineOverspeed",       _engines apply {false}, true];

//Damage ladder latches - cleared by a repair.
_heli setVariable ["bmkhs_engChips",              _engines apply {false}, true];
_heli setVariable ["bmkhs_engFailed",             _engines apply {false}, true];
_heli setVariable ["bmkhs_lowOilPsiFailure",      _engines apply {false}, true];
_heli setVariable ["bmkhs_engOilHealth",          _engines apply {1.0}];
//Oil below its minimum, running with the lever out of OFF - latched, cleared by a repair.
_heli setVariable ["bmkhs_engOilPsiLow",          _engines apply {false}, true];
//Seconds left in the current band, {np, ng, tgt}: -1 none, 0 damage running.
_heli setVariable ["bmkhs_engLimitTimers",        _engines apply {[-1, -1, -1]}];
//The same for the drivetrain the engine's torque loads.
_heli setVariable ["bmkhs_engTqTimer",            _engines apply {-1}];
//useSystems = 0: the engine given the 0.25 fault, -1 until one is.
_heli setVariable ["bmkhs_engFailureResult",      -1];
//Clutch slip - the fraction of torque passed (1 is sound), and each engine's slip clock.
_heli setVariable ["bmkhs_engClutchSlip",         _engines apply {1.0}];
_heli setVariable ["bmkhs_engSlipT",              _engines apply {-1}];
//A random start, so the engines slip out of step.
_heli setVariable ["bmkhs_engSlipWait",           _engines apply {random SYS_SLIP_WAIT_LOW_DMG}];
_heli setVariable ["bmkhs_engSlipDepth",          _engines apply {0}];
//useSystems = 0: when Ng reached idle, -1 until it has.
_heli setVariable ["bmkhs_engIdleSince",          _engines apply {-1}];
//Fuel at the pump, and when its tank ran dry, -1 while it has fuel.
_heli setVariable ["bmkhs_engFuelAvail",          _engines apply {true}];
_heli setVariable ["bmkhs_engStarvedSince",       _engines apply {-1}];

//Outputs
_heli setVariable ["bmkhs_engFuelFlow",                 +_zeros];
_heli setVariable ["bmkhs_engPctNg",              +_zeros];
//Np is state with its own torque balance, not read off the rotor.
_heli setVariable ["bmkhs_engNp",                 +_zeros];
_heli setVariable ["bmkhs_engPctNp",              +_zeros];
//The freewheel - true while the turbine is driving the rotor.
_heli setVariable ["bmkhs_engClutch",             _engines apply {false}];
_heli setVariable ["bmkhs_engPctTq",              +_zeros];
//TGT is state, so it starts at ambient.
_heli setVariable ["bmkhs_engTgt",                _engines apply {_heli getVariable "bmkhs_FAT"}];
_heli setVariable ["bmkhs_engOilPsi",             +_zeros];
_heli setVariable ["bmkhs_engOutputTq",           +_zeros];
//Latched on the OFF -> IDLE/FLY transition; 1.0 is a purged hot section.
_heli setVariable ["bmkhs_engResidualHeat",       _engines apply {1.0}];
_heli setVariable ["bmkhs_engPrevLever",          _engines apply {"OFF"}];
//The lever's tracked position, as a fuel schedule. Travels up, snaps down.
_heli setVariable ["bmkhs_engLeverSched",         +_zeros];
//Np when the governor took over at FLY; negative until it does.
_heli setVariable ["bmkhs_engNpRef",              _engines apply {-1.0}];
//Fuel the TGT and Ng limiters allow; wide open until one is near its limit.
_heli setVariable ["bmkhs_engLimFuel",            _engines apply {_x get "fuelFly"}];
