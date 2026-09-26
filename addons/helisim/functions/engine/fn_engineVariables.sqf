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

//Fields read straight off the engine's class. The key a reader asks for IS the config
//property name, so grepping a knob finds the config, this list and every use of it.
private _numFields = [
     "designRpm"
   , "npFly"
   , "maxFuelFlow"
   , "spoolInertia"
   , "compressorLoad"
   , "massFlowExp"
   , "tgtK"
   , "unfiredDragMult"
   , "unfiredFriction"
   , "thermalMass"
   , "cooling"
   , "soak"
   , "idleTq"
   , "flyTq"
   , "fuelIdle"
   , "fuelFly"
   , "ffwdGain"
   , "ptEfficiency"
   , "lightOffNg"
   , "selfSustNg"
   , "startTgt"
   , "startMinTgt"
   , "hotStartCarry"
   , "startFuelBase"
];
private _arrFields  = ["pid"];
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
    { _engine set [_x, getArray  (_e >> _x)]; } forEach _arrFields;
    { _engine set [_x, getText   (_e >> _x)]; } forEach _textFields;

    _engine set ["damageRoleIndex", getNumber (_e >> "damageRoleIndex")];

    private _name = _engine get "name";
    if (_name isEqualTo "") then { _name = format ["eng %1", _i]; _engine set ["name", _name]; };

    //Starter - air or electric, and what it needs available before the spool will turn.
    private _s = _e >> "Starter";
    _engine set ["starterType",   toLower getText (_s >> "type")];
    _engine set ["starterTorque", getNumber (_s >> "torque")];
    _engine set ["starterGates",  (getArray (_s >> "gate")) apply {_x}];

    //Author-named rating tiers, in declaration order - the first is the reference.
    private _ratings = ("true" configClasses (_e >> "PowerRatings")) apply {
        createHashMapFromArray [
             ["name",          configName _x]
           , ["displayName",   getText   (_x >> "displayName")]
           , ["powerKw",       getNumber (_x >> "powerKw")]
           , ["maxTgt",        getNumber (_x >> "maxTgt")]
           , ["maxNg",         getNumber (_x >> "maxNg")]
           , ["maxOilPsi",     getNumber (_x >> "maxOilPsi")]
           , ["timeLimit",     getNumber (_x >> "timeLimit")]
           , ["unlockBelowTq", getNumber (_x >> "unlockBelowTq")]
        ]
    };
    _engine set ["ratings", _ratings];

    //refTq is derived, never declared: Q = P / w from the base tier's power at governed Np.
    _engine set ["refTq", (((_ratings # 0) get "powerKw") * 1000)
                        / ((_engine get "designRpm") * (_engine get "npFly") * 0.10472)];

    //idleNg is recovered from the idle fuel floor - the start schedule needs it as a divisor.
    _engine set ["idleNg", (((_engine get "fuelIdle") - ((_engine get "idleTq") / (_engine get "ptEfficiency")))
                            / (_engine get "compressorLoad")) ^ 0.5];

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

//COMPAT 1.1.0: the old model reads these flat scalars. Deleted with it at Phase 3b. The
//ones with a new-schema counterpart come from engine 1's block; the rest are its own knobs.
if (_numEngines > 0) then {
    private _e1   = _engines # 0;
    private _rtgs = _e1 get "ratings";
    private _r1   = _rtgs # 0;
    private _rS   = _rtgs # ((count _rtgs) - 1);

    _heli setVariable ["bmkhs_engContPwrKW",    _r1 get "powerKw"];
    _heli setVariable ["bmkhs_engCntgncyPwrKW", _rS get "powerKw"];
    _heli setVariable ["bmkhs_engDesignRPM",    _e1 get "designRpm"];
    _heli setVariable ["bmkhs_engRunNG",        _e1 get "selfSustNg"];
    _heli setVariable ["bmkhs_engMaxTGT_DE",    _r1 get "maxTgt"];
    _heli setVariable ["bmkhs_engMaxTGT_SE",    _rS get "maxTgt"];
    _heli setVariable ["bmkhs_engFlyNP",        _e1 get "npFly"];
    _heli setVariable ["bmkhs_engIdleNG",       _e1 get "idleNg"];
};

_heli setVariable ["bmkhs_engFriction",     getNumber (_config >> "engFriction")];
_heli setVariable ["bmkhs_engGovGain",      getNumber (_config >> "engGovGain")];
_heli setVariable ["bmkhs_engIdleNP",       getNumber (_config >> "engIdleNP")];
_heli setVariable ["bmkhs_engOvrspdNP",     getNumber (_config >> "engOvrspdNP")];
_heli setVariable ["bmkhs_engFlyNG",        getNumber (_config >> "engFlyNG")];

//Governor PID - one per engine, from that engine's own gains.
_heli setVariable ["bmkhs_pid_engine", _engines apply {(_x get "pid") call bmkhs_fnc_pidCreate}];

//RUNTIME STATE - what the model carries frame to frame.
_heli setVariable ["bmkhs_shiftLocked",           false];
_heli setVariable ["bmkhs_isSingleEng",           false];

//Seeded here rather than in systemsVariables, which runs before the engine count is known.
_heli setVariable ["bmkhs_engineOverspeed",       _engines apply {false}, true];

//Outputs
_heli setVariable ["bmkhs_engFF",                 +_zeros];
_heli setVariable ["bmkhs_engPctNG",              +_zeros];
//SEEDS REQUIRED even though nothing READS these: bmkhs_fnc_utilSetArrayVariable does
//`+(_heli getVariable _name)` then `set`, so the array must already exist or it
//throws "Type Number, expected Array". Written per-engine by fn_engine.
_heli setVariable ["bmkhs_engBaseNG",             +_zeros];
_heli setVariable ["bmkhs_engBaseTGT",            +_zeros];
_heli setVariable ["bmkhs_engBaseOilPSI",         +_zeros];
_heli setVariable ["bmkhs_engTrimTq",             +_zeros];
_heli setVariable ["bmkhs_engPctNP",              +_zeros];
_heli setVariable ["bmkhs_engPctTQ",              +_zeros];
_heli setVariable ["bmkhs_engTGT",                +_zeros];
_heli setVariable ["bmkhs_engOilPSI",             +_zeros];

_heli setVariable ["bmkhs_engOutputTq",           +_zeros];

//New model - published beside the old one until Phase 3b drops the gt prefix.
_heli setVariable ["bmkhs_gtEngPctNg",            +_zeros];
_heli setVariable ["bmkhs_gtEngPctNp",            +_zeros];
_heli setVariable ["bmkhs_gtEngPctTq",            +_zeros];
//TGT is state, so it starts at ambient.
_heli setVariable ["bmkhs_gtEngTgt",              _engines apply {_heli getVariable "bmkhs_FAT"}];
_heli setVariable ["bmkhs_gtEngOilPsi",           +_zeros];
_heli setVariable ["bmkhs_gtEngFf",               +_zeros];
_heli setVariable ["bmkhs_gtEngOutputTq",         +_zeros];
_heli setVariable ["bmkhs_gtEngState",            _engines apply {"OFF"}];
//Latched from TGT on the OFF -> IDLE/FLY lever transition; 1.0 is a purged hot section.
_heli setVariable ["bmkhs_gtEngHotFac",           _engines apply {1.0}];
_heli setVariable ["bmkhs_engRatingIdx",          _engines apply {0}];
_heli setVariable ["bmkhs_engRatingName",         _engines apply {((_x get "ratings") # 0) get "displayName"}];
