/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_simpleRotorVariables

Description:
    Reads the simple rotor declarations and initializes the runtime state.

    An aircraft declares one class per rotor under Rotors, and DECLARATION ORDER
    IS THE ROTOR INDEX - the index bmkhs_rtrThrust[] and bmkhs_reqEngTorque[]
    are keyed on. Core loops whatever is declared, so a NOTAR, a coaxial or a
    tandem needs no code here.

    Every default is NEUTRAL, not borrowed: flat 1.0 tables, zero gains. An
    aircraft that declares nothing flies badly but honestly rather than quietly
    inheriting another airframe's handling.

    Field reference: \bmkhs_helisim\simpleRotor.hpp

Parameters:
    _heli   - The helicopter to configure [Object].
    _config - The BMKHS_HeliSim config class for this aircraft [Config].

Returns:
    Nothing.

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

#include "\bmkhs_helisim\functions\core\core.hpp"

//A missing number is the default, NOT zero - getNumber returns 0 for an absent
//entry, and 0 is a meaningful value for most of these fields.
#define NUM_OR(cfg,name,dflt) ([dflt, getNumber (cfg >> name)] select (isNumber (cfg >> name)))
#define TXT_OR(cfg,name,dflt) ([dflt, getText   (cfg >> name)] select (isText   (cfg >> name)))

//A table is {x,y} pairs. Absent or malformed declarations fall back to the flat
//curve passed in, so a rotor that declares no tables still runs the model.
#define TBL_OR(cfg,name,dflt) ([dflt, getArray (cfg >> name)] select (isArray (cfg >> name) && {count getArray (cfg >> name) > 0}))

private _rotorsCfg = _config >> "Rotors";

//Neutral fallbacks. Hoisted out of the macros below because a macro argument
//cannot contain a comma - [[0, 1.0]] would read as two arguments.
private _flatOne   = [[0, 1.0]];
private _linearRamp = [[0, 0.0], [1, 1.0]];

private _rotors = [];

{
    private _cfg  = _x;
    private _type = toUpper (TXT_OR(_cfg, "rotorType", "MAIN"));
    private _isMain = _type == "MAIN";

    //Thrust axis in model space. Z is up (main), X is lateral (tail).
    private _axisDflt = if (_isMain) then {"Z"} else {"X"};
    private _axisName = toUpper (TXT_OR(_cfg, "thrustAxis", _axisDflt));
    private _axis = switch (_axisName) do {
        case "X": {[1.0, 0.0, 0.0]};
        case "Y": {[0.0, 1.0, 0.0]};
        default  {[0.0, 0.0, 1.0]};
    };

    //CCW is the conventional main rotor: reaction torque yaws the nose right.
    private _dirSign = if (toUpper (TXT_OR(_cfg, "direction", "CCW")) == "CW") then {-1.0} else {1.0};

    private _rotor = createHashMapFromArray [
        //---- identity ----
        ["name",              configName _cfg],
        ["isMain",            _isMain],
        ["axis",              _axis],
        ["dirSign",           _dirSign],
        ["position",          getArray (_cfg >> "position")],

        //---- geometry ----
        ["bladeRadius",       NUM_OR(_cfg, "bladeRadius",      1.0)],
        ["rotorInertia",      NUM_OR(_cfg, "rotorInertia",     0.0)],
        ["bladePitchMin",     NUM_OR(_cfg, "bladePitchMin",    0.0)],
        ["bladePitchMax",     NUM_OR(_cfg, "bladePitchMax",    1.0)],
        ["designRpm",         NUM_OR(_cfg, "designRpm",        1.0)],
        ["gearRatio",         NUM_OR(_cfg, "gearRatio",        1.0)],
        ["rpmTrimVal",        NUM_OR(_cfg, "rpmTrimVal",       1.0)],
        ["heightAgl",         NUM_OR(_cfg, "heightAgl",        0.0)],

        //---- force scale ----
        ["baseThrust",        NUM_OR(_cfg, "baseThrust",       0.0)],
        ["maxPower",          NUM_OR(_cfg, "maxPower",         0.0)],
        ["refRpm",            NUM_OR(_cfg, "refRpm",           1.0)],

        //---- tuning ----
        ["groundEffectGain",  NUM_OR(_cfg, "groundEffectGain", 0.0)],
        ["climbGain",         NUM_OR(_cfg, "climbGain",        0.0)],
        ["autoroTorque",      NUM_OR(_cfg, "autoroTorque",     0.0)],
        ["torqueScalar",      NUM_OR(_cfg, "torqueScalar",     0.0)],

        //---- control authority ----
        ["cyclicPitchTorque", NUM_OR(_cfg, "cyclicPitchTorque", 0.0)],
        ["cyclicRollTorque",  NUM_OR(_cfg, "cyclicRollTorque",  0.0)],
        ["pedalYawTorque",    NUM_OR(_cfg, "pedalYawTorque",    0.0)],
        ["rollCouple",        NUM_OR(_cfg, "rollCouple",        0.0)],
        ["thrustTiltRoll",    NUM_OR(_cfg, "thrustTiltRoll",    0.0)],
        ["flapbackLon",       NUM_OR(_cfg, "flapbackLon",       0.0)],
        ["flapbackLat",       NUM_OR(_cfg, "flapbackLat",       0.0)],

        //---- envelope, knots in config -> m/s internally ----
        ["vne",               NUM_OR(_cfg, "vne", 250.0) * KNOTS_TO_MPS],
        ["vbe",               NUM_OR(_cfg, "vbe",  75.0) * KNOTS_TO_MPS],
        ["etl",               NUM_OR(_cfg, "etl",  24.0) * KNOTS_TO_MPS],

        //---- the four tables ----
        //x is in knots for the airspeed tables; converted to m/s here so the
        //model never converts in the hot path.
        ["thrustVsAirspeed",   (TBL_OR(_cfg, "thrustVsAirspeed",  _flatOne))    apply {[(_x select 0) * KNOTS_TO_MPS, _x select 1]}],
        ["powerVsAirspeed",    (TBL_OR(_cfg, "powerVsAirspeed",   _flatOne))    apply {[(_x select 0) * KNOTS_TO_MPS, _x select 1]}],
        ["thrustVsCollective", TBL_OR(_cfg, "thrustVsCollective", _linearRamp)],
        ["powerVsCollective",  TBL_OR(_cfg, "powerVsCollective",  _flatOne)]
    ];

    _rotors pushBack _rotor;
} forEach ("true" configClasses _rotorsCfg);

_heli setVariable ["bmkhs_simpleRotors", _rotors];
_heli setVariable ["bmkhs_simpleRotorCount", count _rotors];

//RUNTIME STATE - what the model carries frame to frame, one slot per rotor.
private _zeros = _rotors apply {0.0};

_heli setVariable ["bmkhs_reqEngTorque",   +_zeros];
_heli setVariable ["bmkhs_rtrThrust",      +_zeros];
_heli setVariable ["bmkhs_rtrMoi",         _rotors apply {_x get "rotorInertia"}];
_heli setVariable ["bmkhs_vrsVelocityMin", +_zeros];
_heli setVariable ["bmkhs_vrsVelocityMax", +_zeros];
_heli setVariable ["bmkhs_rtrRPM",         0.0];

//The transmission reads the main rotor's gear ratio by name.
private _mainIdx = _rotors findIf {_x get "isMain"};
if (_mainIdx < 0) then { _mainIdx = 0; };
if (count _rotors > 0) then {
    _heli setVariable ["bmkhs_mainRotorGearRatio", (_rotors select _mainIdx) get "gearRatio"];
};

/////////////////////////////////////////////////////////////////////////////////////////////
// TRANSITIONAL - the old flat surface
/////////////////////////////////////////////////////////////////////////////////////////////
//fn_simpleRotorMain and fn_simpleRotorTail are still the functions being CALLED, and they
//read the flat mainRtr*/tailRtr* variables. Keep publishing those from the old config
//entries until the call in fn_coreUpdateFlightModel switches over, so the aircraft flies
//exactly as it did while the new model is built beside it.
//
//DELETE THIS BLOCK when fn_simpleRotorMain.sqf and fn_simpleRotorTail.sqf go.
_heli setVariable ["bmkhs_mainRtrPos",          getArray  (_config >> "mainRtrPos")];
_heli setVariable ["bmkhs_mainRtrHeightAgl",    getNumber (_config >> "mainRtrHeightAgl")];
_heli setVariable ["bmkhs_mainRtrDesignRpm",    getNumber (_config >> "mainRtrDesignRpm")];
_heli setVariable ["bmkhs_mainRtrRpmTrimVal",   getNumber (_config >> "mainRtrRpmTrimVal")];
_heli setVariable ["bmkhs_mainRtrNumBlades",    getNumber (_config >> "mainRtrNumBlades")];
_heli setVariable ["bmkhs_mainRtrBladeRadius",  getNumber (_config >> "mainRtrBladeRadius")];
_heli setVariable ["bmkhs_mainRtrBladeChord",   getNumber (_config >> "mainRtrBladeChord")];
_heli setVariable ["bmkhs_mainRtrBladeMass",    getNumber (_config >> "mainRtrBladeMass")];
_heli setVariable ["bmkhs_mainRtrBladeHingeOff",getNumber (_config >> "mainRtrBladeHingeOff")];
_heli setVariable ["bmkhs_mainRtrBladePitchMin",getNumber (_config >> "mainRtrBladePitchMin")];
_heli setVariable ["bmkhs_mainRtrBladePitchMax",getNumber (_config >> "mainRtrBladePitchMax")];
_heli setVariable ["bmkhs_mainRtrBaseThrust",   getNumber (_config >> "mainRtrBaseThrust")];
_heli setVariable ["bmkhs_mainRotorGearRatio",  getNumber (_config >> "mainRtrGearRatio")];
_heli setVariable ["bmkhs_mainRtrFlapbackLon",  getNumber (_config >> "mainRtrFlapbackLon")];
_heli setVariable ["bmkhs_mainRtrFlapbackLat",  getNumber (_config >> "mainRtrFlapbackLat")];
_heli setVariable ["bmkhs_tailRtrPos",          getArray  (_config >> "tailRtrPos")];
_heli setVariable ["bmkhs_tailRtrDesignRpm",    getNumber (_config >> "tailRtrDesignRpm")];
_heli setVariable ["bmkhs_tailRtrRpmTrimVal",   getNumber (_config >> "tailRtrRpmTrimVal")];
_heli setVariable ["bmkhs_tailRtrGearRatio",    getNumber (_config >> "tailRtrGearRatio")];
_heli setVariable ["bmkhs_tailRtrNumBlades",    getNumber (_config >> "tailRtrNumBlades")];
_heli setVariable ["bmkhs_tailRtrBladeRadius",  getNumber (_config >> "tailRtrBladeRadius")];
_heli setVariable ["bmkhs_tailRtrBladeChord",   getNumber (_config >> "tailRtrBladeChord")];
_heli setVariable ["bmkhs_tailRtrBaseThrust",   getNumber (_config >> "tailRtrBaseThrust")];

//The old model wants a 2-slot array whatever is declared, and it treats
//vrsVelocityMin/Max as SCALARS - the new model makes them per-rotor arrays.
//While the old functions are the ones running, the old shapes win.
_heli setVariable ["bmkhs_reqEngTorque",   [0.0, 0.0]];
_heli setVariable ["bmkhs_rtrThrust",      [0.0, 0.0]];
_heli setVariable ["bmkhs_rtrMoi",         [0.0, 0.0]];
_heli setVariable ["bmkhs_vrsVelocityMin", 0.0];
_heli setVariable ["bmkhs_vrsVelocityMax", 0.0];
