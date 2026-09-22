/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_simpleRotorVariables

Description:
    Loads the simple rotor configuration and initialises rotor state.

    Each rotor becomes one hashmap keyed by the config's own property names, so
    adding a knob to the config means naming it in the field list below and
    nothing else - no positional argument to keep in sync, and no second copy of
    the name to go stale.

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
#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

params ["_heli", "_config"];

//Fields read straight off the rotor's class. The key a reader asks for IS the config
//property name, so grepping a knob finds the config, this list and every use of it.
private _numFields = [
     "numBlades"
   , "mastLength"
   , "gearRatio"
   , "torqueTau"
   , "bladeRadius"
   , "bladeChord"
   , "bladeMass"
   , "pitchFlapMin"
   , "pitchFlapMid"
   , "pitchFlapMax"
   , "rollFlapMin"
   ,  "rollFlapMid"
   ,  "rollFlapMax"
   , "coneAngle"
   , "flapBackRollMax"
   , "flapBackPitchMax"
   , "rollGain"
   , "pitchGain"
   , "gndEffValue"
   , "reacTqScalar"
];
private _arrFields   = ["pivot", "rotation"];
private _gridFields  = ["liftCoefTable", "dragCoefTable"];

private _numSimpleRotors = getNumber (_config >> "numSimpleRotors");
private _rotors          = [];

for "_i" from 1 to _numSimpleRotors do {
    private _r = (_config >> "SimpleRotors") >> format ["SimpleRotor%1%2", ["0", ""] select (_i > 9), _i];

    //Hashmap, not a positional array: adding or removing a field cannot silently shift
    //what every reader sees.
    private _rotor = createHashMap;

    //type and direction are named in the config, so nothing downstream assumes rotor 0 is
    //the main one or that every main turns the same way.
    _rotor set ["type", [MAIN, TAIL] select (toLower getText (_r >> "type")      == "tail")];
    _rotor set ["dir",  [CCW,  CW  ] select (toLower getText (_r >> "direction") == "cw")];

    { _rotor set [_x, getNumber (_r >> _x)]; } forEach _numFields;
    { _rotor set [_x, getArray  (_r >> _x)]; } forEach _arrFields;

    //Authored grids, header row and all - the builder validates them and reports by name.
    {
        _rotor set [_x, [getArray (_r >> _x), format ["simple rotor %1 %2", _i - 1, _x]] call bmkhs_fnc_mathBuildInterpGrid];
    } forEach _gridFields;

    _rotors pushBack _rotor;
};

_heli setVariable ["bmkhs_numSimpleRotors", _numSimpleRotors];
_heli setVariable ["bmkhs_simpleRotors",    _rotors];

//Live state - seeded because the torque filter and the transmission read these before
//anything has written them.
_heli setVariable ["bmkhs_reqEngTorque",   [0.0, 0.0]];
_heli setVariable ["bmkhs_rtrThrust",      [0.0, 0.0]];
_heli setVariable ["bmkhs_rtrRPM",         0.0];
_heli setVariable ["bmkhs_rtrMoi",         [0.0, 0.0]];
