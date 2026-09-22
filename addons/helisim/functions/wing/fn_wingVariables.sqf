/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_wingVariables

Description:
    Loads the per-wing configuration. Core loops over numWings, so an aircraft
    declares as many lifting surfaces as it has - wings, fins, stabilators - or
    none at all.

    Each surface becomes one hashmap keyed by the config's own property names, so
    adding a field to the config means naming it in the list below and nothing
    else - no positional argument to keep in sync.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

//Fields read straight off the surface's class. The key a reader asks for IS the config
//property name, so grepping a knob finds the config, this list and every use of it.
private _numFields = [
     "isStabilator"
   , "pitch"
   , "roll"
   , "span"
   , "chord"
   , "sweep"
   , "twist"
   , "tipWidthScalar"
   , "numElements"
   , "chordLinePos"
];
private _arrFields  = ["pos"];
private _textFields = ["name", "airfoil"];

private _numWings = getNumber (_config >> "numWings");
private _wings    = [];

for "_i" from 1 to _numWings do {
    private _w = (_config >> "Wings") >> format ["Wing%1%2", ["0", ""] select (_i > 9), _i];

    //Hashmap, not a positional array: adding or removing a field cannot silently shift
    //what every reader sees.
    private _wing = createHashMap;

    { _wing set [_x, getNumber (_w >> _x)]; } forEach _numFields;
    { _wing set [_x, getArray  (_w >> _x)]; } forEach _arrFields;
    { _wing set [_x, getText   (_w >> _x)]; } forEach _textFields;

    //The surface NAMES itself, so a table or a readout says "vertical fin" rather than
    //wing 3.
    private _name = _wing get "name";
    if (_name isEqualTo "") then { _name = format ["wing %1", _i]; _wing set ["name", _name]; };

    //Section resolved once at init, not per element per frame.
    _wing set ["airfoilTable", [_heli, _wing get "airfoil", _name] call bmkhs_fnc_airfoilGet];

    _wings pushBack _wing;
};

_heli setVariable ["bmkhs_numWings", _numWings];
_heli setVariable ["bmkhs_wings",    _wings];
