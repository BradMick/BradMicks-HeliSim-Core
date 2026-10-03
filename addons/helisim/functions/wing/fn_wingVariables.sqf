/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_wingVariables

Description:
    Loads the per-wing configuration: every class in Wings, so an aircraft
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
     "numElements"
   , "chordLinePos"
];
private _arrFields  = ["panels"];
private _textFields = ["name", "facing", "airfoil"];

private _wings = [];

{
    private _w = _x;
    private _i = _forEachIndex + 1;

    //Hashmap, not a positional array: adding or removing a field cannot silently shift
    //what every reader sees.
    private _wing = createHashMap;

    { _wing set [_x, getNumber (_w >> _x)]; } forEach _numFields;
    { _wing set [_x, getArray  (_w >> _x)]; } forEach _arrFields;
    { _wing set [_x, getText   (_w >> _x)]; } forEach _textFields;
    _wing set ["facing", toLower (_wing get "facing")];

    //The surface NAMES itself, so a table or a readout says "vertical fin" rather than
    //wing 3.
    private _name = _wing get "name";
    if (_name isEqualTo "") then { _name = format ["wing %1", _i]; _wing set ["name", _name]; };

    //Section resolved once at init, not per element per frame.
    _wing set ["airfoilTable", [_heli, _wing get "airfoil", _name] call bmkhs_fnc_airfoilGet];

    _wings pushBack _wing;
} forEach ("true" configClasses (_config >> "Wings"));

_heli setVariable ["bmkhs_numWings", count _wings];
_heli setVariable ["bmkhs_wings",    _wings];

if (local _heli) then {
    _heli setVariable ["bmkhs_stabilatorPosition", 0.0, true];
};
