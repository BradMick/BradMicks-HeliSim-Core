/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_trimVariables

Description:
    Loads the aircraft's declared trim (class Trim - see \bmkhs_helisim\trim.hpp).
    No class Trim is force trim with no beep.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

private _cfg  = _config >> "Trim";
private _trim = createHashMapFromArray [["forceTrim", [1, getNumber (_cfg >> "forceTrim")] select (isNumber (_cfg >> "forceTrim"))]];

//Beep - undeclared, the aircraft has none
private _beep = _cfg >> "Beep";
if (isClass _beep) then {
    private _b = createHashMapFromArray [["gate", getArray (_beep >> "gate")]];
    { _b set [_x select 0, [_x select 1, getNumber (_beep >> (_x select 0))] select (isNumber (_beep >> (_x select 0)))] } forEach [["lag", 0.1]];
    { _b set [_x select 0, [_x select 1, getArray (_beep >> (_x select 0))] select (isArray (_beep >> (_x select 0)))] } forEach [["rate", [0.10, 0.10]], ["moves", ["stick", "stick"]]];
    _b set ["moves", (_b get "moves") apply {toLower _x}];
    _trim set ["beep", _b];
};
_heli setVariable ["bmkhs_trim", _trim];

//Trim switch state as last seen by fn_trimUpdate - the change is what captures or zeroes
_heli setVariable ["bmkhs_trimOn",       true];
//Beep switches held: action name -> [axis, sign, time pressed]
_heli setVariable ["bmkhs_trimBeepHeld", createHashMap];
//The beep's trim travel this frame, {pitch, roll} - fn_trimUpdate
_heli setVariable ["bmkhs_trimBeepDelta", [0.0, 0.0]];
