/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_trimVariables

Description:
    Loads the aircraft's declared trim (class Trim - see \bmkhs_helisim\fmc.hpp).
    Force trim is always there; no class Trim is force trim with no beep.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_config"];
#include "\bmkhs_helisim\fmc.hpp"

private _trim = createHashMap;

//Beep - undeclared, the aircraft has none
private _beep = _config >> "Trim" >> "Beep";
if (isClass _beep) then {
    _trim set ["beep", createHashMapFromArray [
        ["gate", getArray (_beep >> "gate")],
        ["rate", [TRIM_BEEP_RATE, getArray (_beep >> "rate")] select (isArray (_beep >> "rate"))]
    ]];
};
_heli setVariable ["bmkhs_trim", _trim];

//Trim switch state as last seen by fn_trimUpdate - the change is what captures or zeroes
_heli setVariable ["bmkhs_trimOn",       true];
//Beep switches held: action name -> [axis, sign, time pressed, on a flight director target]
_heli setVariable ["bmkhs_trimBeepHeld", createHashMap];
//The beep's travel this frame, {pitch, roll}: the stick's, for the holds; a director target's
_heli setVariable ["bmkhs_trimBeepDelta", [0.0, 0.0]];
_heli setVariable ["bmkhs_trimBeepFd",    [0.0, 0.0]];
