/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputControlHandle

Description:
    Dispatches HeliSim's discrete flight-control actions - force trim, hold
    modes, the flight director and the sticky-input interrupt. Bound from Core's
    CfgUserActions; a cockpit control calls it with the same action name.

Parameters:
    _name  - Action name [String]
    _state - Pressed [Bool]
    _heli  - The helicopter [Object, default the player's]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_name", "_state", ["_heli", vehicle player]];

if !(_heli getVariable ["bmkhs_initialised", false]) exitWith {};

//Flight director - mode toggles, target steps and syncs, on the press
private _fdMode = createHashMapFromArray [
    ["bmkhs_fdRalt", "ralt"], ["bmkhs_fdAlt", "alt"], ["bmkhs_fdAltp", "altp"], ["bmkhs_fdIas", "ias"],
    ["bmkhs_fdHdg", "hdg"], ["bmkhs_fdNav", "nav"], ["bmkhs_fdHvr", "hvr"]
];
private _fdStep = createHashMapFromArray [
    ["bmkhs_fdRaltUp", ["ralt", 1]], ["bmkhs_fdRaltDn", ["ralt", -1]],
    ["bmkhs_fdAltUp",  ["alt",  1]], ["bmkhs_fdAltDn",  ["alt",  -1]],
    ["bmkhs_fdAltpUp", ["altp", 1]], ["bmkhs_fdAltpDn", ["altp", -1]],
    ["bmkhs_fdIasUp",  ["ias",  1]], ["bmkhs_fdIasDn",  ["ias",  -1]],
    ["bmkhs_fdHdgUp",  ["hdg",  1]], ["bmkhs_fdHdgDn",  ["hdg",  -1]]
];
private _fdSync = createHashMapFromArray [
    ["bmkhs_fdRaltSync", "ralt"], ["bmkhs_fdAltSync", "alt"], ["bmkhs_fdAltpSync", "altp"],
    ["bmkhs_fdIasSync", "ias"], ["bmkhs_fdHdgSync", "hdg"]
];
if (_state && {_name in _fdMode}) exitWith { [_heli, _fdMode get _name] call bmkhs_fnc_fmcFdMode };
if (_state && {_name in _fdStep}) exitWith { ([_heli] + (_fdStep get _name)) call bmkhs_fnc_fmcFdStep };
if (_state && {_name in _fdSync}) exitWith { [_heli, _fdSync get _name] call bmkhs_fnc_fmcFdSync };

if (_state) then {
    switch (_name) do {
        case "bmkhs_forceTrim":          { [_heli] call bmkhs_fnc_fmcForceTrimHold; };
        case "bmkhs_holdModeAltitude":   { [_heli] call bmkhs_fnc_fmcAltitudeHoldEnable; };
        case "bmkhs_holdModeAttitude":   { [_heli] call bmkhs_fnc_fmcAttitudeHoldEnable; };
        case "bmkhs_holdModesOff":       { [_heli] call bmkhs_fnc_fmcHoldModesDisable; };
        case "bmkhs_stickyInterrupt":    { [_heli, true] call bmkhs_fnc_stickyInterrupt; };
        case "bmkhs_ctrlVisToggle":      { [_heli] call bmkhs_fnc_ctrlVisToggle; };
    };
} else {
    switch (_name) do {
        case "bmkhs_forceTrim":          { [_heli] call bmkhs_fnc_fmcForceTrimRelease; };
        case "bmkhs_forceTrimPanic":     { [_heli] call bmkhs_fnc_fmcForceTrimReset; };
        case "bmkhs_stickyInterrupt":    { [_heli, false] call bmkhs_fnc_stickyInterrupt; };
    };
};
