/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputControlHandle

Description:
    Dispatches HeliSim's discrete flight-control actions - force trim, hold
    modes and the sticky-input interrupt. Bound from Core's CfgUserActions.

Parameters:
    _name  - Action name [String]
    _state - Pressed [Bool]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_name", "_state"];

private _heli = vehicle player;
if !(_heli getVariable ["bmkhs_initialised", false]) exitWith {};

if (_state) then {
    switch (_name) do {
        case "bmkhs_forceTrim":          { [_heli] call bmkhs_fnc_fmcForceTrimHold; };
        case "bmkhs_holdModeAltitude":   { [_heli] call bmkhs_fnc_fmcAltitudeHoldEnable; };
        case "bmkhs_holdModeAttitude":   { [_heli] call bmkhs_fnc_fmcAttitudeHoldEnable; };
        case "bmkhs_holdModesOff":       { [_heli] call bmkhs_fnc_fmcHoldModesDisable; };
        case "bmkhs_stickyInterrupt":    { [_heli, true] call bmkhs_fnc_stickyInterrupt; };
    };
} else {
    switch (_name) do {
        case "bmkhs_forceTrim":          { [_heli] call bmkhs_fnc_fmcForceTrimRelease; };
        case "bmkhs_forceTrimPanic":     { [_heli] call bmkhs_fnc_fmcForceTrimReset; };
        case "bmkhs_stickyInterrupt":    { [_heli, false] call bmkhs_fnc_stickyInterrupt; };
    };
};
