/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcSetChannel

Description:
    Enables or disables one FMC control channel. The actuator and FMC read
    these every frame to decide whether that axis is augmented.

Parameters:
    _heli    - The helicopter [Object]
    _channel - "pitch", "roll", "yaw", "coll" or "trim" [String]
    _state   - Enabled [Bool]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_channel", "_state"];

private _var = switch (toLower _channel) do {
    case "pitch": { "bmkhs_fmcPitchOn" };
    case "roll":  { "bmkhs_fmcRollOn"  };
    case "yaw":   { "bmkhs_fmcYawOn"   };
    case "coll":  { "bmkhs_fmcCollOn"  };
    case "trim":  { "bmkhs_fmcTrimOn"  };
    default       { "" };
};

if (_var == "") exitWith {};

_heli setVariable [_var, _state, true];
