/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fuelTankVarName

Description:
    Validates a tank's variableName and returns the full variable prefix Core
    publishes it under.

    The aircraft supplies the middle part only - variableName = "fwdTank"
    becomes bmkhs_fwdTank, so the tank's variables are bmkhs_fwdTankMass,
    bmkhs_fwdTankMax and so on. Core owns the bmkhs_ prefix; a config that
    includes it anyway has it stripped rather than doubled.

    Misconfiguration is reported loudly and then worked around, so a bad pack
    produces a readable error instead of tanks that silently share a variable
    or write one nothing reads.

Parameters:
    _tank       - The tank's config entry [Config]
    _kind       - "FuelTank" or "AuxTank", for the error text [String]
    _index      - 1-based tank number, for the error text and fallback [Number]
    _seenNames  - Full names already taken by earlier tanks [Array]

Returns:
    The full variable prefix, e.g. "bmkhs_fwdTank" [String]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_tank", "_kind", "_index", ["_seenNames", []]];

private _raw = getText (_tank >> "variableName");

//Core owns the prefix. Accept it if the author wrote it anyway rather than producing
//bmkhs_bmkhs_something.
if (_raw select [0, 6] == "bmkhs_") then { _raw = _raw select [6] };

private _fallback = format ["bmkhs_%1%2", toLower _kind, _index];

if (_raw == "") exitWith {
    diag_log text format [
        "[BMKHS] FUEL CONFIG ERROR: %1%2 has no variableName. Falling back to %3 - the aircraft's displays will not find this tank.",
        _kind, _index, _fallback
    ];
    _fallback
};

private _name = "bmkhs_" + _raw;

if (_name in _seenNames) exitWith {
    diag_log text format [
        "[BMKHS] FUEL CONFIG ERROR: %1%2 reuses variableName '%3', already taken by an earlier tank. Falling back to %4 - fix the config, two tanks cannot share one variable.",
        _kind, _index, _raw, _fallback
    ];
    _fallback
};

_name
