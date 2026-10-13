/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_utilGateOpen

Description:
    Whether a gate[] holds - the same form as a component gate: each entry a
    variable name, or {circuit, threshold}; all must hold. An empty gate holds.

Parameters:
    _heli - The helicopter [Object]
    _gate - The gate [Array]

Returns:
    Holds [Bool]
---------------------------------------------------------------------------- */
params ["_heli", "_gate"];

(_gate findIf {
    !(if (_x isEqualType []) then {
        ([_heli, _x select 0] call bmkhs_fnc_systemCircuit) >= (_x select 1)
    } else {
        _heli getVariable [_x, false]
    })
}) < 0
