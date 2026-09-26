/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_turboShaftPowerTurbine

Description:
    The free turbine. Shaft torque is the SURPLUS after the compressor takes its
    share, capped by what is demanded - an unloaded engine makes no torque at
    any Ng, which is what a free turbine does.

Parameters:
    _engine   - That engine's config [HashMap]
    _shaft    - The free turbine's share of the gas [Number]
    _gasPower - Gas power from the hot section [Number]
    _absorbed - What the compressor took [Number]

Returns:
    Shaft torque, Nm [Number]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_engine", "_shaft", "_gasPower", "_absorbed"];

(_shaft min ((_gasPower - _absorbed) max 0.0)) * (_engine get "refTq") * (_engine get "ptEfficiency")
