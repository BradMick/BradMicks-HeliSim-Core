/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_turboShaftPowerTurbine

Description:
    The free turbine. It takes a share of the gas generator's output - what the
    compressor leaves, floored so gas moving over the turbine always turns it.

Parameters:
    _engine   - That engine's config [HashMap]
    _gasPower - Gas power from the hot section [Number]
    _absorbed - What the compressor took [Number]
    _npFrac   - Np as a fraction of governed [Number]

Returns:
    [_shaftTq, _gaugeTq] - torque to the drivetrain, and as indicated, Nm [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_engine", "_gasPower", "_absorbed", "_npFrac"];

if (_gasPower <= 0.0) exitWith { [0.0, 0.0] };

private _refTq = _engine get "refTq";
private _share = ((_gasPower - _absorbed) / _gasPower) max (_engine get "ptIdleExtract");

private _shaftTq = _gasPower * _refTq * _share * (_engine get "ptEfficiency");
private _gaugeTq = (_shaftTq / (_npFrac max EPSILON)) min (_refTq * (_engine get "stallTqMult"));

[_shaftTq, _gaugeTq]
