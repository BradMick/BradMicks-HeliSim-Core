/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_turboShaftPowerTurbine

Description:
    The free turbine. Everything the hot section puts out blows over it, and that
    torque accelerates its own speed. Np is state, not the rotor's.

    The freewheel grips on speed alone: the turbine drives the rotor and is never
    driven by it, so a shutdown and an autorotation decouple through the same
    branch.

Parameters:
    _engine    - That engine's config [HashMap]
    _fuelGas   - Heat released by combustion [Number]
    _airGas    - Cold air the compressor is pushing through [Number]
    _compWork  - What the compressor turbine takes out of the gas [Number]
    _np      - Np at the top of the frame, normalised [Number]
    _nrFrac    - Rotor speed as a fraction of governed Np [Number]
    _deltaTime - Frame time [Number]

Returns:
    [_shaftTq, _np, _clutch] - shaft torque in Nm, the stepped Np, and whether
    the freewheel is engaged [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_engine", "_fuelGas", "_airGas", "_compWork", "_np", "_nrFrac", "_deltaTime"];

private _refTq = _engine get "refTq";

//Less what the compressor turbine takes. Motoring, the starter turns the compressor, so nothing is taken.
private _ptGas = (_fuelGas + _airGas - ([0.0, _compWork] select (_fuelGas > 0.0))) max 0.0;

private _shaftTq = _ptGas * _refTq * (_engine get "ptEfficiency");

//Windmilling only - gas flowing over the turbine drives it, so the floor applies just when
//there is none behind it. It finishes the stop, since np^2 alone only asymptotes.
private _npDrag = ((_engine get "ptDrag") * _np * _np)
                + ([0.0, _engine get "ptDragFloor"] select (_shaftTq <= 0.0));
private _npDot  = ((_shaftTq / _refTq) - _npDrag) / (_engine get "ptInertia");
private _npFree = (_np + (_npDot * _deltaTime)) max 0.0;

//Running, the turbine is driven and stays engaged; not running, its drag lets it go.
private _npDriven = _np + (((_shaftTq / _refTq) / (_engine get "ptInertia")) * _deltaTime);
private _clutch   = ([_npFree, _npDriven] select (_fuelGas > 0.0)) >= _nrFrac;
//Engaged, the pair are one shaft and the transmission integrates them together.
_np = [_npFree, _nrFrac] select _clutch;

[_shaftTq, _np, _clutch]
