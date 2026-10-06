/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_turboShaftPowerTurbine

Description:
    The free turbine, stations 4.5 -> 5. The gas expands to ambient across it, and
    that torque accelerates its own speed. Np is state, not the rotor's.

    The freewheel grips on speed alone: the turbine drives the rotor and is never
    driven by it, so a shutdown and an autorotation decouple through the same
    branch.

Parameters:
    _engine    - That engine's config [HashMap]
    _t45       - Gas temperature at its inlet, K [Number]
    _p45       - Gas pressure at its inlet, kPa [Number]
    _p2        - Ambient pressure, kPa [Number]
    _mDot      - Airflow, kg/s [Number]
    _running   - Burning, as opposed to cranked [Boolean]
    _np        - Np at the top of the frame, normalised [Number]
    _nrFrac    - Rotor speed as a fraction of governed Np [Number]
    _deltaTime - Frame time [Number]

Returns:
    [_shaftTq, _np, _clutch, _t5] - shaft torque in Nm, the stepped Np, whether the
    freewheel is engaged, exhaust temperature K [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\engine\engine.hpp"

params ["_engine", "_t45", "_p45", "_p2", "_mDot", "_running", "_np", "_nrFrac", "_deltaTime"];

private _refTq = _engine get "refTq";

private _t5      = _t45;
private _ptPower = 0.0;
if (_p45 > _p2) then {
    private _xh = (GT_GAMMA_HOT - 1) / GT_GAMMA_HOT;
    _t5      = _t45 * (1 - ((_engine get "ptEfficiency") * (1 - ((_p2 / _p45) ^ _xh))));
    _ptPower = (_mDot max 0.0) * GT_CP_HOT * (_t45 - _t5);
};

//Gas power expressed as torque at design Np - the torque gauge's own reference.
private _shaftTq = _ptPower / (_engine get "powerKw") * _refTq;

//Windmilling only - gas flowing over the turbine drives it, so the floor applies just when
//there is none behind it. It finishes the stop, since np^2 alone only asymptotes.
private _npDrag = ((_engine get "ptDrag") * _np * _np)
                + ([0.0, _engine get "ptDragFloor"] select (_shaftTq <= 0.0));
private _npDot  = ((_shaftTq / _refTq) - _npDrag) / (_engine get "ptInertia");
private _npFree = (_np + (_npDot * _deltaTime)) max 0.0;

//A sprag clutch: engaged while the turbine, free, would keep up with the rotor; released the
//moment the rotor overruns it - running or not.
private _clutch   = _npFree >= _nrFrac;
//Engaged, the pair are one shaft and the transmission integrates them together.
_np = [_npFree, _nrFrac] select _clutch;

[_shaftTq, _np, _clutch, _t5]
