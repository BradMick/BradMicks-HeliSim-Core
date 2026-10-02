/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_gasTurbineCombustor

Description:
    Combustor, stations 3 -> 4. Fuel heat raises the compressed air to T4.

Parameters:
    _engine  - That engine's config [HashMap]
    _t3      - Compressor exit temperature, K [Number]
    _mDot    - Airflow, kg/s [Number]
    _fuelCmd - Commanded fuel, normalised [Number]
    _running - Burning, as opposed to cranked [Boolean]

Returns:
    T4, K [Number]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\engine\engine.hpp"

params ["_engine", "_t3", "_mDot", "_fuelCmd", "_running"];

if (!_running) exitWith { _t3 };

private _fuelKgs = _fuelCmd * (_engine get "maxFuelFlow");
_t3 + (_fuelKgs * (_engine get "fuelLhv") * (_engine get "combustorEfficiency") / ((_mDot max 0.001) * GT_CP_HOT))
