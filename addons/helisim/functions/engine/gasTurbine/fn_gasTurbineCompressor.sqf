/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_gasTurbineCompressor

Description:
    Compressor, stations 2 -> 3. Everything follows corrected Ng through the map.

Parameters:
    _engine - That engine's config [HashMap]
    _ng     - Spool speed at the top of the frame [Number]
    _fat    - Ambient temperature, deg C [Number]
    _pAmb   - Ambient pressure, kPa [Number]
    _velFwd - Airspeed into the inlet, m/s [Number]

Returns:
    [_nc, _pr, _mDot, _t3, _p3, _compPower, _ctExpansion, _inletVel] - corrected Ng,
    pressure ratio, airflow kg/s, T3 K, P3 kPa, compressor power kW, the compressor
    turbine's expansion ratio, inlet velocity m/s [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\engine\engine.hpp"

params ["_engine", "_ng", "_fat", "_pAmb", "_velFwd"];

//Ram - forward speed raises the inlet's total temperature and pressure; the inlet keeps ramRecovery
//of the pressure rise, so standing still it sees ambient.
private _tAmb  = _fat + DEG_C_TO_KELVIN;
private _mach2 = ((_velFwd max 0.0) ^ 2) / (GT_GAMMA_COLD * GT_R_AIR * 1000 * _tAmb);
private _ram   = 1 + (((GT_GAMMA_COLD - 1) / 2) * _mach2);
private _t2    = _tAmb * _ram;
private _p2    = _pAmb + ((_engine get "ramRecovery") * ((_pAmb * (_ram ^ (GT_GAMMA_COLD / (GT_GAMMA_COLD - 1)))) - _pAmb));

private _theta = _t2 / GT_STD_TEMP_K;
private _nc    = _ng / (sqrt _theta);

([_engine get "compressorMap", _nc / (_engine get "maxNg")] call bmkhs_fnc_mathLinearInterp)
    params ["", "_prFrac", "_flowFrac", "_eff", "_ctFrac"];

private _designPr = _engine get "pressureRatio";
private _pr       = _designPr * _prFrac;
private _airflow  = ([_engine get "airflowTable", _fat] call bmkhs_fnc_mathLinearInterp) select 1;
private _mDot     = (_engine get "massFlow") * _flowFrac * _airflow * (_p2 / GT_STD_PRESSURE_KPA) / (sqrt _theta);

private _xc        = (GT_GAMMA_COLD - 1) / GT_GAMMA_COLD;
private _t3        = _t2 * (1 + (((_pr ^ _xc) - 1) / _eff));
private _compPower = _mDot * GT_CP_COLD * (_t3 - _t2);

//Logged only, until ram air reads it.
private _inletArea = pi * (((_engine get "inletDiameter") / 2) ^ 2);
private _inletVel  = _mDot / _inletArea / (_p2 / (GT_R_AIR * _t2));

[_nc, _pr, _mDot, _t3, _p2 * _pr, _compPower, _designPr ^ _ctFrac, _inletVel]
