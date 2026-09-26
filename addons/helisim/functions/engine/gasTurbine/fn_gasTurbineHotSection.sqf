/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_gasTurbineHotSection

Description:
    Combustor. TGT is heat released over the mass flow carrying it away, and it
    is STATE - so residual heat, motoring to cool and the hot-start runaway all
    come out of one mechanism rather than three special cases.

Parameters:
    _engine    - That engine's config [HashMap]
    _tgt       - TGT at the top of the frame, deg C [Number]
    _ng        - Spool speed AFTER the spool step [Number]
    _fuelCmd   - Commanded fuel, normalised [Number]
    _residualHeat - Heat left over from the last run, latched when the lever
                 moved; 1.0 is a purged hot section [Number]
    _dens      - Air density as a fraction of a standard day [Number]
    _fat       - Free air temperature, deg C [Number]
    _velY      - Forward speed, m/s [Number]
    _lit       - Is it burning [Boolean]
    _deltaTime - Frame time [Number]

Returns:
    Stepped TGT, deg C [Number]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_engine", "_tgt", "_ng", "_fuelCmd", "_residualHeat", "_dens", "_fat", "_velY", "_lit", "_deltaTime"];

//The POST-step Ng: the combustor sees what the compressor is delivering now. Floored so a
//stopped spool cannot divide by zero.
private _massFlow = ((_ng ^ (_engine get "massFlowExp")) * _dens) max 0.02;

//Fades as Ng comes up, because the spool rising IS the hot section purging. A cold start
//latches ~1.0 and this path is inert.
private _currentHeat = 1.0 + ((_residualHeat - 1.0) * ((1.0 - (_ng / (_engine get "idleNg"))) max 0.0));
private _tgtHot      = [_fat, _fat + (_currentHeat * (_engine get "tgtK") * _fuelCmd / _massFlow)] select _lit;

//coolingCoef is the coefficient; the bracket is the airflow available to carry the heat
//away. Ng is the windmill, stillAirFlow is the floor with the spool stopped, then ram.
private _coolRate = (_engine get "coolingCoef") * (_ng + (_engine get "stillAirFlow") + (_velY / VEL_VNE));
private _rate     = [_coolRate, _engine get "thermalMassCoef"] select (_tgtHot > _tgt);

_tgt + ((_tgtHot - _tgt) * _rate * _deltaTime)
