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
    _running   - Burning, as opposed to cranked [Boolean]
    _deltaTime - Frame time [Number]

Returns:
    Stepped TGT, deg C [Number]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_engine", "_tgt", "_ng", "_fuelCmd", "_residualHeat", "_dens", "_fat", "_velY", "_running", "_deltaTime"];

//The POST-step Ng: the combustor sees what the compressor is delivering now. Floored so a
//stopped spool cannot divide by zero.
private _massFlow = ((_ng ^ (_engine get "massFlowExp")) * _dens) max 0.02;

//Fades as Ng comes up, because the spool rising IS the hot section purging. A cold start
//latches ~1.0 and this path is inert.
private _currentHeat = 1.0 + ((_residualHeat - 1.0) * ((1.0 - (_ng / (_engine get "idleNg"))) max 0.0));
private _tgtHot      = [_fat, _fat + (_currentHeat * (_engine get "tgtK") * _fuelCmd / _massFlow)] select _running;

//coolingCoef is the coefficient; the bracket is the airflow available to carry the heat
//away. Ng is the windmill, stillAirFlow is the floor with the spool stopped, then ram.
//Ram is forward only - air cannot be rammed in backwards.
private _ram      = ((_velY max 0.0) * (_engine get "ramAirCoef"));
private _coolRate = (_engine get "coolingCoef") * (_ng + (_engine get "stillAirFlow") + _ram);
private _rate     = [_coolRate, _engine get "thermalMassCoef"] select (_tgtHot > _tgt);

private _tgtNew = _tgt + ((_tgtHot - _tgt) * _rate * _deltaTime);

//TEMPORARY - remove when the post-shutdown TGT climb is found.
if (bmkhs_sysDebug) then {
    private _last = missionNamespace getVariable ["bmkhs_hotDiagLast", 0];
    if (time > _last + 0.25) then {
        missionNamespace setVariable ["bmkhs_hotDiagLast", time];
        diag_log text format [
            "HOTDIAG run=%1 fat=%2 ng=%3 fuel=%4 mflow=%5 heat=%6 tgtHot=%7 ram=%8 coolRate=%9 rate=%10 tgt=%11->%12 dt=%13",
            _running, _fat toFixed 1, _ng toFixed 4, _fuelCmd toFixed 4,
            _massFlow toFixed 4, _currentHeat toFixed 3, _tgtHot toFixed 1,
            _ram toFixed 6, _coolRate toFixed 6, _rate toFixed 6,
            _tgt toFixed 1, _tgtNew toFixed 1, _deltaTime toFixed 4
        ];
    };
};

_tgtNew
