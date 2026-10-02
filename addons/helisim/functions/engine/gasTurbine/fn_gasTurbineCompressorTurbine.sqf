/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_gasTurbineCompressorTurbine

Description:
    Compressor turbine, stations 4 -> 4.5. Its power against the compressor's steps
    Ng, and TGT is read at its exit.

Parameters:
    _engine       - That engine's config [HashMap]
    _ng           - Spool speed at the top of the frame [Number]
    _tgt          - TGT at the top of the frame, deg C [Number]
    _t4           - Combustor exit temperature, K [Number]
    _p3           - Compressor exit pressure, kPa [Number]
    _mDot         - Airflow, kg/s [Number]
    _compPower    - Compressor power, kW [Number]
    _ctExpansion  - This turbine's expansion ratio [Number]
    _starterTq    - Starter torque on the spool [Number]
    _residualHeat - Heat left from the last run; 1.0 is a purged hot section [Number]
    _fat          - Free air temperature, deg C [Number]
    _velY         - Forward speed, m/s [Number]
    _running      - Burning, as opposed to cranked [Boolean]
    _spooling     - Spooling down, unfired and uncranked [Boolean]
    _deltaTime    - Frame time [Number]

Returns:
    [_ng, _tgt, _t45, _p45] - stepped Ng, stepped TGT deg C, T4.5 K, P4.5 kPa [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\engine\engine.hpp"

params ["_engine", "_ng", "_tgt", "_t4", "_p3", "_mDot", "_compPower", "_ctExpansion", "_starterTq",
        "_residualHeat", "_fat", "_velY", "_running", "_spooling", "_deltaTime"];

private _m       = _mDot max 0.001;
private _xh      = (GT_GAMMA_HOT - 1) / GT_GAMMA_HOT;
private _ctPower = _m * GT_CP_HOT * (_engine get "turbineEfficiency") * _t4 * (1 - (_ctExpansion ^ (-_xh)));
private _t45     = _t4 - (_ctPower / (_m * GT_CP_HOT));

//Spooling down, the unfired compressor's drag is what stops it; the floor finishes the stop.
private _drag  = [0.0, ((_engine get "compDrag") * _ng * _ng) + (_engine get "compDragFloor")] select _spooling;
private _ngDot = (((_ctPower - _compPower) / (_engine get "spoolUnitKw")) + _starterTq - _drag) / (_engine get "spoolInertia");
private _ngNew = [_ng + (_ngDot * _deltaTime), 0.0, 1.1] call BIS_fnc_clamp;

//Fades as Ng comes up, because the spool rising IS the hot section purging.
private _currentHeat = 1.0 + ((_residualHeat - 1.0) * ((1.0 - (_ngNew / (_engine get "idleNg"))) max 0.0));
private _tgtHot      = [_fat, _fat + (_currentHeat * (_t45 - DEG_C_TO_KELVIN - _fat))] select _running;

//The gauge lags the gas. Ng is the windmill, still air the floor once stopped, then ram.
private _coolRate = GT_TGT_COOL_RATE * (_ngNew + GT_TGT_STILL_AIR + ((_velY max 0.0) * GT_TGT_RAM_AIR));
private _rate     = [_coolRate, GT_TGT_HEAT_RATE] select (_tgtHot > _tgt);
private _tgtNew   = _tgt + ((_tgtHot - _tgt) * _rate * _deltaTime);

//TEMPORARY - remove when the post-shutdown TGT climb is found.
if (bmkhs_sysDebug) then {
    private _key  = format ["bmkhs_hotDiagLast_%1", _engine get "name"];
    private _last = missionNamespace getVariable [_key, 0];
    if (time > _last + 0.25) then {
        missionNamespace setVariable [_key, time];
        diag_log text format [
            "HOTDIAG eng=%14 run=%1 fat=%2 ng=%3 t4=%4 t45=%5 heat=%6 tgtHot=%7 ctPwr=%8 coolRate=%9 rate=%10 tgt=%11->%12 dt=%13",
            _running, _fat toFixed 1, _ngNew toFixed 4, (_t4 - DEG_C_TO_KELVIN) toFixed 1,
            (_t45 - DEG_C_TO_KELVIN) toFixed 1, _currentHeat toFixed 3, _tgtHot toFixed 1,
            _ctPower toFixed 1, _coolRate toFixed 6, _rate toFixed 6,
            _tgt toFixed 1, _tgtNew toFixed 1, _deltaTime toFixed 4, _engine get "name"
        ];
    };
};

[_ngNew, _tgtNew, _t45, _p3 / _ctExpansion]
