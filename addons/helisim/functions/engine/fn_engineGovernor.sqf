/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engineGovernor

Description:
    The ECU and the fuel control. At FLY it trims the orifice below the lever's
    to hold Np, anticipating the collective.

Parameters:
    _heli      - The helicopter [Object]
    _index     - Which engine [Number]
    _engine    - That engine's config [HashMap]
    _ng        - Spool speed at the top of the frame [Number]
    _np        - Np at the top of the frame, normalised [Number]
    _tgt       - TGT at the top of the frame, deg C [Number]
    _lever     - Power lever position, OFF / IDLE / FLY [String]
    _fat       - Free air temperature, deg C [Number]
    _deltaTime - Frame time [Number]

Returns:
    [_fuelCmd, _engineLoadShareTq, _orifice] - commanded fuel normalised, this engine's share
    of rotor demand in Nm, and the orifice as the governor has set it [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\engine\engine.hpp"

params ["_heli", "_index", "_engine", "_ng", "_np", "_tgt", "_lever", "_fat", "_deltaTime"];

private _idleNg = _engine get "idleNg";

//Single engine when one engine's torque is below 51% of another's.
private _tqs = _heli getVariable "bmkhs_engPctTq";
private _tqHigh = 0;
{ _tqHigh = _tqHigh max _x } forEach _tqs;
_heli setVariable ["bmkhs_isSingleEng", (_tqs findIf {_x < (_tqHigh * GT_SINGLE_ENG_TQ_RATIO)}) >= 0];

//Each detent's schedule carries its own torque - idle and fly are loaded points.
private _target = switch (_lever) do {
    case "FLY":  { _engine get "fuelFly" };
    case "IDLE": { _engine get "fuelIdle" };
    default      { 0.0 };
};

//Idle to fly travels over leverTravelTime; every other move of the lever is instant.
private _fuelIdle  = _engine get "fuelIdle";
private _fuelFly   = _engine get "fuelFly";
private _fuelSched = _heli getVariable "bmkhs_engLeverSched" select _index;
if (_lever == "FLY" && {_target > _fuelSched}) then {
    _fuelSched = ((_fuelSched max _fuelIdle) + (((_fuelFly - _fuelIdle) / (_engine get "leverTravelTime")) * _deltaTime)) min _target;
} else {
    _fuelSched = _target;
};
[_heli, "bmkhs_engLeverSched", _index, _fuelSched] call bmkhs_fnc_utilSetArrayVariable;

//Lose the ECU and nothing is metering fuel - the engine surges to maximum. Not a shutdown.
private _govPowered = true;
{
    private _ok = if (_x isEqualType []) then {
        ([_heli, _x select 0] call bmkhs_fnc_systemCircuit) >= (_x select 1)
    } else {
        _heli getVariable [_x, false]
    };
    if (!_ok) exitWith { _govPowered = false };
} forEach (_engine get "governorGates");

//The torque motor trims the orifice below the lever's to hold Np; it never opens it past.
private _pid     = _heli getVariable "bmkhs_pid_engine" select _index;
private _npRef   = _heli getVariable "bmkhs_engNpRef" select _index;
private _orifice = _fuelSched;
if (_lever == "FLY" && {_govPowered}) then {
    //Taking over, the governor starts from the fuel already flowing and from Np where it is.
    if (_npRef < 0.0) then {
        _pid set ["integral", _fuelSched / (_pid get "ki")];
        _pid set ["prevError", 0.0];
        _npRef = _np;
    };
    //The Np it holds follows the lever, reaching trim as the lever reaches fly.
    private _npTarget = _npRef + ((1.0 - _npRef) * linearConversion [_fuelIdle, _fuelFly, _fuelSched, 0.0, 1.0, true]);

    //Collective anticipation - from the collective the rotor gets, the pilot's and the holds'
    private _coll     = (((_heli getVariable "bmkhs_collectiveOutput") + (_heli getVariable ["bmkhs_fmcAltHoldCollOut", 0.0])) max 0.0) min 1.0;
    private _integral = _pid get "integral";
    private _govFuel  = ([_pid, _deltaTime, _npTarget, _np] call bmkhs_fnc_pidRun)
                      + (_coll * (_engine get "ffwdGain"));

    //TGT and Ng limiters - the fuel allowed rides above demand and is pulled down by whichever
    //is over its limit. Never below the idle floor.
    private _tgtLim = [_engine get "maxTgt", _engine get "maxTgtSe"] select (_heli getVariable "bmkhs_isSingleEng");
    private _ngLim  = (_engine get "ngLimitMax") min ((_engine get "ngLimitBase") + ((_engine get "ngLimitSlope") * _fat));
    private _err    = ((_tgtLim - _tgt) / GT_TGT_LIMIT_BAND) min ((_ngLim - _ng) / GT_NG_LIMIT_BAND);
    private _lim    = ((_heli getVariable "bmkhs_engLimFuel") select _index) + (GT_LIMIT_GAIN * _err * _deltaTime);
    _lim = _lim min ((_fuelSched min (_govFuel max 0.0)) * GT_LIMIT_TRACK);
    _lim = (_lim max _fuelIdle) min _fuelFly;
    [_heli, "bmkhs_engLimFuel", _index, _lim] call bmkhs_fnc_utilSetArrayVariable;
    private _allowed = _fuelSched min _lim;

    //Minimum flow - the governor never pulls Ng below idle. Rises as Ng falls under it, as the limiters do.
    private _minFuel = ((_heli getVariable "bmkhs_engMinFuel") select _index)
                     + (GT_LIMIT_GAIN * (((_engine get "idleNg") - _ng) / GT_NG_LIMIT_BAND) * _deltaTime);
    _minFuel = (_minFuel max 0.0) min _fuelIdle;
    [_heli, "bmkhs_engMinFuel", _index, _minFuel] call bmkhs_fnc_utilSetArrayVariable;
    if (_govFuel < _minFuel) then {
        _pid set ["integral", _integral];
        _govFuel = _minFuel;
    };

    //While the lever or a limiter is what limits fuel, the integral does not wind up.
    if (_govFuel > _allowed) then {
        _pid set ["integral", _integral];
    } else {
        //Load sharing: an engine below the average of those matched with it trims up. One above is
        //never trimmed down - it gives up load through its own Np governing as the other takes it.
        if ((_heli getVariable "bmkhs_engClutch") select _index) then {
            private _outTq   = _heli getVariable "bmkhs_engOutputTq";
            private _clutch  = _heli getVariable "bmkhs_engClutch";
            private _lvrs    = _heli getVariable "bmkhs_engPowerLeverState";
            private _shares  = [];
            {
                if ((_lvrs select _forEachIndex) == "FLY" && {_clutch select _forEachIndex}) then {
                    _shares pushBack ((_outTq select _forEachIndex) / (_x get "refTq"));
                };
            } forEach (_heli getVariable "bmkhs_engines");
            if (count _shares > 1) then {
                private _avg = 0.0;
                { _avg = _avg + _x } forEach _shares;
                private _mismatch = ((_avg / count _shares) - ((_outTq select _index) / (_engine get "refTq"))) max 0.0;
                private _clamp    = _pid get "ki_clamp";
                _pid set ["integral", ((_pid get "integral")
                    + ((_engine get "loadShareGain") * _mismatch * _deltaTime / (_pid get "ki"))) max -_clamp min _clamp];
            };
        };
    };
    _orifice = _allowed min (_govFuel max 0.0);
} else {
    [_pid] call bmkhs_fnc_pidReset;
    _npRef = -1.0;
    [_heli, "bmkhs_engLimFuel", _index, _fuelFly] call bmkhs_fnc_utilSetArrayVariable;
    [_heli, "bmkhs_engMinFuel", _index, 0.0] call bmkhs_fnc_utilSetArrayVariable;
};
[_heli, "bmkhs_engNpRef", _index, _npRef] call bmkhs_fnc_utilSetArrayVariable;

//Below idle Ng fuel is metered, which is what makes TGT peak above idle during a start and
//fall back as the compressor catches up. Above it this is a no-op.
private _fuelCmd = _orifice;
if (_ng < _idleNg) then {
    private _base = _engine get "startFuelBase";
    _fuelCmd = _fuelCmd * ((_base + ((1.0 - _base) * _ng / _idleNg)) min 1.0);
};

//reqEngTorque is indexed by ROTOR, so it is summed. Shared by capacity, and only among
//engines at FLY - one at idle drives nothing and takes no share of the load.
private _rotorTq = 0.0;
{ _rotorTq = _rotorTq + _x; } forEach (_heli getVariable "bmkhs_reqEngTorque");

private _lvrState      = _heli getVariable "bmkhs_engPowerLeverState";
private _totalEngineTq = 0.0;
{
    if ((_lvrState select _forEachIndex) == "FLY") then { _totalEngineTq = _totalEngineTq + (_x get "refTq") };
} forEach (_heli getVariable "bmkhs_engines");

private _engineLoadShareTq = if (_lever == "FLY" && {_totalEngineTq > 0.0}) then {
    _rotorTq * ((_engine get "refTq") / _totalEngineTq)
} else { 0.0 };

//TEMPORARY - remove when the zero torque output is found.
if (bmkhs_sysDebug) then {
    private _key  = format ["bmkhs_govDiagLast%1", _index];
    private _last = _heli getVariable [_key, 0];
    if (time > _last + 0.25) then {
        _heli setVariable [_key, time];
        diag_log text format [
            "GOVDIAG eng=%9 lvr=%1 sched=%2 orifice=%3 np=%4 powered=%5 cmd=%6 rotorTq=%7 share=%8",
            _lever, _fuelSched toFixed 4, _orifice toFixed 4, _np toFixed 4, _govPowered,
            _fuelCmd toFixed 4, _rotorTq toFixed 1, _engineLoadShareTq toFixed 1, _index + 1
        ];
    };
};

[_fuelCmd, _engineLoadShareTq, _orifice]
