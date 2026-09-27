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

//Each detent's schedule carries its own torque - idle and fly are loaded points.
private _target = switch (_lever) do {
    case "FLY":  { _engine get "fuelFly" };
    case "IDLE": { _engine get "fuelIdle" };
    default      { 0.0 };
};

//The lever travels to fly over GT_LEVER_TRAVEL_SEC and snaps back, so the schedule builds
//over the push rather than stepping with the detent.
private _fuelSched = _heli getVariable "bmkhs_gtEngLeverSched" select _index;
if (_target > _fuelSched) then {
    _fuelSched = (_fuelSched + (((_engine get "fuelFly") / GT_LEVER_TRAVEL_SEC) * _deltaTime)) min _target;
} else {
    _fuelSched = _target;
};
[_heli, "bmkhs_gtEngLeverSched", _index, _fuelSched] call bmkhs_fnc_utilSetArrayVariable;

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
private _pid     = _heli getVariable "bmkhs_gtPidEngine" select _index;
private _orifice = _fuelSched;
if (_lever == "FLY" && {_govPowered}) then {
    private _govFuel = ([_pid, _deltaTime, 1.0, _np] call bmkhs_fnc_pidRun)
                     + ((_heli getVariable "bmkhs_collectiveOutput") * (_engine get "ffwdGain"));
    _orifice = _fuelSched min (_govFuel max 0.0);
} else {
    [_pid] call bmkhs_fnc_pidReset;
};

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
