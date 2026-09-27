/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engineGovernor

Description:
    The ECU. Commands fuel flow from the lever's schedule and the physical
    limiter. The Np trim and collective feed-forward are removed pending rework.

Parameters:
    _heli      - The helicopter [Object]
    _index     - Which engine [Number]
    _engine    - That engine's config [HashMap]
    _ng        - Spool speed at the top of the frame [Number]
    _tgt       - TGT at the top of the frame, deg C [Number]
    _lever     - Power lever position, OFF / IDLE / FLY [String]
    _fat       - Free air temperature, deg C [Number]
    _deltaTime - Frame time [Number]

Returns:
    [_fuelCmd, _engineLoadShareTq] - commanded fuel normalised, and this engine's share of
    rotor demand in Nm [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\engine\engine.hpp"

params ["_heli", "_index", "_engine", "_ng", "_tgt", "_lever", "_fat", "_deltaTime"];

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

//Below idle Ng fuel is metered, which is what makes TGT peak above idle during a start and
//fall back as the compressor catches up. Above it this is a no-op.
private _fuelCmd = _fuelSched;
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

//Nothing restricts fuel here. The lever sets a physical orifice, and an unregulated engine
//runs away until a hard shutdown trips. The governor is what will meter it.

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

if (!_govPowered) then { _fuelCmd = 1.0; };

//TEMPORARY - remove when the zero torque output is found.
if (bmkhs_sysDebug && {_index == 0}) then {
    private _last = _heli getVariable ["bmkhs_govDiagLast", 0];
    if (time > _last + 0.25) then {
        _heli setVariable ["bmkhs_govDiagLast", time];
        diag_log text format [
            "GOVDIAG lvr=%1 sched=%2 idleNg=%3 tgtAuth=%4 ngAuth=%5 powered=%6 cmd=%7 rotorTq=%8 share=%9",
            _lever, _fuelSched toFixed 4, _idleNg toFixed 4,
            _tgtAuth toFixed 4, _ngAuth toFixed 4, _govPowered,
            _fuelCmd toFixed 4, _rotorTq toFixed 1, _engineLoadShareTq toFixed 1
        ];
    };
};

[_fuelCmd, _engineLoadShareTq]
