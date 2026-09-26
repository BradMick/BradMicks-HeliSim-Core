/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engineGovernor

Description:
    The ECU. Commands fuel flow; everything downstream responds to it.

    The power lever sets a minimum fuel SCHEDULE and the governor trims around
    that floor rather than replacing it - at governed speed with the collective
    down it commands nothing, so without a floor the engine cannot light.

    THE LIMITER IS PHYSICAL, NOT A RATING. Nothing clamps torque; torque is
    where the engine tops out once fuel stops going up, and it tops out lower on
    a hot, high day because thin air reaches the TGT limit at less fuel. The
    named tiers are for annunciation and damage, which is a later phase.

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
    [_fuelCmd, _myShare] - commanded fuel normalised, and this engine's share of
    rotor demand in Nm [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\engine\engine.hpp"

params ["_heli", "_index", "_engine", "_ng", "_tgt", "_lever", "_fat", "_deltaTime"];

private _idleNg = _engine get "idleNg";

//Each detent's floor CARRIES ITS OWN TORQUE - idle and fly are loaded points. The free
//turbine's share is taken out of the spool balance, so adding the load here double-counts
//it and the spool settles above its declared Ng.
private _fuelFloor = switch (_lever) do {
    case "FLY":  { _engine get "fuelFly" };
    case "IDLE": { _engine get "fuelIdle" };
    default      { 0.0 };
};

//Below idle Ng fuel is metered, which is what makes TGT peak above idle during a start and
//fall back as the compressor catches up. Above it this is a no-op.
if (_ng < _idleNg) then {
    private _base = _engine get "startFuelBase";
    _fuelFloor = _fuelFloor * ((_base + ((1.0 - _base) * _ng / _idleNg)) min 1.0);
};

private _fuelCmd = _fuelFloor;

//reqEngTorque is indexed by ROTOR, so it is summed. Shared by capacity.
private _rotorTq = 0.0;
{ _rotorTq = _rotorTq + _x; } forEach (_heli getVariable "bmkhs_reqEngTorque");

private _fleetRefTq = 0.0;
{ _fleetRefTq = _fleetRefTq + (_x get "refTq"); } forEach (_heli getVariable "bmkhs_engines");

private _myShare = if (_fleetRefTq > 0.0) then { _rotorTq * ((_engine get "refTq") / _fleetRefTq) } else { 0.0 };

//Trim on a NORMALISED Np fraction, never raw RPM - pidRun returns its gains times whatever
//it is fed, and an RPM-scaled result cannot be added to a 0-1 fuel command.
private _npFrac  = (_heli getVariable "bmkhs_xmsnOutputRpm") / ((_engine get "npFly") * (_engine get "designRpm"));
private _pid     = _heli getVariable "bmkhs_pid_engine" select _index;
private _govTrim = [_pid, _deltaTime, 1.0, _npFrac] call bmkhs_fnc_pidRun;

//Anticipates the load, so Nr does not droop before the governor has an error to react to.
private _ffwd = (_heli getVariable "bmkhs_collectiveOutput") * (_engine get "ffwdGain");

_fuelCmd = _fuelCmd + _ffwd + _govTrim;

//The two things the engine is built not to do: melt the hot section, or run the compressor
//tips through Mach. Each closes over a margin rather than switching at the line, so the
//limiter does not chatter once it is sitting on one. The floor is never limited away - a
//limited engine still idles.
private _tgtAuth = (((_engine get "maxTgt") - _tgt) / GT_TGT_LIMIT_BAND) min 1.0 max 0.0;

//Flat physical speed limit, or the sloped Mach limit where cold air brings it down.
private _ngLimit = (_engine get "maxNg")
                 min ((_engine get "ngLimitBase") + ((_engine get "ngLimitSlope") * _fat));
private _ngAuth  = ((_ngLimit - _ng) / GT_NG_LIMIT_BAND) min 1.0 max 0.0;

_fuelCmd = _fuelFloor + ((_fuelCmd - _fuelFloor) * (_tgtAuth min _ngAuth)) max 0.0;

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

[_fuelCmd, _myShare]
