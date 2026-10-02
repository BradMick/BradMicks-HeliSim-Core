/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_gasTurbineStarter

Description:
    Air turbine or electric motor, per the engine's class Starter. Puts torque
    on the spool while engaged and supplied, and disengages at selfSustNg.

    It cranks for a START or for a held IGNITION OVERRIDE. The override is what
    motors the spool with no fuel, which is the cooling path the hot-start abort
    depends on.

Parameters:
    _heli   - The helicopter [Object]
    _index  - Which engine [Number]
    _engine - That engine's config [HashMap]
    _ng     - Spool speed at the top of the frame [Number]

Returns:
    Torque on the spool, normalised [Number]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_index", "_engine", "_ng"];

private _sw = _heli getVariable [format ["bmkhs_eng%1StartSwVal", _index + 1], 0];

//An aircraft declaring no controls reads 0, so engState is what says a start is running.
private _starting = _sw > 0 || {(_heli getVariable "bmkhs_engState" select _index) == "STARTING"};
private _override = _sw < 0;

//Self-sustaining ends the start - the starter cuts out and the engine is ON.
if (_ng >= (_engine get "selfSustNg")) exitWith {
    if ((_heli getVariable "bmkhs_engState" select _index) == "STARTING") then {
        [_heli, "bmkhs_engState", _index, "ON", true] call bmkhs_fnc_utilSetArrayVariable;
    };
    0.0
};
if (!_starting && {!_override}) exitWith { 0.0 };
//A tripped engine is locked out - the starter will not turn it until a repair resets it.
if ((_heli getVariable "bmkhs_engineOverspeed") select _index) exitWith { 0.0 };
if ((_heli getVariable "bmkhs_engFailed") select _index) exitWith { 0.0 };

//No gate means always supplied; every gate declared has to be on.
private _supplied = true;
{
    private _ok = if (_x isEqualType []) then {
        ([_heli, _x select 0] call bmkhs_fnc_systemCircuit) >= (_x select 1)
    } else {
        _heli getVariable [_x, false]
    };
    if (!_ok) exitWith { _supplied = false };
} forEach (_engine get "starterGates");

//An air turbine - full torque stalled, none at its runaway speed, which is what motoring settles at.
[0.0, (_engine get "starterTorque") * ((1.0 - (_ng / (_engine get "runawayNg"))) max 0.0)] select _supplied
