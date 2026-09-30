/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_turboShaftEngine

Description:
    One turboshaft engine, one frame. Wires the modules together and owns the
    call order: governor -> starter -> cold section -> hot section -> power
    turbine. Config names this assembly, so a new engine type is a folder and a
    config string rather than a switch in the controller.

    Every state flag comes from the Ng at the TOP of the frame, before anything
    updates it.

Parameters:
    _heli   - The helicopter [Object]
    _index  - Which engine [Number]
    _engine - That engine's config [HashMap]

Returns:
    Nothing. Publishes this engine's slot in the bmkhs_eng* arrays.

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\engine\engine.hpp"

params ["_heli", "_index", "_engine"];

private _deltaTime = _heli getVariable "bmkhs_deltaTime";
private _fat       = _heli getVariable "bmkhs_FAT";
private _dens      = (_heli getVariable "bmkhs_rho") / ISA_STD_DAY_AIR_DENSITY;
private _velY      = (_heli getVariable "bmkhs_velModelSpace") select 1;

private _ng           = _heli getVariable "bmkhs_engPctNg"        select _index;
private _np           = _heli getVariable "bmkhs_engNp"           select _index;
private _tgt          = _heli getVariable "bmkhs_engTgt"          select _index;
private _residualHeat = _heli getVariable "bmkhs_engResidualHeat" select _index;

private _lever = _heli getVariable "bmkhs_engPowerLeverState" select _index;
private _sw    = _heli getVariable [format ["bmkhs_eng%1StartSwVal", _index + 1], 0];

//The lever position is when the pilot commits, so this samples the TGT they saw. It fades
//out on Ng in the hot section, because the spool rising IS the hot section purging.
private _prevLever = _heli getVariable "bmkhs_engPrevLever" select _index;
if (_prevLever == "OFF" && {_lever != "OFF"}) then {
    _residualHeat = 1.0 + ((_engine get "residualHeatGain") * _tgt);
    [_heli, "bmkhs_engResidualHeat", _index, _residualHeat] call bmkhs_fnc_utilSetArrayVariable;
};
if (_prevLever != _lever) then {
    [_heli, "bmkhs_engPrevLever", _index, _lever] call bmkhs_fnc_utilSetArrayVariable;
};

//Ng fly weights and the Np electrical trip. Both cut fuel; a repair is what resets them.
private _tripped = _heli getVariable "bmkhs_engineOverspeed" select _index;
if (!_tripped && {_ng >= (_engine get "maxNg") || {_np >= (_engine get "maxNp")}}) then {
    _tripped = true;
    [_heli, "bmkhs_engineOverspeed", _index, true, true] call bmkhs_fnc_utilSetArrayVariable;
};

//The lever check is required - without it fuel keeps burning after shutdown.
private _fuelAvail = _heli getVariable "bmkhs_engFuelAvail" select _index;
private _failed    = _heli getVariable "bmkhs_engFailed" select _index;
private _running   = _ng > (_engine get "lightOffNg") && {_lever != "OFF"} && {_fuelAvail} && {!_tripped} && {!_failed};

private _starterTq = [_heli, _index, _engine, _ng] call bmkhs_fnc_gasTurbineStarter;
private _cranking  = _starterTq > 0.0;
private _spooling  = !_running && {!_cranking};

([_heli, _index, _engine, _ng, _np, _tgt, _lever, _fat, _deltaTime] call bmkhs_fnc_engineGovernor)
    params ["_fuelCmd", "_engineLoadShareTq"];

private _refTq = _engine get "refTq";

([_engine, _ng, _fuelCmd, _starterTq, _dens, _running, _spooling, _deltaTime]
    call bmkhs_fnc_gasTurbineColdSection) params ["_ngNew", "_gasPower", "_compWork", "_airGas"];

_tgt = [_engine, _tgt, _ngNew, _fuelCmd, _residualHeat, _dens, _fat, _velY, _running, _deltaTime]
        call bmkhs_fnc_gasTurbineHotSection;

//The rotor, as a fraction of governed Np - the only place real rpm meets the engine.
private _xmsnRpm = _heli getVariable "bmkhs_xmsnOutputRpm";
private _nrFrac  = _xmsnRpm / ((_engine get "npFly") * (_engine get "designRpm"));

([_engine, _gasPower, _airGas, _compWork, _np, _nrFrac, _deltaTime]
    call bmkhs_fnc_turboShaftPowerTurbine) params ["_tqOut", "_npNew", "_clutch"];

//State follows Ng, so a start that hangs never reads ON and a flameout drops out of it.
private _state = switch (true) do {
    case (_ngNew >= (_engine get "selfSustNg") && _running): { "ON" };
    case (_cranking || {_running}):                          { "STARTING" };
    default                                                  { "OFF" };
};

//TEMPORARY - remove when the zero torque output is found.
if (bmkhs_sysDebug) then {
    private _key   = format ["bmkhs_gtDiagLast%1", _index];
    private _swKey = format ["bmkhs_gtDiagSw%1", _index];
    //A switch press is shorter than the log interval, so the last one seen is held until printed.
    if (_sw != 0) then { _heli setVariable [_swKey, _sw] };
    private _last = _heli getVariable [_key, 0];
    if (time > _last + 0.25) then {
        _heli setVariable [_key, time];
        private _swSeen = _heli getVariable [_swKey, 0];
        _heli setVariable [_swKey, 0];
        diag_log text format [
            "GTDIAG eng=%25 t=%1 lvr=%2 sw=%3 swSeen=%26 latch=%27 fuel=%4 run=%5 crank=%6 spool=%7 ng=%8->%9 np=%10->%11 clutch=%12 nrFrac=%13 gas=%14 comp=%15 air=%16 share=%17 tq=%18 Nr=%19 tgt=%20 state=%21 dt=%22 trip=%23 rtrMdl=%24",
            time toFixed 2, _lever, _sw, _fuelCmd toFixed 4,
            _running, _cranking, _spooling,
            _ng toFixed 4, _ngNew toFixed 4,
            _np toFixed 4, _npNew toFixed 4, _clutch, _nrFrac toFixed 4,
            _gasPower toFixed 4, _compWork toFixed 4,
            _airGas toFixed 4, _engineLoadShareTq toFixed 1,
            _tqOut toFixed 1,
            _xmsnRpm toFixed 0,
            _tgt toFixed 0, _state, _deltaTime toFixed 4,
            _tripped, bmkhs_rotorModel, _index + 1,
            _swSeen, _heli getVariable "bmkhs_engState" select _index
        ];
    };
};

//A slipping clutch passes less, and grabs back - what the rotor gets and the gauge reads.
_tqOut = _tqOut * ((_heli getVariable "bmkhs_engClutchSlip") select _index);

[_heli, "bmkhs_engPctNg",    _index, _ngNew] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engTgt",      _index, _tgt] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engOutputTq", _index, _tqOut] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engPctTq",    _index, _tqOut / _refTq] call bmkhs_fnc_utilSetArrayVariable;

[_heli, "bmkhs_engNp",     _index, _npNew] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engClutch", _index, _clutch] call bmkhs_fnc_utilSetArrayVariable;

//The gauge reads Np against designRpm, so governed Np shows 101% where npFly is 1.01.
[_heli, "bmkhs_engPctNp", _index, _npNew * (_engine get "npFly")] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engFuelFlow",    _index, _fuelCmd * (_engine get "maxFuelFlow")] call bmkhs_fnc_utilSetArrayVariable;

//The pump is on the gas generator shaft, so pressure rises during a start before the engine
//makes any torque - which is why this comes from Ng and not from torque.
[_heli, "bmkhs_engOilPsi", _index,
    (_ngNew * GT_OIL_PSI_SCALE * (_heli getVariable "bmkhs_engOilHealth" select _index)) max 0.0]
        call bmkhs_fnc_utilSetArrayVariable;
