params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

if (currentPilot _heli != player || !local _heli) exitWith {};

private _mainRotorGearRatio = ((_heli getVariable "bmkhs_simpleRotors") select 0) get "gearRatio";

//Every rotor is on the same drivetrain, each referred through its OWN gear ratio.
private _rtrMoi = _heli getVariable "bmkhs_rtrMoi";
private _jEng   = 0.0;
{
    private _gr = ((_heli getVariable "bmkhs_simpleRotors") select _forEachIndex) get "gearRatio";
    if (_gr > 0.0) then { _jEng = _jEng + (_x / (_gr * _gr)) };
} forEach _rtrMoi;
private _outputRpm          = _heli getVariable "bmkhs_xmsnOutputRpm";
private _deltaRpm           = _heli getVariable "bmkhs_xmsnDeltaRpm";

private _totTq   = 0.0;
private _rotorTq = _heli getVariable "bmkhs_reqEngTorque";
{
    _totTq = _totTq + _x;
} forEach _rotorTq;

//Which model's torque turns the rotor.
private _engOutputTq = _heli getVariable (["bmkhs_engOutputTq", "bmkhs_gtEngOutputTq"] select bmkhs_engineModel);
private _engInputTq  = 0.0;
if (bmkhs_engineModel == 1) then {
    //Only an engaged freewheel drives the rotor - a decoupled turbine turns nothing.
    private _clutch = _heli getVariable "bmkhs_gtEngClutch";
    {
        if (_clutch select _forEachIndex) then { _engInputTq = _engInputTq + _x };
    } forEach _engOutputTq;
} else {
    {
        _engInputTq = _engInputTq + _x;
    } forEach _engOutputTq;
};

// Lumped at the engine shaft, dimensionally correct and framerate-independent:
//   alpha = tau_net / J_eng           [rad/s^2]
//   J_eng = J_rotor / GR^2            (rotor inertia referred to engine shaft)
//   d(rpm) = alpha * dt * (60 / 2pi)  (rad/s -> rpm over the frame)
private _deltaTime = _heli getVariable "bmkhs_deltaTime";
private _alpha     = if (_jEng == 0.0) then { 0.0 } else { (_engInputTq - _totTq) / _jEng };
_deltaRpm          = _alpha * _deltaTime * (60.0 / (2.0 * pi));

if (_outputRpm < 0.0) then {
    _outputRpm = 0.0;
} else {
    _outputRpm = _outputRpm + _deltaRpm;
};

//The rotor brake. It will not act above ROTOR_BRAKE_MAX_NR - the switch moves and the crew
//gets the caution, but the system refuses to apply it, so there is nothing to damage. Below
//that, BRAKE drags the rotor down over ROTOR_BRAKE_STOP_SEC and LOCK holds it at zero, which
//is what makes a locked-rotor start work: Nr reads 0 while the engines run and Np is normal.
private _brakePos  = _heli getVariable ["bmkhs_rotorBrakeVal", 0];
private _designRpm = _heli getVariable ["bmkhs_engDesignRPM", 20900];

//The brake coming off is the only thing that clears the locked-rotor start latch.
if (_brakePos == 0) then {
    [_heli, "bmkhs_rtrBrkStartLatch", 0] call bmkhs_fnc_utilUpdateNetworkGlobal;
};
if (_brakePos >= 1 && {_outputRpm < (_designRpm * ROTOR_BRAKE_MAX_NR)}) then {
    if (_brakePos >= 2) then {
        _outputRpm = 0.0;
        _deltaRpm  = 0.0;
    } else {
        private _deltaTime = _heli getVariable "bmkhs_deltaTime";
        _outputRpm = (_outputRpm - ((_designRpm / ROTOR_BRAKE_STOP_SEC) * _deltaTime)) max 0.0;
    };
};

//systemChat format ["_outputRpm = %1", _outputRpm];

_heli setVariable ["bmkhs_xmsnOutputRpm", _outputRpm];
_heli setVariable ["bmkhs_xmsnDeltaRpm",  _deltaRpm];

//TEMPORARY - remove when the zero torque output is found.
if (bmkhs_sysDebug) then {
    private _last = _heli getVariable ["bmkhs_xmsnDiagLast", 0];
    if (time > _last + 0.25) then {
        _heli setVariable ["bmkhs_xmsnDiagLast", time];
        diag_log text format [
            "XMDIAG t=%1 model=%2 engIn=%3 rotorReq=%4 net=%5 jEng=%6 gr=%7 dRpm=%8 rpm=%9 brake=%10 clutch=%11 np=%12",
            time toFixed 2, bmkhs_engineModel,
            _engInputTq toFixed 1, _totTq toFixed 1, (_engInputTq - _totTq) toFixed 1,
            _jEng toFixed 3, _mainRotorGearRatio toFixed 3,
            _deltaRpm toFixed 2, _outputRpm toFixed 0, _brakePos,
            _heli getVariable "bmkhs_gtEngClutch", _heli getVariable "bmkhs_gtEngNp"
        ];
    };
};

//systemChat format ["_totTq = %1 -- _engInputTq = %2 -- _deltaRpm = %3", _totTq, _engInputTq, _deltaRpm];
//systemChat format ["_outputRpm = %1 -- _deltaRpm = %2", _outputRpm, _deltaRpm];
