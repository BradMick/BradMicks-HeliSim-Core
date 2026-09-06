#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli", "_engNum"];

private _continuosPower     = _heli getVariable "bmkhs_engContPwrKW";
private _contingencyPower   = _heli getVariable "bmkhs_engCntgncyPwrKW";
private _designRpm          = _heli getVariable "bmkhs_engDesignRPM";
private _engFriction        = _heli getVariable "bmkhs_engFriction";

private _npIdleRef          = _heli getVariable "bmkhs_engIdleNP";
private _npFlyRef           = _heli getVariable "bmkhs_engFlyNP";

private _deltaTime          = _heli getVariable "bmkhs_deltaTime";
private _rho                = _heli getVariable "bmkhs_RHO";
private _engState           = _heli getVariable "bmkhs_engState" select _engNum;
private _engPowerLeverState = _heli getVariable "bmkhs_engPowerLeverState" select _engNum;
private _xmsnRpm            = _heli getVariable "bmkhs_xmsnOutputRpm";
private _collectiveOutput   = _heli getVariable "bmkhs_collectiveOutput";
private _engPid             = _heli getVariable "bmkhs_pid_engine" select _engNum;
//_engPid set ["kp", E_KP];
//_engPid set ["ki", E_KI];
//_engPid set ["kd", E_KD];
private _rtrTqReq           = _heli getVariable "bmkhs_reqEngTorque" select 0;
private _engOverspeed       = _heli getVariable "bmkhs_engineOverspeed" select _engNum;
private _onGnd              = [_heli] call bmkhs_fnc_stateOnGround;
private _engPctNp           = _heli getVariable "bmkhs_engPctNP" select _engNum;

private _npTrimRef          = 0.0;
private _throttle           = 0.0;
if (_engPowerLeverState == "IDLE") then {
    _throttle  = 0.09;
    _npTrimRef = _npIdleRef;
};
if (_engPowerLeverState == "FLY") then {
    _throttle  = 0.18;
    _npTrimRef = _npFlyRef;
};
_npTrimRef     = _npTrimRef * _designRpm;

private _engLimitTQ  = 0.0;
private _isSingleEng = _heli getVariable "bmkhs_isSingleEng";
private _maxTQ_DE    = _heli getVariable "bmkhs_maxTQ_DE";
private _maxTQ_SE    = _heli getVariable "bmkhs_maxTQ_SE";

if (_isSingleEng) then {
    _engLimitTQ = _maxTQ_SE;
} else {
    _engLimitTQ = _maxTQ_DE;
};

private _engRefTq           = (_continuosPower * 1000) / 0.105 / _designRpm;
private _maxPowerInWatts    = (_continuosPower * 1000) * _engLimitTQ;
private _maxTorque          = _maxPowerInWatts / ((_designRpm * _npFlyRef) * 0.105);
private _trimTq             = 0.0;
private _tqOutput           = 0.0;
private _engPctTq           = 0.0;

if (_engState == "OFF") then {
    _engPctNp = [_engPctNp, 0.0, _deltaTime] call BIS_fnc_lerp;
};

if (_engState in ["STARTING", "ON"]) then {
    if (_engOverspeed) then {
        _engPctNp = [_engPctNp, 1.22, 1.5 * _deltaTime] call BIS_fnc_lerp;
        _engPctTq = _rtrTqReq/ _engRefTq;

        if (_engPctNP >= (_heli getVariable "bmkhs_engOvrspdNP")) then {
            _engState     = "OFF";
            [_heli, "bmkhs_engState", _engNum, _engState, true] call bmkhs_fnc_utilSetArrayVariable;
            [_heli, "bmkhs_engineOverspeed", _engNum, false, true] call bmkhs_fnc_utilSetArrayVariable;
        };
    } else {
        if (_xmsnRpm > _npTrimRef || _xmsnRpm < _npTrimRef) then {
            _trimTq = [_engPid, _deltaTime, _npTrimRef, _xmsnRpm] call bmkhs_fnc_pidRun;
            _trimTq = [_trimTq, _maxTorque * -0.5, _maxTorque * 0.5] call BIS_fnc_clamp;
        };
        _tqOutput = (_engRefTq * _throttle) + (_engRefTq * _collectiveOutput) + _trimTq;
        _tqOutput = [_tqOutput, 0.0, _maxTorque] call BIS_fnc_clamp;

        _engPctNp = _xmsnRpm / _designRpm;
        _engPctTq = _tqOutput/ _engRefTq;
    };
};

[_heli, "bmkhs_engOutputTq", _engNum, _tqOutput, true] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engPctNP",    _engNum, _engPctNp] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engPctTQ", _engNum, _engPctTq + ([_heli, _engNum] call bmkhs_fnc_systemTorqueJitter)] call bmkhs_fnc_utilSetArrayVariable;
//systemChat format ["_engRefTq = %1 -- _maxPowerInWatts = %2 -- _maxOutputTq = %3", _engRefTq, _maxPowerInWatts, _maxTorque];
//systemChat format ["Engine %3 -- _trimTq = %1 -- _tqOutput = %2 -- _maxTorque = %4", _trimTq toFixed 2, _tqOutput toFixed 2, _engNum, _maxTorque];
