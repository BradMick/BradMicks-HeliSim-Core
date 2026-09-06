#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\systems\systems.hpp"

params ["_heli", "_engNum"];

private _cfg           = configOf _heli;
private _sfmPlusConfig = _cfg >> "BMKHS_HeliSim";

// ── Physical constants ────────────────────────────────────────────────────────
private _continuousPower  = _heli getVariable "bmkhs_engContPwrKW";
private _designRpm        = _heli getVariable "bmkhs_engDesignRPM";
private _npFlyRef         = _heli getVariable "bmkhs_engFlyNP";
private _npIdleRef        = _heli getVariable "bmkhs_engIdleNP";
private _ngFlyRef         = _heli getVariable "bmkhs_engFlyNG";
private _ngIdleRef        = _heli getVariable "bmkhs_engIdleNG";

// Reference torque at 100% Np: Q = P / omega
private _engRefTq = (_continuousPower * 1000) / (_designRpm * _npFlyRef * 0.10472);

// ── Frame state ───────────────────────────────────────────────────────────────
private _deltaTime          = _heli getVariable "bmkhs_deltaTime";
private _engState           = _heli getVariable "bmkhs_engState"          select _engNum;
private _engPowerLeverState = _heli getVariable "bmkhs_engPowerLeverState" select _engNum;
private _engPctNG           = _heli getVariable "bmkhs_engPctNG"          select _engNum;
private _engPctNP           = _heli getVariable "bmkhs_engPctNP"          select _engNum;
private _engPctTQ           = _heli getVariable "bmkhs_engPctTQ"          select _engNum;
private _engPid             = _heli getVariable "bmkhs_pid_engine"        select _engNum;
private _engOverspeed       = _heli getVariable "bmkhs_engineOverspeed"      select _engNum;
private _isSingleEng        = _heli getVariable "bmkhs_isSingleEng";
private _xmsnRpm            = _heli getVariable "bmkhs_xmsnOutputRpm";
private _collectiveOutput   = _heli getVariable "bmkhs_collectiveOutput";

// ── Torque limits ─────────────────────────────────────────────────────────────
private _maxTQ_DE   = _heli getVariable "bmkhs_maxTQ_DE";
private _maxTQ_SE   = _heli getVariable "bmkhs_maxTQ_SE";
private _engLimitTQ = [_maxTQ_DE, _maxTQ_SE] select (_isSingleEng);

// ── Outputs ───────────────────────────────────────────────────────────────────
private _tqOutput = 0.0;
private _engFF    = _heli getVariable "bmkhs_engFF" select _engNum;

if (_engState in ["STARTING", "ON"]) then {

    if (_engPowerLeverState == "FLY") then {

        if (_engOverspeed) then {
            _engPctNP = [_engPctNP, 1.22, 1.5 * _deltaTime] call BIS_fnc_lerp;
            _tqOutput = 0.0;
            _engPctTQ = _tqOutput / _engRefTq;

            if (_engPctNP >= (_heli getVariable "bmkhs_engOvrspdNP")) then {
                [_heli, "bmkhs_engState",     _engNum, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
                [_heli, "bmkhs_engineOverspeed", _engNum, false, true] call bmkhs_fnc_utilSetArrayVariable;
            };
        } else {
            // ── Available shaft power from Ng ─────────────────────────────────
            private _ngPowerFraction = [_engPctNG / _ngFlyRef, 0.0, 1.0] call BIS_fnc_clamp;
            private _availablePowerW = _continuousPower * 1000 * _ngPowerFraction;
            private _omegaGov        = _npFlyRef * _designRpm * 0.10472;
            private _availTq         = _availablePowerW / _omegaGov;

            // ── Load feed-forward — this engine's share of rotor demand ───────
            // The engine's steady-state torque target is the rotor's actual power
            // demand referred to the engine shaft (already gently smoothed in
            // fn_rotor). This is the baseline the governor trims around so NR
            // holds reference with no standing offset.
            private _rotorTqReq = 0.0;
            { _rotorTqReq = _rotorTqReq + _x; } forEach (_heli getVariable "bmkhs_reqEngTorque");
            private _numActive  = [2, 1] select (_isSingleEng);
            private _myShareTq  = _rotorTqReq / _numActive;
            //DIAG (torque scale): OGE hover should read ~84% per engine; user sees ~49%. Print the
            //raw rotor demand + this engine's share + refTq so we see the true factor. Remove when done.
            if (_engNum == 0) then {
                systemChat format ["ENG rotorTqReq=%1 share=%2 refTq=%3 pct=%4", _rotorTqReq toFixed 0, _myShareTq toFixed 0, _engRefTq toFixed 0, (_myShareTq / _engRefTq) toFixed 3];
            };

            // ── Isochronous governor trim ─────────────────────────────────────
            // Proportional trim on normalised Np error. The feed-forward carries
            // the bulk of the load; this corrects the residual error that the
            // spool lag leaves behind.
            private _npFrac  = _xmsnRpm / (_npFlyRef * _designRpm);  // 1.0 = on-speed
            private _npErr   = 1.0 - _npFrac;                         // positive = underspeed
            private _govGain = _heli getVariable "bmkhs_engGovGain";
            private _govCorr = [_npErr * _govGain, -0.5, 0.5] call BIS_fnc_clamp;

            // ── Engine spool dynamics ─────────────────────────────────────────
            // A real turboshaft cannot deliver new power instantly — Ng must
            // spool. We model that by lagging the ENTIRE torque demand (load
            // feed-forward + governor trim) through a first-order spool filter.
            //
            // This lag is what produces realistic NR behaviour: when cyclic tilts
            // the disc and rotor demand spikes, the engine lags behind for ~0.5 s,
            // so NR droops a percent or so, the governor adds trim, and NR
            // recovers. It also tames the torque response — the raw BET demand
            // spike from a cyclic input is filtered instead of hitting the
            // transmission at full magnitude.
            private _tqTarget = _myShareTq + (_availTq * _govCorr);
            _tqTarget = [_tqTarget, 0.0, _engRefTq * _engLimitTQ] call BIS_fnc_clamp;

            private _tqPrev  = _heli getVariable "bmkhs_engOutputTq" select _engNum;
            private _tqAlpha = 1.0 - exp (-_deltaTime / 0.5);
            _tqOutput = _tqPrev + (_tqTarget - _tqPrev) * _tqAlpha;
            _tqOutput = [_tqOutput, 0.0, _engRefTq * _engLimitTQ] call BIS_fnc_clamp;

            // ── Fuel flow schedule (display) ──────────────────────────────────
            private _wfDemand = if (_availTq > 0.0) then { [_tqOutput / _availTq, 0.0, 1.0] call BIS_fnc_clamp } else { 0.0 };
            _engFF = [getArray (_sfmPlusConfig >> "engFFTable"), _wfDemand] call bmkhs_fnc_mathLinearInterp select 1;

            _engPctNP = _xmsnRpm / _designRpm;
            _engPctTQ = _tqOutput / _engRefTq;
        };

    } else {
        // IDLE: engine runs at idle Ng/Np, minimal torque output
        private _engIdleTQ = getNumber (_sfmPlusConfig >> "engIdleTQ");
        private _npTrimRef = _npIdleRef * _designRpm;
        private _govTrim   = [_engPid, _deltaTime, _npTrimRef, _xmsnRpm] call bmkhs_fnc_pidRun;
        _govTrim   = [_govTrim, 0.0, _engRefTq * 0.15] call BIS_fnc_clamp;
        _tqOutput  = _govTrim;
        _engFF     = [getArray (_sfmPlusConfig >> "engFFTable"), _engIdleTQ] call bmkhs_fnc_mathLinearInterp select 1;

        _engPctNP = _xmsnRpm / _designRpm;
        _engPctTQ = _tqOutput / _engRefTq;
    };

} else {
    _engPctNP = [_engPctNP, 0.0, _deltaTime] call BIS_fnc_lerp;
    _tqOutput = 0.0;
    _engPctTQ = 0.0;
    _engFF    = 0.0;
};

// ── Write outputs ─────────────────────────────────────────────────────────────
[_heli, "bmkhs_engOutputTq", _engNum, _tqOutput,  true] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engPctNP",    _engNum, _engPctNP       ] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engFF",       _engNum, _engFF          ] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engPctTQ",    _engNum, _engPctTQ + ([_heli, _engNum] call bmkhs_fnc_systemTorqueJitter)] call bmkhs_fnc_utilSetArrayVariable;
