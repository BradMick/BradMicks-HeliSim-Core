/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engineController

Description:
    Monitors and controls engine states.

Parameters:
    _heli      - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\systems\systems.hpp"

private _config         = configOf _heli >> "BMKHS_HeliSim";
private _configVehicles = configOf _heli;

//Starts run off bleed air, whatever is supplying it. True by default so an airframe that
//models no pneumatics starts as before.
private _pneuAvail = _heli getVariable ["bmkhs_pneuAvail", true];
private _onGnd     = [_heli] call bmkhs_fnc_stateOnGround;

private _engState  = _heli getVariable "bmkhs_engState";
private _eng1State = _engState select 0;
private _eng2State = _engState select 1;

private _engPwrLvrState  = _heli getVariable "bmkhs_engPowerLeverState";
private _eng1PwrLvrState = _engPwrLvrState select 0;
private _eng2PwrLvrState = _engPwrLvrState select 1;

private _eng1Np  = _heli getVariable "bmkhs_engPctNP" select 0;
private _eng2Np  = _heli getVariable "bmkhs_engPctNP" select 1;
private _rtrRPM  = _heli getVariable "bmkhs_rtrRPM";

private _eng1TQ   = _heli getVariable "bmkhs_engPctTQ" select 0;
private _eng2TQ   = _heli getVariable "bmkhs_engPctTQ" select 1;
private _engPctTQ = _eng1TQ max _eng2TQ;
private _eng1FuelAvail = _heli getVariable ["bmkhs_eng1FuelAvail", true];
private _eng2FuelAvail = _heli getVariable ["bmkhs_eng2FuelAvail", true];

private _shiftLocked = _heli getVariable "bmkhs_shiftLocked";
private _useSystems  = _heli getVariable ["bmkhs_useSystems", false];
private _isSingleEng     = _heli getVariable "bmkhs_isSingleEng";
//private _isAutorotating  = _heli getVariable "bmkhs_isAutorotating";


if (local _heli) then {

    /*
    if (([_heli, "mainRotor"] call bmkhs_fnc_damageGet) < 1.0) then {
        private _lastRtdUpdate = _heli getVariable ["bmkhs_lastUpdate", 0];
        if (cba_missionTime > _lastRtdUpdate + MIN_TIME_BETWEEN_UPDATES) then {
            private _realRPM = (_heli animationPhase "mainRotorRPM") * 1.08 / 10;
            if (_realRPM > _rtrRPM && _rtrRPM < 0.9) then {
                [_heli, "mainRotor", 0.9] call bmkhs_fnc_damageSet;
            } else {
                [_heli, "mainRotor", 0.0] call bmkhs_fnc_damageSet;
                _heli engineOn true;
            };
            _heli setVariable ["bmkhs_lastUpdate", cba_missionTime];
        };
    } else {
        _heli engineOn false;
    };

    if (_eng1State == "OFF" && _eng2State == "OFF" && _rtrRPM < 0.5) then {
        _heli engineOn false;
        [_heli, "mainRotor", 0.9] call bmkhs_fnc_damageSet;
    };
    */
    if (_useSystems) then {
        //The cockpit switches are LEVELS the engine reads, not commands pushed at it. A
        //start switch held at Start (+1) begins the sequence; ignition override (-1) aborts
        //it. The power lever's own value is what the throttle follows, and FLY against a set
        //rotor brake is refused here - the switch moves, the engine declines to drive it.
        private _brakeOn = (_heli getVariable ["bmkhs_rotorBrakeVal", 0]) > 0;
        {
            private _e   = _x;
            private _st  = _engState select _e;
            private _sw  = _heli getVariable [format ["bmkhs_eng%1StartSwVal", _e + 1], 0];
            private _lvr = _heli getVariable [format ["bmkhs_eng%1PwrLvrVal",  _e + 1], 0];

            if (_sw > 0 && {_st == "OFF"}) then {
                [_heli, "bmkhs_engState", _e, "STARTING", true] call bmkhs_fnc_utilSetArrayVariable;
                //A start begun with the brake set latches its caution off until the brake
                //comes off - a locked-rotor start is deliberate the whole way through.
                if (_brakeOn) then {
                    [_heli, "bmkhs_rtrBrkStartLatch", 1] call bmkhs_fnc_utilUpdateNetworkGlobal;
                };
            };
            if (_sw < 0 && {_st == "STARTING"}) then {
                [_heli, "bmkhs_engState", _e, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
            };

            private _want = switch (true) do {
                case (_lvr >= 1.0): {"FLY"};
                case (_lvr > 0.0):  {"IDLE"};
                default             {"OFF"};
            };
            if (_want != (_engPwrLvrState select _e)) then {
                [_heli, "bmkhs_engPowerLeverState", _e, _want, true] call bmkhs_fnc_utilSetArrayVariable;
                if (_want == "OFF" && {_st == "ON"}) then {
                    [_heli, "bmkhs_engState", _e, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
                };
            };
        } forEach [0, 1];

        //A running engine is a bleed air source. The aircraft gates its bleed component on
        //this; whether that matters is the airframe's business, not the engine's.
        private _lvrState = _heli getVariable "bmkhs_engPowerLeverState";
        [_heli, "bmkhs_engBleedAvail",
            (_lvrState select 0) == "FLY" || {(_lvrState select 1) == "FLY"}]
                call bmkhs_fnc_utilUpdateNetworkGlobal;

        //With a start procedure, the engine runs when the procedure says so. Holding the
        //rotor off until then is what stops the player spinning it up with the throttle.
        if (([_heli, "mainRotor"] call bmkhs_fnc_damageGet) > 0.9) then {
            _heli engineOn false;
        } else {
            if (_eng1State != "OFF" || _eng2State != "OFF" || _rtrRPM >= 0.5) then {
                _heli engineOn true;
            } else {
                _heli engineOn false;
            };
        };
        if (_eng1State == "OFF" && _eng2State == "OFF" && _rtrRPM < 0.1) then { //prevents player holding shift causing Rotor spinning
            _heli engineOn false;
            [_heli, "mainRotor", 0.9] call bmkhs_fnc_damageSet;
            if (!_shiftLocked) then {
                _heli setVariable ["bmkhs_shiftLocked", true];
            };
        } else {
            if (_shiftLocked) then {
                _heli setVariable ["bmkhs_shiftLocked", false];
                [_heli, "mainRotor", 0] call bmkhs_fnc_damageSet;
            };
        };
    } else {
        //No start procedure to wait for, so Arma's own start is the signal - the player
        //moves the throttle, the aircraft wakes up, and the engine state follows rather
        //than holding it off forever.
        //STARTING, not ON - the engine model spools Ng from there and flips itself to ON
        //at running speed. Setting ON directly skips the spool, which surges the rotor and
        //trips the engine-out warning against an Ng still climbing from zero.
        //Everything wakes together: buses, pressure and the engines. Nothing is simulated
        //without systems, so these are set once here rather than solved.
        private _awake = isEngineOn _heli;
        if ((_heli getVariable ["bmkhs_acBusOn", false]) isNotEqualTo _awake) then {
            [_heli, "bmkhs_acBusOn",    _awake] call bmkhs_fnc_utilUpdateNetworkGlobal;
            [_heli, "bmkhs_dcBusOn",    _awake] call bmkhs_fnc_utilUpdateNetworkGlobal;
            [_heli, "bmkhs_battBusOn",  _awake] call bmkhs_fnc_utilUpdateNetworkGlobal;
            [_heli, "bmkhs_priHydPsi",  [0.0, 3000.0] select _awake] call bmkhs_fnc_utilUpdateNetworkGlobal;
            [_heli, "bmkhs_utilHydPsi", [0.0, 3000.0] select _awake] call bmkhs_fnc_utilUpdateNetworkGlobal;
        };

        if (_awake) then {
            //Through the control so the lever animates over its normal travel - setting the
            //state directly snaps it, and the rotor surges with it.
            if (_eng1State == "OFF") then {
                [_heli, "bmkhs_engState", 0, "STARTING", true] call bmkhs_fnc_utilSetArrayVariable;
                [_heli, "bmkhs_engPowerLeverState", 0, "FLY", true] call bmkhs_fnc_utilSetArrayVariable;
                ["eng1PwrLvr", 2, _heli] call bmkhs_fnc_controlSet;
            };
            if (_eng2State == "OFF") then {
                [_heli, "bmkhs_engState", 1, "STARTING", true] call bmkhs_fnc_utilSetArrayVariable;
                [_heli, "bmkhs_engPowerLeverState", 1, "FLY", true] call bmkhs_fnc_utilSetArrayVariable;
                ["eng2PwrLvr", 2, _heli] call bmkhs_fnc_controlSet;
            };
        } else {
            if (_eng1State != "OFF") then {
                [_heli, "bmkhs_engState", 0, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
            };
            if (_eng2State != "OFF") then {
                [_heli, "bmkhs_engState", 1, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
            };
        };
        if (_shiftLocked) then {
            _heli setVariable ["bmkhs_shiftLocked", false];
            [_heli, "mainRotor", 0] call bmkhs_fnc_damageSet;
        };
    };
};


if !_pneuAvail then {
    if (_eng1State == "STARTING") then {
		[_heli, "bmkhs_engState", 0, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
    };
    if (_eng2State == "STARTING") then {
		[_heli, "bmkhs_engState", 1, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
    };
};

// Single engine when: one engine is OFF/damaged, or one power lever is at IDLE
// while the other is at FLY (lever-induced single engine operation).
private _eng1Active = (_eng1State in ["STARTING","ON"]) && (_eng1PwrLvrState == "FLY");
private _eng2Active = (_eng2State in ["STARTING","ON"]) && (_eng2PwrLvrState == "FLY");
_isSingleEng = !(_eng1Active && _eng2Active);
_heli setVariable ["bmkhs_isSingleEng", _isSingleEng];

if (isMultiplayer && (currentPilot _heli == player || local _heli) && (_heli getVariable "bmkhs_lastTimePropagated") + 0.1 < time) then {
    {
        _heli setVariable [_x, _heli getVariable _x, true];
    } forEach [
        "bmkhs_apuRPM_pct",
        "bmkhs_engFF",
        "bmkhs_engPctNG",
        "bmkhs_engPctNP",
        "bmkhs_engPctTQ",
        "bmkhs_engBaseTGT",
        "bmkhs_engTGT",
        "bmkhs_engBaseOilPSI",
        "bmkhs_engOilPSI",
        "bmkhs_engState",
        "bmkhs_engFF",
        "bmkhs_collectiveOutput",
        "bmkhs_xmsnOutputRpm",
        "bmkhs_xmsnDeltaRpm"
    ];
    _heli setVariable ["bmkhs_lastTimePropagated", time, true];
};

if (currentPilot _heli == player || local _heli) then {
    [_heli, 0] call bmkhs_fnc_engine;
    [_heli, 1] call bmkhs_fnc_engine;

    if (bmkhs_rotorModel == 1) then {
        [_heli, 0] call bmkhs_fnc_engineBET;
        [_heli, 1] call bmkhs_fnc_engineBET;
    } else {
        [_heli, 0] call bmkhs_fnc_engine2;
        [_heli, 1] call bmkhs_fnc_engine2;
    };
};

private _no1EngDmg = [_heli, "engines", 0] call bmkhs_fnc_damageGet;
private _no2EngDmg = [_heli, "engines", 1] call bmkhs_fnc_damageGet;

if (_no1EngDmg > SYS_ENG_DMG_THRESH || !_eng1FuelAvail) then {
	[_heli, "bmkhs_engState", 0, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
};

if (_no2EngDmg > SYS_ENG_DMG_THRESH || !_eng2FuelAvail) then {
	[_heli, "bmkhs_engState", 1, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
};

//Autorotation handler
/*
private _velXY = vectorMagnitude [velocityModelSpace _heli # 0, velocityModelSpace _heli # 1];
if (   _engPctTQ < 0.10
    && !_onGnd
    && _rtrRPM > EPSILON) then {
    _heli setVariable ["bmkhs_isAutorotating", true];
} else {
    _heli setVariable ["bmkhs_isAutorotating", false];
};
*/
//systemChat format ["_isAutorotating = %1", _heli getVariable "bmkhs_isAutorotating"];
//End Autorotation handler
