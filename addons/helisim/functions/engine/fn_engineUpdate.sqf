/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engineUpdate

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
#include "\bmkhs_helisim\functions\engine\engine.hpp"

//Starts run off bleed air, whatever is supplying it. True by default so an airframe that
//models no pneumatics starts as before.
private _pneuAvail = _heli getVariable ["bmkhs_pneuAvail", true];

private _engState       = _heli getVariable "bmkhs_engState";
private _engPwrLvrState = _heli getVariable "bmkhs_engPowerLeverState";
private _allOff         = (_engState findIf {_x != "OFF"}) < 0;
private _rtrRPM         = _heli getVariable "bmkhs_rtrRpm";

private _shiftLocked = _heli getVariable "bmkhs_shiftLocked";
private _useSystems  = _heli getVariable ["bmkhs_useSystems", false];
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

    if (_allOff && _rtrRPM < 0.5) then {
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
        private _failed  = _heli getVariable "bmkhs_engFailed";
        {
            private _e   = _forEachIndex;
            private _st  = _x;
            private _sw  = _heli getVariable [format ["bmkhs_eng%1StartSwVal", _e + 1], 0];
            private _lvr = _heli getVariable [format ["bmkhs_eng%1PwrLvrVal",  _e + 1], 0];

            if (_sw > 0 && {_st == "OFF"} && {!(_failed select _e)}) then {
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
        } forEach _engState;

        //A running engine is a bleed air source. The aircraft gates its bleed component on
        //this; whether that matters is the airframe's business, not the engine's.
        [_heli, "bmkhs_engBleedAvail", "FLY" in (_heli getVariable "bmkhs_engPowerLeverState")]
            call bmkhs_fnc_utilUpdateNetworkGlobal;

        //With a start procedure, the engine runs when the procedure says so. Holding the
        //rotor off until then is what stops the player spinning it up with the throttle.
        if (([_heli, "mainRotor"] call bmkhs_fnc_damageGet) > 0.9) then {
            _heli engineOn false;
        } else {
            if (!_allOff || _rtrRPM >= 0.5) then {
                _heli engineOn true;
            } else {
                _heli engineOn false;
            };
        };
        if (_allOff && _rtrRPM < 0.1) then { //prevents player holding shift causing Rotor spinning
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
            private _failed = _heli getVariable "bmkhs_engFailed";
            {
                if (_x == "OFF" && {!(_failed select _forEachIndex)}) then {
                    [_heli, "bmkhs_engState", _forEachIndex, "STARTING", true] call bmkhs_fnc_utilSetArrayVariable;
                    [_heli, "bmkhs_engPowerLeverState", _forEachIndex, "IDLE", true] call bmkhs_fnc_utilSetArrayVariable;
                    [format ["eng%1PwrLvr", _forEachIndex + 1], 1, _heli] call bmkhs_fnc_controlSet;
                };
            } forEach _engState;

            //Started at IDLE; to FLY once Ng has held at idle.
            {
                private _e = _forEachIndex;
                if ((_engState select _e) == "ON" && {(_engPwrLvrState select _e) == "IDLE"}) then {
                    private _since = _heli getVariable "bmkhs_engIdleSince" select _e;
                    if ((_heli getVariable "bmkhs_engPctNg" select _e) < (GT_IDLE_STABLE_FRAC * (_x get "idleNg"))) then {
                        _since = -1;
                    } else {
                        if (_since < 0) then { _since = time };
                        if (time >= _since + GT_IDLE_TO_FLY_SEC) then {
                            [_heli, "bmkhs_engPowerLeverState", _e, "FLY", true] call bmkhs_fnc_utilSetArrayVariable;
                            [format ["eng%1PwrLvr", _e + 1], 2, _heli] call bmkhs_fnc_controlSet;
                            _since = -1;
                        };
                    };
                    [_heli, "bmkhs_engIdleSince", _e, _since] call bmkhs_fnc_utilSetArrayVariable;
                };
            } forEach (_heli getVariable "bmkhs_engines");
        } else {
            {
                if (_x != "OFF") then {
                    [_heli, "bmkhs_engState", _forEachIndex, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
                };
            } forEach _engState;
        };
        if (_shiftLocked) then {
            _heli setVariable ["bmkhs_shiftLocked", false];
            [_heli, "mainRotor", 0] call bmkhs_fnc_damageSet;
        };
    };
};


if !_pneuAvail then {
    {
        if (_x == "STARTING") then {
            [_heli, "bmkhs_engState", _forEachIndex, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
        };
    } forEach _engState;
};

if (isMultiplayer && (currentPilot _heli == player || local _heli) && (_heli getVariable "bmkhs_lastTimePropagated") + 0.1 < time) then {
    {
        _heli setVariable [_x, _heli getVariable _x, true];
    } forEach [
        "bmkhs_apuRpm_pct",
        "bmkhs_engFuelFlow",
        "bmkhs_engPctNg",
        "bmkhs_engPctNp",
        "bmkhs_engPctTq",
        "bmkhs_engTgt",
        "bmkhs_engOilPsi",
        "bmkhs_engLimitTimers",
        "bmkhs_engTqTimer",
        "bmkhs_engState",
        "bmkhs_collectiveOutput",
        "bmkhs_xmsnOutputRpm",
        "bmkhs_xmsnDeltaRpm"
    ];
    _heli setVariable ["bmkhs_lastTimePropagated", time, true];
};

[_heli, _heli getVariable "bmkhs_deltaTime"] call bmkhs_fnc_engineFuelAvail;

if (currentPilot _heli == player || local _heli) then {
    //Config names the assembly, so a new engine type is a folder and a config string.
    {
        private _fnc = missionNamespace getVariable [format ["bmkhs_fnc_%1", _x get "engineType"], {}];
        [_heli, _forEachIndex, _x] call _fnc;
    } forEach (_heli getVariable "bmkhs_engines");
};

if (local _heli) then {
    [_heli, _heli getVariable "bmkhs_deltaTime"] call bmkhs_fnc_engineDamage;
};

private _engFailed = _heli getVariable "bmkhs_engFailed";
private _fuelAvail = _heli getVariable "bmkhs_engFuelAvail";
private _overspeed = _heli getVariable "bmkhs_engineOverspeed";

{
    if (_x || {_overspeed select _forEachIndex} || {!(_fuelAvail select _forEachIndex)}) then {
        [_heli, "bmkhs_engState", _forEachIndex, "OFF", true] call bmkhs_fnc_utilSetArrayVariable;
    };
} forEach _engFailed;

//Oil pressure low - running or failed, the lever out of OFF, below the minimum. Latched until a repair.
if (local _heli) then {
    private _state = _heli getVariable "bmkhs_engState";
    private _lever = _heli getVariable "bmkhs_engPowerLeverState";
    private _oil   = _heli getVariable "bmkhs_engOilPsi";
    private _low   = _heli getVariable "bmkhs_engOilPsiLow";
    {
        private _isLow = ((_state select _forEachIndex) == "ON" || {_engFailed select _forEachIndex})
            && {(_lever select _forEachIndex) != "OFF"}
            && {(_oil select _forEachIndex) < ((_x get "oilPsiLimits") select 0)};
        if (_isLow && {!(_low select _forEachIndex)}) then {
            [_heli, "bmkhs_engOilPsiLow", _forEachIndex, true, true] call bmkhs_fnc_utilSetArrayVariable;
        };
    } forEach (_heli getVariable "bmkhs_engines");
};
