/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engineVariables

Description:
    Defines core engine variables.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

if (!(_heli getVariable ["bmkhs_engineInitialised", false]) && local _heli) then {
    _heli setVariable ["bmkhs_engineInitialised", true, true];

    //Everything starts OFF, with or without modelled systems. Without them there is no
    //start PROCEDURE - no battery, APU or generators to sequence - but Arma's own startup
    //still runs, and driving it from OFF is what spins the rotor up over that window.
    //The graph then follows: Nr turns the transmission, which drives the accessories,
    //which build pressure. Seeding it already-running skipped all of that and left the
    //values reading as failures until the model caught up.
    //Cold and dark either way: the levers start OFF. With no systems the controller moves
    //them to FLY when the player wakes the aircraft, through the same path a click takes
    //so the animation travels rather than snapping.
    private _lever = "OFF";

    _heli setVariable ["bmkhs_engPowerLeverState",    [_lever, _lever], true]; //OFF, IDLE, FLY
    _heli setVariable ["bmkhs_engState",              ["OFF", "OFF"],   true]; //OFF, STARTING, ON
};

if(isMultiplayer) then {
    _heli setVariable ["bmkhs_lastTimePropagated", 0];
};

//CONFIG - what the aircraft IS.
//Engine - power, governing and limits
_heli setVariable ["bmkhs_engContPwrKW",    getNumber (_config >> "engContPwrKW")];
_heli setVariable ["bmkhs_engCntgncyPwrKW", getNumber (_config >> "engCntgncyPwrKW")];
_heli setVariable ["bmkhs_engDesignRPM",    getNumber (_config >> "engDesignRPM")];
_heli setVariable ["bmkhs_engFriction",     getNumber (_config >> "engFriction")];
_heli setVariable ["bmkhs_engGovGain",      getNumber (_config >> "engGovGain")];
_heli setVariable ["bmkhs_engRunNG",        getNumber (_config >> "engRunNG")];
_heli setVariable ["bmkhs_engMaxTGT_DE",    getNumber (_config >> "engMaxTGT_DE")];
_heli setVariable ["bmkhs_engMaxTGT_SE",    getNumber (_config >> "engMaxTGT_SE")];
//Np/Ng references already exist as engIdleNP/engFlyNP/engOvrspdNP/engIdleNG/engFlyNG
_heli setVariable ["bmkhs_engIdleNP",       getNumber (_config >> "engIdleNP")];
_heli setVariable ["bmkhs_engFlyNP",        getNumber (_config >> "engFlyNP")];
_heli setVariable ["bmkhs_engOvrspdNP",     getNumber (_config >> "engOvrspdNP")];
_heli setVariable ["bmkhs_engIdleNG",       getNumber (_config >> "engIdleNG")];
_heli setVariable ["bmkhs_engFlyNG",        getNumber (_config >> "engFlyNG")];

//Governor PID - one per engine
private _engPidGains = getArray (_config >> "pidEngine");
_heli setVariable ["bmkhs_pid_engine", [ _engPidGains call bmkhs_fnc_pidCreate
                                       , _engPidGains call bmkhs_fnc_pidCreate]];

//RUNTIME STATE - what the model carries frame to frame.
_heli setVariable ["bmkhs_shiftLocked",           false];
_heli setVariable ["bmkhs_isSingleEng",           false];
//_heli setVariable ["bmkhs_isAutorotating",        false];

//Outputs
_heli setVariable ["bmkhs_engFF",                 [0.0, 0.0]];
_heli setVariable ["bmkhs_engPctNG",              [0.0, 0.0]];
//SEEDS REQUIRED even though nothing READS these: bmkhs_fnc_utilSetArrayVariable does
//`+(_heli getVariable _name)` then `set`, so the array must already exist or it
//throws "Type Number, expected Array". Written per-engine by fn_engine.
_heli setVariable ["bmkhs_engBaseNG",             [0.0, 0.0]];
_heli setVariable ["bmkhs_engBaseTGT",            [0.0, 0.0]];
_heli setVariable ["bmkhs_engBaseOilPSI",         [0.0, 0.0]];
_heli setVariable ["bmkhs_engTrimTq",             [0.0, 0.0]];
_heli setVariable ["bmkhs_engPctNP",              [0.0, 0.0]];
_heli setVariable ["bmkhs_engPctTQ",              [0.0, 0.0]];
_heli setVariable ["bmkhs_engTGT",                [0.0, 0.0]];
_heli setVariable ["bmkhs_engOilPSI",             [0.0, 0.0]];

_heli setVariable ["bmkhs_engOutputTq",           [0.0, 0.0]];
