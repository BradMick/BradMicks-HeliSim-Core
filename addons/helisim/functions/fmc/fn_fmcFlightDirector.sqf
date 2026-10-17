/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcFlightDirector

Description:
    Flies the engaged flight director modes. Each axis it holds replaces that
    axis's hold output in fn_fmc:

        vertical      ralt / alt / altp  - altitude error to a climb rate, climb rate
                                           to collective, never past continuous torque
        longitudinal  ias                - airspeed to a pitch attitude, pitch to cyclic
                      hvr                - slows to a stop over the ground, then hands
                                           the aircraft to the attitude hold's position hold
        lateral       hdg / nav          - a standard rate bank for the airspeed, rolling
                                           out onto the heading, bank to cyclic; below
                                           bankAboveKts ground speed, or in HVR, heading
                                           to pedal

    Every command is eased onto at its declared rate - a step in a target is not a step
    in a control. Works in m, m/s and deg; the config's pilot units convert as read.
    ALTP climbs to its target and, inside captureFt, hands over to ALT holding it.
    With the gate shut every mode drops.

Parameters:
    _heli - The helicopter [Object]
    _fd   - The FlightDirector feature [HashMap]
    _on   - Declared and its gate holding [Boolean]

Returns:
    The axes it holds and their outputs - any of "coll", "pitch", "roll", "yaw"
    [HashMap]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_fd", "_on"];
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\fmc.hpp"

if (count _fd == 0) exitWith {createHashMap};

private _modes   = _fd get "modes";
private _engaged = { (_this in _modes) && {_heli getVariable ["bmkhs_fd_" + _this, false]} };
private _refs    = ["climbRef", "iasRef", "pitchRef", "bankRef", "hvrRef", "hvrHeld", "hvrBeeped"];

if (!_on) exitWith {
    { if (_x call _engaged) then { [_heli, _x] call bmkhs_fnc_fmcFdMode } } forEach _modes;
    { [_fd get _x] call bmkhs_fnc_pidReset } forEach ["vs", "ias", "pitch", "roll", "yaw", "hvrPitch", "hvrRoll"];
    { _fd deleteAt _x } forEach _refs;
    _fd set ["log", [0, 0, 0, 0]];
    createHashMap
};

private _deltaTime = _heli getVariable "bmkhs_deltaTime";
private _hdgNow    = getDir _heli;
(_heli call BIS_fnc_getPitchBank) params ["_pitchNow", "_bankNow"];
//Airspeed, forward through the air; ground velocity, model space - m/s
private _airspeed = (_heli getVariable "bmkhs_velModelSpace") select 1;
(_heli getVariable "bmkhs_velModelSpaceNoWind") params ["_gndVelX", "_gndVelY"];
private _gndSpeed = _heli getVariable "bmkhs_gndSpeed";
private _climbNow = _heli getVariable "bmkhs_velClimb";
//ALT / ALTP fly pressure altitude - the environment's base altitude, as the altimeter reads
private _altNow   = (_heli getVariable "bmkhs_barAlt") * FEET_TO_METERS;

//A command eased toward what is wanted at no more than _rate a second, from where the
//aircraft is when the mode starts
private _slew = {
    params ["_key", "_want", "_rate", "_now"];
    private _ref  = _fd getOrDefault [_key, _now];
    private _step = _rate * _deltaTime;
    _ref = _ref + (((_want - _ref) max -_step) min _step);
    _fd set [_key, _ref];
    _ref
};

//The active waypoint - bearing and distance, for the cockpit and NAV
private _wpt = _heli getVariable ["bmkhs_fdWaypoint", []];
private _wptBrg = -1;
if (_wpt isNotEqualTo []) then {
    _wptBrg = round (_heli getDir _wpt);
    [_heli, "bmkhs_fdWptBearing",  _wptBrg] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "bmkhs_fdWptDistance", round (_heli distance2D _wpt)] call bmkhs_fnc_utilUpdateNetworkGlobal;
} else {
    [_heli, "bmkhs_fdWptBearing",  -1] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "bmkhs_fdWptDistance", -1] call bmkhs_fnc_utilUpdateNetworkGlobal;
    if ("nav" call _engaged) then { [_heli, "nav"] call bmkhs_fnc_fmcFdMode };
};

//Force trim pressed - the director stands aside on the cyclic
private _override = _heli getVariable "bmkhs_forceTrimInterupted";
if (_override) then { _fd set ["hvrHeld", false]; _fd deleteAt "hvrBeeped" };
//Cyclic past the attitude hold's breakout - that axis only
private _bo      = ((_heli getVariable "bmkhs_fmc") getOrDefault ["AttitudeHold", createHashMap]) getOrDefault ["breakout", FMC_ATT_BREAKOUT];
private _ovPitch = _override || {abs (_heli getVariable "bmkhs_cyclicFwdAft") > _bo};
private _ovRoll  = _override || {abs (_heli getVariable "bmkhs_cyclicLeftRight") > _bo};

//HVR holding on the attitude hold - when that drops, so does HVR
if ("hvr" call _engaged && {_fd getOrDefault ["hvrHeld", false]} && {!(_heli getVariable "bmkhs_attHoldActive")}) then {
    [_heli, "hvr"] call bmkhs_fnc_fmcFdMode;
};
//HVR is a low speed mode
if ("hvr" call _engaged && {_gndSpeed > FMC_FD_HVR_BELOW_KTS * KNOTS_TO_MPS}) then {
    [_heli, "hvr"] call bmkhs_fnc_fmcFdMode;
};

//Speed floor - every mode but RALT and HVR drops 2 kt below it
private _floor = (_fd get "floorKts") * KNOTS_TO_MPS;
if (_floor > 0 && {_airspeed < _floor - FMC_FD_FLOOR_DROP_KTS * KNOTS_TO_MPS}) then {
    { if (_x call _engaged) then { [_heli, _x] call bmkhs_fnc_fmcFdMode } } forEach (_modes - ["ralt", "hvr"]);
};

//ALTP capture - inside the band it becomes ALT on the same target
if ("altp" call _engaged && {"alt" in _modes}
    && {abs (_altNow - (_heli getVariable "bmkhs_fdTgt_altp")) < ((_fd get "captureFt") * FEET_TO_METERS)}) then {
    [_heli, "alt", _heli getVariable "bmkhs_fdTgt_altp"] call bmkhs_fnc_fmcFdTarget;
    [_heli, "alt", false] call bmkhs_fnc_fmcFdMode;
};

//Beep moves the IAS / HDG target; in HVR, the ground velocity it holds
private _beep    = _heli getVariable ["bmkhs_trimBeepFd", [0.0, 0.0]];
private _beeping = (_beep findIf {_x != 0}) >= 0;
private _sgn     = { [0, [1, -1] select (_this < 0)] select (_this != 0) };
private _hvr     = "hvr" call _engaged;
if (_hvr) then {
    if (_beeping) then {
        private _step = FMC_FD_HVR_BEEP_KTS * KNOTS_TO_MPS * _deltaTime;
        private _ref  = if (_fd getOrDefault ["hvrHeld", false]) then { [0, 0] } else { _fd getOrDefault ["hvrRef", [_gndVelX, _gndVelY]] };
        //Beep + is forward / left; model X is right
        _fd set ["hvrRef", [(_ref select 0) - ((_beep select 1) call _sgn) * _step, (_ref select 1) + ((_beep select 0) call _sgn) * _step]];
        _fd set ["hvrHeld", false];
        _fd set ["hvrBeeped", true];
    };
} else {
    {
        _x params ["_mode", "_d", "_rate"];
        if (_d != 0 && {_mode call _engaged}) then {
            [_heli, _mode, (_heli getVariable ("bmkhs_fdTgt_" + _mode)) + (_d call _sgn) * _rate * _deltaTime, false] call bmkhs_fnc_fmcFdTarget;
        };
    } forEach [
        ["ias",  _beep select 0,  (_fd get "iasAccelKts") * KNOTS_TO_MPS],
        ["hdg", -(_beep select 1), _fd get "turnRateDps"]
    ];
};

private _out = createHashMap;
//What it is asking for, for the flight log - {climb m/s, pitch, bank, torque headroom}
private _log = [0, 0, 0, 0];
private _cycAuth = _fd get "cycAuthority";

//Vertical
private _vMode = ["ralt", "alt", "altp"] select {_x call _engaged};
if (_vMode isNotEqualTo []) then {
    _vMode = _vMode select 0;
    private _cur   = if (_vMode == "ralt") then { _heli getVariable "bmkhs_radAlt" } else { _altNow };
    private _err   = (_heli getVariable ("bmkhs_fdTgt_" + _vMode)) - _cur;
    private _vsMax = (_fd get "vsMaxFpm") * FPM_TO_MPS;
    //altGain is fpm per ft - per second either way
    private _want  = ((_err * (_fd get "altGain") * FPM_TO_MPS / FEET_TO_METERS) max -_vsMax) min _vsMax;

    //Never past continuous torque - the engine's first book limit, single-engine where that
    //applies. Inside tqBand of it, the climb allowed is the present one plus the headroom left
    //times tqFpmGain: a little more with room, the present climb at the limit, less over it -
    //so torque eases onto the limit and stops there. It limits what is wanted, and the climb
    //rate eases onto that like any other change.
    private _engines = _heli getVariable ["bmkhs_engines", []];
    if (_engines isNotEqualTo []) then {
        private _key  = ["tqLimits", "tqLimitsSe"] select (_heli getVariable ["bmkhs_isSingleEng", false]);
        private _room = ((((_engines select 0) get _key) select 0) select 0) - (selectMax (_heli getVariable "bmkhs_engPctTq"));
        _log set [3, _room];
        if (_room < (_fd get "tqBand")) then {
            _want = _want min (_climbNow + (_room * (_fd get "tqFpmGain") * FPM_TO_MPS));
        };
    };
    private _cmd   = ["climbRef", _want, (_fd get "vsAccelFpm") * FPM_TO_MPS, _climbNow] call _slew;

    _log set [0, _cmd];
    private _coll = [_fd get "vs", _deltaTime, _cmd, _climbNow] call bmkhs_fnc_pidRun;
    _out set ["coll", (_coll max -(_fd get "collAuthority")) min (_fd get "collAuthority")];
} else {
    [_fd get "vs"] call bmkhs_fnc_pidReset;
    _fd deleteAt "climbRef";
};

//Longitudinal - IAS
if ("ias" call _engaged && {!_ovPitch}) then {
    private _maxPitch = _fd get "maxPitchDeg";
    private _spd = ["iasRef", _heli getVariable "bmkhs_fdTgt_ias", (_fd get "iasAccelKts") * KNOTS_TO_MPS, _airspeed] call _slew;
    //Slow is nose down
    private _wantPitch = -([_fd get "ias", _deltaTime, _spd, _airspeed] call bmkhs_fnc_pidRun);
    _wantPitch = (_wantPitch max -_maxPitch) min _maxPitch;
    _wantPitch = ["pitchRef", _wantPitch, _fd get "pitchRateDps", _pitchNow] call _slew;
    _log set [1, _wantPitch];
    private _pitch = -([_fd get "pitch", _deltaTime, 0.0, _pitchNow - _wantPitch] call bmkhs_fnc_pidRun);
    _out set ["pitch", (_pitch max -_cycAuth) min _cycAuth];
} else {
    if ("ias" call _engaged) then { _out set ["pitch", 0.0] };
    [_fd get "ias"] call bmkhs_fnc_pidReset;
    [_fd get "pitch"] call bmkhs_fnc_pidReset;
    _fd deleteAt "iasRef";
    _fd deleteAt "pitchRef";
};

//Longitudinal - HVR. The ground velocity it started with, eased to nothing; stopped, the
//attitude hold's position hold holds the spot. A beeped velocity is held; beeped back to
//nothing, it holds the spot again.
if (_hvr && {!_override} && {!(_fd getOrDefault ["hvrHeld", false])}) then {
    private _ref = _fd getOrDefault ["hvrRef", [_gndVelX, _gndVelY]];
    private _mag = vectorMagnitude [_ref select 0, _ref select 1, 0];
    private _new = _mag;
    if (_fd getOrDefault ["hvrBeeped", false]) then {
        if (!_beeping && {_mag < FMC_FD_HVR_HOLD_KTS * KNOTS_TO_MPS}) then { _new = 0; _fd deleteAt "hvrBeeped" };
    } else {
        _new = (_mag - ((_fd get "hvrDecelKts") * KNOTS_TO_MPS * _deltaTime)) max 0;
    };
    _ref = if (_mag > 0) then { [(_ref select 0) * (_new / _mag), (_ref select 1) * (_new / _mag)] } else { [0, 0] };
    _fd set ["hvrRef", _ref];

    private _roll  = [_fd get "hvrRoll",  _deltaTime, -(_ref select 0), -_gndVelX] call bmkhs_fnc_pidRun;
    private _pitch = [_fd get "hvrPitch", _deltaTime,  (_ref select 1),  _gndVelY] call bmkhs_fnc_pidRun;
    _out set ["roll",  (_roll  max -_cycAuth) min _cycAuth];
    _out set ["pitch", (_pitch max -_cycAuth) min _cycAuth];
    if (_ovRoll)  then { _out set ["roll",  0.0]; [_fd get "hvrRoll"]  call bmkhs_fnc_pidReset };
    if (_ovPitch) then { _out set ["pitch", 0.0]; [_fd get "hvrPitch"] call bmkhs_fnc_pidReset };

    if (_new == 0 && {!(_fd getOrDefault ["hvrBeeped", false])} && {(_heli getVariable "bmkhs_attHoldSubMode") == "pos"}) then {
        _fd set ["hvrHeld", true];
        //The spot it stops on
        _heli setVariable ["bmkhs_attHoldDesiredPos", getPos _heli, true];
        _heli setVariable ["bmkhs_posIntX", 0.0];
        _heli setVariable ["bmkhs_posIntY", 0.0];
        if !(_heli getVariable "bmkhs_attHoldActive") then { [_heli] call bmkhs_fnc_fmcAttitudeHoldEnable };
    };
} else {
    if (_hvr && _override) then { _out set ["roll", 0.0]; _out set ["pitch", 0.0] };
    [_fd get "hvrRoll"]  call bmkhs_fnc_pidReset;
    [_fd get "hvrPitch"] call bmkhs_fnc_pidReset;
    _fd deleteAt "hvrRef";
    if (!_hvr) then { _fd deleteAt "hvrHeld"; _fd deleteAt "hvrBeeped" };
};

//Lateral - HDG, or NAV to the waypoint
private _lMode = ["hdg", "nav"] select {_x call _engaged};
if (_lMode isNotEqualTo []) then {
    private _tgtHdg = if ((_lMode select 0) == "nav") then { _wptBrg } else { _heli getVariable "bmkhs_fdTgt_hdg" };
    private _hdgErr = [_tgtHdg - _hdgNow] call CBA_fnc_simplifyAngle180;
    if (!_hvr && {_gndSpeed > ((_fd get "bankAboveKts") * KNOTS_TO_MPS)}) then {
        //Standard rate for the airspeed - tan(bank) = V * rate / g - rolling out as the heading nears
        private _rateBank = atan ((_airspeed max 0) * ((_fd get "turnRateDps") * (pi / 180)) / 9.80665);
        private _bank     = ((_rateBank min (_fd get "maxBankDeg")) min ((abs _hdgErr) * (_fd get "bankPerDeg")));
        if (_ovRoll) then {
            _out set ["roll", 0.0];
            [_fd get "roll"] call bmkhs_fnc_pidReset;
            _fd deleteAt "bankRef";
        } else {
            private _wantBank = ["bankRef", _bank * ([1, -1] select (_hdgErr < 0)), _fd get "rollRateDps", _bankNow] call _slew;
            _log set [2, _wantBank];
            private _roll = -([_fd get "roll", _deltaTime, 0.0, _bankNow - _wantBank] call bmkhs_fnc_pidRun);
            _out set ["roll", (_roll max -_cycAuth) min _cycAuth];
        };
        [_fd get "yaw"] call bmkhs_fnc_pidReset;
    } else {
        //Slow, or hovering - pedal turns it. Wings level unless HVR has the cyclic.
        if (!_hvr && {_ovRoll}) then {
            _out set ["roll", 0.0];
            [_fd get "roll"] call bmkhs_fnc_pidReset;
            _fd deleteAt "bankRef";
        };
        if (!_hvr && {!_ovRoll}) then {
            private _wantBank = ["bankRef", 0.0, _fd get "rollRateDps", _bankNow] call _slew;
            _log set [2, _wantBank];
            private _roll = -([_fd get "roll", _deltaTime, 0.0, _bankNow - _wantBank] call bmkhs_fnc_pidRun);
            _out set ["roll", (_roll max -_cycAuth) min _cycAuth];
        } else {
            [_fd get "roll"] call bmkhs_fnc_pidReset;
            _fd deleteAt "bankRef";
        };
        private _yaw = [_fd get "yaw", _deltaTime, 0.0, [_hdgNow - _tgtHdg] call CBA_fnc_simplifyAngle180] call bmkhs_fnc_pidRun;
        _out set ["yaw", (_yaw max -(_fd get "pedAuthority")) min (_fd get "pedAuthority")];
    };
} else {
    [_fd get "roll"] call bmkhs_fnc_pidReset;
    [_fd get "yaw"] call bmkhs_fnc_pidReset;
    _fd deleteAt "bankRef";
};

_fd set ["log", _log];
_out
