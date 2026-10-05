/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcFlightDirector

Description:
    Flies the engaged flight director modes. Each axis it holds replaces that
    axis's hold output in fn_fmc:

        vertical      ralt / alt / altp  - altitude error to a climb rate, climb rate
                                           to collective, never past continuous torque
        longitudinal  ias                - airspeed to a pitch attitude, pitch to cyclic
                      hvr                - the aircraft's attitude hold; nothing here
        lateral       hdg / nav          - a standard rate bank for the speed, rolling
                                           out onto the heading, bank to cyclic; below
                                           bankAboveKts, heading to pedal

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

if (count _fd == 0) exitWith {createHashMap};

private _modes  = _fd get "modes";
private _engaged = { (_this in _modes) && {_heli getVariable ["bmkhs_fd_" + _this, false]} };

if (!_on) exitWith {
    { if (_x call _engaged) then { [_heli, _x] call bmkhs_fnc_fmcFdMode } } forEach _modes;
    { [_fd get _x] call bmkhs_fnc_pidReset } forEach ["vs", "ias", "pitch", "roll", "yaw"];
    createHashMap
};

private _dt     = _heli getVariable "bmkhs_deltaTime";
private _kts    = _heli getVariable "bmkhs_vel2D";
private _hdgNow = getDir _heli;
(_heli call BIS_fnc_getPitchBank) params ["_pitchNow", "_bankNow"];

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

//HVR is the attitude hold - when that drops, so does HVR
if ("hvr" call _engaged && {!(_heli getVariable "bmkhs_attHoldActive")}) then { [_heli, "hvr"] call bmkhs_fnc_fmcFdMode };

//ALTP capture - inside the band it becomes ALT on the same target
private _altFt = ((getPosASL _heli) select 2) * METERS_TO_FEET;
if ("altp" call _engaged && {"alt" in _modes}
    && {abs (_altFt - (_heli getVariable "bmkhs_fdTgt_altp")) < (_fd get "captureFt")}) then {
    [_heli, "alt", _heli getVariable "bmkhs_fdTgt_altp"] call bmkhs_fnc_fmcFdTarget;
    [_heli, "alt"] call bmkhs_fnc_fmcFdMode;
};

private _out = createHashMap;

//Vertical
private _vMode = ["ralt", "alt", "altp"] select {_x call _engaged};
if (_vMode isNotEqualTo []) then {
    _vMode = _vMode select 0;
    private _cur = if (_vMode == "ralt") then { (_heli getVariable "bmkhs_radAltRaw") * METERS_TO_FEET } else { _altFt };
    private _err = (_heli getVariable ("bmkhs_fdTgt_" + _vMode)) - _cur;
    private _fpm = ((_err * (_fd get "altGain")) max -(_fd get "vsMaxFpm")) min (_fd get "vsMaxFpm");

    //Never past continuous torque - the engine's first book limit, single-engine where that
    //applies. Inside tqBand of it, the climb asked for beyond the present one tapers with the
    //headroom left, to none at the limit; over it, a little less than the present one.
    private _engines = _heli getVariable ["bmkhs_engines", []];
    private _climb   = _heli getVariable "bmkhs_velClimb";
    if (_engines isNotEqualTo [] && {_fpm > _climb}) then {
        private _key  = ["tqLimits", "tqLimitsSe"] select (_heli getVariable ["bmkhs_isSingleEng", false]);
        private _room = ((((_engines select 0) get _key) select 0) select 0) - (selectMax (_heli getVariable "bmkhs_engPctTq"));
        if (_room < (_fd get "tqBand")) then {
            _fpm = _climb + ((_fpm - _climb) * ((_room / (_fd get "tqBand")) max 0)) - (((-_room) max 0) * (_fd get "tqFpmGain"));
        };
    };

    private _coll = [_fd get "vs", _dt, _fpm, _climb] call bmkhs_fnc_pidRun;
    _out set ["coll", (_coll max -(_fd get "collAuthority")) min (_fd get "collAuthority")];
} else {
    [_fd get "vs"] call bmkhs_fnc_pidReset;
};

//Longitudinal - IAS. HVR flies through the attitude hold.
if ("ias" call _engaged) then {
    private _maxPitch = _fd get "maxPitchDeg";
    //Slow is nose down
    private _wantPitch = -([_fd get "ias", _dt, _heli getVariable "bmkhs_fdTgt_ias", _kts] call bmkhs_fnc_pidRun);
    _wantPitch = (_wantPitch max -_maxPitch) min _maxPitch;
    private _pitch = -([_fd get "pitch", _dt, 0.0, _pitchNow - _wantPitch] call bmkhs_fnc_pidRun);
    _out set ["pitch", (_pitch max -(_fd get "cycAuthority")) min (_fd get "cycAuthority")];
} else {
    [_fd get "ias"] call bmkhs_fnc_pidReset;
    [_fd get "pitch"] call bmkhs_fnc_pidReset;
};

//Lateral - HDG, or NAV to the waypoint
private _lMode = ["hdg", "nav"] select {_x call _engaged};
if (_lMode isNotEqualTo []) then {
    private _tgtHdg = if ((_lMode select 0) == "nav") then { _wptBrg } else { _heli getVariable "bmkhs_fdTgt_hdg" };
    private _hdgErr = [_tgtHdg - _hdgNow] call CBA_fnc_simplifyAngle180;
    private _maxBank = _fd get "maxBankDeg";
    if (_kts > (_fd get "bankAboveKts")) then {
        //Standard rate for the speed - tan(bank) = V * rate / g - rolling out as the heading nears
        private _rateBank = atan ((_kts * KNOTS_TO_MPS) * ((_fd get "turnRateDps") * (pi / 180)) / 9.80665);
        private _bank     = (_rateBank min _maxBank) min ((abs _hdgErr) * (_fd get "bankPerDeg"));
        private _wantBank = _bank * ([1, -1] select (_hdgErr < 0));
        private _roll = -([_fd get "roll", _dt, 0.0, _bankNow - _wantBank] call bmkhs_fnc_pidRun);
        _out set ["roll", (_roll max -(_fd get "cycAuthority")) min (_fd get "cycAuthority")];
        [_fd get "yaw"] call bmkhs_fnc_pidReset;
    } else {
        //Slow - wings level, pedal turns it
        private _roll = -([_fd get "roll", _dt, 0.0, _bankNow] call bmkhs_fnc_pidRun);
        _out set ["roll", (_roll max -(_fd get "cycAuthority")) min (_fd get "cycAuthority")];
        private _yaw = [_fd get "yaw", _dt, 0.0, [_hdgNow - _tgtHdg] call CBA_fnc_simplifyAngle180] call bmkhs_fnc_pidRun;
        _out set ["yaw", (_yaw max -(_fd get "pedAuthority")) min (_fd get "pedAuthority")];
    };
} else {
    [_fd get "roll"] call bmkhs_fnc_pidReset;
    [_fd get "yaw"] call bmkhs_fnc_pidReset;
};

_out
