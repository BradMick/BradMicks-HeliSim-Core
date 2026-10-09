/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_debugFlightLog

Description:
    With the Flight Log setting on, writes the flight to the RPT 10 times a
    second as comma-separated BMKHSLOG lines. A BMKHSLOG_HDR line names the
    columns, once per aircraft and whenever the log is switched on.

    The RPT cuts a line at about 1020 characters, so a header or row longer
    than 900 goes out in pieces, split between columns: the first under its
    own tag, the rest as BMKHSLOG_HDR+ and BMKHSLOG+ lines that continue it.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

if (!bmkhs_flightLog) exitWith { _heli setVariable ["bmkhs_flightLogHdr", false] };
if (time < (_heli getVariable ["bmkhs_flightLogLast", -1]) + 0.1) exitWith {};
_heli setVariable ["bmkhs_flightLogLast", time];

private _fmc = _heli getVariable "bmkhs_fmc";
private _fd  = _fmc getOrDefault ["FlightDirector", createHashMap];
private _fdModes   = _fd getOrDefault ["modes", []];
private _fdTargets = keys (_fd getOrDefault ["targets", createHashMap]);
_fdTargets sort true;

//The aircraft's own damage roles, every member, and every component it declares - so a
//failure shows its whole chain: damage, what the solve made of it, what that took down
private _dmgPoints = _heli getVariable ["bmkhs_damagePoints", createHashMap];
private _roles     = keys _dmgPoints;
_roles sort true;
private _dmgCols   = [];
{
    private _role = _x;
    private _n    = count (_dmgPoints get _role);
    for "_i" from 0 to (_n - 1) do { _dmgCols pushBack [_role, _i, [_i + 1, ""] select (_n <= 1)] };
} forEach _roles;
//Drive parts whose damage Core keeps rather than a hitpoint
private _ownDmg = ((_heli getVariable ["bmkhs_sysTorqued", []]) select {(_x getOrDefault ["damageVar", ""]) != ""})
                    apply {[_x get "varName", _x get "damageVar"]};
private _sysVars = [];
{ { _sysVars pushBackUnique (_x get "varName") } forEach (_heli getVariable [_x, []]) }
    forEach ["bmkhs_sysProducers", "bmkhs_sysConverters", "bmkhs_sysStorage", "bmkhs_sysNamed", "bmkhs_sysConsumers"];

private _cols = [
    "t", "id", "realism",
    "kts", "gsKts", "altFt", "radAltFt", "vsFpm", "pitch", "bank", "hdg", "betaG", "betaDeg",
    "pRateDps", "rRateDps", "yRateDps",
    "tq1", "tq2", "np1", "np2", "nr",
    "cyc", "cycLR", "pedal", "coll", "ftPitch", "ftRoll", "ftYaw",
    "sasP", "sasR", "sasY", "attP", "attR", "hdgY", "altC", "mixP", "mixR", "mixY",
    "sasAv", "attAv", "altAv", "hdgAv", "fdAv",
    "attOn", "attSub", "altOn", "altSub", "hdgOn", "hdgSub", "ftInt",
    "fdModes", "fdFpm", "fdWantPitch", "fdWantBank", "fdTqRoom",
    "engOn", "engState", "engOverspeed", "engTqTimer"
] + (_fdTargets apply {"tgt_" + _x})
  + (_dmgCols apply {"dmg_" + (_x select 0) + ([str (_x select 2), ""] select ((_x select 2) isEqualTo ""))})
  + (_ownDmg apply {"dmg_" + ((_x select 0) select [6])})
  + (_sysVars apply {"sys_" + (_x select [6])});

//Writes [tag, columns] as lines under 900 characters, the pieces after the first tagged tag+
private _logLines = {
    params ["_tag", "_items"];
    private _buf = [];
    private _len = 0;
    private _cont = "";
    {
        if (_len + (count _x) + 1 > 900 && {_buf isNotEqualTo []}) then {
            diag_log text (_tag + _cont + "," + (_buf joinString ","));
            _cont = "+";
            _buf = [];
            _len = 0;
        };
        _buf pushBack _x;
        _len = _len + (count _x) + 1;
    } forEach _items;
    diag_log text (_tag + _cont + "," + (_buf joinString ","));
};

if !(_heli getVariable ["bmkhs_flightLogHdr", false]) then {
    _heli setVariable ["bmkhs_flightLogHdr", true];
    ["BMKHSLOG_HDR", _cols] call _logLines;
};

private _f = { _this toFixed 3 };
private _b = { [0, 1] select _this };
(_heli call BIS_fnc_getPitchBank) params ["_pitch", "_bank"];
(_heli getVariable "bmkhs_angVelModelSpace") params ["_wx", "_wy", "_wz"];
(_heli getVariable "bmkhs_engPctTq") params [["_tq1", 0], ["_tq2", 0]];
(_heli getVariable "bmkhs_engPctNp") params [["_np1", 0], ["_np2", 0]];
(_fd getOrDefault ["log", [0, 0, 0, 0]]) params ["_climb", "_wantPitch", "_wantBank", "_tqRoom"];
//Targets in ft and kt, like the rest of the log
private _tgtUnits = createHashMapFromArray [["ralt", METERS_TO_FEET], ["alt", METERS_TO_FEET], ["altp", METERS_TO_FEET], ["ias", MPS_TO_KNOTS]];

private _row = [
    time toFixed 2, netId _heli, str bmkhs_helisimRealismSetting,
    (((_heli getVariable "bmkhs_velModelSpace") select 1) * MPS_TO_KNOTS) toFixed 1,
    ((_heli getVariable "bmkhs_gndSpeed") * MPS_TO_KNOTS) toFixed 1,
    (((getPosASL _heli) select 2) * METERS_TO_FEET) toFixed 1,
    ((_heli getVariable "bmkhs_radAlt") * METERS_TO_FEET) toFixed 1,
    ((_heli getVariable "bmkhs_velClimb") * MPS_TO_FPM) toFixed 0,
    _pitch toFixed 2, _bank toFixed 2, (getDir _heli) toFixed 2,
    (_heli getVariable "bmkhs_aero_beta_g") call _f, (_heli getVariable "bmkhs_aero_beta_deg") toFixed 2,
    (deg _wx) toFixed 2, (deg _wy) toFixed 2, (deg _wz) toFixed 2,
    _tq1 call _f, _tq2 call _f, _np1 call _f, _np2 call _f, (_heli getVariable "bmkhs_rtrRpm") call _f,
    (_heli getVariable "bmkhs_cyclicFwdAft") call _f, (_heli getVariable "bmkhs_cyclicLeftRight") call _f,
    (_heli getVariable "bmkhs_pedalLeftRight") call _f, (_heli getVariable "bmkhs_collectiveOutput") call _f,
    (_heli getVariable "bmkhs_forceTrimPosPitch") call _f, (_heli getVariable "bmkhs_forceTrimPosRoll") call _f,
    (_heli getVariable "bmkhs_forceTrimPosYaw") call _f,
    (_heli getVariable "bmkhs_fmcSasPitchOut") call _f, (_heli getVariable "bmkhs_fmcSasRollOut") call _f,
    (_heli getVariable "bmkhs_fmcSasYawOut") call _f,
    (_heli getVariable "bmkhs_fmcAttHoldCycPitchOut") call _f, (_heli getVariable "bmkhs_fmcAttHoldCycRollOut") call _f,
    (_heli getVariable "bmkhs_fmcHdgHoldPedalYawOut") call _f, (_heli getVariable "bmkhs_fmcAltHoldCollOut") call _f,
    (_heli getVariable "bmkhs_mixPitchOut") call _f, (_heli getVariable "bmkhs_mixRollOut") call _f,
    (_heli getVariable "bmkhs_mixYawOut") call _f,
    str ((_heli getVariable "bmkhs_fmcSasAvail") call _b), str ((_heli getVariable "bmkhs_fmcAttHoldAvail") call _b),
    str ((_heli getVariable "bmkhs_fmcAltHoldAvail") call _b), str ((_heli getVariable "bmkhs_fmcHdgHoldAvail") call _b),
    str ((_heli getVariable "bmkhs_fmcFdAvail") call _b),
    str ((_heli getVariable "bmkhs_attHoldActive") call _b), _heli getVariable "bmkhs_attHoldSubMode",
    str ((_heli getVariable "bmkhs_altHoldActive") call _b), _heli getVariable "bmkhs_altHoldSubMode",
    str ((_heli getVariable "bmkhs_hdgHoldActive") call _b), _heli getVariable "bmkhs_hdgHoldSubMode",
    str ((_heli getVariable "bmkhs_forceTrimInterupted") call _b),
    ((_fdModes select {_heli getVariable ["bmkhs_fd_" + _x, false]}) joinString "|"),
    (_climb * MPS_TO_FPM) toFixed 0, _wantPitch toFixed 2, _wantBank toFixed 2, _tqRoom call _f,
    str (isEngineOn _heli call _b),
    (_heli getVariable ["bmkhs_engState", []]) joinString "|",
    ((_heli getVariable ["bmkhs_engineOverspeed", []]) apply {_x call _b}) joinString "|",
    ((_heli getVariable ["bmkhs_engTqTimer", []]) apply {_x toFixed 1}) joinString "|"
] + (_fdTargets apply { ((_heli getVariable ["bmkhs_fdTgt_" + _x, 0]) * (_tgtUnits getOrDefault [_x, 1])) toFixed 0 })
  + (_dmgCols apply { ([_heli, _x select 0, _x select 1] call bmkhs_fnc_damageGet) call _f })
  + (_ownDmg apply { (_heli getVariable [_x select 1, 0]) call _f })
  + (_sysVars apply {
        private _v = _heli getVariable [_x, 0];
        if (_v isEqualType true) then { str (_v call _b) } else { if (_v isEqualType 0) then { _v call _f } else { "?" } }
    });

["BMKHSLOG", _row] call _logLines;
