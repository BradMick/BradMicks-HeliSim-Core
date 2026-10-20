/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcVariables

Description:
    Loads the aircraft's declared FMC (class FMC - see \bmkhs_helisim\fmc.hpp), the
    casual assists' gains and the control mixes.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_config"];
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\fmc.hpp"

//Gains are declared per pilot unit; the loops run in SI. [gains, pilot units per SI unit]
private _pidSi = {
    params ["_gains", "_perSi"];
    _gains params ["_kp", "_ki", "_kd", "_limit"];
    [_kp * _perSi, _ki * _perSi, _kd * _perSi, _limit] call bmkhs_fnc_pidCreate
};

//The FMC - one entry per declared feature; an undeclared feature does not exist. Field
//reference and defaults: \bmkhs_helisim\fmc.hpp. The gains a feature runs are required; the rest are 0.
private _fmc = createHashMap;
private _known = createHashMapFromArray [
    ["AttitudeHold", ["pos", "vel", "att", "spd"]],
    ["HeadingHold",  ["hdg", "yaw", "trn"]]
];
private _subDefault = createHashMapFromArray [
    ["AttitudeHold", FMC_ATT_SUB_MODES],
    ["HeadingHold",  FMC_HDG_SUB_MODES]
];
{
    _x params ["_name", "_pids", "_nums", "_arrs", "_needs"];
    private _cfg = _config >> "FMC" >> _name;
    if (isClass _cfg) then {
        private _f = createHashMapFromArray [["gate", getArray (_cfg >> "gate")]];
        { _f set [_x select 0, [_x select 1, getNumber (_cfg >> (_x select 0))] select (isNumber (_cfg >> (_x select 0)))] } forEach _nums;
        { _f set [_x select 0, [_x select 1, getArray (_cfg >> (_x select 0))] select (isArray (_cfg >> (_x select 0)))] } forEach _arrs;
        if (_name in _known) then {
            private _sub = (_f get "subModes") select {_x in (_known get _name)};
            _f set ["subModes", [_sub, _subDefault get _name] select (_sub isEqualTo [])];
        };
        private _missing = [];
        {
            _x params ["_pid", "_perSi"];
            private _gains = getArray (_cfg >> _pid);
            if (count _gains >= 4) then {
                _f set [_pid, [_gains, _perSi] call _pidSi];
            } else {
                _f set [_pid, [0, 0, 0, 0] call bmkhs_fnc_pidCreate];
                _missing pushBack _pid;
            };
        } forEach _pids;
        _missing = _missing arrayIntersect (_f call _needs);
        if (_missing isEqualTo []) then {
            _fmc set [_name, _f];
        } else {
            (format ["[BMKHS] FMC CONFIG ERROR: %1 needs gains {kp, ki, kd, limit}: %2. Skipped.", _name, _missing]) call bmkhs_fnc_utilConfigError;
        };
    };
} forEach [
    ["Sas",          [["pitch", RAD_TO_DEG], ["roll", RAD_TO_DEG], ["yaw", RAD_TO_DEG]],     //deg/s
                     [],
                     [["authority", FMC_SAS_AUTHORITY]],
                     { ["pitch", "roll", "yaw"] }],
    ["AttitudeHold", [["posPitch", MPS_TO_KNOTS], ["posRoll", MPS_TO_KNOTS], ["attPitch", 1], ["attRoll", 1],
                      ["spdPitch", MPS_TO_KNOTS]],                                          //kt, deg
                     [["posBelowKts", FMC_ATT_POS_BELOW_KTS], ["velBelowKts", FMC_ATT_VEL_BELOW_KTS], ["attBelowKts", FMC_ATT_ATT_BELOW_KTS],
                      ["authority", FMC_ATT_AUTHORITY], ["alwaysOn", FMC_ATT_ALWAYS_ON], ["spdAboveKts", FMC_ATT_SPD_ABOVE_KTS],
                      ["breakout", FMC_ATT_BREAKOUT]],
                     [["subModes", []], ["trimGain", FMC_ATT_TRIM_GAIN]],
                     {
                         private _s = _this get "subModes";
                         ([[], ["posPitch", "posRoll"]] select ("pos" in _s || {"vel" in _s}))
                         + ([[], ["attPitch", "attRoll"]] select ("att" in _s || {"spd" in _s}))
                         + ([[], ["spdPitch"]] select ("spd" in _s))
                     }],
    ["AltitudeHold", [["rad", METERS_TO_FEET], ["bar", METERS_TO_FEET]],                    //ft
                     [["radBelowFt", FMC_ALT_RAD_BELOW_FT], ["radBelowKts", FMC_ALT_RAD_BELOW_KTS], ["engageFpm", FMC_ALT_ENGAGE_FPM],
                      ["collBand", FMC_ALT_COLL_BAND], ["dropAboveTq", FMC_ALT_DROP_ABOVE_TQ]],
                     [],
                     { ["rad", "bar"] }],
    ["HeadingHold",  [["hdg", 1], ["trn", 1]],                                              //deg, g
                     [["hdgBelowKts", FMC_HDG_BELOW_KTS], ["blendToKts", FMC_HDG_BLEND_TO_KTS], ["authority", FMC_HDG_AUTHORITY]],
                     [["subModes", []], ["breakout", FMC_HDG_BREAKOUT]],
                     {
                         private _s = _this get "subModes";
                         ([[], ["hdg"]] select ("hdg" in _s)) + ([[], ["trn"]] select ("yaw" in _s || {"trn" in _s}))
                     }],
    ["FlightDirector", [["vs", MPS_TO_FPM], ["ias", MPS_TO_KNOTS], ["pitch", 1], ["roll", 1], ["yaw", 1],
                        ["hvrPitch", MPS_TO_KNOTS], ["hvrRoll", MPS_TO_KNOTS]],                 //fpm, kt, deg; hvr kt to deg
                     [["altGain", FMC_FD_ALT_GAIN], ["vsMaxFpm", FMC_FD_VS_MAX_FPM], ["vsAccelFpm", FMC_FD_VS_ACCEL_FPM],
                      ["tqBand", FMC_FD_TQ_BAND], ["tqFpmGain", FMC_FD_TQ_FPM_GAIN], ["captureFt", FMC_FD_CAPTURE_FT],
                      ["iasAccelKts", FMC_FD_IAS_ACCEL_KTS], ["maxPitchDeg", FMC_FD_MAX_PITCH_DEG], ["pitchRateDps", FMC_FD_PITCH_RATE_DPS],
                      ["maxBankDeg", FMC_FD_MAX_BANK_DEG], ["turnRateDps", FMC_FD_TURN_RATE_DPS], ["bankPerDeg", FMC_FD_BANK_PER_DEG],
                      ["rollRateDps", FMC_FD_ROLL_RATE_DPS], ["bankAboveKts", FMC_FD_BANK_ABOVE_KTS],
                      ["hvrDecelKts", FMC_FD_HVR_DECEL_KTS], ["floorKts", FMC_FD_FLOOR_KTS],
                      ["collAuthority", FMC_FD_COLL_AUTHORITY], ["cycAuthority", FMC_FD_CYC_AUTHORITY], ["pedAuthority", FMC_FD_PED_AUTHORITY]],
                     [["modes", []]],
                     {
                         private _m = (_this get "modes") select {_x in ["ralt", "alt", "altp", "ias", "hdg", "nav", "hvr"]};
                         _this set ["modes", _m];
                         ([[], ["vs"]] select ((_m findIf {_x in ["ralt", "alt", "altp"]}) >= 0))
                         + ([[], ["ias", "pitch"]] select ("ias" in _m))
                         + ([[], ["roll", "yaw"]] select ((_m findIf {_x in ["hdg", "nav"]}) >= 0))
                         + ([[], ["hvrPitch", "hvrRoll", "pitch", "roll"]] select ("hvr" in _m))
                     }]
];

//The flight director's targets - {min, max, step, wraps} each. Declared in ft, kt and deg;
//held in m, m/s and deg.
private _fd = _fmc getOrDefault ["FlightDirector", createHashMap];
if (count _fd > 0) then {
    //HVR hands the hover to the attitude hold's position hold
    private _ah = _fmc getOrDefault ["AttitudeHold", createHashMap];
    if ("hvr" in (_fd get "modes") && {count _ah == 0 || {count getArray (_config >> "FMC" >> "AttitudeHold" >> "posPitch") < 4}}) then {
        "[BMKHS] FMC CONFIG ERROR: FlightDirector hvr needs AttitudeHold posPitch[] and posRoll[]. hvr skipped." call bmkhs_fnc_utilConfigError;
        _fd set ["modes", (_fd get "modes") - ["hvr"]];
    };
    private _targets = createHashMap;
    {
        _x params ["_name", "_si"];
        private _t = getArray (_config >> "FMC" >> "FlightDirector" >> "Targets" >> _name);
        if (count _t >= 3) then { _targets set [_name, [(_t select 0) * _si, (_t select 1) * _si, (_t select 2) * _si, (_t param [3, 0]) > 0]] };
    } forEach [["ralt", FEET_TO_METERS], ["alt", FEET_TO_METERS], ["altp", FEET_TO_METERS], ["ias", KNOTS_TO_MPS], ["hdg", 1]];
    _fd set ["targets", _targets];
};
_heli setVariable ["bmkhs_fmc", _fmc];
//The aircraft's active waypoint, posASL or [] - an input the aircraft writes
if (isNil {_heli getVariable "bmkhs_fdWaypoint"}) then { _heli setVariable ["bmkhs_fdWaypoint", []] };

_heli setVariable ["bmkhs_autoAttLevelPitch", getNumber (_config >> "autoAttLevelPitch")];
_heli setVariable ["bmkhs_autoAttRollLimit", getNumber (_config >> "autoAttRollLimit")];
_heli setVariable ["bmkhs_pid_autoAttPitch", (getArray (_config >> "pidAutoAttPitch")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_autoAttRoll", (getArray (_config >> "pidAutoAttRoll")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_autoPedalHdg", (getArray (_config >> "pidAutoPedalHdg")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_autoPedalNtt", (getArray (_config >> "pidAutoPedalNtt")) call bmkhs_fnc_pidCreate];
_heli setVariable ["bmkhs_pid_autoPedalAero", (getArray (_config >> "pidAutoPedalAero")) call bmkhs_fnc_pidCreate];

//FMC output state
_heli setVariable ["bmkhs_fmcAttHoldCycPitchOut", 0.0];
_heli setVariable ["bmkhs_fmcSasPitchOut",        0.0];
_heli setVariable ["bmkhs_fmcSasRollOut",         0.0];
_heli setVariable ["bmkhs_fmcHdgHoldPedalYawOut", 0.0];
_heli setVariable ["bmkhs_fmcSasYawOut",          0.0];
_heli setVariable ["bmkhs_fmcAltHoldCollOut",     0.0];
_heli setVariable ["bmkhs_fmcAttHoldCycRollOut",  0.0];
_heli setVariable ["bmkhs_mixPitchOut",           0.0];
_heli setVariable ["bmkhs_mixRollOut",            0.0];
_heli setVariable ["bmkhs_mixYawOut",             0.0];
_heli setVariable ["bmkhs_mixCollOut",            0.0];

//Control mixing - the aircraft's declared mixes, read once. A mix with no table or an
//unknown source or target is skipped.
private _mixes = [];
{
    private _target = ["pitch", "roll", "yaw", "collective"] find toLower getText (_x >> "target");
    private _source = toLower getText (_x >> "source");
    private _table  = getArray (_x >> "table");
    if (_target >= 0 && {_source in ["collective", "pedal"]} && {_table isNotEqualTo []}) then {
        _mixes pushBack (createHashMapFromArray [
            ["name",     configName _x],
            ["source",   _source],
            ["target",   _target],
            ["table",    _table],
            ["airspeed", getArray (_x >> "airspeed")],
            ["gate",     getArray (_x >> "gate")]
        ]);
    } else {
        (format ["[BMKHS] MIXING CONFIG ERROR: %1 needs source, target and table[]. Skipped.", configName _x]) call bmkhs_fnc_utilConfigError;
    };
} forEach ("true" configClasses (_config >> "ControlMixing"));
_heli setVariable ["bmkhs_mixes", _mixes];

//Modes and holds - networked; bmkhs_fnc_utilSeed publishes them from the owner only.
//FMC
[_heli, "bmkhs_fmcPitchOn",             true,  true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_fmcRollOn",              true,  true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_fmcYawOn",               true,  true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_fmcCollOn",              true,  true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_fmcTrimOn",              true,  true] call bmkhs_fnc_utilSeed;
//Declared and its gate holding - set by fn_fmc
[_heli, "bmkhs_fmcSasAvail",            false, true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_fmcAttHoldAvail",        false, true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_fmcAltHoldAvail",        false, true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_fmcHdgHoldAvail",        false, true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_fmcFdAvail",             false, true] call bmkhs_fnc_utilSeed;
//Flight director - a mode flag for each declared mode, a target for each declared target
if (count _fd > 0) then {
    { [_heli, "bmkhs_fd_" + _x, false, true] call bmkhs_fnc_utilSeed } forEach (_fd get "modes");
    { [_heli, "bmkhs_fdTgt_" + _x, _y select 0, true] call bmkhs_fnc_utilSeed } forEach (_fd get "targets");
};
[_heli, "bmkhs_fdWptBearing",           -1,    true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_fdWptDistance",          -1,    true] call bmkhs_fnc_utilSeed;
//Force Trim - the positions travel in the packed state, so they are not published here
[_heli, "bmkhs_forceTrimInterupted",    false, true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_forceTrimPosPitch",      0.0]         call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_forceTrimPosRoll",       0.0]         call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_forceTrimPosYaw",        0.0]         call bmkhs_fnc_utilSeed;
//Attitude Hold
[_heli, "bmkhs_attHoldActive",          false, true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_attHoldDesiredPos",      getPos _heli, true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_attHoldDesiredVel",      [0.0, 0.0], true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_attHoldDesiredAtt",      [0.0, 0.0], true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_attHoldDesiredSpd",      0.0,   true] call bmkhs_fnc_utilSeed;   //KTAS, m/s
[_heli, "bmkhs_attHoldSubMode",         "pos", true] call bmkhs_fnc_utilSeed;   //pos, vel, att, spd
//Altitude Hold
[_heli, "bmkhs_altHoldActive",          false, true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_altHoldDesiredAlt",      0.0,   true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_altHoldSubMode",         "rad", true] call bmkhs_fnc_utilSeed;   //rad, bar
[_heli, "bmkhs_altHoldCollRef",         0.0,   true] call bmkhs_fnc_utilSeed;
//Heading Hold
[_heli, "bmkhs_hdgHoldActive",          false, true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_hdgHoldDesiredHdg",      0.0,   true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_hdgHoldDesiredSideslip", 0.0,   true] call bmkhs_fnc_utilSeed;
[_heli, "bmkhs_hdgHoldSubMode",         "hdg", true] call bmkhs_fnc_utilSeed;    //hdg, trn, yaw, aut
