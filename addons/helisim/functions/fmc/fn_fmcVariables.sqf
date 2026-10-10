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

//The FMC - one entry per declared feature; an undeclared feature does not exist. Field
//reference: \bmkhs_helisim\fmc.hpp
private _fmc = createHashMap;
{
    _x params ["_name", "_pids", "_nums", "_arrs"];
    private _cfg = _config >> "FMC" >> _name;
    if (isClass _cfg) then {
        private _f = createHashMapFromArray [["gate", getArray (_cfg >> "gate")]];
        private _ok = true;
        {
            private _gains = getArray (_cfg >> _x);
            if (count _gains < 4) then { _ok = false } else { _f set [_x, _gains call bmkhs_fnc_pidCreate] };
        } forEach _pids;
        { _f set [_x select 0, [_x select 1, getNumber (_cfg >> (_x select 0))] select (isNumber (_cfg >> (_x select 0)))] } forEach _nums;
        { _f set [_x select 0, [_x select 1, getArray (_cfg >> (_x select 0))] select (isArray (_cfg >> (_x select 0)))] } forEach _arrs;
        if (_ok) then {
            _fmc set [_name, _f];
        } else {
            diag_log format ["[BMKHS] FMC CONFIG ERROR: %1 needs every gain array {kp, ki, kd, ki_clamp}: %2. Skipped.", _name, _pids];
        };
    };
} forEach [
    ["Sas",          ["pitch", "roll", "yaw"],
                     [],
                     [["authority", [0.2, 0.1, 0.1]]]],
    ["AttitudeHold", ["posPitch", "posRoll", "attPitch", "attRoll"],
                     [["posBelowKts", 5], ["velBelowKts", 40], ["attBelowKts", 30], ["authority", 0.1]],
                     []],
    ["AltitudeHold", ["rad", "bar"],
                     [["radBelowFt", 1428], ["radBelowKts", 40], ["engageFpm", 200], ["collBand", 0.05], ["dropAboveTq", 0.98]],
                     []],
    ["HeadingHold",  ["hdg", "trn"],
                     [["hdgBelowKts", 5], ["blendToKts", 40], ["authority", 0.1]],
                     [["breakout", [0.05, 0.10, 0.20]]]],
    ["FlightDirector", ["vs", "ias", "pitch", "roll", "yaw", "hvrPitch", "hvrRoll"],
                     [["altGain", 10], ["vsMaxFpm", 1000], ["vsAccelFpm", 200], ["tqBand", 0.1], ["tqFpmGain", 1000], ["captureFt", 50],
                      ["iasAccelKts", 2], ["maxPitchDeg", 15], ["pitchRateDps", 3],
                      ["maxBankDeg", 30], ["turnRateDps", 3], ["bankPerDeg", 1], ["rollRateDps", 5], ["bankAboveKts", 20],
                      ["hvrDecelKts", 2],
                      ["collAuthority", 1.0], ["cycAuthority", 0.1], ["pedAuthority", 0.1]],
                     [["modes", []]]]
];

//The flight director's targets - {min, max, step, wraps} each, and only the modes Core knows.
//Declared in ft, kt and deg; held in m, m/s and deg.
private _fd = _fmc getOrDefault ["FlightDirector", createHashMap];
if (count _fd > 0) then {
    _fd set ["modes", (_fd get "modes") select {_x in ["ralt", "alt", "altp", "ias", "hdg", "nav", "hvr"]}];
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

_heli setVariable ["bmkhs_posIntX",     0.0];
_heli setVariable ["bmkhs_posIntY",     0.0];

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

//Control mixing - the aircraft's declared mixes, read once. A mix with no table or an
//unknown source or target is skipped.
private _mixes = [];
{
    private _target = ["pitch", "roll", "yaw"] find toLower getText (_x >> "target");
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
        diag_log format ["[BMKHS] MIXING CONFIG ERROR: %1 needs source, target and table[]. Skipped.", configName _x];
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
[_heli, "bmkhs_attHoldSubMode",         "pos", true] call bmkhs_fnc_utilSeed;   //pos, vel, att
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
