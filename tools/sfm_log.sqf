//Flight recorder for a VANILLA helicopter on the standard flight model: the pilot's inputs, what the
//aircraft does, and its flight config. Every frame, to the RPT. Debug console, LOCAL EXEC. Run again to stop.
//  SFMLOG_INFO  - aircraft, mass, flight model, weather       SFMLOG_CFG   - every number in its config
//  SFMLOG_ANIMS - every animation it has                      SFMLOG_HDR   - the column names
//  SFMLOG       - one row per frame ("SFMLOG+" continues a row)
if (!isNil "bmkhs_sfmLogEH") exitWith {
    removeMissionEventHandler ["EachFrame", bmkhs_sfmLogEH];
    bmkhs_sfmLogEH = nil;
    diag_log text "SFMLOG_END";
    hint "SFM log stopped";
};

private _veh = vehicle player;
if !(_veh isKindOf "Helicopter") exitWith { hint "Get in a helicopter first" };

//Writes [tag, items] as lines under 900 characters, the pieces after the first tagged tag+
private _lines = {
    params ["_tag", "_items"];
    private _buf  = [];
    private _len  = 0;
    private _cont = "";
    {
        if (_len + (count _x) + 1 > 900 && {_buf isNotEqualTo []}) then {
            diag_log text (_tag + _cont + "," + (_buf joinString ","));
            _cont = "+";
            _buf  = [];
            _len  = 0;
        };
        _buf pushBack _x;
        _len = _len + (count _x) + 1;
    } forEach _items;
    diag_log text (_tag + _cont + "," + (_buf joinString ","));
};

//Every number and number array a config class holds, inherited ones included
private _cfgDump = {
    params ["_cfg", "_label"];
    private _items = [_label];
    {
        if (isNumber _x) then { _items pushBack format ["%1=%2", configName _x, getNumber _x] };
        if (isArray _x) then {
            private _a = getArray _x;
            if (_a isNotEqualTo [] && {count _a <= 64} && {(_a findIf {!(_x isEqualType 0)}) < 0}) then {
                _items pushBack format ["%1=%2", configName _x, (_a apply {str _x}) joinString "|"];
            };
        };
    } forEach configProperties [_cfg, "true", true];
    ["SFMLOG_CFG", _items] call _lines;
};

private _cfg = configOf _veh;
private _vec = { (_this apply {_x toFixed 3}) joinString "|" };

["SFMLOG_INFO", [
    "type=" + typeOf _veh,
    "name=" + getText (_cfg >> "displayName"),
    "simulation=" + getText (_cfg >> "simulation"),
    "massKg=" + ((getMass _veh) toFixed 1),
    "com=" + ((getCenterOfMass _veh) call _vec),
    "boundingCenter=" + ((boundingCenter _veh) call _vec),
    "boundingBoxMin=" + (((0 boundingBoxReal _veh) select 0) call _vec),
    "boundingBoxMax=" + (((0 boundingBoxReal _veh) select 1) call _vec),
    "crew=" + str (count crew _veh),
    "fuel=" + ((fuel _veh) toFixed 3),
    "wind=" + (wind call _vec),
    "gusts=" + (gusts toFixed 2),
    "overcast=" + (overcast toFixed 2),
    "rain=" + (rain toFixed 2),
    "world=" + worldName,
    "accTime=" + (accTime toFixed 2)
]] call _lines;

[_cfg, "vehicle"] call _cfgDump;
{ [_x, "Wheels/" + configName _x] call _cfgDump } forEach configProperties [_cfg >> "Wheels", "isClass _x", true];

private _animNames = animationNames _veh;
["SFMLOG_ANIMS", _animNames] call _lines;

//The actions a helicopter pilot can be giving - an unbound or unknown one reads 0
private _actions = [
    "HeliCyclicForward", "HeliCyclicBack", "HeliCyclicLeft", "HeliCyclicRight",
    "HeliCollectiveRaise", "HeliCollectiveLower", "HeliCollectiveRaiseCont", "HeliCollectiveLowerCont",
    "HeliRudderLeft", "HeliRudderRight",
    "HeliForward", "HeliBack", "HeliLeft", "HeliRight", "HeliUp", "HeliDown",
    "HeliThrottlePos", "HeliThrottleNeg", "HeliWheelsBrake",
    "HeliTrimForward", "HeliTrimBackward", "HeliTrimLeft", "HeliTrimRight", "HeliTrimRudderLeft", "HeliTrimRudderRight",
    "AutoHover", "AutoHoverCancel", "LookAround", "LookAroundToggle"
];

//The animations that move with the controls, rotors and gear
private _animKeys = ["cyclic", "stick", "collective", "pedal", "lever", "rudder", "elevator", "stab", "rotor", "gear", "wheel", "damper", "throttle", "horizon"];
private _anims = (_animNames select { private _n = toLower _x; (_animKeys findIf {_x in _n}) >= 0 }) select [0, 40];

//Animation sources the engine drives directly
private _sources = [
    "rpm", "rotorH", "rotorV", "rotorHFullyDestroyed", "cyclicForward", "cyclicAside", "collective", "pedal", "rudder",
    "elevator", "aileron", "gear", "throttle", "altRadar", "altBaro", "speed", "vertSpeed", "horizonBank", "horizonDive",
    "gmeter", "fuel", "damper", "wheel"
];

//What the engine feeds its own sounds - rotor speed, thrust and the like
private _sounds = (getAllSoundControllers _veh) apply {_x select 0};

["SFMLOG_HDR",
    ["t", "tick", "dt", "frame"]
    + (_actions apply {"in_" + _x})
    + ["autoHover", "engineOn", "touchingGround", "freeLook"]
    + ["posX", "posY", "posZasl", "altAGL", "altATL"]
    + ["velX", "velY", "velZ", "velRight", "velFwd", "velUp", "speedKmh"]
    + ["pitch", "bank", "hdg"]
    + ["dirX", "dirY", "dirZ", "upX", "upY", "upZ"]
    + ["wX", "wY", "wZ", "wPitchDps", "wRollDps", "wYawDps"]
    + ["windX", "windY", "windZ", "slopeX", "slopeY", "slopeZ"]
    + ["fuel", "damage", "massKg"]
    + (_anims apply {"anim_" + _x})
    + (_sources apply {"src_" + _x})
    + (_sounds apply {"snd_" + _x})
] call _lines;

bmkhs_sfmLogEH = addMissionEventHandler ["EachFrame", {
    _thisArgs params ["_veh", "_actions", "_anims", "_sources", "_lines"];
    if (!alive _veh || {vehicle player != _veh}) exitWith {
        removeMissionEventHandler ["EachFrame", _thisEventHandler];
        bmkhs_sfmLogEH = nil;
        diag_log text "SFMLOG_END";
        hint "SFM log stopped";
    };
    if (isGamePaused) exitWith {};

    private _b = { str ([0, 1] select _this) };
    (_veh call BIS_fnc_getPitchBank) params ["_pitch", "_bank"];
    private _pos = getPosASL _veh;

    private _row = [time toFixed 3, diag_tickTime toFixed 3, diag_deltaTime toFixed 4, str diag_frameNo];
    _row append (_actions apply {(inputAction _x) toFixed 3});
    _row append [(isAutoHoverOn _veh) call _b, (isEngineOn _veh) call _b, (isTouchingGround _veh) call _b, freeLook call _b];
    _row append ((_pos + [(getPos _veh) select 2, (getPosATL _veh) select 2]) apply {_x toFixed 3});
    _row append (((velocity _veh) + (velocityModelSpace _veh) + [speed _veh]) apply {_x toFixed 3});
    _row append ([_pitch, _bank, getDir _veh] apply {_x toFixed 3});
    _row append (((vectorDir _veh) + (vectorUp _veh)) apply {_x toFixed 5});
    //World, then model: + pitch forward, + roll right, + yaw right, deg/s
    _row append (((angularVelocity _veh) + (angularVelocityModelSpace _veh)) apply {(deg _x) toFixed 3});
    _row append ((wind + (surfaceNormal _pos)) apply {_x toFixed 3});
    _row append [(fuel _veh) toFixed 4, (damage _veh) toFixed 3, (getMass _veh) toFixed 1];
    _row append (_anims apply {(_veh animationPhase _x) toFixed 3});
    _row append (_sources apply {(_veh animationSourcePhase _x) toFixed 3});
    _row append ((getAllSoundControllers _veh) apply {(_x select 1) toFixed 3});

    ["SFMLOG", _row] call _lines;
}, [_veh, _actions, _anims, _sources, _lines]];

hint format ["SFM log running on %1", typeOf _veh];
