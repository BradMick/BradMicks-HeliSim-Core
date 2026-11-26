//Environment probe: reads everything the environment model takes from Arma and writes it to the RPT, so the
//units, the ranges and what drives each value can be read off the log. Editor preview (singleplayer), debug
//console, LOCAL EXEC, then close the pause menu - it runs while the game runs. It CHANGES the mission's time,
//wind, gusts, overcast and rain as it goes and puts them back at the end, so use a scratch mission. About
//5 minutes of game time. Run again to stop early.
//Where it has got to stays on screen: the step, what it is doing now, the time gone and to go, and each step
//as done, NOW or to do. Each step is also named in the chat as it starts, and it says FINISHED at the end.
//ACE: runs with it loaded. ACE's wind simulation is logged as it is, with gusts off and full, then held off by
//its own ace_weather_disableWindSimulation switch for the runs that set the wind, and let go again after. ACE's
//temperature and humidity are brought up to the moment and logged beside Arma's.
//  ENVLOG,INFO   - world, position, every value once, mods that own the weather, Core's own air data
//  ENVLOG,WIND   - the wind 10 times a second: as the mission has it, under ACE's wind with gusts 0 and 1, then
//                  with ACE's wind held off: gusts 0 and 1 on the mission's wind, and on 5 m/s set blowing east
//  ENVLOG,DAY    - every half hour of today: sun, ambientTemperature, the sunrise/sunset function
//  ENVLOG,MONTH  - the 15th of every month at 03:00 and 15:00
//  ENVLOG,CLOUD  - overcast 0, 0.5 and 1: what reads back, the temperature, cloud density up the column
//  ENVLOG,RAIN   - rain on, then off, once a second: rain, humidity, fog, temperature
//  ENVLOG,STEP   - each step as it starts          ENVLOG,END - finished, or stopped
//A row is ENVLOG,<block>,<mission seconds since the start>,name=value,... Arrays read [a|b|c]. A command this
//game version lacks reads "unsupported".
if (!isNil "bmkhs_envLogScript") exitWith {
    terminate bmkhs_envLogScript;
    if (!isNil "bmkhs_envLogTicker") then { terminate bmkhs_envLogTicker };
    bmkhs_envLogScript = nil;
    bmkhs_envLogStatus = nil;
    call bmkhs_envLogRestore;
    diag_log text "ENVLOG,END,stopped";
    systemChat "Environment log: STOPPED before the end - time and weather put back";
    hint "ENVIRONMENT LOG - STOPPED\nStopped before the end.\nTime and weather put back.\n\nRun the script again to start over.";
};

//What the mission had, and how it goes back
bmkhs_envLogOrig    = [date, wind, gusts, overcast, rain, missionNamespace getVariable ["ace_weather_disableWindSimulation", false]];
bmkhs_envLogRestore = {
    bmkhs_envLogOrig params ["_date", "_wind", "_gusts", "_overcast", "_rain", "_aceWindHeld"];
    if (isClass (configFile >> "CfgPatches" >> "ace_weather")) then { ace_weather_disableWindSimulation = _aceWindHeld };
    setDate _date;
    setWind [_wind select 0, _wind select 1, false];
    0 setGusts _gusts;
    0 setRain _rain;
    0 setOvercast _overcast;
    forceWeatherChange;
};

//The steps, and the seconds of game time each is expected to take - the wind runs are two more with ACE
bmkhs_envLogSteps = [
    ["the mission as it is",     1],
    ["wind and gusts",           [125, 175] select (isClass (configFile >> "CfgPatches" >> "ace_weather"))],
    ["the day, every half hour", 30],
    ["the year, month by month", 15],
    ["overcast and cloud",       35],
    ["rain and humidity",        45]
];
//Where the log has got to: [step, what it is doing now, when the step started, when the log started]
bmkhs_envLogStatus = [1, "starting", time, time];

//On screen once a second
bmkhs_envLogTicker = [] spawn {
    private _clock = {
        private _seconds = round _this;
        format ["%1:%2%3", floor (_seconds / 60), ["", "0"] select ((_seconds mod 60) < 10), _seconds mod 60]
    };
    while { !isNil "bmkhs_envLogStatus" } do {
        bmkhs_envLogStatus params ["_step", "_doing", "_stepStart", "_start"];
        private _left = (((bmkhs_envLogSteps select (_step - 1)) select 1) - (time - _stepStart)) max 0;
        { if (_forEachIndex + 1 > _step) then { _left = _left + (_x select 1) } } forEach bmkhs_envLogSteps;
        private _lines = [
            "ENVIRONMENT LOG - RUNNING",
            format ["Step %1 of %2: %3", _step, count bmkhs_envLogSteps, (bmkhs_envLogSteps select (_step - 1)) select 0],
            format ["Now: %1", _doing],
            format ["%1 gone, about %2 to go", (time - _start) call _clock, _left call _clock],
            ""
        ];
        {
            private _state = if (_forEachIndex + 1 < _step) then { "done" } else { ["to do", "NOW"] select (_forEachIndex + 1 == _step) };
            _lines pushBack format ["%1. %2 - %3", _forEachIndex + 1, _x select 0, _state];
        } forEach bmkhs_envLogSteps;
        _lines append ["", "It runs only while the game is unpaused.", "Run the script again to stop it."];
        hintSilent (_lines joinString "\n");
        sleep 1;
    };
};

bmkhs_envLogScript = [] spawn {
    private _t0 = time;
    bmkhs_envLogOrig params ["_date0", "_wind0", "_gusts0"];
    _date0 params ["_year", "_month", "_day"];
    private _pos = getPosASL player;

    //A value as text: numbers to 4 places, an array's elements between | marks
    private _fmt = {
        if (_this isEqualType 0) exitWith { _this toFixed 4 };
        if (_this isEqualType "") exitWith { _this };
        if (_this isEqualType []) exitWith { "[" + ((_this apply { _x call _fmt }) joinString "|") + "]" };
        str _this
    };

    private _log = {
        params ["_block", "_items"];
        diag_log text ((["ENVLOG", _block, (time - _t0) toFixed 2] + _items) joinString ",");
    };

    //A step starts: on screen, in the chat and in the RPT
    private _step = {
        params ["_index"];
        private _name = (bmkhs_envLogSteps select (_index - 1)) select 0;
        bmkhs_envLogStatus = [_index, "starting", time, _t0];
        systemChat format ["Environment log: step %1 of %2 - %3", _index, count bmkhs_envLogSteps, _name];
        ["STEP", [format ["step=%1", _index], format ["name=%1", _name]]] call _log;
    };
    //What the step is doing now
    private _now = { bmkhs_envLogStatus set [1, _this] };

    //supportInfo is trusted only if it knows a command every version has; where it does not, every command is tried
    private _maskOk = (supportInfo "n:dayTime") isNotEqualTo [];

    //[name, supportInfo mask, expression] -> [name, code]
    private _prep = {
        _this apply {
            _x params ["_name", "_mask", "_expr"];
            [_name, if (_maskOk && {(supportInfo _mask) isEqualTo []}) then { { "unsupported" } } else { compile _expr }]
        }
    };

    //One row of a block: each reading as name=value, after any lead items
    private _row = {
        params ["_block", "_set", ["_lead", []]];
        [_block, _lead + (_set apply {
            private _value = call (_x select 1);
            format ["%1=%2", _x select 0, if (isNil "_value") then { "nil" } else { _value call _fmt }]
        })] call _log;
    };

    private _windSet = [
        ["wind",    "n:wind",    "wind"],
        ["windSpd", "n:wind",    "vectorMagnitude wind"],
        ["windDir", "n:windDir", "windDir"],
        ["windStr", "n:windStr", "windStr"],
        ["gusts",   "n:gusts",   "gusts"]
    ] call _prep;

    private _sunSet = [
        ["date",               "n:date",               "date"],
        ["dayTime",            "n:dayTime",            "dayTime"],
        ["sunOrMoon",          "n:sunOrMoon",          "sunOrMoon"],
        ["sunriseSunset",      "n:date",               "if (isNil 'BIS_fnc_sunriseSunsetTime') then { 'no function' } else { date call BIS_fnc_sunriseSunsetTime }"],
        ["ambientTemperature", "n:ambientTemperature", "ambientTemperature"],
        ["getLighting",        "n:getLighting",        "getLighting"]
    ] call _prep;

    private _wxSet = [
        ["overcast",           "n:overcast",           "overcast"],
        ["overcastForecast",   "n:overcastForecast",   "overcastForecast"],
        ["rain",               "n:rain",               "rain"],
        ["rainParams",         "n:rainParams",         "rainParams"],
        ["humidity",           "n:humidity",           "humidity"],
        ["fog",                "n:fog",                "fog"],
        ["fogParams",          "n:fogParams",          "fogParams"],
        ["ambientTemperature", "n:ambientTemperature", "ambientTemperature"]
    ] call _prep;

    //ACE's own temperature and humidity, brought up to the moment and logged as a row of the block it sits in
    private _ace    = isClass (configFile >> "CfgPatches" >> "ace_weather");
    private _aceSet = [
        ["ace_weather_currentTemperature", "n:worldName", "missionNamespace getVariable ['ace_weather_currentTemperature', 'none']"],
        ["ace_weather_currentHumidity",    "n:worldName", "missionNamespace getVariable ['ace_weather_currentHumidity', 'none']"],
        ["ace_weather_currentOvercast",    "n:worldName", "missionNamespace getVariable ['ace_weather_currentOvercast', 'none']"]
    ] call _prep;
    private _aceRow = {
        params ["_block", ["_lead", []]];
        if (!_ace) exitWith {};
        if (!isNil "ace_weather_fnc_updateTemperature") then { call ace_weather_fnc_updateTemperature };
        if (!isNil "ace_weather_fnc_updateHumidity")    then { call ace_weather_fnc_updateHumidity };
        [_block, _aceSet, _lead] call _row;
    };

    //Once: where and when, who else owns the weather, and what Core makes of it now
    1 call _step;
    "reading every value once" call _now;
    ["INFO", [
        ["world",             "n:worldName",         "worldName"],
        ["latitude",          "n:worldName",         "getNumber (configFile >> 'CfgWorlds' >> worldName >> 'latitude')"],
        ["longitude",         "n:worldName",         "getNumber (configFile >> 'CfgWorlds' >> worldName >> 'longitude')"],
        ["posASL",            "n:worldName",         "_pos"],
        ["terrainASL",        "n:worldName",         "getTerrainHeightASL _pos"],
        ["timeMultiplier",    "n:timeMultiplier",    "timeMultiplier"],
        ["accTime",           "n:accTime",           "accTime"],
        ["nextWeatherChange", "n:nextWeatherChange", "nextWeatherChange"],
        ["lightnings",        "n:lightnings",        "lightnings"],
        ["supportInfoTrusted", "n:worldName",        "_maskOk"]
    ] call _prep] call _row;
    ["INFO", [
        ["aceWeatherLoaded",                  "n:worldName", "_ace"],
        ["ace_weather_enabled",               "n:worldName", "missionNamespace getVariable ['ace_weather_enabled', 'none']"],
        ["ace_weather_windSimulation",        "n:worldName", "missionNamespace getVariable ['ace_weather_windSimulation', 'none']"],
        ["ace_weather_disableWindSimulation", "n:worldName", "missionNamespace getVariable ['ace_weather_disableWindSimulation', 'none']"],
        ["ace_weather_updateInterval",        "n:worldName", "missionNamespace getVariable ['ace_weather_updateInterval', 'none']"]
    ] call _prep] call _row;
    ["INFO"] call _aceRow;
    ["INFO", [
        ["bmkhs_helisimEnvironment",       "n:worldName", "missionNamespace getVariable ['bmkhs_helisimEnvironment', 'none']"],
        ["vehicle",                        "n:worldName", "typeOf vehicle player"],
        ["bmkhs_fat",                      "n:worldName", "(vehicle player) getVariable ['bmkhs_fat', 'none']"],
        ["bmkhs_barAlt",                   "n:worldName", "(vehicle player) getVariable ['bmkhs_barAlt', 'none']"],
        ["bmkhs_rho",                      "n:worldName", "(vehicle player) getVariable ['bmkhs_rho', 'none']"],
        ["bmkhs_windSpeed",                "n:worldName", "(vehicle player) getVariable ['bmkhs_windSpeed', 'none']"],
        ["bmkhs_windDirFrom",              "n:worldName", "(vehicle player) getVariable ['bmkhs_windDirFrom', 'none']"]
    ] call _prep] call _row;
    ["INFO", _sunSet]  call _row;
    ["INFO", _windSet] call _row;
    ["INFO", _wxSet]   call _row;

    //The wind 10 times a second for a stretch of mission time, counted off on screen as one of the wind runs
    private _runs    = [5, 7] select _ace;
    private _runNo   = 0;
    private _windRun = {
        params ["_lead", "_seconds", "_what"];
        _runNo = _runNo + 1;
        format ["%1 - run %2 of %3, %4 s each", _what, _runNo, _runs, _seconds] call _now;
        private _end = time + _seconds;
        while { time < _end } do {
            ["WIND", _windSet, _lead] call _row;
            sleep 0.1;
        };
    };

    //As the mission has it. Then, where ACE is loaded, its wind simulation left running with gusts off and full,
    //and after that held off, so the wind set below is the wind read back.
    2 call _step;
    [["run=mission"], 20, "the wind as the mission has it"] call _windRun;
    if (_ace) then {
        {
            0 setGusts _x;
            format ["ACE wind on, gusts %1 - letting it settle", _x] call _now;
            sleep 5;
            [["run=aceWind", format ["gustsSet=%1", _x]], 20, format ["ACE wind on, gusts %1", _x]] call _windRun;
        } forEach [0, 1];
        ace_weather_disableWindSimulation = true;
    };
    //Gusts off and full: on the mission's own wind, and on 5 m/s set blowing east
    {
        _x params ["_forced", "_gustsSet"];
        if (_forced) then {
            setWind [5, 0, true];
        } else {
            setWind [_wind0 select 0, _wind0 select 1, false];
        };
        0 setGusts _gustsSet;
        private _what = format ["%1%2, gusts %3", ["", "ACE wind held off, "] select _ace, ["the mission wind", "5 m/s set blowing east"] select _forced, _gustsSet];
        (_what + " - letting it settle") call _now;
        sleep 5;
        [[format ["run=%1", ["free", "forced5east"] select _forced], format ["gustsSet=%1", _gustsSet]], 20, _what] call _windRun;
    } forEach [[false, 0], [false, 1], [true, 0], [true, 1]];
    setWind [_wind0 select 0, _wind0 select 1, false];
    0 setGusts _gusts0;
    if (_ace) then { ace_weather_disableWindSimulation = bmkhs_envLogOrig select 5 };

    //Today, every half hour
    3 call _step;
    for "_i" from 0 to 47 do {
        private _hour   = floor (_i / 2);
        private _minute = (_i mod 2) * 30;
        format ["%1:%2 today - %3 of 48", _hour, ["00", "30"] select (_minute > 0), _i + 1] call _now;
        setDate [_year, _month, _day, _hour, _minute];
        sleep 0.5;
        ["DAY", _sunSet] call _row;
        ["DAY"] call _aceRow;
    };

    //The year, before dawn and mid afternoon
    4 call _step;
    for "_m" from 1 to 12 do {
        {
            format ["month %1 of 12, the 15th at %2:00", _m, _x] call _now;
            setDate [_year, _m, 15, _x, 0];
            sleep 0.5;
            ["MONTH", _sunSet] call _row;
            ["MONTH"] call _aceRow;
        } forEach [3, 15];
    };
    setDate _date0;

    //Clear, half and full overcast: what reads back, the temperature at 03:00 and 15:00, and the cloud over the
    //player every 250 m from 0 to 8000 as the cloud commands give it
    5 call _step;
    private _hasDensity  = !_maskOk || {(supportInfo "u:simulCloudDensity") isNotEqualTo []};
    private _hasInClouds = !_maskOk || {(supportInfo "u:simulInClouds") isNotEqualTo []};
    private _density     = if (_hasDensity)  then { compile "simulCloudDensity _this" } else { {} };
    private _inClouds    = if (_hasInClouds) then { compile "simulInClouds _this" }     else { {} };
    {
        private _overcastSet = _x;
        private _what        = format ["overcast %1 - %2 of 3", _overcastSet, _forEachIndex + 1];
        (_what + ", changing the sky - the game may freeze for a moment") call _now;
        0 setOvercast _overcastSet;
        forceWeatherChange;
        sleep 8;
        {
            (_what + format [", the temperature at %1:00", _x]) call _now;
            setDate [_year, _month, _day, _x, 0];
            sleep 0.5;
            ["CLOUD", _wxSet,  [format ["overcastSet=%1", _overcastSet]]] call _row;
            ["CLOUD", _sunSet, [format ["overcastSet=%1", _overcastSet]]] call _row;
            ["CLOUD", [format ["overcastSet=%1", _overcastSet]]] call _aceRow;
        } forEach [3, 15];
        (_what + ", the cloud up the column") call _now;
        {
            _x params ["_name", "_has", "_code"];
            if (_has) then {
                private _column = [];
                for "_z" from 0 to 8000 step 250 do {
                    private _value = [_pos select 0, _pos select 1, _z] call _code;
                    _column pushBack (if (isNil "_value") then { "nil" } else { _value call _fmt });
                };
                ["CLOUD", [format ["overcastSet=%1", _overcastSet], format ["%1Every250mFrom0=%2", _name, _column joinString "|"]]] call _log;
            } else {
                ["CLOUD", [format ["overcastSet=%1", _overcastSet], format ["%1=unsupported", _name]]] call _log;
            };
        } forEach [["simulCloudDensity", _hasDensity, _density], ["simulInClouds", _hasInClouds, _inClouds]];
    } forEach [0, 0.5, 1];
    setDate _date0;

    //Full overcast is still set: rain on for 20 s, then off for 20 s
    6 call _step;
    {
        private _rainSet = _x;
        0 setRain _rainSet;
        forceWeatherChange;
        for "_i" from 1 to 20 do {
            format ["rain %1 - second %2 of 20", ["off", "on"] select (_rainSet > 0), _i] call _now;
            ["RAIN", _wxSet, [format ["rainSet=%1", _rainSet]]] call _row;
            sleep 1;
        };
    } forEach [1, 0];

    //Finished: the counter off the screen first, so the last word is this one
    terminate bmkhs_envLogTicker;
    bmkhs_envLogStatus = nil;
    call bmkhs_envLogRestore;
    ["END", ["finished"]] call _log;
    bmkhs_envLogScript = nil;
    private _took = round (time - _t0);
    systemChat format ["Environment log: FINISHED - all %1 steps, %2 s. Time and weather put back.", count bmkhs_envLogSteps, _took];
    hint format ["ENVIRONMENT LOG - FINISHED\nAll %1 steps done in %2 s.\nTime and weather put back.\n\nNothing more to do in game.", count bmkhs_envLogSteps, _took];
};
