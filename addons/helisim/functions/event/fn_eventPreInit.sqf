#include "\bmkhs_helisim\functions\core\core.hpp"

#define BMKHS_SETTINGS_CATEGORY "BradMick's HeliSim"

[
    "bmkhs_cyclicCenterTrimMode",
    "CHECKBOX",
    ["Cyclic Center Trim Mode", "When enabled, the cyclic is locked out until re-centered"],
    [BMKHS_SETTINGS_CATEGORY, "Controls"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_pedalCenterTrimMode",
    "CHECKBOX",
    ["Pedal Center Trim Mode", "When enabled, the pedals are locked out until re-centered"],
    [BMKHS_SETTINGS_CATEGORY, "Controls"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_springlessCyclic",
    "CHECKBOX",
    ["Springless Cyclic", "When enabled, cyclic force trim is disabled. This is for users with force feedback or springless HOTAS"],
    [BMKHS_SETTINGS_CATEGORY, "Controls"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_springlessPedals",
    "CHECKBOX",
    ["Springless Pedals", "When enabled, pedal force trim is disabled. This is for users with force feedback or springless pedals. This option also disables Heading Hold"],
    [BMKHS_SETTINGS_CATEGORY, "Controls"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_keyboardStickyPitch",
    "CHECKBOX",
    ["Keyboard Sticky Pitch", "Keyboard only. When enabled, the key accumulates control position while held, in place of the rate command on this axis"],
    [BMKHS_SETTINGS_CATEGORY, "Keyboard"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_keyboardStickyRoll",
    "CHECKBOX",
    ["Keyboard Sticky Roll", "Keyboard only. When enabled, the key accumulates control position while held, in place of the rate command on this axis"],
    [BMKHS_SETTINGS_CATEGORY, "Keyboard"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_keyboardGamepadStickyYaw",
    "CHECKBOX",
    ["Keyboard/Gamepad Sticky Yaw", "Keyboard/Gamepad pedals only. When enabled, the key or button accumulates control position while held, in place of the rate command on this axis"],
    [BMKHS_SETTINGS_CATEGORY, "Keyboard"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_autoPedal",
    "CHECKBOX",
    ["Keyboard/Gamepad & Mouse Auto Pedal", "Keyboard/Gamepad and Mouse pedals only - ignored while HOTAS pedals are in use. DOES NOT WORK WITH STICKY YAW! When enabled, the pedals are automatically managed by AI"],
    [BMKHS_SETTINGS_CATEGORY, "Keyboard"],
    [true],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_autoTaxi",
    "CHECKBOX",
    ["Keyboard & Mouse Auto Taxi", "Keyboard and Mouse only. DOES NOT WORK WITH STICKY PITCH! When enabled, on the wheels the pitch keys or mouse hold a taxi speed - 6 kt forward or backward - and released they stop the aircraft"],
    [BMKHS_SETTINGS_CATEGORY, "Keyboard"],
    [true],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_mouseSense",
    "SLIDER",
    ["Mouse Sensitivity", "Controls the sensitivity of the Mouse Pitch / Roll / Pedal binds"],
    [BMKHS_SETTINGS_CATEGORY, "Controls"],
    [0.1, 1.0, 1.0, 1],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_helisimEnvironment",
    "LIST",
    ["Aircraft Environmental Settings", "Standard day is Sea Level and 15 deg C.\nEurope is a base altitude of 800ft, with a Summer temperature of 18 deg C and a Winter temperature of -3 deg C.\nThe Middle East is a base altitude of 1,800ft, with a Summer temperature of 31 deg C and a Winter temperature of 14 deg C.\nCentral Asia is a base altitude of 5000ft, with a Summer temperature of 22 deg C and a winter temperature of -4 deg C.\nAsia is a base altitude of 3100ft, with a Summer temperature of 21 deg C and a Winter temperature of -2 deg C."],
    [BMKHS_SETTINGS_CATEGORY, "Flight model"],
    //Each selection carries its own tooltip - without one the list shows the selection's number
    [[ISA_STD, EUROPE_SUMMER, EUROPE_WINTER, MIDDLE_EAST_SUMMER, MIDDLE_EAST_WINTER, CENTRAL_ASIA_SUMMER, CENTRAL_ASIA_WINTER, ASIA_SUMMER, ASIA_WINTER], [
        ["Standard Day",          "Sea Level and 15 deg C."],
        ["Europe - Summer",       "A base altitude of 800ft and a temperature of 18 deg C."],
        ["Europe - Winter",       "A base altitude of 800ft and a temperature of -3 deg C."],
        ["Middle East - Summer",  "A base altitude of 1,800ft and a temperature of 31 deg C."],
        ["Middle East - Winter",  "A base altitude of 1,800ft and a temperature of 14 deg C."],
        ["Central Asia - Summer", "A base altitude of 5000ft and a temperature of 22 deg C."],
        ["Central Asia - Winter", "A base altitude of 5000ft and a temperature of -4 deg C."],
        ["Asia - Summer",         "A base altitude of 3100ft and a temperature of 21 deg C."],
        ["Asia - Winter",         "A base altitude of 3100ft and a temperature of -2 deg C."]
    ], 1],
    //The server's, for every client: every aircraft in the mission is in the same environment
    1
] call CBA_fnc_addSetting;

[
    "bmkhs_vrsWarning",
    "CHECKBOX",
    ["Enable VRS Warning", "When enabled, will alert the pilot to the onset of VRS"],
    [BMKHS_SETTINGS_CATEGORY, "Display"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_testGwtEnabled",
    "CHECKBOX",
    ["Fixed Test Gross Weight", "When enabled, the aircraft is held at the test weight below regardless of fuel and stores. For testing, so a result is not confounded by burn-off."],
    [BMKHS_SETTINGS_CATEGORY, "Testing"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_testGwtLbs",
    "EDITBOX",
    ["Test Gross Weight (lbs)", "The gross weight held when Fixed Test Gross Weight is enabled, clamped to the aircraft's declared empty and max gross weights. CG is scaled to match."],
    [BMKHS_SETTINGS_CATEGORY, "Testing"],
    ["18000"],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_windDisabled",
    "CHECKBOX",
    ["Disable Wind", "When enabled, the flight model sees still air regardless of the mission weather. For testing, so a result is not confounded by whatever the wind is doing."],
    [BMKHS_SETTINGS_CATEGORY, "Testing"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_fmDebug",
    "CHECKBOX",
    ["Enable FM Debugging", "Displays debug output for troubleshooting FM issues"],
    [BMKHS_SETTINGS_CATEGORY, "Testing"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_sysDebug",
    "CHECKBOX",
    ["Enable Systems Debugging", "Shows every circuit, what feeds it, and whether each component is awake or asleep. Takes over the hint from FM debugging while on."],
    [BMKHS_SETTINGS_CATEGORY, "Testing"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_engDisplay",
    "CHECKBOX",
    ["Enable Engine Display", "Engine readout - torque, Np, Nr, TGT, Ng, oil and the active rating. Always shown on an aircraft without a systems model; this adds it to one that has its own cockpit display, so the two can be compared."],
    [BMKHS_SETTINGS_CATEGORY, "Testing"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_forcesDebug",
    "CHECKBOX",
    ["Enable Forces Readout", "Window listing every force the model applies each frame - what each rotor, panel and surface produces, and where it acts relative to the centre of mass. Needs FM Debugging on, which is what makes the contributors publish."],
    [BMKHS_SETTINGS_CATEGORY, "Testing"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_flightLog",
    "CHECKBOX",
    ["Enable Flight Log", "Writes the flight to the RPT 10 times a second - air data, attitude, engines, the pilot's controls and trims, and every FMC output, mode and target - as BMKHSLOG lines under a BMKHSLOG_HDR header, for analysis."],
    [BMKHS_SETTINGS_CATEGORY, "Testing"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_rotorModel",
    "LIST",
    ["Rotor Model", "Selects the rotor aerodynamic model. Simple is stable and performant. BET (Blade Element Theory) is higher fidelity with induced flow, dissymmetry of lift, and flapping dynamics."],
    [BMKHS_SETTINGS_CATEGORY, "Flight model"],
    [[0, 1], ["Simple", "BET [INOP]"], 0],
    0
] call CBA_fnc_addSetting;

//Flight control indicator
[
    "bmkhs_ctrlVisColor",
    "LIST",
    ["Control Input Visualiser: Colour Scheme", "Colour theme for the Control Input Visualiser indicators"],
    [BMKHS_SETTINGS_CATEGORY, "Display"],
    [[0, 1, 2, 3, 4, 5], [
        "Default (Green / Orange / Red)",
        "NVG (All Green)",
        "Monochrome (White / Grey)",
        "Amber (Amber / Yellow / Red)",
        "Blue Force (Cyan / White / Yellow)",
        "High Contrast (White / Yellow / Red)"
    ], 0],
    2
] call CBA_fnc_addSetting;

//Auto pedal and sticky yaw cannot both own the pedals, so selecting one clears the other.
["bmkhs_autoPedal", {
    params ["_value"];
    if (_value) then {
        ["bmkhs_keyboardGamepadStickyYaw", false, false, true] call CBA_settings_fnc_set;
    };
}] call CBA_settings_fnc_addEventHandler;
["bmkhs_keyboardGamepadStickyYaw", {
    params ["_value"];
    if (_value) then {
        ["bmkhs_autoPedal", false, false, true] call CBA_settings_fnc_set;
    };
}] call CBA_settings_fnc_addEventHandler;
//Auto taxi and sticky pitch cannot both own the pitch keys on the wheels, the same.
["bmkhs_autoTaxi", {
    params ["_value"];
    if (_value) then {
        ["bmkhs_keyboardStickyPitch", false, false, true] call CBA_settings_fnc_set;
    };
}] call CBA_settings_fnc_addEventHandler;
["bmkhs_keyboardStickyPitch", {
    params ["_value"];
    if (_value) then {
        ["bmkhs_autoTaxi", false, false, true] call CBA_settings_fnc_set;
    };
}] call CBA_settings_fnc_addEventHandler;

bmkhs_keyboardCollective         = true;
bmkhs_keyboardCollectivePrevious = true;
//Control scheme last used on each axis: kb | hotasGamepad | mouse - the pedal's, kbGamepad | hotas | mouse
bmkhs_pitchScheme                = "kb";
bmkhs_rollScheme                 = "kb";
bmkhs_pedalScheme                = "kbGamepad";
bmkhs_lastFrameGetIn            = false;
//The environment's weather, one for every aircraft this machine flies: when it was last solved, the
//setting's base altitude (ft), base FAT, swing and dew point (deg C) and average wind (m/s), where the
//day is in its cycle, what the overcast makes of it, and the rain - how far it has taken the air to
//saturation (0..1), its rate (mm/h) and when it was last stepped (never, until the first solve).
//fn_environmentWeather solves it.
bmkhs_envWeatherT                = -ENV_WEATHER_PERIOD;
bmkhs_envBaseAlt                 = 0.0;
bmkhs_envBaseFat                 = 0.0;
bmkhs_envSwing                   = 0.0;
bmkhs_envDewPoint                = 0.0;
bmkhs_envAvgWind                 = 0.0;
bmkhs_envCycle                   = false;
bmkhs_envIsDay                   = true;
bmkhs_envShare                   = 0.0;
bmkhs_envLayerSec                = 0.0;
bmkhs_envCloudDay                = 1.0;
bmkhs_envCloudNight              = 1.0;
bmkhs_envRain                    = 0.0;
bmkhs_envRainRate                = 0.0;
bmkhs_envRainT                   = -1.0;

//The aircraft this player crews but does not own. A pack ticks only what its machine owns, so
//Core hands this one to coreUpdate itself - there is nothing for a pack to add.
if (hasInterface) then {
    bmkhs_crewedRemoteHandler = addMissionEventHandler ["EachFrame", {
        private _heli = vehicle player;
        if (!local _heli && {alive _heli} && {_heli getVariable ["bmkhs_initialised", false]}) then {
            [_heli] call bmkhs_fnc_coreUpdate;
        };
    }];
};

//private _nonAnalogEvents = ["Activate", "Deactivate"];
//
//{
//    addUserActionEventHandler ["bmkhs_kbCollectiveUp", _x, {bmkhs_keyboardCollective = true;}];
//    addUserActionEventHandler ["bmkhs_kbCollectiveDn", _x, {bmkhs_keyboardCollective = true;}];
//} forEach _nonAnalogEvents;
//
//private _analogEvents = ["Analog"];
//
//{
//    addUserActionEventHandler ["bmkhs_collectiveUp", _x, {bmkhs_keyboardCollective = false;}];
//    addUserActionEventHandler ["bmkhs_collectiveDn", _x, {bmkhs_keyboardCollective = false;}];
//} forEach _analogEvents;
