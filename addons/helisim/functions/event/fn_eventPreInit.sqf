#include "\bmkhs_helisim\functions\core\core.hpp"

#define BMKHS_SETTINGS_CATEGORY "BradMick's HeliSim"

[
    "bmkhs_helisimRealismSetting",
    "LIST",
    ["Aircraft Realism Settings", "Switch helisim between different realism levels, Casual recomneded for keyboard"],
    [BMKHS_SETTINGS_CATEGORY, "Flight model"],
    [[CASUAL, REALISTIC],["Casual", "Realistic"],0],
    0
] call CBA_fnc_addSetting;

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
    ["Keyboard Sticky Pitch", "DO NOT USE THIS IF USING HOTAS OR GAMEPAD! When enabled, keyboard input is continously updated while the input key is held down"],
    [BMKHS_SETTINGS_CATEGORY, "Keyboard"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_keyboardStickyRoll",
    "CHECKBOX",
    ["Keyboard Sticky Roll", "DO NOT USE THIS IF USING HOTAS OR GAMEPAD! When enabled, keyboard input is continously updated while the input key is held down"],
    [BMKHS_SETTINGS_CATEGORY, "Keyboard"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_keyboardStickyYaw",
    "CHECKBOX",
    ["Keyboard Sticky Yaw", "DO NOT USE THIS IF USING HOTAS OR GAMEPAD! When enabled, keyboard input is continously updated while the input key is held down"],
    [BMKHS_SETTINGS_CATEGORY, "Keyboard"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_autoPedal",
    "CHECKBOX",
    ["Keyboard Auto Pedal", "DOES NOT WORK WITH STICKY YAW! When enabled, the pedals are automatically managed by AI"],
    [BMKHS_SETTINGS_CATEGORY, "Keyboard"],
    [true],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_autoPitch",
    "CHECKBOX",
    ["Keyboard Auto Pitch", "When enabled, the aircraft is held at its level flight pitch attitude. CASUAL only."],
    [BMKHS_SETTINGS_CATEGORY, "Keyboard"],
    [true],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_autoRoll",
    "CHECKBOX",
    ["Keyboard Auto Roll", "When enabled, the roll key commands a bank angle up to the airframe's limit and returns to wings level on release. CASUAL only."],
    [BMKHS_SETTINGS_CATEGORY, "Keyboard"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_mouseAsJoystick",
    "CHECKBOX",
    ["Mouse as Joystick", "DO NOT USE THIS IF USING HOTAS OR GAMEPAD! Enables the mouse sensitivity option and stops input in freelook"],
    [BMKHS_SETTINGS_CATEGORY, "Controls"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_mouseSense",
    "SLIDER",
    ["Mouse Sensitivity", "Controls the sensitivity of the Mouse when used as a Joystick"],
    [BMKHS_SETTINGS_CATEGORY, "Controls"],
    [0.1, 1.0, 1.0, 1],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_helisimEnvironment",
    "LIST",
    ["Aircraft Environmental Settings", "Standard day is Sea Level and 15 deg C.\nEurope is a base altitude of 800ft, with a Summer temperature of 20 deg C and a Winter temperature of 0 deg C.\nThe Middle East is a base altitude of 1,800ft and a temperature of 30 deg C.\nCentral Asia is a base altitude of 5000ft, with a Summer temperature of 30 deg C and a winter temperature of -5 deg C.\nAsia is a base altitude of 3100ft and a temperature of 25 deg C."],
    [BMKHS_SETTINGS_CATEGORY, "Flight model"],
    [[ISA_STD, EUROPE_SUMMER, EUROPE_WINTER, MIDDLE_EAST, CENTRAL_ASIA_SUMMER, CENTRAL_ASIA_WINTER, ASIA],["Standard Day", "Europe - Summer", "Europe - Winter", "Middle East", "Central Asia - Summer", "Central Asia - Winter", "Asia"],1],
    0
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
    "bmkhs_forcesDebug",
    "CHECKBOX",
    ["Enable Forces Readout", "Window listing every force the model applies each frame - what each rotor, panel and surface produces, and where it acts relative to the centre of mass. Needs FM Debugging on, which is what makes the contributors publish."],
    [BMKHS_SETTINGS_CATEGORY, "Testing"],
    [false],
    2
] call CBA_fnc_addSetting;

[
    "bmkhs_rotorModel",
    "LIST",
    ["Rotor Model", "Selects the rotor aerodynamic model. Simple is stable and performant. BET (Blade Element Theory) is higher fidelity with induced flow, dissymmetry of lift, and flapping dynamics."],
    [BMKHS_SETTINGS_CATEGORY, "Flight model"],
    [[0, 1], ["Simple", "BET [WIP]"], 0],
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

//An auto assist and a sticky axis cannot both own the same control, so selecting an auto
//clears the sticky settings it conflicts with.
["bmkhs_autoPitch", {
    params ["_value"];
    if (_value) then {
        ["bmkhs_keyboardStickyPitch", false, false, true] call CBA_settings_fnc_set;
    };
}] call CBA_settings_fnc_addEventHandler;

["bmkhs_autoRoll", {
    params ["_value"];
    if (_value) then {
        ["bmkhs_keyboardStickyRoll", false, false, true] call CBA_settings_fnc_set;
    };
}] call CBA_settings_fnc_addEventHandler;

["bmkhs_autoPedal", {
    params ["_value"];
    if (_value) then {
        ["bmkhs_keyboardStickyYaw", false, false, true] call CBA_settings_fnc_set;
    };
}] call CBA_settings_fnc_addEventHandler;

bmkhs_keyboardCollective         = true;
bmkhs_keyboardCollectivePrevious = true;

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
