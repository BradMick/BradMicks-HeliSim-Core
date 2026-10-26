/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_utilConfigError

Description:
    A config error - to the RPT, and on screen once per session.

Parameters:
    _msg - The error [String]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_msg"];

diag_log text _msg;
if (isNil "bmkhs_configErrorsShown") then { bmkhs_configErrorsShown = createHashMap };
if (hasInterface && {!(_msg in bmkhs_configErrorsShown)}) then {
    bmkhs_configErrorsShown set [_msg, true];
    systemChat _msg;
};
