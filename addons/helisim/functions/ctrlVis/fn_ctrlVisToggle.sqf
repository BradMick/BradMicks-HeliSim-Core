/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_ctrlVisToggle

Description:
    Shows or hides the flight control indicator.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli"];

if !(_heli getVariable ["bmkhs_initialised", false]) exitWith {};

private _layer = "bmkhs_ctrlvis" call BIS_fnc_rscLayer;

if !(isNull (uiNamespace getVariable ["bmkhs_ctrlvis", displayNull])) then {
    //Hide - clear the cached statics so they are recomputed on next open
    _layer cutText ["", "PLAIN", 0, false];
    uiNamespace setVariable ["bmkhs_ctrlvis",        displayNull];
    uiNamespace setVariable ["bmkhs_ctrlVisColors",  []];
    uiNamespace setVariable ["bmkhs_ctrlVisCircleW", nil];
} else {
    _layer cutRsc ["bmkhs_ctrlvis", "PLAIN", 0, false];
};
