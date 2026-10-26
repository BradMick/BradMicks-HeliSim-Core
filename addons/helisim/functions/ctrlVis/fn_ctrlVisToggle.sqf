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
    uiNamespace setVariable ["bmkhs_ctrlvisHeli",    objNull];
} else {
    _layer cutRsc ["bmkhs_ctrlvis", "PLAIN", 0, false];
    uiNamespace setVariable ["bmkhs_ctrlvisHeli", _heli];
    //Closes once the player is no longer crew of the aircraft that opened it - the update
    //stops for a remote or shut-down aircraft, so it cannot do this itself
    if (isNil "bmkhs_ctrlVisWatch") then {
        bmkhs_ctrlVisWatch = addMissionEventHandler ["EachFrame", {
            private _heli = uiNamespace getVariable ["bmkhs_ctrlvisHeli", objNull];
            if (isNull (uiNamespace getVariable ["bmkhs_ctrlvis", displayNull])) exitWith {
                removeMissionEventHandler ["EachFrame", _thisEventHandler];
                bmkhs_ctrlVisWatch = nil;
            };
            if (isNull _heli || {!alive player} || {driver _heli != player && {gunner _heli != player}}) then {
                ("bmkhs_ctrlvis" call BIS_fnc_rscLayer) cutText ["", "PLAIN", 0, false];
                uiNamespace setVariable ["bmkhs_ctrlvis",        displayNull];
                uiNamespace setVariable ["bmkhs_ctrlVisColors",  []];
                uiNamespace setVariable ["bmkhs_ctrlVisCircleW", nil];
                uiNamespace setVariable ["bmkhs_ctrlvisHeli",    objNull];
                removeMissionEventHandler ["EachFrame", _thisEventHandler];
                bmkhs_ctrlVisWatch = nil;
            };
        }];
    };
};
