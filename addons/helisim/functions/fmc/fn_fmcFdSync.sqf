/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcFdSync

Description:
    Sets a flight director target to where the aircraft is now.

Parameters:
    _heli   - The helicopter [Object]
    _target - "ralt", "alt", "altp", "ias" or "hdg" [String]
    _snap   - Snap to the target's step [Bool, default true]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_target", ["_snap", true]];
#include "\bmkhs_helisim\functions\core\core.hpp"

private _now = switch (_target) do {
    case "ralt": { _heli getVariable "bmkhs_radAlt" };
    case "alt";
    case "altp": { (_heli getVariable "bmkhs_barAlt") * FEET_TO_METERS };
    case "ias":  { (_heli getVariable "bmkhs_velModelSpace") select 1 };
    case "hdg":  { getDir _heli };
    default      { nil };
};
if (isNil "_now") exitWith {};

[_heli, _target, _now, _snap] call bmkhs_fnc_fmcFdTarget;
