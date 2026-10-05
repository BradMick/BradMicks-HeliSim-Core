/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcFdSync

Description:
    Sets a flight director target to where the aircraft is now.

Parameters:
    _heli   - The helicopter [Object]
    _target - "ralt", "alt", "altp", "ias" or "hdg" [String]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_target"];
#include "\bmkhs_helisim\functions\core\core.hpp"

private _now = switch (_target) do {
    case "ralt": { (_heli getVariable "bmkhs_radAltRaw") * METERS_TO_FEET };
    case "alt";
    case "altp": { ((getPosASL _heli) select 2) * METERS_TO_FEET };
    case "ias":  { _heli getVariable "bmkhs_vel2D" };
    case "hdg":  { getDir _heli };
    default      { nil };
};
if (isNil "_now") exitWith {};

[_heli, _target, _now] call bmkhs_fnc_fmcFdTarget;
