/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stateAltitude

Description:
    Returns the current AGL and MSL altitude of the helicopter

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

private _barAlt  = _heli getVariable "bmkhs_PA";
_barAlt = [_barAlt, 0.0, 20000] call bis_fnc_clamp;

//Both in METRES - the flight model works in metres, and a caller that wants feet
//says so. RAW is what it is; the other carries the altimeter's own rounding.
private _radAltRaw = getPos _heli # 2;

private _radAlt    = _radAltRaw;
if (_radAlt > RADALT_ROUND_ABOVE) then {
    _radAlt = round (_radAlt / RADALT_ROUND_STEP) * RADALT_ROUND_STEP;
};
_radAlt     = [_radAlt, 0.0, RADALT_MAX] call bis_fnc_clamp;

_heli setVariable ["bmkhs_barAlt",    _barAlt];
_heli setVariable ["bmkhs_radAlt",    _radAlt];
_heli setVariable ["bmkhs_radAltRaw", _radAltRaw];
