/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_environmentWetBulb

Description:
    The wet-bulb temperature of air: the temperature it reaches once rain has
    saturated it. The rain evaporates into the air until it can hold no more,
    and the heat for the evaporation comes out of the air itself, so it ends
    up cooler as well as wetter (Stull, Practical Meteorology, eq. 4.17 and
    4.18).

Parameters:
    _temperature - The air's temperature, deg C [Number]
    _vapour      - Its vapour pressure, hPa [Number]
    _pressure    - Its pressure, hPa [Number]

Returns:
    The wet-bulb temperature, deg C [Number]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_temperature", "_vapour", "_pressure"];

//The heat the air gives up in cooling to the wet-bulb is the heat of the vapour it gains in saturating
//there. Halved in from 40 deg C below: 14 halvings is within 0.003 deg C.
private _mixing = (VAPOUR_MASS_RATIO * _vapour) / (_pressure - _vapour);
private _lo     = _temperature - 40.0;
private _hi     = _temperature;
for "_i" from 1 to 14 do {
    private _mid       = (_lo + _hi) / 2.0;
    //Saturation vapour pressure, hPa (WMO-No. 8)
    private _saturated = 6.112 * exp ((17.62 * _mid) / (243.12 + _mid));
    if ((SPECIFIC_HEAT_AIR * (_temperature - _mid)) > (LATENT_HEAT_VAPOUR * (((VAPOUR_MASS_RATIO * _saturated) / (_pressure - _saturated)) - _mixing))) then { _lo = _mid } else { _hi = _mid };
};

(_lo + _hi) / 2.0
