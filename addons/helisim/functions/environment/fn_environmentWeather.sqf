/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_environmentWeather

Description:
    The environment setting's own numbers, and what the mission's clock and
    sky make of the day: the weather, one for every aircraft this machine
    flies. Its globals are declared in fn_eventPreInit, and fn_environment
    calls this once every ENV_WEATHER_PERIOD.

    The setting is the server's, so every machine solves the same one.

Parameters:
    None

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

//The setting: base altitude ft, base FAT deg C - its mean - with its daily swing and dew point, deg C,
//and the average wind, m/s, and cloud, 0..1, that swing was measured in. A region is the mean of its
//countries' averages, summer June to August and winter December to February.
private _baseAlt  = 0.0;
private _baseFAT  = 0.0;
private _swing    = 0.0;
private _dewPoint = 0.0;
private _avgWind  = 0.0;
private _avgCloud = 0.0;
switch (bmkhs_helisimEnvironment) do {
    //Fixed and dry: no swing, and nothing reads its dew point, wind or cloud
    case ISA_STD: {
        _baseAlt  = 0;
        _baseFAT  = 15.0;
    };
    case EUROPE_SUMMER: {
        _baseAlt  = 800;
        _baseFAT  = 18.0;
        _swing    = 5.0;
        _dewPoint = 12.0;
        _avgWind  = 2.7;
        _avgCloud = 0.55;
    };
    case EUROPE_WINTER: {
        _baseAlt  = 800;
        _baseFAT  = -3.0;
        _swing    = 3.0;
        _dewPoint = -4.0;
        _avgWind  = 3.6;
        _avgCloud = 0.79;
    };
    case MIDDLE_EAST_SUMMER: {
        _baseAlt  = 1800;
        _baseFAT  = 31.0;
        _swing    = 7.0;
        _dewPoint = 14.0;
        _avgWind  = 3.2;
        _avgCloud = 0.25;
    };
    case MIDDLE_EAST_WINTER: {
        _baseAlt  = 1800;
        _baseFAT  = 14.0;
        _swing    = 5.0;
        _dewPoint = 6.0;
        _avgWind  = 2.7;
        _avgCloud = 0.41;
    };
    case CENTRAL_ASIA_SUMMER: {
        _baseAlt  = 5000;
        _baseFAT  = 22.0;
        _swing    = 7.0;
        _dewPoint = 8.0;
        _avgWind  = 2.5;
        _avgCloud = 0.31;
    };
    case CENTRAL_ASIA_WINTER: {
        _baseAlt  = 5000;
        _baseFAT  = -4.0;
        _swing    = 5.0;
        _dewPoint = -10.0;
        _avgWind  = 2.4;
        _avgCloud = 0.61;
    };
    case ASIA_SUMMER: {
        _baseAlt  = 3100;
        _baseFAT  = 21.0;
        _swing    = 5.0;
        _dewPoint = 16.0;
        _avgWind  = 1.9;
        _avgCloud = 0.63;
    };
    case ASIA_WINTER: {
        _baseAlt  = 3100;
        _baseFAT  = -2.0;
        _swing    = 5.0;
        _dewPoint = -7.0;
        _avgWind  = 2.5;
        _avgCloud = 0.43;
    };
};

//The day. No sunrise or no sunset, as inside a polar circle, is no daily cycle.
private _dayTime  = dayTime;
(date call BIS_fnc_sunriseSunsetTime) params ["_sunrise", "_sunset"];
private _cycle    = _sunrise >= 0.0 && {_sunset > _sunrise};
private _isDay    = true;
//By night, the share of the dawn's drop the surface has reached; by day, the share of it still to burn off
private _share    = 0.0;
//Seconds the night's cold layer has been cooling, by night, or had been by dawn, by day
private _layerSec = 0.0;
if (_cycle) then {
    private _dayLength   = _sunset - _sunrise;
    private _nightLength = 24.0 - _dayLength;
    _isDay = _dayTime >= _sunrise && {_dayTime < _sunset};
    if (_isDay) then {
        //The sun's heating so far, as a share of the day's (Stull 18.2), burns the night's cold layer off
        //from the ground up: 1 - u + u ln u of its heat is gone when u of the dawn drop is left at the surface.
        private _heat = (1.0 - (cos (180.0 * (_dayTime - _sunrise) / _dayLength))) / 2.0;
        private _lo   = 0.0;
        private _hi   = 1.0;
        for "_i" from 1 to 20 do {
            private _mid = (_lo + _hi) / 2.0;
            if (((1.0 - _mid) + (_mid * (ln _mid))) > _heat) then { _lo = _mid } else { _hi = _mid };
        };
        _share    = (_lo + _hi) / 2.0;
        _layerSec = _nightLength * 3600.0;
    } else {
        //The surface cools as the square root of the time since sunset (Stull 18.1, 18.4, 18.5)
        private _since = _dayTime - _sunset;
        if (_since < 0.0) then { _since = _since + 24.0 };
        _share    = sqrt (_since / _nightLength);
        _layerSec = _since * 3600.0;
    };
};

//Cloud against the region's average: the day's warming by the sunlight it lets through, the night's
//cooling by the heat it lets out
private _overcast   = overcast;
private _cloudDay   = (1.0 - (ENV_CLOUD_DAY * _overcast))   / (1.0 - (ENV_CLOUD_DAY * _avgCloud));
private _cloudNight = (1.0 - (ENV_CLOUD_NIGHT * _overcast)) / (1.0 - (ENV_CLOUD_NIGHT * _avgCloud));

//Rain. The mission's rain, 0..1, is a rain rate through three even bands - light, moderate and heavy. The
//rain water that rate holds in the air, and what it evaporates into the air, are the Kessler scheme's
//(Klemp and Wilhelmson 1978, as the WRF model's module_mp_kessler has them). Evaporating, it takes the air
//cooler and wetter toward its wet-bulb temperature, while the air mixes back out at ENV_RAIN_RECOVERY:
//bmkhs_envRain is how far it is there, 0..1. Solved in the setting's own air - its mean FAT and dew
//point at its base altitude. Standard Day has none.
private _rainRate = 0.0;
if (bmkhs_helisimEnvironment == ISA_STD) then {
    bmkhs_envRain  = 0.0;
    bmkhs_envRainT = -1.0;
} else {
    _rainRate = ([[
         [0.0,       0.0]
        ,[1.0 / 3.0, ENV_RAIN_LIGHT]
        ,[2.0 / 3.0, ENV_RAIN_MODERATE]
        ,[1.0,       ENV_RAIN_TOP]
    ], rain] call bmkhs_fnc_mathLinearInterp) select 1;

    //Saturation vapour pressure, hPa, at a temperature in deg C (WMO-No. 8)
    private _saturation    = { 6.112 * exp ((17.62 * _this) / (243.12 + _this)) };
    //The setting's air, and its wet-bulb: pressure hPa, mixing ratios kg/kg
    private _pressure      = ISA_PRESSURE_HPA * ((1.0 - ((ISA_LAPSE * _baseAlt * FEET_TO_METERS) / ISA_TEMP_K)) ^ ISA_EXPONENT);
    private _vapour        = (_dewPoint call _saturation) min (_baseFAT call _saturation);
    private _mixing        = (VAPOUR_MASS_RATIO * _vapour) / (_pressure - _vapour);
    private _wetBulb       = [_baseFAT, _vapour, _pressure] call bmkhs_fnc_environmentWetBulb;
    private _wetBulbVapour = _wetBulb call _saturation;
    private _toSaturate    = (((VAPOUR_MASS_RATIO * _wetBulbVapour) / (_pressure - _wetBulbVapour)) - _mixing) max 1.0e-9;

    //The rate, 1/s, at which the rain closes what is left to saturation, with the air _this of the way there
    private _closing = {
        private _far        = _this min 0.999;
        private _temp       = _baseFAT - (_far * (_baseFAT - _wetBulb));
        private _mix        = _mixing + (_far * _toSaturate);
        private _saturated  = _temp call _saturation;
        private _mixSat     = (VAPOUR_MASS_RATIO * _saturated) / (_pressure - _saturated);
        private _rho        = (_pressure * 100.0) / (GAS_CONSTANT_DRY_AIR * (_temp + DEG_C_TO_KELVIN));
        //Rain water, g/cm3, for the rate: the rate is the water times its fall speed,
        //36.34 (water)^0.1364 sqrt(sea level density / density) m/s, turned round
        private _water      = 0.001 * ((_rainRate / (3600.0 * 36.34 * (0.001 ^ 0.1364) * (sqrt (ISA_STD_DAY_AIR_DENSITY / _rho)))) ^ (1.0 / 1.1364));
        //What it evaporates, kg/kg s - pressure in Pa and density in g/cm3, as the scheme has them
        private _evaporated = (((1.6 + (124.9 * (_water ^ 0.2046))) * (_water ^ 0.525)) / ((2.55e8 / ((_pressure * 100.0) * _mixSat)) + 5.4e5)) * (((_mixSat - _mix) max 0.0) / ((0.001 * _rho) * _mixSat));
        _evaporated / (_toSaturate * (1.0 - _far))
    };

    if (bmkhs_envRainT < 0.0) then {
        //The first solve: a mission that opens in rain has been in it a while, so the air starts settled,
        //where the rain's closing and the mixing balance
        private _lo = 0.0;
        private _hi = 1.0;
        for "_i" from 1 to 20 do {
            private _mid = (_lo + _hi) / 2.0;
            if (((_mid call _closing) * (1.0 - _mid)) > (_mid / ENV_RAIN_RECOVERY)) then { _lo = _mid } else { _hi = _mid };
        };
        bmkhs_envRain = (_lo + _hi) / 2.0;
    } else {
        //Toward where the two balance, at the rate of the two together
        private _rate = (bmkhs_envRain call _closing) + (1.0 / ENV_RAIN_RECOVERY);
        bmkhs_envRain = [bmkhs_envRain, 1.0 - (1.0 / (ENV_RAIN_RECOVERY * _rate)), CBA_missionTime - bmkhs_envRainT, 1.0 / _rate] call bmkhs_fnc_utilSmooth;
    };
    bmkhs_envRainT = CBA_missionTime;
};

bmkhs_envRainRate   = _rainRate;
bmkhs_envBaseAlt    = _baseAlt;
bmkhs_envBaseFat    = _baseFAT;
bmkhs_envSwing      = _swing;
bmkhs_envDewPoint   = _dewPoint;
bmkhs_envAvgWind    = _avgWind;
bmkhs_envCycle      = _cycle;
bmkhs_envIsDay      = _isDay;
bmkhs_envShare      = _share;
bmkhs_envLayerSec   = _layerSec;
bmkhs_envCloudDay   = _cloudDay;
bmkhs_envCloudNight = _cloudNight;
