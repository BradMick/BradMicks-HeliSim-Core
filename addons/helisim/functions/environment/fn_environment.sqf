/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_environment

Description:
    The air the aircraft is in: pressure altitude, free air temperature,
    pressure, dew point and density where it is now, and the wind and gust on
    it.

    The environment setting is the base - a region's elevation, mean
    temperature, daily swing and dew point. The mission moves it: the clock
    through the day, overcast and wind the size of the swing, fog the moisture,
    rain both - cooler and wetter, toward the air's wet-bulb temperature.
    Standard Day is the standard atmosphere, dry, whatever the mission does.

    Pressure is the standard atmosphere's at the pressure altitude. The free
    air is the setting's mean, lapsed at the standard rate. The day's swing
    sits on the ground: the afternoon's air falls at the dry adiabatic rate
    above the ground until it meets the free air, and stays aloft overnight;
    the night cools the surface under it in a layer that thins exponentially
    with height (Stull, Practical Meteorology, eq. 18.3), and the morning burns
    that layer off from the ground up.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

//The weather - the setting's numbers, the day and the sky - is one for every aircraft this machine flies,
//solved once every ENV_WEATHER_PERIOD by whichever of them gets here first
if (CBA_missionTime >= bmkhs_envWeatherT + ENV_WEATHER_PERIOD) then {
    bmkhs_envWeatherT = CBA_missionTime;
    call bmkhs_fnc_environmentWeather;
};
private _baseAlt     = bmkhs_envBaseAlt;
private _baseFAT     = bmkhs_envBaseFat;
private _swing       = bmkhs_envSwing;
private _dewPoint    = bmkhs_envDewPoint;
private _avgWind     = bmkhs_envAvgWind;
private _cycle       = bmkhs_envCycle;
private _isDay       = bmkhs_envIsDay;
private _share       = bmkhs_envShare;
private _layerSec    = bmkhs_envLayerSec;
private _cloudDay    = bmkhs_envCloudDay;
private _cloudNight  = bmkhs_envCloudNight;
private _rain        = bmkhs_envRain;
private _standardDay = bmkhs_helisimEnvironment == ISA_STD;
//fn_environmentVariables solves this once at init, before the frame clock is seeded: nothing is eased then
private _deltaTime   = _heli getVariable ["bmkhs_deltaTime", 0.0];

//Saturation vapour pressure, hPa, at a temperature in deg C (WMO-No. 8)
private _saturation  = { 6.112 * exp ((17.62 * _this) / (243.12 + _this)) };
//The standard atmosphere's pressure, hPa, at a pressure altitude in m
private _isaPressure = { ISA_PRESSURE_HPA * ((1.0 - ((ISA_LAPSE * _this) / ISA_TEMP_K)) ^ ISA_EXPONENT) };
//The dew point, deg C, of a vapour pressure in hPa - absolute zero in dry air
private _dewPointOf  = {
    if (_this <= 0.0) exitWith { -DEG_C_TO_KELVIN };
    private _g = ln (_this / 6.112);
    (243.12 * _g) / (17.62 - _g)
};

//Height above the map's sea level, and above the ground - the sea where there is no ground. The height
//above ground is eased, so a cliff edge is a drift in temperature and not a step.
private _posASL    = getPosASL _heli;
private _asl       = _posASL select 2;
private _aglNow    = (_asl - ((getTerrainHeightASL _posASL) max 0.0)) max 0.0;
private _agl       = [_heli getVariable "bmkhs_envAgl", _aglNow, _deltaTime, ENV_AGL_TIME] call bmkhs_fnc_utilSmooth;
_heli setVariable ["bmkhs_envAgl", _agl];
private _groundAsl = _asl - _agl;

//The steady wind: the mission's, its jumps eased
private _windNow         = [wind select 0, wind select 1, 0.0];
private _windSteady      = [_heli getVariable "bmkhs_windSteady", _windNow, _deltaTime, ENV_WIND_TIME] call bmkhs_fnc_utilSmooth;
_heli setVariable ["bmkhs_windSteady", _windSteady];
private _windSteadySpeed = vectorMagnitude _windSteady;

//Exact - the physics computes from these; a display rounds them for itself
//Pressure altitude, ft, and the standard atmosphere's pressure there
private _altitude    = _baseAlt + (_asl * METERS_TO_FEET);
private _pressure    = (_altitude * FEET_TO_METERS) call _isaPressure;

//Free air: the setting's mean, lapsed
private _temperature = _baseFAT - (ISA_LAPSE * _asl);

//The setting's vapour at its base thins with pressure
private _pressureBase = (_baseAlt * FEET_TO_METERS) call _isaPressure;
private _vapourBase   = _dewPoint call _saturation;

if (!_standardDay && {_cycle}) then {
    //The afternoon's surface above the mean and the dawn's below it are the measured swing at the region's
    //average cloud, each narrowed or widened by the cloud there is. The night's whole drop from the
    //afternoon is deeper in less wind than the region's average and shallower in more (Stull 18.4, 18.5).
    private _windNight   = _windSteadySpeed max ENV_NIGHT_WIND_MIN;
    private _rise        = _swing * _cloudDay;
    private _drop        = (_rise + (_swing * _cloudNight)) * ((_avgWind / _windNight) ^ 0.75);
    //A calm night cools the surface no further than its dew point - or the average dawn, where that is the colder
    private _groundFree  = _baseFAT - (ISA_LAPSE * _groundAsl);
    private _dewGround   = (_vapourBase * (((_baseAlt * FEET_TO_METERS) + _groundAsl) call _isaPressure) / _pressureBase) call _dewPointOf;
    private _dropLimit   = ((_groundFree + _rise) - (_dewGround min (_groundFree - _swing))) max 0.0;
    //The night layer's e-folding height
    private _layerHeight = (ENV_NIGHT_LAYER_A * (_windNight ^ 0.75) * (sqrt _layerSec)) max 0.1;
    private _deficit     = if (_isDay) then {
        //Mixed from the ground up to where the dawn's layer is weaker than what is left of it at the surface
        private _dawn = _drop min _dropLimit;
        (_share * _dawn) min (_dawn * exp (-_agl / _layerHeight))
    } else {
        ((_drop * _share) min _dropLimit) * exp (-_agl / _layerHeight)
    };
    //The afternoon's air falls at the dry adiabatic rate above the ground until it meets the free air
    _temperature = _temperature + ((_rise - ((DRY_ADIABATIC_LAPSE - ISA_LAPSE) * _agl)) max 0.0) - _deficit;
};

//Vapour: none on a standard day; saturated in fog; never more than the air can hold
private _vapour      = 0.0;
//What the rain has taken off the temperature, deg C
private _rainCooling = 0.0;
if (!_standardDay) then {
    _vapour = (_vapourBase * _pressure / _pressureBase) min (_temperature call _saturation);
    //Rain: the air here is _rain of the way to its wet-bulb temperature - cooler, and wetter by the vapour
    //whose evaporation cooled it. Before the fog, so the air is as cool under the fog's top as over it.
    if (_rain > 0.0) then {
        private _wetBulb       = [_temperature, _vapour, _pressure] call bmkhs_fnc_environmentWetBulb;
        private _wetBulbVapour = _wetBulb call _saturation;
        private _mixing        = (VAPOUR_MASS_RATIO * _vapour) / (_pressure - _vapour);
        _mixing      = _mixing + (_rain * (((VAPOUR_MASS_RATIO * _wetBulbVapour) / (_pressure - _wetBulbVapour)) - _mixing));
        _rainCooling = _rain * (_temperature - _wetBulb);
        _temperature = _temperature - _rainCooling;
        _vapour      = ((_mixing * _pressure) / (VAPOUR_MASS_RATIO + _mixing)) min (_temperature call _saturation);
    };
    //Fog: saturated below its top
    if (_agl < fog * ENV_FOG_TOP) then { _vapour = _temperature call _saturation };
};
private _airDensity = ((_pressure - (VAPOUR_DENSITY_DEFICIT * _vapour)) * 100.0) / (GAS_CONSTANT_DRY_AIR * (_temperature + DEG_C_TO_KELVIN));

_heli setVariable ["bmkhs_barAlt",   _altitude];
_heli setVariable ["bmkhs_fat",      _temperature];
_heli setVariable ["bmkhs_pressure", _pressure];
_heli setVariable ["bmkhs_dewPoint", _vapour call _dewPointOf];
_heli setVariable ["bmkhs_rho",      _airDensity];
_heli setVariable ["bmkhs_rainCooling", _rainCooling];

//What the cockpit reads and the airframe feels: the steady wind - still air with wind disabled, so a
//test is not confounded by the mission weather. The direction is the one it blows FROM.
private _windFelt    = [_windSteady, [0.0, 0.0, 0.0]] select bmkhs_windDisabled;
private _windSpeed   = vectorMagnitude _windFelt;
private _windDirFrom = if (_windSpeed > 0.0) then {
    (round ((((_windFelt select 0) atan2 (_windFelt select 1)) + 540.0) mod 360.0)) mod 360
} else { 0 };

_heli setVariable ["bmkhs_windSpeed",   _windSpeed];
_heli setVariable ["bmkhs_windDirFrom", _windDirFrom];

//World-space wind and the gust on it, for the airspeed fn_stateVelocities works out
private _velWindWorldSpace = [0.0, 0.0, 0.0];
private _velGustWorldSpace = [0.0, 0.0, 0.0];

if (bmkhs_rotorModel == 0) then {
    _velWindWorldSpace = _windFelt;
    _velGustWorldSpace = [_heli, _windFelt, _agl, _deltaTime] call bmkhs_fnc_environmentGusts;
};

_heli setVariable ["bmkhs_velWindWorldSpace", _velWindWorldSpace];
_heli setVariable ["bmkhs_velGustWorldSpace", _velGustWorldSpace];
