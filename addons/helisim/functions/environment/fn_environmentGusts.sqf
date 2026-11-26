/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_environmentGusts

Description:
    The gust on the steady wind: the Dryden turbulence model of MIL-F-8785C.

    Its strength follows the wind. Below 1,000 ft above the ground the
    vertical gust is a tenth of the wind at 20 ft - Arma's wind stands for
    it - and the gusts along and across the wind are stronger the nearer the
    ground; a gust's size grows with height. Above 2,000 ft the
    specification's table gives the strength (figure 7: light, moderate and
    severe, which it pairs with 15, 30 and 45 kt of wind), read between its
    rows by the wind. Arma's gusts setting scales the result, none at 0 and
    the specification's own at 1.

    The specification has the aircraft fly through the gusts at its
    airspeed. Here that is its speed through the air mass, so in a hover the
    gusts blow past at the wind's speed.

Parameters:
    _heli      - The helicopter [Object]
    _wind      - The steady wind, world space m/s [Array]
    _agl       - Height above the ground, m [Number]
    _deltaTime - Frame time, s [Number]

Returns:
    The gust, world space m/s [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli", "_wind", "_agl", "_deltaTime"];

private _windSpeed = vectorMagnitude _wind;
private _severity  = gusts;
if (_severity <= 0.0 || {_windSpeed <= 0.0}) exitWith {
    _heli setVariable ["bmkhs_gustState", [0.0, 0.0, 0.0]];
    [0.0, 0.0, 0.0]
};

//Height above ground, and the speed through the air mass
private _h = _agl max GUST_HEIGHT_MIN;
private _v = (vectorMagnitude ((velocity _heli) vectorDiff _wind)) max GUST_AIRSPEED_MIN;

//Below 1,000 ft: strength from the wind, size from the height. The specification's shape is per foot.
private _low   = _h min GUST_LOW_TOP;
private _shape = 0.177 + (0.000823 * METERS_TO_FEET * _low);
private _lu    = _low / (_shape ^ 1.2);
private _lw    = _low;
private _sw    = 0.1 * _windSpeed;
private _su    = _sw / (_shape ^ 0.4);
if (_h > GUST_LOW_TOP) then {
    //Above 2,000 ft: the specification's table - [height ft, light, moderate, severe], ft/s - read at this
    //height and between its rows by the wind; from 1,000 to 2,000 ft, between the two
    ([[
        [  500.0,  6.6,  8.6, 15.6],
        [ 1750.0,  6.9,  9.6, 17.6],
        [ 3750.0,  7.4, 10.6, 23.0],
        [ 7500.0,  6.7, 10.1, 23.6],
        [15000.0,  4.6,  8.0, 22.1],
        [25000.0,  2.7,  6.6, 20.0],
        [35000.0,  0.4,  5.0, 16.0],
        [45000.0,  0.0,  4.2, 15.1],
        [55000.0,  0.0,  2.7, 12.1],
        [65000.0,  0.0,  0.0,  7.9],
        [75000.0,  0.0,  0.0,  6.2],
        [80000.0,  0.0,  0.0,  5.1]
    ], _h * METERS_TO_FEET] call bmkhs_fnc_mathLinearInterp) params ["", "_light", "_moderate", "_severe"];
    private _steps   = [0.0, _light * FEET_TO_METERS, _moderate * FEET_TO_METERS, _severe * FEET_TO_METERS];
    private _at      = ((_windSpeed / GUST_WIND_STEP) max 0.0) min 3.0;
    private _lowStep = (floor _at) min 2;
    private _high    = (_steps select _lowStep) + (((_steps select (_lowStep + 1)) - (_steps select _lowStep)) * (_at - _lowStep));
    private _blend   = (((_h - GUST_LOW_TOP) / (GUST_HIGH_BASE - GUST_LOW_TOP)) max 0.0) min 1.0;
    _lu = GUST_LOW_TOP + (_blend * (GUST_HIGH_SIZE - GUST_LOW_TOP));
    _lw = _lu;
    _su = _sw + (_blend * (_high - _sw));
    _sw = _su;
};
_su = _su * _severity;
_sw = _sw * _severity;

//Each component eases toward zero and is pushed by its own noise, at a rate set by how fast the aircraft
//crosses a gust: the specification's filters (Yeager, NASA CR-1998-206937, eq. 30-32), stepped exactly so
//the gusts are the same at any frame rate
private _noise = { (sqrt (-2.0 * (ln ((random 1.0) max 0.000001)))) * (cos (random 360.0)) };
private _keepU = exp (-_deltaTime * _v / _lu);
private _keepV = exp (-2.0 * _deltaTime * _v / _lu);
private _keepW = exp (-2.0 * _deltaTime * _v / _lw);
(_heli getVariable "bmkhs_gustState") params ["_gustU", "_gustV", "_gustW"];
_gustU = (_keepU * _gustU) + (_su * (sqrt (1.0 - (_keepU * _keepU))) * (call _noise));
_gustV = (_keepV * _gustV) + (_su * (sqrt (1.0 - (_keepV * _keepV))) * (call _noise));
_gustW = (_keepW * _gustW) + (_sw * (sqrt (1.0 - (_keepW * _keepW))) * (call _noise));
_heli setVariable ["bmkhs_gustState", [_gustU, _gustV, _gustW]];

//Along the wind, across it, and vertical
private _alongX = (_wind select 0) / _windSpeed;
private _alongY = (_wind select 1) / _windSpeed;
[
    (_alongX * _gustU) + (_alongY * _gustV),
    (_alongY * _gustU) - (_alongX * _gustV),
    _gustW
]
