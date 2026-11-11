/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_utilSmooth

Description:
    Smooths a noisy value over a time in seconds, the same at any frame rate.

Parameters:
    _previous  - The smoothed value last frame [Number or Array].
    _new       - This frame's reading [Number or Array].
    _deltaTime - Frame time, s [Number].
    _time      - Smoothing time, s [Number].

Returns:
    The smoothed value [Number or Array].

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_previous", "_new", "_deltaTime", "_time"];

//The share of the way to the new reading this frame covers
private _share = 1 - exp (-_deltaTime / _time);

if (_new isEqualType []) exitWith { _previous vectorAdd ((_new vectorDiff _previous) vectorMultiply _share) };
_previous + (_new - _previous) * _share
