/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_suspensionDebug

Description:
    Draws every wheel where its strut has it, along the stroke from full
    extension to static by the last compression. Full extension off the ground.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

private _compressed = _heli getVariable "bmkhs_wheelCompression";

{
    private _radius = _x get "radius";
    private _axle   = (_x get "extended") vectorAdd ((_x get "stroke") vectorMultiply (((_compressed select _forEachIndex) max 0.0) / (_x get "sag")));
    private _tread  = _axle vectorAdd [0.0, 0.0, -_radius];

    //Wheel
    [_heli, 24, _axle, [0.0, 90.0, 0.0], _radius, "white"] call bmkhs_fnc_debugDrawCircle;
    [_heli, _axle vectorDiff [0.00, _radius, 0.00], _axle vectorAdd [0.00, _radius, 0.00], "green"] call bmkhs_fnc_debugDrawLine;
    [_heli, _axle vectorDiff [0.00, 0.00, _radius], _axle vectorAdd [0.00, 0.00, _radius], "blue"]  call bmkhs_fnc_debugDrawLine;

    //Contact point (bottom of the wheel)
    [_heli, 24, _tread, [0.0,  90.0, 0.0], 0.05, "white"] call bmkhs_fnc_debugDrawCircle;
    [_heli, 24, _tread, [-90.0, 0.0, 0.0], 0.05, "white"] call bmkhs_fnc_debugDrawCircle;
    [_heli, 24, _tread, [0.0,   0.0, 0.0], 0.05, "white"] call bmkhs_fnc_debugDrawCircle;
    [_heli, _tread vectorDiff [0.05, 0.00, 0.00], _tread vectorAdd [0.05, 0.00, 0.00], "red"]   call bmkhs_fnc_debugDrawLine;
    [_heli, _tread vectorDiff [0.00, 0.05, 0.00], _tread vectorAdd [0.00, 0.05, 0.00], "green"] call bmkhs_fnc_debugDrawLine;
    [_heli, _tread vectorDiff [0.00, 0.00, 0.05], _tread vectorAdd [0.00, 0.00, 0.05], "blue"]  call bmkhs_fnc_debugDrawLine;

    //Ray (top of the wheel to the bottom)
    [_heli, _axle vectorAdd [0.0, 0.0, _radius], _tread, "red"] call bmkhs_fnc_debugDrawLine;
} forEach (_heli getVariable "bmkhs_wheels");
