/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stateVelocities

Description:

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    _gndSpeed, m/s
    _vel2D, m/s, simulates velocity from pitot/static sources
    _vel3D, m/s, simulates velocity from an air data sensor
    _velClimb, m/s
    _velModelSpace, m/s, unified local velocity for the simulation to use
    _velWorldSpace, m/s, unified velocity for the simulation to use

    All m/s, unrounded - a display converts.

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

//Wind
private _velWindWorldSpace   = _heli getVariable "bmkhs_velWindWorldSpace";
private _velWindWorldSpaceX  = _velWindWorldSpace select 0;
private _velWindWorldSpaceY  = _velWindWorldSpace select 1;
//Wind SPEED/DIRECTION for display are published by fn_environment (single source of truth) -
//NOT re-derived here. Only the world-space wind VECTOR is used below, to rotate into model
//space for the airspeed calculation.
private _hdg                 = direction _heli;

private _velWindModelSpaceX  = (_velWindWorldSpaceX * cos _hdg) - (_velWindWorldSpaceY * sin _hdg);
private _velWindModelSpaceY  = (_velWindWorldSpaceX * sin _hdg) + (_velWindWorldSpaceY * cos _hdg);
private _velWindModelSpace   = [_velWindModelSpaceX, _velWindModelSpaceY, 0.0];

//Velocity model space
(velocityModelSpace _heli) params ["_velModelSpaceX", "_velModelSpaceY", "_velModelSpaceZ"];
private _velModelSpace       = [_velModelSpaceX, _velModelSpaceY, _velModelSpaceZ] vectorDiff _velWindModelSpace;
private _velModelSpaceNoWind = [_velModelSpaceX, _velModelSpaceY, _velModelSpaceZ];
//Ground speed
private _gndSpeed            = vectorMagnitude [_velModelSpaceNoWind select 0, _velModelSpaceNoWind select 1, 0.0];
//3D velocity of the aircraft
private _vel3D               = vectorMagnitude _velModelSpace;
//2D velocity of the aircraft - the pitot reads 0 to 180 kt
private _vel2D               = [_velModelSpace select 1, 0.0, 180.0 * KNOTS_TO_MPS] call BIS_fnc_clamp;
//Velocity world space
(velocity _heli) params ["_velWorldSpaceX", "_velWorldSpaceY", "_velWorldSpaceZ"];
private _velWorldSpace       = [_velWorldSpaceX, _velWorldSpaceY, _velWorldSpaceZ] vectorDiff _velWindWorldSpace;
private _velWorldSpaceNoWind = [_velWorldSpaceX, _velWorldSpaceY, _velWorldSpaceZ];
//Climb velocity
private _velClimb              = _velWorldSpace select 2;
//Angular velocity in model space
private _angVelModelSpace      = angularVelocityModelSpace _heli;
//Angular velocity in world space
private _angVelWorldSpace      = angularVelocity _heli;
//Smoothed copies for display - the aircraft flies on the values above
private _deltaTime                   = _heli getVariable "bmkhs_deltaTime";
private _velModelSpaceNoWindFiltered = [_heli getVariable "bmkhs_velModelSpaceNoWindFiltered", _velModelSpaceNoWind, _deltaTime, VEL_SMOOTH_TIME] call bmkhs_fnc_utilSmooth;
private _velWorldSpaceNoWindFiltered = [_heli getVariable "bmkhs_velWorldSpaceNoWindFiltered", _velWorldSpaceNoWind, _deltaTime, VEL_SMOOTH_TIME] call bmkhs_fnc_utilSmooth;

_heli setVariable ["bmkhs_gndSpeed",            _gndSpeed];
_heli setVariable ["bmkhs_vel2D",               _vel2D];
_heli setVariable ["bmkhs_vel3D",               _vel3D];
_heli setVariable ["bmkhs_velWindModelSpace",   _velWindModelSpace];
_heli setVariable ["bmkhs_velModelSpace",       _velModelSpace];
_heli setVariable ["bmkhs_velModelSpaceNoWind", _velModelSpaceNoWind];
_heli setVariable ["bmkhs_velWorldSpace",       _velWorldSpace];
_heli setVariable ["bmkhs_velWorldSpaceNoWind", _velWorldSpaceNoWind];
_heli setVariable ["bmkhs_velClimb",            _velClimb];
_heli setVariable ["bmkhs_angVelModelSpace",    _angVelModelSpace];
_heli setVariable ["bmkhs_velModelSpaceNoWindFiltered", _velModelSpaceNoWindFiltered];
_heli setVariable ["bmkhs_velWorldSpaceNoWindFiltered", _velWorldSpaceNoWindFiltered];
_heli setVariable ["bmkhs_velWorldSpaceFiltered",       _velWorldSpaceNoWindFiltered vectorDiff _velWindWorldSpace];
//windDirFrom / windSpeed are published by fn_environment (single source of truth).
