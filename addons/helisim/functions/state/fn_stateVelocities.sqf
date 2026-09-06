/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stateVelocities

Description:

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    _gndSpeed, kts
    _vel2D, kts, simulates velocity from pitot/static sources
    _vel3D, kts, simulates velocity from an air data sensor
    _velModelSpace, m/s, unified local velocity for the simulation to use
    _velWorldSpace, m/s, unified velocity for the simulation to use

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
private _velModelSpaceX_avg  = _heli getVariable "bmkhs_velModelSpaceX_avg";
private _velModelSpaceY_avg  = _heli getVariable "bmkhs_velModelSpaceY_avg";
private _velModelSpaceZ_avg  = _heli getVariable "bmkhs_velModelSpaceZ_avg";
private _velModelSpaceX      = velocityModelSpace _heli select 0;
_velModelSpaceX              = [_velModelSpaceX_avg, _velModelSpaceX] call bmkhs_fnc_utilSmoothAverage;
private _velModelSpaceY      = velocityModelSpace _heli select 1;
_velModelSpaceY              = [_velModelSpaceY_avg, _velModelSpaceY] call bmkhs_fnc_utilSmoothAverage;
private _velModelSpaceZ      = velocityModelSpace _heli select 2;
_velModelSpaceZ              = [_velModelSpaceZ_avg, _velModelSpaceZ] call bmkhs_fnc_utilSmoothAverage;
private _velModelSpace       = [_velModelSpaceX, _velModelSpaceY, _velModelSpaceZ] vectorDiff _velWindModelSpace;
private _velModelSpaceNoWind = [_velModelSpaceX, _velModelSpaceY, _velModelSpaceZ];
//Ground speed
private _gndSpeed            = round(vectorMagnitude [_velModelSpaceNoWind select 0, _velModelSpaceNoWind select 1] * MPS_TO_KNOTS);
//3D velocity of the aircraft
private _vel3D               = round(MPS_TO_KNOTS * vectorMagnitude _velModelSpace);
//2D velocity of the aircraft
private _vel2D               = [round(MPS_TO_KNOTS * (_velModelSpace select 1)), 0.0, 180.0] call BIS_fnc_clamp;
//Velocity world space
private _velWorldSpaceX_avg  = _heli getVariable "bmkhs_velWorldSpaceX_avg";
private _velWorldSpaceY_avg  = _heli getVariable "bmkhs_velWorldSpaceY_avg";
private _velWorldSpaceZ_avg  = _heli getVariable "bmkhs_velWorldSpaceZ_avg";

private _velWorldSpaceX      = velocity _heli select 0;
_velWorldSpaceX              = [_velWorldSpaceX_avg, _velWorldSpaceX] call bmkhs_fnc_utilSmoothAverage;
private _velWorldSpaceY      = velocity _heli select 1;
_velWorldSpaceY              = [_velWorldSpaceY_avg, _velWorldSpaceY] call bmkhs_fnc_utilSmoothAverage;
private _velWorldSpaceZ      = velocity _heli select 2;
_velWorldSpaceZ              = [_velWorldSpaceZ_avg, _velWorldSpaceZ] call bmkhs_fnc_utilSmoothAverage;
private _velWorldSpace       = [_velWorldSpaceX, _velWorldSpaceY, _velWorldSpaceZ] vectorDiff _velWindWorldSpace;
private _velWorldSpaceNoWind = [_velWorldSpaceX, _velWorldSpaceY, _velWorldSpaceZ];
//Climb velocity
private _velClimb              = (_velWorldSpace select 2) * MPS_TO_FPM;
//Angular velocity in model space
private _angVelModelSpaceX_avg = _heli getVariable "bmkhs_angVelModelSpaceX_avg";
private _angVelModelSpaceY_avg = _heli getVariable "bmkhs_angVelModelSpaceY_avg";
private _angVelModelSpaceZ_avg = _heli getVariable "bmkhs_angVelModelSpaceZ_avg";

private _angVelModelSpaceX     = angularVelocityModelSpace _heli select 0;
_angVelModelSpaceX             = [_angVelModelSpaceX_avg, _angVelModelSpaceX] call bmkhs_fnc_utilSmoothAverage;
private _angVelModelSpaceY     = angularVelocityModelSpace _heli select 1;
_angVelModelSpaceY             = [_angVelModelSpaceY_avg, _angVelModelSpaceY] call bmkhs_fnc_utilSmoothAverage;
private _angVelModelSpaceZ     = angularVelocityModelSpace _heli select 2;
_angVelModelSpaceZ             = [_angVelModelSpaceZ_avg, _angVelModelSpaceZ] call bmkhs_fnc_utilSmoothAverage;
private _angVelModelSpace      = [_angVelModelSpaceX, _angVelModelSpaceY, _angVelModelSpaceZ];
//Angular velocity in world space
private _angVelWorldSpace      = angularVelocity _heli;

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
//windDirFrom / windSpeedKts are published by fn_environment (single source of truth).
