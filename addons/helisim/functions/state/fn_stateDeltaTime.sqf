/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stateDeltaTime

Description:
    Custom time handler

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

//Game time, not the real clock: Arma steps at most 0.1 s a frame and spreads a longer frame over
//the frames after it, so only game time matches the step the forces are applied over
private _now       = time;
private _deltaTime = ((_now - (_heli getVariable "bmkhs_previousTime")) max 0.0) min 0.1;

_heli setVariable ["bmkhs_previousTime", _now];
_heli setVariable ["bmkhs_deltaTime",    _deltaTime];
