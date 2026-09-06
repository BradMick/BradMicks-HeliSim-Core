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

private _deltaTime     = _heli getVariable "bmkhs_deltaTime";
private _previousTime  = _heli getVariable "bmkhs_previousTime";
private _deltaTime_avg = _heli getVariable "bmkhs_deltaTime_avg";

private _currentTime = diag_tickTime;
_deltaTime           = _currentTime - _previousTime;
_previousTime        = _currentTime;

_deltaTime           = _deltaTime * accTime;
_deltaTime           = _deltaTime min 0.1;

_heli setVariable ["bmkhs_previousTime", _previousTime];
_heli setVariable ["bmkhs_deltaTime",    _deltaTime];
