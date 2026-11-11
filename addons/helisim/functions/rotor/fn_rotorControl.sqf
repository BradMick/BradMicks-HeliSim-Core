#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

params ["_heli", "_type", "_pitchMin", "_pitchMid", "_pitchMax", "_rollMin", "_rollMid", "_rollMax", "_collMin", "_collMid", "_collMax"];

private _pitchInput             = _heli getVariable "bmkhs_fltCtrlPitchOut";
private _pitchFeather			= 0.0;

private _rollInput              = _heli getVariable "bmkhs_fltCtrlRollOut";
private _rollFeather			= 0.0;

private _yawInput               = _heli getVariable "bmkhs_fltCtrlYawOut";

private _collInput              = _heli getVariable "bmkhs_fltCtrlCollOut";
private _collFeather            = 0.0;

switch (_type) do {
	case MAIN: {
		_pitchFeather  = [-1, 1, _pitchInput, _pitchMin, _pitchMid, _pitchMax] call bmkhs_fnc_mathLinearInterpFromCenter;
		_rollFeather   = [-1, 1, _rollInput,  _rollMin,  _rollMid,  _rollMax]  call bmkhs_fnc_mathLinearInterpFromCenter;
		_collFeather   = linearConversion[ 0, 1, _collInput,  _collMin, _collMax, true];
	};
	case TAIL: {
		_collFeather   = [-1, 1, -_yawInput, _collMin, _collMid, _collMax] call bmkhs_fnc_mathLinearInterpFromCenter;
	};
};

[_pitchFeather, _rollFeather, _collFeather];
