#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

params ["_heli", "_type", "_pitchMin", "_pitchMid", "_pitchMax", "_rollMin", "_rollMid", "_rollMax", "_collMin", "_collMid", "_collMax", "_dragMin", "_dragMid", "_dragMax"];

private _cyclicFwdAft           = _heli getVariable "bmkhs_cyclicFwdAft";
private _forceTrimPosPitch      = _heli getVariable "bmkhs_forceTrimPosPitch";
private _sasPitchOut            = _heli getVariable "bmkhs_fmcSasPitchOut";
private _attHoldCycPitchOut     = _heli getVariable "bmkhs_fmcAttHoldCycPitchOut";
private _pitchInput             = ([_cyclicFwdAft, _forceTrimPosPitch] call bmkhs_fnc_inputGetInterp) + _sasPitchOut + _attHoldCycPitchOut;
_pitchInput                     = [_pitchInput, -1.0, 1.0] call BIS_fnc_clamp;
private _pitchFeather			= 0.0;

private _cyclicLeftRight        = _heli getVariable "bmkhs_cyclicLeftRight";
private _forceTrimPosRoll       = _heli getVariable "bmkhs_forceTrimPosRoll";
private _sasRollOut             = _heli getVariable "bmkhs_fmcSasRollOut";
private _attHoldCycRollOut      = _heli getVariable "bmkhs_fmcAttHoldCycRollOut";
private _rollInput              = ([_cyclicLeftRight, _forceTrimPosRoll] call bmkhs_fnc_inputGetInterp) + _sasRollOut + _attHoldCycRollOut;
_rollInput                      = [_rollInput, -1.0, 1.0] call BIS_fnc_clamp;
private _rollFeather			= 0.0;

private _pedalLeftRight         = _heli getVariable "bmkhs_pedalLeftRight";
private _forceTrimPosYaw        = _heli getVariable "bmkhs_forceTrimPosYaw";
private _sasYawOut              = _heli getVariable "bmkhs_fmcSasYawOut";
private _hdgHoldPedalYawOut     = _heli getVariable "bmkhs_fmcHdgHoldPedalYawOut";
private _yawInput               = ([_pedalLeftRight, _forceTrimPosYaw] call bmkhs_fnc_inputGetInterp) + _sasYawOut + _hdgHoldPedalYawOut;
_yawInput                       = [_yawInput, -1.0, 1.0] call BIS_fnc_clamp;

private _collectiveOut          = _heli getVariable "bmkhs_collectiveOutput";
private _altHoldCollOut         = _heli getVariable "bmkhs_fmcAltHoldCollOut";
private _collInput              = _collectiveOut + _altHoldCollOut;
private _collOutput 			= 0.0;

switch (_type) do {
	case MAIN: {
		_pitchFeather = [-1, 1, _pitchInput, _pitchMin, _pitchMid, _pitchMax] call bmkhs_fnc_mathLinearInterpFromCenter;
		_rollFeather  = [-1, 1, _rollInput,  _rollMin,  _rollMid,  _rollMax]  call bmkhs_fnc_mathLinearInterpFromCenter;
		_collOutput   = [_collInput, 0.0, 1.0] call BIS_fnc_clamp;
	};
	case TAIL: {
		_collOutput   = [-_yawInput, -1.0, 1.0] call BIS_fnc_clamp;
	};
};

[_pitchFeather, _rollFeather, _collOutput];
