#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

params ["_heli", "_type", "_pitchMin", "_pitchMid", "_pitchMax", "_rollMin", "_rollMid", "_rollMax", "_collMin", "_collMid", "_collMax"];

//COMMANDED control positions = pilot stick + force-trim reference + SCAS. The pilot inputs
//(cyclicFwdAft / cyclicLeftRight / pedalLeftRight / collectiveOutput) are ALREADY passed
//through the hydraulic actuator LAG upstream in fn_getInput (fnc_actuator: crisp when FMC/
//hydraulics are good, lagged when not), so no servo lag is applied here - that would double it.
//The SAS terms (SCAS rate/command augmentation) are published FMC-gated in fn_fmc (zeroed on
//an axis when that axis's FMC is off), so adding them here is safe when FMC is off = 0.
//Per axis: pilot stick + force-trim reference + SAS (rate damping) + the FMC HOLD output
//(attitude hold on pitch/roll, heading hold on yaw, altitude hold on collective). The hold and
//SAS outputs are FMC-gated + primary-hydraulics-gated in fn_fmc (zeroed when their channel/hyd
//is unavailable), so summing them here is safe (=0 when inactive).
private _cyclicFwdAft           = _heli getVariable "bmkhs_cyclicFwdAft";
private _forceTrimPosPitch      = _heli getVariable "bmkhs_forceTrimPosPitch";
private _sasPitchOut            = _heli getVariable "bmkhs_fmcSasPitchOut";
private _attHoldCycPitchOut     = _heli getVariable "bmkhs_fmcAttHoldCycPitchOut";
private _pitchInput             = _cyclicFwdAft + _forceTrimPosPitch + _sasPitchOut + _attHoldCycPitchOut;
private _pitchFeather			= 0.0;

private _cyclicLeftRight        = _heli getVariable "bmkhs_cyclicLeftRight";
private _forceTrimPosRoll       = _heli getVariable "bmkhs_forceTrimPosRoll";
private _sasRollOut             = _heli getVariable "bmkhs_fmcSasRollOut";
private _attHoldCycRollOut      = _heli getVariable "bmkhs_fmcAttHoldCycRollOut";
private _rollInput              = _cyclicLeftRight + _forceTrimPosRoll + _sasRollOut + _attHoldCycRollOut;
private _rollFeather			= 0.0;

private _pedalLeftRight         = _heli getVariable "bmkhs_pedalLeftRight";
private _forceTrimPosYaw        = _heli getVariable "bmkhs_forceTrimPosYaw";
private _sasYawOut              = _heli getVariable "bmkhs_fmcSasYawOut";
private _hdgHoldPedalYawOut     = _heli getVariable "bmkhs_fmcHdgHoldPedalYawOut";
private _yawInput               = _pedalLeftRight + _forceTrimPosYaw + _sasYawOut + _hdgHoldPedalYawOut;

private _collectiveOut          = _heli getVariable "bmkhs_collectiveOutput";
private _altHoldCollOut         = _heli getVariable "bmkhs_fmcAltHoldCollOut";
private _collInput              = _collectiveOut + _altHoldCollOut;
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
