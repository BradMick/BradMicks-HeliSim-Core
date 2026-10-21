#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

params ["_heli", "_rotor"];

private _type                   = _rotor get "type";

private _cyclicFwdAft           = _heli getVariable "bmkhs_cyclicFwdAft";
private _forceTrimPosPitch      = _heli getVariable "bmkhs_forceTrimPosPitch";
private _sasPitchOut            = _heli getVariable "bmkhs_fmcSasPitchOut";
private _attHoldCycPitchOut     = _heli getVariable "bmkhs_fmcAttHoldCycPitchOut";
private _pitchInput             = ([_cyclicFwdAft, _forceTrimPosPitch] call bmkhs_fnc_inputGetInterp) + _sasPitchOut + _attHoldCycPitchOut + (_heli getVariable "bmkhs_mixPitchOut");
_pitchInput                     = [_pitchInput, -1.0, 1.0] call BIS_fnc_clamp;
private _pitchOutput			= 0.0;

private _cyclicLeftRight        = _heli getVariable "bmkhs_cyclicLeftRight";
private _forceTrimPosRoll       = _heli getVariable "bmkhs_forceTrimPosRoll";
private _sasRollOut             = _heli getVariable "bmkhs_fmcSasRollOut";
private _attHoldCycRollOut      = _heli getVariable "bmkhs_fmcAttHoldCycRollOut";
private _autoAttCycRollOut      = _heli getVariable "bmkhs_autoAttCycRollOut";
private _rollInput              = ([_cyclicLeftRight, _forceTrimPosRoll] call bmkhs_fnc_inputGetInterp) + _sasRollOut + _attHoldCycRollOut + _autoAttCycRollOut + (_heli getVariable "bmkhs_mixRollOut");
_rollInput                      = [_rollInput, -1.0, 1.0] call BIS_fnc_clamp;
private _rollOutput 			= 0.0;

private _pedalLeftRight         = _heli getVariable "bmkhs_pedalLeftRight";
private _forceTrimPosYaw        = _heli getVariable "bmkhs_forceTrimPosYaw";
private _sasYawOut              = _heli getVariable "bmkhs_fmcSasYawOut";
private _hdgHoldPedalYawOut     = _heli getVariable "bmkhs_fmcHdgHoldPedalYawOut";
private _yawInput               = ([_pedalLeftRight, _forceTrimPosYaw] call bmkhs_fnc_inputGetInterp) + _sasYawOut + _hdgHoldPedalYawOut + (_heli getVariable "bmkhs_mixYawOut");
_yawInput                       = [_yawInput, -1.0, 1.0] call BIS_fnc_clamp;

private _collectiveOut          = _heli getVariable "bmkhs_collectiveOutput";
private _altHoldCollOut         = _heli getVariable "bmkhs_fmcAltHoldCollOut";
private _collInput              = _collectiveOut + _altHoldCollOut + (_heli getVariable "bmkhs_mixCollOut");
private _collOutput 			= 0.0;
//Tandem only - the rotor applies these through tandemPitchLiftCoef and tandemYawLiftCoef.
private _tandemPitchOutput      = 0.0;
private _tandemYawOutput        = 0.0;
//Mast pitch added to the rotor's rotation[] pitch, deg - the tandem's LCT
private _mastPitchOutput        = 0.0;

switch (_heli getVariable "bmkhs_controlScheme") do {
	case CONVENTIONAL: {
		switch (_type) do {
			case MAIN: {
				_pitchOutput = [_pitchInput, -1.0, 1.0] call BIS_fnc_clamp;
				_rollOutput  = [_rollInput,  -1.0, 1.0] call BIS_fnc_clamp;
				_collOutput  = [_collInput, 0.0, 1.0] call BIS_fnc_clamp;
			};
			case TAIL: {
				_collOutput  = [_yawInput, -1.0, 1.0] call BIS_fnc_clamp;
			};
			case PROPELLER: {
			};
			case PROPROTOR: {
			};
		};
	};
	case TANDEM: {
		//Fore/aft cyclic moves lift from one rotor to the other - it does not touch collective, so
		//it does not move torque. Pedal is lateral cyclic on each rotor, opposite ways: blade pitch
		//and disc tilt. Forward stick takes lift off the front rotor and puts it on the rear;
		//right pedal tilts the front disc right and the rear disc left. Lateral cyclic and
		//collective are the conventional ones.
		private _sign = switch (_rotor get "position") do {
			case FRONT: { -1.0 };
			case REAR:  {  1.0 };
			default     {  0.0 };
		};
		_rollOutput        = [_rollInput, -1.0, 1.0] call BIS_fnc_clamp;
		_collOutput        = [_collInput, 0.0, 1.0] call BIS_fnc_clamp;
		_tandemPitchOutput = _sign * _pitchInput;
		_tandemYawOutput   = _sign * _yawInput;

		//LCT - the mast tilts on its schedule (airspeed by pressure altitude), + extends (forward).
		//The actuator travels its full stroke in lctTime seconds. Casual and realistic alike.
		//On the ground the proximity switches drive it to GND - 0, perpendicular to the mast.
		private _lctSchedule = _rotor get "lctSchedule";
		if (_lctSchedule isNotEqualTo []) then {
			private _kts    = (_heli getVariable "bmkhs_vel2D") * MPS_TO_KNOTS;
			private _target = if ([_heli] call bmkhs_fnc_stateOnGround) then { 0.0 } else {
				[_lctSchedule, _kts, _heli getVariable "bmkhs_barAlt"] call bmkhs_fnc_mathLinearInterp2D
			};
			private _lctPos = _rotor getOrDefault ["lctPos", _target];
			private _time   = _rotor get "lctTime";
			if (_time > 0) then {
				private _step = ((_rotor get "lctStroke") / _time) * (_heli getVariable "bmkhs_deltaTime");
				_lctPos = _lctPos + (((_target - _lctPos) max (0 - _step)) min _step);
			} else {
				_lctPos = _target;
			};
			_rotor set ["lctPos", _lctPos];
			_mastPitchOutput = 0 - _lctPos;
		};
	};
	case COAXIAL: {
	};
	case TILTROTOR: {
	};
	case COMPOUND: {
	};
};

[_pitchOutput, _rollOutput, _collOutput, _tandemPitchOutput, _tandemYawOutput, _mastPitchOutput];
