#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

params ["_heli", "_rotor"];

private _type                   = _rotor get "type";

private _pitchInput             = _heli getVariable "bmkhs_fltCtrlPitchOut";
private _pitchOutput			= 0.0;

private _rollInput              = _heli getVariable "bmkhs_fltCtrlRollOut";
private _rollOutput 			= 0.0;

private _yawInput               = _heli getVariable "bmkhs_fltCtrlYawOut";

private _collInput              = _heli getVariable "bmkhs_fltCtrlCollOut";
private _collOutput 			= 0.0;
//Tandem only - the rotor applies these through tandemPitchLiftCoef and tandemYawLiftCoef.
private _tandemPitchOutput      = 0.0;
private _tandemYawOutput        = 0.0;
//LCT, deg of longitudinal cyclic - the tandem's
private _lctOutput              = 0.0;

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

		//LCT - the mast tilts on its schedule (airspeed by pressure altitude), - forward, + aft.
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
			_lctOutput = _lctPos;
		};
	};
	case COAXIAL: {
	};
	case TILTROTOR: {
	};
	case COMPOUND: {
	};
};

[_pitchOutput, _rollOutput, _collOutput, _tandemPitchOutput, _tandemYawOutput, _lctOutput];
