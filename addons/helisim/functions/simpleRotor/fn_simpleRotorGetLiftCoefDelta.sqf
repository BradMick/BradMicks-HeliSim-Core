#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

//One blade's lift coefficient change from the controls, mixed for the control scheme into a
//roll and a pitch part. fn_simpleRotor adds the two.
params ["_heli", "_rotor", "_psi", "_controls", "_liftCoef"];

_controls params ["_pitchOutput", "_rollOutput", "_collOutput", "_tandemPitchOutput", "_tandemYawOutput"];

private _rollLiftCoef  = 0.0;
private _pitchLiftCoef = 0.0;

switch (_heli getVariable "bmkhs_controlScheme") do {
	case CONVENTIONAL: {
		_rollLiftCoef  = _rollOutput  * (cos _psi) * (_rotor get "rollLiftCoef");
		_pitchLiftCoef = _pitchOutput * (sin _psi) * (_rotor get "pitchLiftCoef");
	};
	case TANDEM: {
		//Pedal is lateral cyclic on each rotor, opposite ways; fore/aft stick is lift moved from
		//one rotor to the other, the same on every blade.
		private _roll  = _rollOutput * (cos _psi) * (_rotor get "rollLiftCoef");
		private _pitch = (_pitchOutput * (sin _psi) * (_rotor get "pitchLiftCoef"))
		               + (_tandemPitchOutput * (_rotor get "tandemPitchLiftCoef"));
		private _yaw   = _tandemYawOutput * (cos _psi) * (_rotor get "tandemYawLiftCoef");
		_rollLiftCoef  = _roll + _yaw;
		_pitchLiftCoef = _pitch;
	};
	case COAXIAL: {
	};
	case TILTROTOR: {
	};
	case COMPOUND: {
	};
};

[_rollLiftCoef, _pitchLiftCoef];
