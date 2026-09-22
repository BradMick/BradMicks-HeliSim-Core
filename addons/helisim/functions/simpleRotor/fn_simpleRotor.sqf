/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_simpleRotor

Description:
    Simple rotor provides a simple, grounded in reality simulation of a
    helicopters rotor. Translational Lift, Ground Effect and Vortex Ring State
    are all simulated.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

params [ "_heli"
       , "_rotorIndex"
       , "_type"
       , "_dir"
       , "_numBlades"
       , "_gearRatio"
       , "_pivot"
       , "_mastLength"
       , "_rot"
       , "_pitchMin"
       , "_pitchMid"
       , "_pitchMax"
       , "_cyclicPitchGain"
       ,"_rollMin"
       , "_rollMid"
       , "_rollMax"
       , "_cyclicRollGain"
       , "_flapGainLat"
       , "_flapTimeConst"
       , "_bladeRadius"
       , "_bladeChord"
       , "_bladeMass"
       , "_liftCoefTable"
       , "_dragCoefTable"
       ];

if (!local _heli) exitWith {};

private _deltaTime          = _heli getVariable "bmkhs_deltaTime";

([ _heli
 , _type
 , _pitchMin
 , _pitchMid
 , _pitchMax
 , _rollMin
 , _rollMid
 , _rollMax ] call bmkhs_fnc_simpleRotorControl)
	params [ "_pitchOutput"
		   , "_rollOutput"
		   , "_collOutput"];

//Disc tilt
private _flapTimeConstLon   = _flapTimeConst select 0;
private _flapTimeConstLat   = _flapTimeConst select 1;
private _flapLonTarget      = _pitchFeather;
private _flapLatTarget      = _rollFeather;
private _flapLon            = [(_heli getVariable "bmkhs_simpleRotorFlapLon") select _rotorIndex, _flapLonTarget, (_deltaTime / _flapTimeConstLon)] call BIS_fnc_lerp;
private _flapLat            = [(_heli getVariable "bmkhs_simpleRotorFlapLat") select _rotorIndex, _flapLatTarget, (_deltaTime / _flapTimeConstLat)] call BIS_fnc_lerp;
[_heli, "bmkhs_simpleRotorFlapLon", _rotorIndex, _flapLon] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_simpleRotorFlapLat", _rotorIndex, _flapLat] call bmkhs_fnc_utilSetArrayVariable;

private _p              = _rot select 0;
private _r              = _rot select 1;
private _y              = _rot select 2;
//Mast frame
private _fVec           = [[0.0, 1.0, 0.0], _p, _r, _y] call bmkhs_fnc_mathVectorRotate;
private _rVec           = [[1.0, 0.0, 0.0], _p, _r, _y] call bmkhs_fnc_mathVectorRotate;
private _uVec           = [[0.0, 0.0, 1.0], _p, _r, _y] call bmkhs_fnc_mathVectorRotate;
//Calculate positions
private _pos     		= _pivot vectorAdd (_uVec vectorMultiply _mastLength);
private _heliCom 		= getCenterOfMass _heli;
//Environment
private _altitude       = _heli getVariable "bmkhs_PA";
private _temperature    = _heli getVariable "bmkhs_FAT";
private _dryAirDensity  = _heli getVariable "bmkhs_RHO";

//Velocity in hub axes
private _velModel = _heli getVariable "bmkhs_velModelSpace";
private _velX     = _velModel vectorDotProduct _rVec;
private _velY     = _velModel vectorDotProduct _fVec;
private _velZ     = _velModel vectorDotProduct _uVec;
private _velXY    = vectorMagnitude [_velX, _velY] min VEL_VNE;
if ([_velX]  call bmkhs_fnc_mathIsNAN || [_velX]  call bmkhs_fnc_mathIsINF) then { _velX  = 0.0; };
if ([_velY]  call bmkhs_fnc_mathIsNAN || [_velY]  call bmkhs_fnc_mathIsINF) then { _velY  = 0.0; };
if ([_velXY] call bmkhs_fnc_mathIsNAN || [_velXY] call bmkhs_fnc_mathIsINF) then { _velXY = 0.0; };
if ([_velZ]  call bmkhs_fnc_mathIsNAN || [_velZ]  call bmkhs_fnc_mathIsINF) then { _velZ  = 0.0; };

private _xmsnRpm        = _heli getVariable "bmkhs_xmsnOutputRpm";
private _rpm            = _xmsnRpm / _gearRatio;
private _omega          = if (_rpm == 0.0) then { 0.0 } else { (2.0 * pi) * (_rpm / 60.0) };
private _bladeArea      = _bladeRadius * _bladeChord;
private _rotorArea      = pi * (_bladeRadius * _bladeRadius);
private _tipVel         = _omega * _bladeRadius;
private _bladeRad_75    = _bladeRadius * 0.75;
private _bladeVel_75    = _omega * _bladeRad_75;

//Coefficients off the control/airspeed surfaces
private _liftCoef    = [_liftCoefTable, _collOutput, _velXY] call bmkhs_fnc_mathLinearInterp2D;
private _bladeLift   = _liftCoef * 0.5 * _dryAirDensity * _bladeArea * (_bladeVel_75 * _bladeVel_75);

private _dragCoef    = [_dragCoefTable, _collOutput, _velXY] call bmkhs_fnc_mathLinearInterp2D;
private _bladeDrag   = _dragCoef * 0.5 * _dryAirDensity * _bladeArea * (_bladeVel_75 * _bladeVel_75);
private _thrust      = _bladeLift * _numBlades;

//Induced velocity
private _viScalar = 1.0;
if (_velZ < -VEL_VRS && _velXY < VEL_ETL) then {
    _viScalar = 0.0;
} else {
    //private _denom = linearConversion [VEL_ETL, VEL_VNE, _velXY, VEL_VRS, VEL_VRS * 2.5, true];
    _viScalar = 1 - (_velZ / VEL_VRS);//_denom);
};

//Ground effect - strongest on the deck, gone by one rotor diameter up
private _heightAgl    = _heli getVariable "bmkhs_radAltRaw";
private _gndEffLimit  = _bladeRadius * 2.0;
private _gndEffScalar = if (_heightAgl >= _gndEffLimit) then { 1.0 } else {
    1.0 + ((GND_EFF - 1.0) * (1.0 - ((_heightAgl max 0.0) / _gndEffLimit)))
};

_thrust               = _thrust * _viScalar * _gndEffScalar;
//private _curGwt       = _heli getVariable "bmkhs_GWT";
//private _thrustMax    = _curGwt * GRAVITY * 2.5;
//_thrust               = _thrust min _thrustMax;

private _rotorThrust  = _uVec vectorMultiply (_thrust * _deltaTime);
private _bladeThrust  = _rotorThrust vectorMultiply 0.25;

//Dissymetry of lift
private _advanceRatio       = if (_tipVel > 1.0) then { _velXY / _tipVel } else { 0.0; };
private _windAzimuth        = if (_velXY > 0.01) then { _velX atan2 _velY } else { 0.0 };
private _flapBackRollAngle  = LAT_ARR select _rotorIndex;
private _flapBackPitchAngle = LON_ARR select _rotorIndex;
private _flapBackRoll       = _flapBackRollAngle  * _advanceRatio;
private _flapBackPitch      = _flapBackPitchAngle * _advanceRatio;

//Rotor cone angle and blade roll/pitch thrust fractions
private _collCone  = _collOutput * 12.0;
//TEMP
if (_type == TAIL) then { _collCone = 0.0; };
//END TEMP
private _rollFrac  = (sin _rollFeather)  * _cyclicRollGain;
private _pitchFrac = (sin _pitchFeather) * _cyclicPitchGain;

for "_i" from 0 to 3 do {
    private _psi       = _i * 90.0;
    //Local right and forward vectors
    private _locRVec   = [_rVec, _uVec, _psi] call bmkhs_fnc_mathVectorRotateAroundAxis;
    private _locFVec   = [_fVec, _uVec, _psi] call bmkhs_fnc_mathVectorRotateAroundAxis;
    //Flap back angles
    private _fbRoll    = _flapBackRoll  * (cos (_psi - _windAzimuth));
    private _fbPitch   = _flapBackPitch * (sin (_psi - _windAzimuth));
    //Blade flap angles
    private _rollFlap  = (_rollFeather  * (cos _psi)) + _fbRoll;
    private _pitchFlap = (_pitchFeather * (sin _psi)) + _fbPitch;
    private _bladeFlap = _collCone + _rollFlap + _pitchFlap;
    //Build the blade and thrust position
    private _blade             = [_pos vectorAdd  (_locRVec vectorMultiply _bladeRadius), _locFVec, _bladeFlap] call bmkhs_fnc_mathVectorRotateAroundAxis;
    private _bladeThrustPos    = _pos vectorAdd ((_blade vectorDiff _pos) vectorMultiply 0.75);
    //Finally, build the thrust vector
    private _thrustScalar      = 1.0
                               + (_rollFrac  * (cos _psi))
                               + (_pitchFrac * (sin _psi))
                               - ((sin _fbRoll)  * _cyclicRollGain)
                               - ((sin _fbPitch) * _cyclicPitchGain);
    //_bladeThrustScalar         = [_bladeThrustScalar, -BLADE_SCALE_MAX, BLADE_SCALE_MAX] call BIS_fnc_clamp;
    //private _thrustScalar      = 1.0 + _bladeThrustScalar;
    private _bladeThrustVector = [_bladeThrust vectorMultiply _thrustScalar, _locFVec, _bladeFlap] call bmkhs_fnc_mathVectorRotateAroundAxis;

    //Apply thrust at the thrust position
    //_heli addForce [_heli vectorModelToWorld _bladeThrustVector, _bladeThrustPos];// vectorDiff _heliCom];
    _heli addForce [_heli vectorModelToWorld _bladeThrustVector, _pos];

    if (BMKHS_FM_DEBUG) then {
        [_heli, _pos, _blade, "white"] call bmkhs_fnc_debugDrawLine;
        [_heli, _bladeThrustPos, _bladeThrustPos vectorAdd (_bladeThrustVector vectorMultiply (1.0 / 300.0)), "green"] call bmkhs_fnc_debugDrawLine;
		[_heli, _bladeThrustPos,   0.5, "red"] call bmkhs_fnc_debugDrawCross;
	};

	if (BMKHS_FORCES_DEBUG) then {
		private _acc = _heli getVariable ["bmkhs_dbgForces", []];
		_acc pushBack [format ["%1 blade %2", ["main","tail"] select (_type == TAIL), _i], _bladeThrustVector, _bladeThrustPos];
		_heli setVariable ["bmkhs_dbgForces", _acc];
	};
};

//Rotor torque & main rotor reaction torque
    private _torqueSign      = [1.0, -1.0] select (_dir == CW);
    private _bladeTorque     = _bladeDrag * _bladeRad_75;
    private _rotorTorque     = _bladeTorque * _numBlades;
if (_type == MAIN) then {
    private _reactionTorque  = _rotorTorque * _torqueSign * _deltaTime;
    _reactionTorque = _reactionTorque * REAC_TQ;
    
    //Apply main rotor reaction torque
    //TEST
    //private _moment = [20000.0 * _pitchFeather * _deltaTime, 10000 * _rollFeather * _deltaTime, 0.0];
    //_heli addTorque (_heli vectorModelToWorld _moment);
    //END TEST
    _heli addTorque (_heli vectorModelToWorld (_uVec vectorMultiply _reactionTorque));

	if (BMKHS_FORCES_DEBUG) then {
		private _acc = _heli getVariable ["bmkhs_dbgForces", []];
		_acc pushBack ["main react", [0,0,0], [0,0,0], _uVec vectorMultiply _reactionTorque];
		_heli setVariable ["bmkhs_dbgForces", _acc];
	};
};

[_heli, _rotorIndex, _rotorTorque, _gearRatio, _numBlades, _bladeMass, _bladeRadius, _deltaTime] call bmkhs_fnc_simpleRotorTorque;

if (BMKHS_FM_DEBUG) then {
	[_heli, _pos, _pos vectorAdd _rVec, "red"]     call bmkhs_fnc_debugDrawLine;
	[_heli, _pos, _pos vectorAdd _fVec, "green"]   call bmkhs_fnc_debugDrawLine;
	[_heli, _pos, _pos vectorAdd _uVec, "blue"]    call bmkhs_fnc_debugDrawLine;
};
