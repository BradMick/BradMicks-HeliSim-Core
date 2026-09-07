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

params ["_heli", "_rotorIndex", "_pivot", "_rot", "_type", "_dir", "_numBlades", "_mastLength", "_gearRatio", "_bladeRadius", "_bladeChord", "_bladeMass", "_thrustCoefMin", "_thrustCoefMid", "_thrustCoefMax", "_pitchMin", "_pitchMid", "_pitchMax", "_rollMin", "_rollMid", "_rollMax", "_flapTimeConst", "_bladeCd0", "_inducedKappa", "_cyclicGain", "_rollGain", "_hitPoint"];

if (!local _heli) exitWith {};

private _deltaTime          = _heli getVariable "bmkhs_deltaTime";

([ _heli
 , _type
 , _pitchMin
 , _pitchMid
 , _pitchMax
 , _rollMin
 , _rollMid
 , _rollMax
 , _thrustCoefMin
 , _thrustCoefMid
 , _thrustCoefMax ] call bmkhs_fnc_simpleRotorControl)
	params [ "_pitchFeather"
		   , "_rollFeather"
		   , "_thrustCoef"];

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
private _mVec           = [[0.0, 0.0, 1.0], _p, _r, _y] call bmkhs_fnc_mathVectorRotate;
//Disc frame
private _discRot        = [_p - _flapLon, _r - _flapLat, _y];
private _uVec           = [[0.0, 0.0, 1.0], _discRot select 0, _discRot select 1, _discRot select 2] call bmkhs_fnc_mathVectorRotate;
//Calculate positions
private _pos     		= _pivot vectorAdd (_mVec vectorMultiply _mastLength);
private _heliCom 		= getCenterOfMass _heli;
//Environment
private _altitude       = _heli getVariable "bmkhs_PA";
private _temperature    = _heli getVariable "bmkhs_FAT";
private _dryAirDensity  = _heli getVariable "bmkhs_RHO";

private _xmsnRpm        = _heli getVariable "bmkhs_xmsnOutputRpm";
private _rpm            = _xmsnRpm / _gearRatio;
private _omega          = if (_rpm == 0.0) then { 0.0 } else { (2.0 * pi) * (_rpm / 60.0) };
private _area           = pi * (_bladeRadius * _bladeRadius);
private _tipVel         = _omega * _bladeRadius;

private _thrust         = _thrustCoef * 0.5 * _dryAirDensity * _area * (_tipVel * _tipVel);

//Velocity in hub axes
private _velModel = _heli getVariable "bmkhs_velModelSpace";
private _velX     = _velModel vectorDotProduct _rVec;
private _velY     = _velModel vectorDotProduct _fVec;
private _velZ     = _velModel vectorDotProduct _mVec;
private _velXY    = vectorMagnitude [_velX, _velY] min VEL_VNE;
if ([_velXY] call bmkhs_fnc_mathIsNAN || [_velXY] call bmkhs_fnc_mathIsINF) then { _velXY = 0.0; };
if ([_velZ]  call bmkhs_fnc_mathIsNAN || [_velZ]  call bmkhs_fnc_mathIsINF) then { _velZ  = 0.0; };

//Induced velocity
private _viScalar = 1.0;
if (_velZ < -VEL_VRS && _velXY < VEL_ETL) then {
    _viScalar = 0.0;
} else {
    _viScalar = 1 - (_velZ / VEL_VRS);
};

//Thrust vector
_thrust = _thrust * _viScalar;
private _thrustVector = _uVec vectorMultiply (_thrust * _deltaTime);

private _deltaPos = [0,0,0];
private _moment   = [0,0,0];
if (_type == MAIN) then {
	private _rollFrac  = (sin _rollFeather)  * _cyclicGain;
	private _pitchFrac = (sin _pitchFeather) * _cyclicGain;
	//Local thrust
	private _locThrustVec = _thrustVector vectorMultiply 0.25;
	//Right force pos
	private _a_forcePosRight = _pos vectorAdd  (_rVec vectorMultiply (_bladeRadius * 0.75));
	_deltaPos 	      = _a_forcePosRight vectorDiff _heliCom;
	private _a_moment = (_locThrustVec vectorMultiply  _rollFrac) vectorCrossProduct _deltaPos;
	//Forward force pos
	private _b_forcePosFwd = _pos vectorAdd  (_fVec vectorMultiply (_bladeRadius * 0.75));
	_deltaPos         = _b_forcePosFwd vectorDiff _heliCom;
	private _b_moment = (_locThrustVec vectorMultiply -_pitchFrac) vectorCrossProduct _deltaPos;
	//Left force pos
	private _c_forcePosLeft = _pos vectorDiff (_rVec vectorMultiply (_bladeRadius * 0.75));
	_deltaPos         = _c_forcePosLeft vectorDiff _heliCom;
	private _c_moment = (_locThrustVec vectorMultiply -_rollFrac) vectorCrossProduct _deltaPos;
	//Aft force pos
	private _d_forcePosAft = _pos vectorDiff (_fVec vectorMultiply (_bladeRadius * 0.75));
	_deltaPos         = _d_forcePosAft vectorDiff _heliCom;
	private _d_moment = (_locThrustVec vectorMultiply  _pitchFrac) vectorCrossProduct _deltaPos;
	
	//Total moment
	_moment   = _a_moment vectorAdd _b_moment vectorAdd _c_moment vectorAdd _d_moment;

		if (BMKHS_FM_DEBUG) then {
		[_heli, _a_forcePosRight, 0.5, "red"]   call bmkhs_fnc_debugDrawCross;
		[_heli, _b_forcePosFwd,   0.5, "green"] call bmkhs_fnc_debugDrawCross;
		[_heli, _c_forcePosLeft,  0.5, "red"]   call bmkhs_fnc_debugDrawCross;
		[_heli, _d_forcePosAft,   0.5, "green"] call bmkhs_fnc_debugDrawCross;
	};
} else {
	_deltaPos = _pos vectorDiff _heliCom;
	_moment   = _thrustVector vectorCrossProduct _deltaPos;
	_moment set [1, (_moment select 1) * _rollGain];
};

_heli addForce  [_heli vectorModelToWorld _thrustVector, _pos];
_heli addTorque (_heli vectorModelToWorld _moment);

systemChat format ["simpleRotor%1: _thrust = %2", _rotorIndex, _thrust toFixed 0];
systemChat format ["simpleRotor%1: _moment = [%2, %3, %4]", _rotorIndex, _moment select 0 toFixed 0, _moment select 1 toFixed 0, _moment select 2 toFixed 0];

if (BMKHS_FM_DEBUG) then {
	[_heli, _pos, _pos vectorAdd _rVec, "red"]     call bmkhs_fnc_debugDrawLine;
	[_heli, _pos, _pos vectorAdd _fVec, "green"]   call bmkhs_fnc_debugDrawLine;
	[_heli, _pos, _pos vectorAdd _uVec, "blue"]    call bmkhs_fnc_debugDrawLine;

	[_heli, 24, _pos, _discRot, _bladeRadius, "white"] call bmkhs_fnc_debugDrawCircle;
};
