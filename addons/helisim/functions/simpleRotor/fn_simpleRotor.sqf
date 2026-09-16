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

params ["_heli", "_rotorIndex", "_pivot", "_rot", "_type", "_dir", "_numBlades", "_mastLength", "_gearRatio", "_bladeRadius", "_bladeChord", "_bladeMass", "_thrustCoefMin", "_thrustCoefMid", "_thrustCoefMax", "_pitchMin", "_pitchMid", "_pitchMax", "_rollMin", "_rollMid", "_rollMax", "_flapTimeConst", "_dragCoefMin", "_dragCoefMid", "_dragCoefMax", "_inducedKappa", "_cyclicPitchGain", "_cyclicRollGain","_rollGain", "_hitPoint"];

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
 , _thrustCoefMax
 , _dragCoefMin
 , _dragCoefMid
 , _dragCoefMax ] call bmkhs_fnc_simpleRotorControl)
	params [ "_pitchFeather"
		   , "_rollFeather"
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
private _bladeArea      = _bladeRadius * _bladeChord;
private _rotorArea      = pi * (_bladeRadius * _bladeRadius);
private _tipVel         = _omega * _bladeRadius;
private _bladeRad_75    = _bladeRadius * 0.75;
private _bladeVel_75    = _omega * _bladeRad_75;

//Lift coef - collective down the side, airspeed (m/s) across the top
private _liftCoefTable =
[
//  Coll \ A/S    0.00    10.29    20.58    36.01    46.30    51.44    61.73    66.88    72.02
     ["A/S",    0.00,   10.29,   20.58,   36.01,   46.30,   51.44,   61.73,   66.88,   72.02]
    ,[ 0.00,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000]
    ,[ 0.20,  0.1666,  0.1695,  0.1826,  0.1904,  0.1841,  0.1786,  0.1666,  0.1541,  0.1252]
    ,[ 0.40,  0.2331,  0.2389,  0.2653,  0.2809,  0.2681,  0.2571,  0.2331,  0.2081,  0.1503]
    ,[ 0.64,  0.2997,  0.3084,  0.3479,  0.3713,  0.3522,  0.3357,  0.2997,  0.2622,  0.1755]
    ,[ 0.80,  0.3330,  0.4100,  0.5340,  0.6100,  0.5790,  0.5713,  0.4860,  0.3928,  0.2915]
    ,[ 1.00,  0.4120,  0.4120,  0.4120,  0.4120,  0.4120,  0.4120,  0.4120,  0.4120,  0.4120]
];

//Drag coef - collective down the side, airspeed (m/s) across the top
private _dragCoefTable =
[
//  Coll \ A/S    0.00    10.29    20.58    36.01    46.30    51.44    61.73    66.88    72.02
     ["A/S",    0.00,   10.29,   20.58,   36.01,   46.30,   51.44,   61.73,   66.88,   72.02]
    ,[ 0.00,  0.0085,  0.0085,  0.0065,  0.0005,  0.0005,  0.0005,  0.0005,  0.0005,  0.0005]
    ,[ 0.20,  0.0206,  0.0190,  0.0160,  0.0106,  0.0107,  0.0108,  0.0117,  0.0117,  0.0117]
    ,[ 0.40,  0.0326,  0.0296,  0.0255,  0.0206,  0.0208,  0.0211,  0.0229,  0.0229,  0.0229]
    ,[ 0.64,  0.0447,  0.0401,  0.0350,  0.0307,  0.0310,  0.0314,  0.0341,  0.0341,  0.0341]
    ,[ 0.80,  0.0474,  0.0474,  0.0474,  0.0474,  0.0474,  0.0474,  0.0474,  0.0474,  0.0474]
    ,[ 1.00,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000]
];

//Velocity in hub axes
private _velModel = _heli getVariable "bmkhs_velModelSpace";
private _velX     = _velModel vectorDotProduct _rVec;
private _velY     = _velModel vectorDotProduct _fVec;
private _velZ     = _velModel vectorDotProduct _mVec;
private _velXY    = vectorMagnitude [_velX, _velY] min VEL_VNE;
if ([_velXY] call bmkhs_fnc_mathIsNAN || [_velXY] call bmkhs_fnc_mathIsINF) then { _velXY = 0.0; };
if ([_velZ]  call bmkhs_fnc_mathIsNAN || [_velZ]  call bmkhs_fnc_mathIsINF) then { _velZ  = 0.0; };

//A TAIL reads pedal, not collective, and its curve is nonlinear and asymmetric -
//left and right pedal do not have the same authority.
if (_type == TAIL) then {
    _liftCoefTable =
    [
    //  Pedal \ A/S   0.00    10.29    20.58    36.01    46.30    51.44    61.73    66.88    72.02
         ["A/S",    0.00,   10.29,   20.58,   36.01,   46.30,   51.44,   61.73,   66.88,   72.02]
        ,[-1.00, -1.6086, -1.6338, -1.7483, -1.8162, -1.7608, -1.7130, -1.6086, -1.5662, -1.5462]
        ,[-0.50, -0.8405, -0.8657, -0.9802, -1.0481, -0.9927, -0.9449, -0.8405, -0.7981, -0.7781]
        ,[ 0.00, -0.0724, -0.0976, -0.2121, -0.2800, -0.2246, -0.1768, -0.0724, -0.0300, -0.0100]
        ,[ 0.50,  0.6957,  0.6705,  0.5560,  0.4881,  0.5435,  0.5913,  0.6957,  0.7381,  0.7581]
        ,[ 1.00,  1.4638,  1.4386,  1.3241,  1.2562,  1.3116,  1.3594,  1.4638,  1.5062,  1.5262]
    ];
    _dragCoefTable =
    [
    //  Pedal \ A/S   0.00    10.29    20.58    36.01    46.30    51.44    61.73    66.88    72.02
         ["A/S",    0.00,   10.29,   20.58,   36.01,   46.30,   51.44,   61.73,   66.88,   72.02]
        ,[-1.00,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110]
        ,[ 0.00,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110]
        ,[ 1.00,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110]
    ];
};

//Coefficients off the control/airspeed surfaces
private _liftGrid    = [_liftCoefTable, "liftCoefTable"] call bmkhs_fnc_mathBuildInterpGrid;
private _liftCoef    = [_liftGrid, _collOutput, _velXY] call bmkhs_fnc_mathLinearInterp2D;
private _bladeLift   = _liftCoef * 0.5 * _dryAirDensity * _bladeArea * (_bladeVel_75 * _bladeVel_75);

private _dragGrid    = [_dragCoefTable, "dragCoefTable"] call bmkhs_fnc_mathBuildInterpGrid;
private _dragCoef    = [_dragGrid, _collOutput, _velXY] call bmkhs_fnc_mathLinearInterp2D;
private _bladeDrag   = _dragCoef * 0.5 * _dryAirDensity * _bladeArea * (_bladeVel_75 * _bladeVel_75);

private _thrust      = _bladeLift * _numBlades;

private _torqueSign     = [-1.0, 1.0] select (_dir == CW);
private _bladeTorque    = _bladeDrag * _bladeRad_75;
private _rotorTorque    = _bladeTorque * _numBlades;
private _reactionTorque = (_bladeTorque * _numBlades) * _torqueSign * _deltaTime;

//Induced velocity
private _viScalar = 1.0;
if (_velZ < -VEL_VRS && _velXY < VEL_ETL) then {
    _viScalar = 0.0;
} else {
    _viScalar = 1 - (_velZ / VEL_VRS);
};

_thrust = _thrust * _viScalar;
private _thrustVector = _uVec vectorMultiply (_thrust * _deltaTime);

private _deltaPos = [0,0,0];
private _moment   = [0,0,0];
if (_type == MAIN) then {
	private _rollFrac  = (sin _rollFeather)  * _cyclicRollGain;
	private _pitchFrac = (sin _pitchFeather) * _cyclicPitchGain;
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
	
	//Total moment - the four thrust couples plus the drag torque about the mast
	_moment   = _a_moment vectorAdd _b_moment vectorAdd _c_moment vectorAdd _d_moment vectorAdd (_mVec vectorMultiply _reactionTorque);

	if (BMKHS_FM_DEBUG) then {
		[_heli, _a_forcePosRight, 0.5, "red"]   call bmkhs_fnc_debugDrawCross;
		[_heli, _b_forcePosFwd,   0.5, "green"] call bmkhs_fnc_debugDrawCross;
		[_heli, _c_forcePosLeft,  0.5, "red"]   call bmkhs_fnc_debugDrawCross;
		[_heli, _d_forcePosAft,   0.5, "green"] call bmkhs_fnc_debugDrawCross;
	};
} else {

	_deltaPos = _pos vectorDiff _heliCom;
	_moment   = (_thrustVector vectorCrossProduct _deltaPos);// vectorAdd (_mVec vectorMultiply _reactionTorque);
	_moment set [1, (_moment select 1) * _rollGain];
};

systemChat format ["v3 SR %1: _thrust = [%2, %3, %4] _moment = [%5, %6, %7]", _rotorIndex, (_thrustVector select 0) toFixed 0, (_thrustVector select 1) toFixed 0, (_thrustVector select 2) toFixed 0, (_moment select 0) toFixed 0, (_moment select 1) toFixed 0, (_moment select 2) toFixed 0];


_heli addForce  [_heli vectorModelToWorld _thrustVector, _pos];
_heli addTorque (_heli vectorModelToWorld _moment);

//Refer to the engine shaft and publish what the transmission and gauge read.
[_heli, _rotorIndex, _rotorTorque, _gearRatio, _numBlades, _bladeMass, _bladeRadius, _deltaTime] call bmkhs_fnc_simpleRotorTorque;

if (BMKHS_FM_DEBUG) then {
	[_heli, _pos, _pos vectorAdd _rVec, "red"]     call bmkhs_fnc_debugDrawLine;
	[_heli, _pos, _pos vectorAdd _fVec, "green"]   call bmkhs_fnc_debugDrawLine;
	[_heli, _pos, _pos vectorAdd _uVec, "blue"]    call bmkhs_fnc_debugDrawLine;

	[_heli, 24, _pos, _discRot, _bladeRadius, "white"] call bmkhs_fnc_debugDrawCircle;
};
