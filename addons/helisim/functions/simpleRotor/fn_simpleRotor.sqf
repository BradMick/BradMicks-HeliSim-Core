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

params ["_heli", "_rotorIndex", "_rotor"];

if (!local _heli) exitWith {};

//Keys are the config's own property names - see helisim_simpleRotor.hpp.
private _type             = _rotor get "type";
private _dir              = _rotor get "dir";
private _numBlades        = _rotor get "numBlades";
private _gearRatio        = _rotor get "gearRatio";
private _pivot            = _rotor get "pivot";
private _mastLength       = _rotor get "mastLength";
private _rot              = _rotor get "rotation";
private _flapLonMin       = _rotor get "pitchFlapMin";
private _flapLonMid       = _rotor get "pitchFlapMid";
private _flapLonMax       = _rotor get "pitchFlapMax";
private _flapLatMin       = _rotor get "rollFlapMin";
private _flapLatMid       = _rotor get "rollFlapMid";
private _flapLatMax       = _rotor get "rollFlapMax";
private _bladeRadius      = _rotor get "bladeRadius";
private _bladeChord       = _rotor get "bladeChord";
private _bladeMass        = _rotor get "bladeMass";
private _liftCoefTable    = _rotor get "liftCoefTable";
private _dragCoefTable    = _rotor get "dragCoefTable";
private _reacTqScalar     = _rotor get "reacTqScalar";
private _flapBackRollMax  = _rotor get "flapBackRollMax";
private _flapBackPitchMax = _rotor get "flapBackPitchMax";
private _gndEffValue      = _rotor get "gndEffValue";
private _rollLiftCoef     = _rotor get "rollLiftCoef";
private _pitchLiftCoef    = _rotor get "pitchLiftCoef";
private _coneAngle        = _rotor get "coneAngle";
private _autoTorque       = _rotor get "autoTorque";

//Delta time
private _deltaTime          = _heli getVariable "bmkhs_deltaTime";

//Control outputs
([_heli, _type] call bmkhs_fnc_simpleRotorControl)
	params [ "_pitchOutput"
		   , "_rollOutput"
		   , "_collOutput"];

//Disc tilt
private _flapLon = [-1, 1, _pitchOutput, _flapLonMin, _flapLonMid, _flapLonMax] call bmkhs_fnc_mathLinearInterpFromCenter;
private _flapLat = [-1, 1, _rollOutput,  _flapLatMin, _flapLatMid, _flapLatMax] call bmkhs_fnc_mathLinearInterpFromCenter;
//Rotor orientation
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
private _velModel    = _heli getVariable "bmkhs_velModelSpace";
private _angVelModel = _heli getVariable "bmkhs_angVelModelSpace";
private _velX        = _velModel vectorDotProduct _rVec;
private _velY        = _velModel vectorDotProduct _fVec;
private _velZ        = _velModel vectorDotProduct _uVec;
private _velXY       = vectorMagnitude [_velX, _velY] min VEL_VNE;
if ([_velX]  call bmkhs_fnc_mathIsNAN || [_velX]  call bmkhs_fnc_mathIsINF) then { _velX  = 0.0; };
if ([_velY]  call bmkhs_fnc_mathIsNAN || [_velY]  call bmkhs_fnc_mathIsINF) then { _velY  = 0.0; };
if ([_velXY] call bmkhs_fnc_mathIsNAN || [_velXY] call bmkhs_fnc_mathIsINF) then { _velXY = 0.0; };
if ([_velZ]  call bmkhs_fnc_mathIsNAN || [_velZ]  call bmkhs_fnc_mathIsINF) then { _velZ  = 0.0; };

//Rpm, and blade geometry/velocity
private _xmsnRpm        = _heli getVariable "bmkhs_xmsnOutputRpm";
private _rpm            = _xmsnRpm / _gearRatio;
private _omega          = if (_rpm == 0.0) then { 0.0 } else { (2.0 * pi) * (_rpm / 60.0) };
private _bladeArea      = _bladeRadius * _bladeChord;
private _rotorArea      = pi * (_bladeRadius * _bladeRadius);
private _tipVel         = _omega * _bladeRadius;
private _bladeRad_75    = _bladeRadius * 0.75;
private _bladeVel_75    = _omega * _bladeRad_75;

//Rotor cone angle
private _collCone       = _collOutput * _coneAngle;

//Dissymetry of lift - casual has none, the disc does not tilt with speed
if (bmkhs_helisimRealismSetting != REALISTIC) then {
    _flapBackRollMax  = 0.0;
    _flapBackPitchMax = 0.0;
    _reacTqScalar     = 0.0;
};

//"Flapback" a.k.a. "Blow Back"
private _advanceRatio       = if (_tipVel > 1.0) then { _velXY / _tipVel } else { 0.0; };
private _windAzimuth        = if (_velXY > 0.01) then { _velX atan2 _velY } else { 0.0 };
private _flapBackRollAngle  = _flapBackRollMax  * _advanceRatio;
private _flapBackPitchAngle = _flapBackPitchMax * _advanceRatio;

//Total torque of the four blade positions
private _viScalarDenom  = linearConversion [-7.62, -19.30, _velZ, VEL_VRS, VEL_VRS * 0.1, true];
private _rotorTorque    = 0.0;
private _torqueSign     = [1.0, -1.0] select (_dir == CW);

for "_i" from 0 to 3 do {
    private _psi       = _i * 90.0;
    //Local right and forward vectors
    private _locRVec   = [_rVec, _uVec, _psi] call bmkhs_fnc_mathVectorRotateAroundAxis;
    private _locFVec   = [_fVec, _uVec, _psi] call bmkhs_fnc_mathVectorRotateAroundAxis;
    //Flap back angles
    private _fbRoll    = _flapBackRollAngle  * (cos (_psi - _windAzimuth));
    private _fbPitch   = _flapBackPitchAngle * (sin (_psi - _windAzimuth));
    //Blade flap angles
    private _rollFlap  = (_flapLat * (cos _psi)) + _fbRoll;
    private _pitchFlap = (_flapLon * (sin _psi)) + _fbPitch;
    private _bladeFlap = _collCone + _rollFlap + _pitchFlap;
    //Build the blade and thrust position
    private _bladeOffset       = [_locRVec vectorMultiply _bladeRadius, _locFVec, _bladeFlap] call bmkhs_fnc_mathVectorRotateAroundAxis;
    private _blade             = _pos vectorAdd _bladeOffset;
    private _bladeThrustPos    = _pos vectorAdd (_bladeOffset vectorMultiply 0.75);

    //Because the model is 4 fixed points, we have to scale based on the number of blades
    private _bladeScalar    = _numBlades / 4;
    //Blade lift
    private _liftCoef       = [_liftCoefTable, _collOutput, _velXY] call bmkhs_fnc_mathLinearInterp2D;
    //Blade lift
    private _bladeLift      = _liftCoef * 0.5 * _dryAirDensity * _bladeArea * (_bladeVel_75 * _bladeVel_75);
    _bladeLift              = _bladeLift * _bladeScalar;
    //Differential lift from cyclic application
    private _liftCoefDelta  = (_rollOutput  * (cos _psi) * _rollLiftCoef)
                            + (_pitchOutput * (sin _psi) * _pitchLiftCoef);
    private _bladeLiftDelta = _liftCoefDelta * 0.5 * _dryAirDensity * _bladeArea * (_bladeVel_75 * _bladeVel_75);
    _bladeLiftDelta         = _bladeLiftDelta * _bladeScalar;
    //Blade drag
    private _dragCoef       = [_dragCoefTable, _collOutput, _velXY] call bmkhs_fnc_mathLinearInterp2D;
    private _bladeDrag      = _dragCoef * 0.5 * _dryAirDensity * _bladeArea * (_bladeVel_75 * _bladeVel_75);
    _bladeDrag              = _bladeDrag * _bladeScalar;
    //Total rotor torque - drag brakes the rotor, upflow in a descent drives it
    _rotorTorque            = _rotorTorque + (_bladeDrag * _bladeRad_75);
    if (_type == MAIN) then {
        private _upflow     = (-_velZ) max 0.0;
        _rotorTorque        = _rotorTorque - (_autoTorque * _upflow * (1.0 - _collOutput) * _bladeScalar);
    };

    //Induced velocity
    private _viScalar = 1.0;
    if (_velZ < -VEL_VRS && _velXY < VEL_ETL) then {
        _viScalar = 0.0;
    } else {
        _viScalar = 1 - (_velZ / _viScalarDenom);
    };

    //Ground effect - strongest on the deck, gone by one rotor diameter up
    private _heightAgl    = _heli getVariable "bmkhs_radAltRaw";
    private _gndEffLimit  = _bladeRadius * 2.0;
    private _gndEffScalar = if (_heightAgl >= _gndEffLimit) then { 1.0 } else {
        1.0 + ((_gndEffValue - 1.0) * (1.0 - ((_heightAgl max 0.0) / _gndEffLimit)))
    };

    //Build the lift vector
    _bladeLift               = (_bladeLift * _viScalar * _gndEffScalar) + _bladeLiftDelta;
    private _bladeLiftVector = [_uVec vectorMultiply (_bladeLift * _deltaTime), _locFVec, _bladeFlap] call bmkhs_fnc_mathVectorRotateAroundAxis;
    //Build the drag vector
    private _bladeDragVector = _locFVec vectorMultiply (-_bladeDrag * _deltaTime * _torqueSign * _reacTqScalar);

    //Apply thrust at the thrust position
    _heli addForce [_heli vectorModelToWorld _bladeLiftVector, _bladeThrustPos];
    _heli addForce [_heli vectorModelToWorld _bladeDragVector, _bladeThrustPos];

    if (BMKHS_FM_DEBUG) then {
        [_heli, _pos, _blade, "white"] call bmkhs_fnc_debugDrawLine;
        [_heli, _bladeThrustPos, _bladeThrustPos vectorAdd (_bladeLiftVector vectorMultiply (1.0 / 100.0)), "green"] call bmkhs_fnc_debugDrawLine;
        [_heli, _bladeThrustPos, _bladeThrustPos vectorAdd (_bladeDragVector vectorMultiply (1.0 / 100.0)), "red"] call bmkhs_fnc_debugDrawLine;
		[_heli, _bladeThrustPos,   0.5, "red"] call bmkhs_fnc_debugDrawCross;
	};

	if (BMKHS_FORCES_DEBUG) then {
		private _acc = _heli getVariable ["bmkhs_dbgForces", []];
		_acc pushBack [format ["%1 blade %2", ["main","tail"] select (_type == TAIL), _i], _bladeLiftVector vectorAdd _bladeDragVector, _bladeThrustPos vectorDiff _heliCom];
		_heli setVariable ["bmkhs_dbgForces", _acc];
	};
};

[_heli, _rotorIndex, _rotorTorque, _gearRatio, _numBlades, _bladeMass, _bladeRadius, _rotor get "torqueTau", _deltaTime] call bmkhs_fnc_simpleRotorTorque;

if (BMKHS_FM_DEBUG) then {
	[_heli, _pos, _pos vectorAdd _rVec, "red"]     call bmkhs_fnc_debugDrawLine;
	[_heli, _pos, _pos vectorAdd _fVec, "green"]   call bmkhs_fnc_debugDrawLine;
	[_heli, _pos, _pos vectorAdd _uVec, "blue"]    call bmkhs_fnc_debugDrawLine;
};
