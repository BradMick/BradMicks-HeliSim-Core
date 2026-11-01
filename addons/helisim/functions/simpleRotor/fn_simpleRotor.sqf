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
#include "\bmkhs_helisim\functions\damage\gStress.hpp"

params ["_heli", "_rotorIndex", "_rotor", "_destroyed"];

if (!local _heli) exitWith {};

//Keys are the config's own property names - see helisim_simpleRotor.hpp.
private _type             = _rotor get "type";
private _dir              = _rotor get "dir";
private _numBlades        = _rotor get "numBlades";
private _gearRatio        = _rotor get "gearRatio";
private _pivot            = _rotor get "pivot";
private _mastLength       = _rotor get "mastLength";
private _rot              = _rotor get "rotation";
//Casual sets a tail rotor upright - no cant, so it yaws without pitching or rolling the aircraft.
if (bmkhs_helisimRealismSetting != REALISTIC && {_type == TAIL}) then {
    _rot = [0.0, [90.0, -90.0] select ((_rot select 1) < 0), _rot select 2];
};
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
private _gndEffValue      = _rotor get "gndEffValue";
private _coneAngle        = _rotor get "coneAngle";
private _controlMap       = _rotor get "controlMap";

//Delta time
private _deltaTime          = _heli getVariable "bmkhs_deltaTime";

//Control outputs
private _controls = [_heli, _rotor] call bmkhs_fnc_simpleRotorControl;
_controls
	params [ "_pitchOutput"
		   , "_rollOutput"
		   , "_collOutput"
		   , "_tandemPitchOutput"
		   , "_tandemYawOutput"
		   , "_lctOutput"];

//Disc tilt - the LCT is longitudinal cyclic, - forward as rotation[]
private _flapLon = ([-1, 1, _pitchOutput, _flapLonMin, _flapLonMid, _flapLonMax] call bmkhs_fnc_mathLinearInterpFromCenter) - _lctOutput;
private _flapLat = [-1, 1, _rollOutput,  _flapLatMin, _flapLatMid, _flapLatMax] call bmkhs_fnc_mathLinearInterpFromCenter;
//Tandem yaw is lateral cyclic too - it tilts the disc through the same range, on top of roll's
_flapLat         = _flapLat + ([-1, 1, _tandemYawOutput, _flapLatMin, _flapLatMid, _flapLatMax] call bmkhs_fnc_mathLinearInterpFromCenter);
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
private _altitude       = _heli getVariable "bmkhs_barAlt";
private _temperature    = _heli getVariable "bmkhs_fat";
private _dryAirDensity  = _heli getVariable "bmkhs_rho";

//Velocity in hub axes
private _velModel    = _heli getVariable "bmkhs_velModelSpace";
private _angVelModel = _heli getVariable "bmkhs_angVelModelSpace";
private _velX        = _velModel vectorDotProduct _rVec;
private _velY        = _velModel vectorDotProduct _fVec;
private _velZ        = _velModel vectorDotProduct _uVec;
private _velXY       = vectorMagnitude [_velX, _velY] min VEL_VNE;
if ([_velX]  call bmkhs_fnc_mathIsNan || [_velX]  call bmkhs_fnc_mathIsInf) then { _velX  = 0.0; };
if ([_velY]  call bmkhs_fnc_mathIsNan || [_velY]  call bmkhs_fnc_mathIsInf) then { _velY  = 0.0; };
if ([_velXY] call bmkhs_fnc_mathIsNan || [_velXY] call bmkhs_fnc_mathIsInf) then { _velXY = 0.0; };
if ([_velZ]  call bmkhs_fnc_mathIsNan || [_velZ]  call bmkhs_fnc_mathIsInf) then { _velZ  = 0.0; };

//Rpm, and blade geometry/velocity
private _autoTorque     = _rotor get "autoTorque";
private _xmsnRpm        = _heli getVariable "bmkhs_xmsnOutputRpm";
private _rpm            = _xmsnRpm / _gearRatio;
//A destroyed rotor produces nothing
if (_destroyed) then {
    _autoTorque = 0.0;
    _xmsnRpm    = 0.0;
    _rpm        = 0.0;
};
private _omega          = if (_rpm == 0.0) then { 0.0 } else { (2.0 * pi) * (_rpm / 60.0) };
private _bladeArea      = _bladeRadius * _bladeChord;
private _rotorArea      = pi * (_bladeRadius * _bladeRadius);
private _tipVel         = _omega * _bladeRadius;
private _bladeRad_75    = _bladeRadius * 0.75;
private _bladeVel_75    = _omega * _bladeRad_75;

//Rotor cone angle
private _collCone       = _collOutput * _coneAngle;

//Table key - the control through the rotor's optional controlMap, which shapes the response
//between the table's rows. Cone, autorotation and mixing still see the control itself.
private _tableKey       = if (_controlMap isEqualTo []) then { _collOutput } else {
    ([_controlMap, _collOutput] call bmkhs_fnc_mathLinearInterp) select 1
};

//Dissymetry of lift - casual has none, the disc does not tilt with speed
private _realistic      = bmkhs_helisimRealismSetting == REALISTIC;
if (!_realistic) then {
    _reacTqScalar       = 0.0;
};

//Blade lift coefficient - the same at all four positions. Air down through a lifting rotor's disc
//costs angle of attack, taken off the collective at the blade pitch range.
private _liftKey        = _tableKey;
if (_type != TAIL && {_bladeVel_75 > 1.0}) then {
    _liftKey            = _tableKey - ((deg ((_velZ max 0.0) / _bladeVel_75)) / BLADE_PITCH_RANGE);
};
private _liftCoef       = [_liftCoefTable, _liftKey, _velXY] call bmkhs_fnc_mathLinearInterp2D;
//Axial speed along the thrust actually produced - a tail rotor's reverses with pedal
private _axialVel       = _velZ * ([1.0, -1.0] select (_liftCoef < 0));

//Total torque of the four blade positions
private _viScalarDenom  = linearConversion [-7.62, -19.30, _axialVel, VEL_VRS, VEL_VRS * 0.1, true];
private _rotorTorque    = 0.0;
private _torqueSign     = [1.0, -1.0] select (_dir == CW);

//Induced velocity factor - the hub's. A lifting rotor's climb inflow is in the lift key above, so
//only descent and VRS act here; a tail rotor takes both.
private _viAxial        = if (_type == TAIL) then { _axialVel } else { _axialVel min 0.0 };
private _viScalar       = if (_axialVel < -VEL_VRS && _velXY < VEL_ETL) then { 0.0 } else { 1 - (_viAxial / _viScalarDenom) };

//Because the model is 4 fixed points, we have to scale based on the number of blades
private _bladeScalar    = _numBlades / 4;

//Failed blades - each spread over the two positions it is passing. Psi runs clockwise from above.
private _liftMults  = [1.0, 1.0, 1.0, 1.0];
private _pcShares   = [0.0, 0.0, 0.0, 0.0];
private _cycMults   = [1.0, 1.0, 1.0, 1.0];
private _dragMults  = [1.0, 1.0, 1.0, 1.0];
private _separated  = 0;
private _blades     = (_heli getVariable "bmkhs_bladeState") select _rotorIndex;
private _azimuth    = (((_heli getVariable "bmkhs_rtrAzimuth") select _rotorIndex)
                       + (([-1.0, 1.0] select (_dir == CW)) * _rpm * 6.0 * _deltaTime)) mod 360.0;
[_heli, "bmkhs_rtrAzimuth", _rotorIndex, _azimuth] call bmkhs_fnc_utilSetArrayVariable;
if ((_blades findIf {(_x select 0) != BLADE_OK}) > -1) then {
    private _share = 4.0 / _numBlades;
    {
        _x params ["_mode", "_track"];
        if (_mode == BLADE_OK) then { continue };
        if (_mode == BLADE_SEPARATED) then { _separated = _separated + 1 };
        private _bladeAz = _azimuth + (_forEachIndex * 360.0 / _numBlades);
        private _m = switch (_mode) do {
            case BLADE_OUT_OF_TRACK: { 1.0 + _track };
            case BLADE_TIP_CAP:      { (1.0 + _track) * BLADE_TIP_CAP_LIFT };
            default                  { 0.0 };
        };
        for "_i" from 0 to 3 do {
            private _diff = abs (((((_bladeAz - (_i * 90.0)) mod 360.0) + 540.0) mod 360.0) - 180.0);
            private _w    = ((1.0 - (_diff / 90.0)) max 0.0) * _share;
            if (_w == 0.0) then { continue };
            _liftMults set [_i, (_liftMults select _i) + (_w * (_m - 1.0))];
            if (_mode == BLADE_PC_LINK)   then { _pcShares set [_i, (_pcShares select _i) + _w] };
            if (_mode >= BLADE_PC_LINK)   then { _cycMults set [_i, (_cycMults select _i) - _w] };
            if (_mode == BLADE_SEPARATED) then { _dragMults set [_i, (_dragMults select _i) - _w] };
        };
    } forEach _blades;
    _liftMults = _liftMults apply { _x max 0.0 };
    _cycMults  = _cycMults  apply { _x max 0.0 };
    _dragMults = _dragMults apply { _x max 0.0 };
};
//A snapped PC link flies at zero collective
private _pcLiftCoef = if (_pcShares isEqualTo [0.0, 0.0, 0.0, 0.0]) then { 0.0 } else {
    [_liftCoefTable, 0.0 - (_tableKey - _liftKey), _velXY] call bmkhs_fnc_mathLinearInterp2D
};
//Dynamic pressure on a blade, at 75% radius, times its area
private _bladeDynPressure = 0.5 * _dryAirDensity * _bladeArea * (_bladeVel_75 * _bladeVel_75);

//Flap back - the disc tilts aft with speed and sideways with coning, main rotors only
private _lonTilt        = _flapLon;
private _latTilt        = _flapLat;
if (_realistic && {_type == MAIN} && {_tipVel > 1.0}) then {
    private _mu          = _velXY / _tipVel;
    private _windAzimuth = if (_velXY > 0.01) then { _velX atan2 _velY } else { 0.0 };
    //Induced velocity - momentum theory on this rotor's thrust
    private _thrust      = abs (4.0 * _liftCoef * _bladeDynPressure * _bladeScalar * _viScalar);
    private _vi          = 0.0;
    if (_thrust > 0.0) then {
        private _vh2     = _thrust / (2.0 * _dryAirDensity * _rotorArea);
        _vi              = sqrt _vh2;
        for "_n" from 1 to 30 do {
            private _vt  = sqrt ((_velXY * _velXY) + ((_velZ + _vi) * (_velZ + _vi)));
            _vi          = (0.5 * _vi) + (0.5 * (_vh2 / (_vt max 0.5)));
        };
    };
    private _lambda      = (_velZ + _vi) / _tipVel;
    private _theta0      = rad (_collOutput * BLADE_PITCH_RANGE);
    //Aft, away from the relative wind
    private _a1          = deg (_mu * (((8.0 / 3.0) * _theta0) - (2.0 * _lambda)) / (1.0 - (0.5 * _mu * _mu)));
    //Lateral - follows the advancing side, so a clockwise rotor's is mirrored
    private _b1          = ((4.0 / 3.0) * _mu * _collCone / (1.0 + (0.5 * _mu * _mu))) * ([1.0, -1.0] select (_dir == CW));
    _lonTilt             = _lonTilt + (_b1 * (sin _windAzimuth)) - (_a1 * (cos _windAzimuth));
    _latTilt             = _latTilt + (_b1 * (cos _windAzimuth)) + (_a1 * (sin _windAzimuth));
};

//Disc frame - the whole disc tilts: longitudinal about the lateral axis, then lateral about
//the tilted forward axis
private _discUVec       = [_uVec, _rVec, _lonTilt] call bmkhs_fnc_mathVectorRotateAroundAxis;
private _discFVec       = [_fVec, _rVec, _lonTilt] call bmkhs_fnc_mathVectorRotateAroundAxis;
_discUVec               = [_discUVec, _discFVec, _latTilt] call bmkhs_fnc_mathVectorRotateAroundAxis;
private _discRVec       = [_rVec, _discFVec, _latTilt] call bmkhs_fnc_mathVectorRotateAroundAxis;

//Blade frames in the tilted disc, lift coned along the blade, and each blade's axial flow along
//its own lift - per-blade inflow
private _thrustSign     = [1.0, -1.0] select (_liftCoef < 0);
private _bladeFrames    = [];
private _bladeInflow    = [];
for "_i" from 0 to 3 do {
    private _psi        = _i * 90.0;
    private _locRVec    = [_discRVec, _discUVec, _psi] call bmkhs_fnc_mathVectorRotateAroundAxis;
    private _locFVec    = [_discFVec, _discUVec, _psi] call bmkhs_fnc_mathVectorRotateAroundAxis;
    private _liftDir    = [_discUVec, _locFVec, _collCone] call bmkhs_fnc_mathVectorRotateAroundAxis;
    _bladeFrames pushBack [_locRVec, _locFVec, _liftDir];
    _bladeInflow pushBack ([0.0, -((((_velModel vectorDotProduct _liftDir) * _thrustSign) - _axialVel) / VEL_VRS)] select _realistic);
};
//Average held to the hub's
private _inflowMean     = 0.0;
{ _inflowMean = _inflowMean + _x; } forEach _bladeInflow;
_inflowMean             = _inflowMean / 4;

for "_i" from 0 to 3 do {
    private _psi       = _i * 90.0;
    (_bladeFrames select _i) params ["_locRVec", "_locFVec", "_liftDir"];
    private _bladeFlap = _collCone;
    //Build the blade and thrust position
    private _bladeOffset       = [_locRVec vectorMultiply _bladeRadius, _locFVec, _bladeFlap] call bmkhs_fnc_mathVectorRotateAroundAxis;
    private _blade             = _pos vectorAdd _bladeOffset;
    private _bladeThrustPos    = _pos vectorAdd (_bladeOffset vectorMultiply 0.75);

    //Blade lift
    private _bladeLift      = ((_liftCoef * (_liftMults select _i)) + (_pcLiftCoef * (_pcShares select _i))) * _bladeDynPressure;
    _bladeLift              = _bladeLift * _bladeScalar;

    //Differential lift from the controls, mixed for the control scheme
    ([_heli, _rotor, _psi, _controls, _liftCoef] call bmkhs_fnc_simpleRotorGetLiftCoefDelta)
        params ["_rollLiftCoef", "_pitchLiftCoef"];
    private _liftCoefDelta  = _rollLiftCoef + _pitchLiftCoef;
    private _bladeLiftDelta = _liftCoefDelta * _bladeDynPressure * (_cycMults select _i);
    _bladeLiftDelta         = _bladeLiftDelta * _bladeScalar;

    //Blade drag
    private _dragCoef       = [_dragCoefTable, _tableKey, _velXY] call bmkhs_fnc_mathLinearInterp2D;
    private _bladeDrag      = _dragCoef * _bladeDynPressure * (_dragMults select _i);
    _bladeDrag              = _bladeDrag * _bladeScalar;
    
    //Total rotor torque - drag brakes the rotor, upflow in a descent drives it
    _rotorTorque            = _rotorTorque + (_bladeDrag * _bladeRad_75);
    if (_type == MAIN) then {
        private _upflow     = (-_velZ) max 0.0;
        _rotorTorque        = _rotorTorque - (_autoTorque * _upflow * (1.0 - _collOutput) * _bladeScalar);
    };

    //Ground effect - strongest on the deck, gone by one rotor diameter up
    private _heightAgl    = _heli getVariable "bmkhs_radAlt";
    private _gndEffLimit  = _bladeRadius * 2.0;
    private _gndEffScalar = if (_heightAgl >= _gndEffLimit) then { 1.0 } else {
        1.0 + ((_gndEffValue - 1.0) * (1.0 - ((_heightAgl max 0.0) / _gndEffLimit)))
    };

    //Build the lift vector
    _bladeLift               = (_bladeLift * (_viScalar + (_bladeInflow select _i) - _inflowMean) * _gndEffScalar) + _bladeLiftDelta;
    private _bladeLiftVector = _liftDir vectorMultiply (_bladeLift * _deltaTime);
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
		private _acc = _heli getVariable "bmkhs_dbgForces";
		_acc pushBack [format ["%1%2 blade %3", ["main","tail"] select (_type == TAIL), _rotorIndex, _i], _bladeLiftVector vectorAdd _bladeDragVector, _bladeThrustPos vectorDiff _heliCom];
		_heli setVariable ["bmkhs_dbgForces", _acc];
	};
};

//A separated blade's centrifugal pull is unopposed
private _imbalance = [0.0, 0.0, 0.0];
if (_separated > 0) then {
    private _pull = _bladeMass * (_bladeRadius * BLADE_CG_FRACTION) * (_omega * _omega);
    {
        if ((_x select 0) == BLADE_SEPARATED) then {
            private _radial = [_discRVec, _discUVec, _azimuth + (_forEachIndex * 360.0 / _numBlades)] call bmkhs_fnc_mathVectorRotateAroundAxis;
            _imbalance = _imbalance vectorDiff (_radial vectorMultiply _pull);
        };
    } forEach _blades;
    _heli addForce [_heli vectorModelToWorld (_imbalance vectorMultiply _deltaTime), _pos];
    if (BMKHS_FORCES_DEBUG) then {
        private _acc = _heli getVariable "bmkhs_dbgForces";
        _acc pushBack [format ["%1%2 imbalance", ["main","tail"] select (_type == TAIL), _rotorIndex], _imbalance vectorMultiply _deltaTime, _pos vectorDiff _heliCom];
        _heli setVariable ["bmkhs_dbgForces", _acc];
    };
};
[_heli, "bmkhs_rtrImbalance", _rotorIndex, vectorMagnitude _imbalance] call bmkhs_fnc_utilSetArrayVariable;

[_heli, _rotorIndex, _rotorTorque, _gearRatio, _numBlades - _separated, _bladeMass, _bladeRadius, _rotor get "torqueTau", _deltaTime, _destroyed] call bmkhs_fnc_simpleRotorTorque;

if (BMKHS_FM_DEBUG) then {
	[_heli, _pos, _pos vectorAdd _rVec, "red"]     call bmkhs_fnc_debugDrawLine;
	[_heli, _pos, _pos vectorAdd _fVec, "green"]   call bmkhs_fnc_debugDrawLine;
	[_heli, _pos, _pos vectorAdd _uVec, "blue"]    call bmkhs_fnc_debugDrawLine;
};
