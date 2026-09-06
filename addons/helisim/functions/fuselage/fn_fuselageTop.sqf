#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

private _cfg            = configOf _heli;
private _sfmPlusConfig  = _cfg >> "BMKHS_HeliSim";

private _deltaTime      = _heli getVariable "bmkhs_deltaTime";
private _heliCom        = getCenterOfMass _heli;
private _rho            = _heli getVariable "bmkhs_rho";
private _debugLineScale = 1.0 / 30.0;
private _position       = _heli getVariable "bmkhs_fuselagePosition";
private _rotation       = _heli getVariable "bmkhs_fuselageTopRotation";
//private _dragCoefTable  = _heli getVariable "bmkhs_fuselageTopDragCoefTable";
private _airfoilTable   = [_heli, _heli getVariable ["bmkhs_fuselageAirfoil", ""], "fuselage top"] call bmkhs_fnc_airfoilGet;
private _count          = _heli getVariable "bmkhs_fuselageTopCount";
private _coords         = _heli getVariable "bmkhs_fuselageTop";

private _pitch          = _rotation select 0;
private _roll           = _rotation select 1;
private _yaw            = _rotation select 2;

private _vecRight = [[1.0, 0.0, 0.0], _pitch, _roll, _yaw] call bmkhs_fnc_mathRotateVector;
private _vecFwd   = [[0.0, 1.0, 0.0], _pitch, _roll, _yaw] call bmkhs_fnc_mathRotateVector;
private _vecUp    = [[0.0, 0.0, 1.0], _pitch, _roll, _yaw] call bmkhs_fnc_mathRotateVector;


for "_i" from 0 to (_count - 1) do {
    private _verts = _coords select _i;
    private _v1    = _verts select 0;
    private _v2    = _verts select 1;
    private _v3    = _verts select 2;
    private _v4    = _verts select 3;

    private _a = _position vectorAdd (_vecRight vectorMultiply (_v1 select 0)) vectorAdd (_vecFwd vectorMultiply (_v1 select 1)) vectorAdd (_vecUp vectorMultiply (_v1 select 2));
    private _b = _position vectorAdd (_vecRight vectorMultiply (_v2 select 0)) vectorAdd (_vecFwd vectorMultiply (_v2 select 1)) vectorAdd (_vecUp vectorMultiply (_v2 select 2));
    private _c = _position vectorAdd (_vecRight vectorMultiply (_v3 select 0)) vectorAdd (_vecFwd vectorMultiply (_v3 select 1)) vectorAdd (_vecUp vectorMultiply (_v3 select 2));
    private _d = _position vectorAdd (_vecRight vectorMultiply (_v4 select 0)) vectorAdd (_vecFwd vectorMultiply (_v4 select 1)) vectorAdd (_vecUp vectorMultiply (_v4 select 2));

    private _f = _d vectorDiff ((_d vectorDiff _c) vectorMultiply 0.5);
    private _g = _a vectorDiff ((_a vectorDiff _b) vectorMultiply 0.5);

    private _e = _g vectorAdd ((_f vectorDiff _g) vectorMultiply 0.5);

	private _chordLine = _vecFwd;
	_chordLine         = vectorNormalized _chordLine;

    private _up 	   = _vecUp;
    _up         	   = vectorNormalized _up;

	private _right	   = _vecRight;
    _right         	   = vectorNormalized _right;

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e, _e vectorAdd _chordLine, "white"] call bmkhs_fnc_debugDrawLine;
	[_heli, _e, _e vectorAdd _up,	 	 "white"] call bmkhs_fnc_debugDrawLine;
	[_heli, _e, _e vectorAdd _right,     "white"] call bmkhs_fnc_debugDrawLine;
    };

    private _velModelSpace    = (_heli getVariable "bmkhs_velModelSpace")    vectorMultiply -1.0;
    private _angVelModelSpace = (_heli getVariable "bmkhs_angVelModelSpace") vectorMultiply -1.0;
    private _deltaPos    	  = _e vectorDiff _heliCom;

	private _relWindY    	  = _velModelSpace select 1;
	private _relWindZ    	  = _velModelSpace select 2;
	private _locRelWindZ      = (_angVelModelSpace select 0) * -(_deltaPos select 1);

	private _relWind		  = [0.0, _relWindY, _relWindZ + _locRelWindZ];

    //systemChat format ["_relWind = [%1, %2, %3]", _relWind select 0 toFixed 2, _relWind select 1 toFixed 2, _relWind select 2 toFixed 2];

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e vectorDiff (vectorNormalized _relWind), _e, "red"] call bmkhs_fnc_debugDrawLine;
    };

    private _relWindNormalized = vectorNormalized _relWind;

	private _aoa = (_relWindNormalized select 2) atan2 (_relWindNormalized select 1);

    //Lift coefficient
    private _area        = [_a, _b, _c, _d] call bmkhs_fnc_mathGetArea;
    private _CL          = [_airfoilTable, _aoa] call bmkhs_fnc_mathLinearInterp select 1;
    private _v            = (vectorMagnitude _relWind) min VEL_VNE;
    private _lift         = _CL * 0.5 * _rho * _area * (_v * _v);

    //Drag coefficient
    private _CD          = [_airfoilTable, _aoa] call bmkhs_fnc_mathLinearInterp select 2;
    private _drag         = _CD * 0.5 * _rho * _area * (_relWindZ * _relWindZ);

    private _liftVector = _relWindNormalized vectorCrossProduct _up;
    _liftVector = _liftVector vectorCrossProduct _relWindNormalized;
    _liftVector = vectorNormalized _liftVector;
    _liftVector = _liftVector vectorMultiply (_lift * _deltaTime);

    private _dragVector = _relWind;
    _dragVector = (vectorNormalized _dragVector) vectorMultiply -1.0;
    _dragVector = _dragVector vectorMultiply (_drag * _deltaTime);

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e vectorAdd (_liftVector vectorMultiply _debugLineScale), _e, "green"] call bmkhs_fnc_debugDrawLine;
    [_heli, _e vectorAdd (_dragVector vectorMultiply _debugLineScale), _e, "red"]   call bmkhs_fnc_debugDrawLine;
    };

    _heli addForce [_heli vectorModelToWorld _liftVector, _heliCom];
    _heli addForce [_heli vectorModelToWorld _dragVector, _heliCom];

    //This panel's OWN force and moment (F x r about the CoM), as named locals.
    private _force  = _liftVector vectorAdd _dragVector;
    private _moment = _force vectorCrossProduct _deltaPos;

    //Accumulate this panel's force into the component total.

    _heli addTorque (_heli vectorModelToWorld _moment);

    if (BMKHS_FM_DEBUG) then {
    //Draw the wing
    [_heli, _a, _b, "red"]   call bmkhs_fnc_debugDrawLine;
    [_heli, _b, _c, "white"] call bmkhs_fnc_debugDrawLine;
    [_heli, _c, _d, "red"]   call bmkhs_fnc_debugDrawLine;
    [_heli, _d, _a, "white"] call bmkhs_fnc_debugDrawLine;
    };
};
