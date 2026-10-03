#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

private _cfg            = configOf _heli;
private _sfmPlusConfig  = _cfg >> "BMKHS_HeliSim";

private _deltaTime      = _heli getVariable "bmkhs_deltaTime";
private _heliCom        = getCenterOfMass _heli;
private _rho            = _heli getVariable "bmkhs_rho";
private _debugLineScale = 1.0 / 30.0;

private _panelSet       = (_heli getVariable "bmkhs_fuselagePanels") get "fuselageSide";
private _facing         = [[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]] select ((_panelSet get "facing") == "left");
private _dragCoefTable  = _panelSet get "dragCoefTable";
private _airfoilTable   = [_heli, _heli getVariable ["bmkhs_fuselageAirfoil", ""], "fuselage side"] call bmkhs_fnc_airfoilGet;
private _count          = _panelSet get "count";
private _coords         = _panelSet get "panels";

for "_i" from 0 to (_count - 1) do {
    private _verts = _coords select _i;
    private _a     = _verts select 0;
    private _b     = _verts select 1;
    private _c     = _verts select 2;
    private _d     = _verts select 3;

    private _f = _d vectorDiff ((_d vectorDiff _c) vectorMultiply 0.5);
    private _g = _a vectorDiff ((_a vectorDiff _b) vectorMultiply 0.5);

    private _e = _g vectorAdd ((_f vectorDiff _g) vectorMultiply 0.5);

    //Normal from the quad itself, pointed the way it faces.
    private _up = vectorNormalized ((_c vectorDiff _a) vectorCrossProduct (_d vectorDiff _b));
    if ((_up vectorDotProduct _facing) < 0.0) then { _up = _up vectorMultiply -1.0; };

    private _chordLine = vectorNormalized ([0.0, 1.0, 0.0] vectorDiff (_up vectorMultiply (_up select 1)));
    private _right     = _chordLine vectorCrossProduct _up;

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e, _e vectorAdd _chordLine, "white"] call bmkhs_fnc_debugDrawLine;
	[_heli, _e, _e vectorAdd _up,	 	 "white"] call bmkhs_fnc_debugDrawLine;
	[_heli, _e, _e vectorAdd _right,     "white"] call bmkhs_fnc_debugDrawLine;
    };

    private _velModelSpace    = (_heli getVariable "bmkhs_velModelSpace")    vectorMultiply -1.0;
    private _angVelModelSpace = (_heli getVariable "bmkhs_angVelModelSpace") vectorMultiply -1.0;
    private _deltaPos    	  = _e vectorDiff _heliCom;

	private _relWindX    	  = _velModelSpace select 0;
	private _relWindY    	  = _velModelSpace select 1;
	private _locRelWindX      = (_angVelModelSpace select 2) * -(_deltaPos select 1);

	private _relWind		  = [_relWindX + _locRelWindX, _relWindY, 0.0];

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e vectorDiff (vectorNormalized _relWind), _e, "red"] call bmkhs_fnc_debugDrawLine;
    };

    private _relWindNormalized = vectorNormalized _relWind;

	private _aoa = (_relWindNormalized vectorDotProduct _up) atan2 (_relWindNormalized vectorDotProduct _chordLine);

    //Lift coefficient
    private _area        = [_a, _b, _c, _d] call bmkhs_fnc_mathGetArea;
    private _CL          = [_airfoilTable, _aoa] call bmkhs_fnc_mathLinearInterp select 1;
    private _v            = (vectorMagnitude _relWind) min VEL_VNE;
    private _lift         = _CL * 0.5 * _rho * _area * (_v * _v);

    //Drag coefficient
    private _CD          =  [_dragCoefTable, _aoa] call bmkhs_fnc_mathLinearInterp select 1;
    private _relWindN     = _velModelSpace vectorDotProduct _up;
    private _drag         = _CD * 0.5 * _rho * _area * (_relWindN * _relWindN);

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

    if (BMKHS_FORCES_DEBUG) then {
        private _acc = _heli getVariable "bmkhs_dbgForces";
        _acc pushBack ["fuse side", _liftVector vectorAdd _dragVector, _e vectorDiff _heliCom];
        _heli setVariable ["bmkhs_dbgForces", _acc];
    };

    _heli addForce [_heli vectorModelToWorld _liftVector, _e];
    _heli addForce [_heli vectorModelToWorld _dragVector, _e];

    if (BMKHS_FM_DEBUG) then {
    //Draw the wing
    [_heli, _a, _b, "red"]   call bmkhs_fnc_debugDrawLine;
    [_heli, _b, _c, "white"] call bmkhs_fnc_debugDrawLine;
    [_heli, _c, _d, "red"]   call bmkhs_fnc_debugDrawLine;
    [_heli, _d, _a, "white"] call bmkhs_fnc_debugDrawLine;
    };
};
