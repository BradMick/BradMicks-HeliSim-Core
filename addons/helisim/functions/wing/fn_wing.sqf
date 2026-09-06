#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\systems\systems.hpp"

params ["_heli","_wingPos","_pitch","_roll","_span","_chord","_sweep","_twist","_tipWidthScalar",["_isStab", false],["_wingIndex", 0],["_airfoilTable", []]];

if (!local _heli) exitWith {};

private _cfg           = configOf _heli;
private _sfmPlusConfig = _cfg >> "BMKHS_HeliSim";

private _deltaTime      = _heli getVariable "bmkhs_deltaTime";
private _rho            = _heli getVariable "bmkhs_rho";
private _heliCom        = getCenterOfMass _heli;
private _numElements    = (_heli getVariable "bmkhs_wingNumElements")  select _wingIndex;
private _chordLinePos   = (_heli getVariable "bmkhs_wingChordLinePos") select _wingIndex;

private _debugLineScale = 1.0 / 30.0;

//Wing coords
//  + A-------------+-------------B
//    |             |             |
//    F-------------E-------------G
//    |             |             |
//  - D-------------+-------------C

private _A_wingRootLeadingEdge  = [];
private _B_wingTipLeadingEdge   = [];
private _C_wingTipTrailingEdge  = [];
private _D_wingRootTrailingEdge = [];


if (_isStab) then {
    private _stabDamage = [_heli, "stabilator"] call bmkhs_fnc_damageGet;
    private _dcBusOn    = _heli getVariable ["bmkhs_dcBusOn", true];

    private _stabOutputTable = [[]];
    private _desiredTheta    = 0.0;
    private _theta           = _heli getVariable "bmkhs_stabilatorPosition";

    private _intStabTable = [getArray (_sfmPlusConfig >> "heliSimStabTable"), (_heli getVariable "bmkhs_collectiveOutput")] call bmkhs_fnc_mathLinearInterp;
    _stabOutputTable = [
                        [15.43, _intStabTable select 1]   //30kts
                       ,[20.58, _intStabTable select 2]   //40kts
                       ,[25.72, _intStabTable select 3]   //50kts
                       ,[29.58, _intStabTable select 4]   //57.5kts
                       ,[41.16, _intStabTable select 5]   //80kts
                       ,[42.44, _intStabTable select 6]   //82.5kts
                       ,[51.44, _intStabTable select 7]   //100kts
                       ,[59.16, _intStabTable select 8]   //115kts
                       ,[61.73, _intStabTable select 9]   //120kts
                       ,[72.02, _intStabTable select 10]  //140kts
                       ,[77.17, _intStabTable select 11]  //150kts
                       ,[82.31, _intStabTable select 12]  //160kts
                       ,[84.88, _intStabTable select 13]  //165kts
                       ,[92.60, _intStabTable select 14]  //180kts
                       ];

    if (_stabDamage < SYS_STAB_DMG_THRESH && _dcBusOn) then {
        _desiredTheta = [_stabOutputTable, (_heli getVariable "bmkhs_vel2D") * KNOTS_TO_MPS] call bmkhs_fnc_mathLinearInterp select 1;
        _theta        = [_theta, _desiredTheta, (1.0 / 1.5) * _deltaTime] call BIS_fnc_lerp;
        _heli setVariable ["bmkhs_stabilatorPosition", _theta];
    };

    _heli animate ["Hstab", _theta];

    private _vectorRight   = [1.0, 0.0, 0.0];
    private _vectorForward = [0.0, 1.0, 0.0];

    // Stab leading edge is _wingPos; trailing edges are built rearward then rotated about _wingPos
    _A_wingRootLeadingEdge  = _wingPos vectorDiff (_vectorRight vectorMultiply (_span * 0.5));
    _B_wingTipLeadingEdge   = _wingPos vectorAdd  (_vectorRight vectorMultiply (_span * 0.5));
    _C_wingTipTrailingEdge  = _B_wingTipLeadingEdge  vectorDiff (_vectorForward vectorMultiply _chord);
    _D_wingRootTrailingEdge = _A_wingRootLeadingEdge vectorDiff (_vectorForward vectorMultiply _chord);

    private _stabTheta  = _theta;
    private _stabRoot   = _A_wingRootLeadingEdge vectorDiff _D_wingRootTrailingEdge;
    _stabRoot           = [_stabRoot, _vectorRight, _stabTheta] call bmkhs_fnc_mathVectorRotateAroundAxis;
    _D_wingRootTrailingEdge = _A_wingRootLeadingEdge vectorDiff _stabRoot;

    private _stabTip    = _B_wingTipLeadingEdge vectorDiff _C_wingTipTrailingEdge;
    _stabTip            = [_stabTip, _vectorRight, _stabTheta] call bmkhs_fnc_mathVectorRotateAroundAxis;
    _C_wingTipTrailingEdge = _B_wingTipLeadingEdge vectorDiff _stabTip;

} else {
    private _vectorRight   = [[1.0, 0.0, 0.0], _pitch, _roll, 0.0] call bmkhs_fnc_mathVectorRotate;
    private _vectorForward = [[0.0, 1.0, 0.0], _pitch, _roll, 0.0] call bmkhs_fnc_mathVectorRotate;

    private _wingRootCenter = _wingPos       vectorDiff (_vectorRight   vectorMultiply (_span * 0.5));
    private _wingTipCenter  = _wingPos       vectorAdd  (_vectorRight   vectorMultiply (_span * 0.5));
    _wingTipCenter          = _wingTipCenter vectorAdd  (_vectorForward vectorMultiply _sweep);

    _A_wingRootLeadingEdge  = _wingRootCenter vectorAdd  (_vectorForward vectorMultiply  (_chord * 0.5));
    _B_wingTipLeadingEdge   = _wingTipCenter  vectorAdd  (_vectorForward vectorMultiply ((_chord * 0.5) * _tipWidthScalar));
    _C_wingTipTrailingEdge  = _wingTipCenter  vectorDiff (_vectorForward vectorMultiply ((_chord * 0.5) * _tipWidthScalar));
    _D_wingRootTrailingEdge = _wingRootCenter vectorDiff (_vectorForward vectorMultiply  (_chord * 0.5));

    private _wingTip       = _B_wingTipLeadingEdge vectorDiff _C_wingTipTrailingEdge;
    _wingTip               = [_wingTip, _vectorRight, _twist] call bmkhs_fnc_mathVectorRotateAroundAxis;
    _B_wingTipLeadingEdge  = _wingTipCenter vectorAdd  (_wingTip vectorMultiply 0.5);
    _C_wingTipTrailingEdge = _wingTipCenter vectorDiff (_wingTip vectorMultiply 0.5);

};

for "_j" from 0 to (_numElements - 1) do {
    private _a = _A_wingRootLeadingEdge  vectorAdd ((_B_wingTipLeadingEdge  vectorDiff _A_wingRootLeadingEdge)  vectorMultiply (_j / _numElements));
    private _b = _A_wingRootLeadingEdge  vectorAdd ((_B_wingTipLeadingEdge  vectorDiff _A_wingRootLeadingEdge)  vectorMultiply ((_j + 1) / _numElements));
    private _c = _D_wingRootTrailingEdge vectorAdd ((_C_wingTipTrailingEdge vectorDiff _D_wingRootTrailingEdge) vectorMultiply ((_j + 1) / _numElements));
    private _d = _D_wingRootTrailingEdge vectorAdd ((_C_wingTipTrailingEdge vectorDiff _D_wingRootTrailingEdge) vectorMultiply (_j / _numElements));

    if (BMKHS_FM_DEBUG) then {
    [_heli, _b, _c,   "white"] call bmkhs_fnc_debugDrawLine;
    [_heli, _d, _a,   "white"] call bmkhs_fnc_debugDrawLine;
    };

    private _f = _d vectorAdd ((_a vectorDiff _d) vectorMultiply (1.0 - _chordLinePos));
    private _g = _c vectorAdd ((_b vectorDiff _c) vectorMultiply (1.0 - _chordLinePos));

    if (BMKHS_FM_DEBUG) then {
    [_heli, _f, _g,   "green"] call bmkhs_fnc_debugDrawLine;
    };

    private _e = _f vectorAdd ((_g vectorDiff _f) vectorMultiply 0.5);

    private _chordLine   = (_a vectorAdd ((_b vectorDiff _a) vectorMultiply 0.5)) vectorDiff (_d vectorAdd ((_c vectorDiff _d) vectorMultiply 0.5));
    private _chordLength = vectorMagnitude _chordLine;
    _chordLine           = vectorNormalized _chordLine;

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e, _e vectorAdd _chordLine, "blue"] call bmkhs_fnc_debugDrawLine;
    };

    private _relativeWind = (_heli getVariable "bmkhs_velModelSpace") vectorMultiply -1.0;

    private _fromAeroCenterToCOM = _e vectorDiff _heliCOM;
    private _angularVel          = (_heli getVariable "bmkhs_angVelModelSpace");

    private _localRelWind = _angularVel vectorCrossProduct _fromAeroCenterToCOM;
    _localRelWind         = _localRelWind vectorMultiply -1.0;
    _relativeWind         = _relativeWind vectorAdd _localRelWind;

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e vectorDiff (vectorNormalized _relativeWind), _e, "red"] call bmkhs_fnc_debugDrawLine;
    };

    private _up = (vectorNormalized (_g vectorDiff _f)) vectorCrossProduct _chordLine;
    _up         = vectorNormalized _up;
    if (_span < 0.0) then {
        _up = _up vectorMultiply -1.0;
    };

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e, _e vectorAdd _up, "white"] call bmkhs_fnc_debugDrawLine;
    };

    private _relWindY = _chordLine vectorDotProduct _relativeWind;
    private _relWindZ = _up vectorDotProduct _relativeWind;
    _relativeWind     = (_chordLine vectorMultiply _relWindY) vectorAdd (_up vectorMultiply _relWindZ);

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e vectorDiff (vectorNormalized _relativeWind), _e, "green"] call bmkhs_fnc_debugDrawLine;
    };

    private _relativeWindNormalized = vectorNormalized _relativeWind;
    private _AoA                    = _chordLine vectorDotProduct (_relativeWindNormalized vectorMultiply -1.0);
    _AoA = [_AoA, -1.0, 1.0] call BIS_fnc_clamp;
    _AoA = acos _AoA;

    private _yAxisDotRelativeWind = _up vectorDotProduct _relativeWindNormalized;
    if (_yAxisDotRelativeWind < 0.0) then {
        _AoA = _AoA * -1.0;
    };

    //Lift coefficient
    private _area  = [_a, _b, _c, _d] call bmkhs_fnc_mathGetArea;
    private _CL    = [_airfoilTable, _AoA] call bmkhs_fnc_mathLinearInterp select 1;
    private _v     = vectorMagnitude _relativeWind;
    private _lift  = _CL * 0.5 * _rho * _area * (_v * _v);

    //Drag coefficient
    private _CD    = [_airfoilTable, _AoA] call bmkhs_fnc_mathLinearInterp select 2;
    private _drag  = _CD * 0.5 * _rho * _area * (_v * _v);

    private _liftVector = _relativeWindNormalized vectorCrossProduct _up;
    _liftVector = _liftVector vectorCrossProduct _relativeWindNormalized;
    _liftVector = vectorNormalized _liftVector;
    _liftVector = _liftVector vectorMultiply (_lift * _deltaTime);

    private _dragVector = _relativeWind;
    _dragVector = (vectorNormalized _dragVector) vectorMultiply -1.0;
    _dragVector = _dragVector vectorMultiply (_drag * _deltaTime);

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e vectorAdd (_liftVector vectorMultiply _debugLineScale), _e, "green"] call bmkhs_fnc_debugDrawLine;
    [_heli, _e vectorAdd (_dragVector vectorMultiply _debugLineScale), _e, "red"]   call bmkhs_fnc_debugDrawLine;
    };

    _heli addForce [_heli vectorModelToWorld _liftVector, _heliCom];
    _heli addForce [_heli vectorModelToWorld _dragVector, _heliCom];

    //This element's OWN force (lift+drag) and moment (F x r about the CoM), as
    //named locals.
    private _force  = _liftVector vectorAdd _dragVector;
    private _moment = _force vectorCrossProduct _fromAeroCenterToCOM;

    _heli addTorque (_heli vectorModelToWorld _moment);
};
/////////////////////////////////////////////////////////////////////////////////////////////
// Debug                /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
if (BMKHS_FM_DEBUG) then {
[_heli, _A_wingRootLeadingEdge,  _B_wingTipLeadingEdge,   "red"]   call bmkhs_fnc_debugDrawLine;
[_heli, _B_wingTipLeadingEdge,   _C_wingTipTrailingEdge,  "white"] call bmkhs_fnc_debugDrawLine;
[_heli, _C_wingTipTrailingEdge,  _D_wingRootTrailingEdge, "white"] call bmkhs_fnc_debugDrawLine;
[_heli, _D_wingRootTrailingEdge, _A_wingRootLeadingEdge,  "white"] call bmkhs_fnc_debugDrawLine;
};
