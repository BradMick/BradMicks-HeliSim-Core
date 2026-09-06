#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\rotor\rotor.hpp"
#include "\bmkhs_helisim\functions\systems\systems.hpp"

params ["_heli", "_rotorIndex", "_pivot", "_rot", "_type", "_dir", "_numBlades", "_numElements", "_mastLength", "_gearRatio", "_flapTimeConst", "_inflowAlpha", "_delta3", "_airfoilTable", "_bladeCutout", "_bladeLength", "_bladeChord", "_bladeTwist", "_bladeMass", "_pitchMin", "_pitchMid", "_pitchMax", "_rollMin", "_rollMid", "_rollMax", "_collMin", "_collMid", "_collMax", "_animSource", "_hitPoint", "_dmgThreshold"];

if (!local _heli) exitWith {};

private _deltaTime = _heli getVariable "bmkhs_deltaTime";

([ _heli
 , _type
 , _pitchMin
 , _pitchMid
 , _pitchMax
 , _rollMin
 , _rollMid
 , _rollMax
 , _collMin
 , _collMid
 , _collMax ] call bmkhs_fnc_rotorControl)
	params [ "_pitchFeather"
		   , "_rollFeather"
		   , "_collFeather"];

private _p = _rot select 0;
private _r = _rot select 1;
private _y = _rot select 2;

private _fVec = [[0.0, 1.0, 0.0], _p, _r, _y] call bmkhs_fnc_mathVectorRotate;
private _rVec = [[1.0, 0.0, 0.0], _p, _r, _y] call bmkhs_fnc_mathVectorRotate;
private _uVec = [[0.0, 0.0, 1.0], _p, _r, _y] call bmkhs_fnc_mathVectorRotate;

private _pos          = _pivot vectorAdd (_uVec vectorMultiply _mastLength);
private _xmsnRpm      = _heli getVariable "bmkhs_xmsnOutputRpm";
private _rpm          = _xmsnRpm / _gearRatio;
private _omega        = if (_rpm == 0.0) then { 0.0 } else { (2.0 * pi) * (_rpm / 60.0) };
private _rotorAzimuth = (_heli animationSourcePhase _animSource) * 360.0 * (if (_dir == CW) then { 1 } else { -1 });

if (_heli getHitPointDamage _hitPoint < _dmgThreshold && currentPilot _heli == player) then {

// Reset accumulators before blade loop
[_heli, "bmkhs_rotorReactionTorque", _rotorIndex, 0.0] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_rotorThrustAccum",    _rotorIndex, 0.0] call bmkhs_fnc_utilSetArrayVariable;
for "_ei" from 0 to (_numElements - 1) do {
    [_heli, "bmkhs_rotorInducedFlowAccum", _rotorIndex, _ei, 0.0] call bmkhs_fnc_utilSetMultiArrayVariable;
};

// Blade flapping dynamics
([ _heli
 , _rotorIndex
 , _type
 , _deltaTime
 , _numBlades
 , _omega
 , _bladeMass
 , _bladeLength
 , _bladeCutout
 , _flapTimeConst
 , _rVec
 , _fVec
 , _uVec ] call bmkhs_fnc_rotorFlapDynamics)
	params ["_beta0", "_a1", "_b1"];

// Tail rotor has no flapping hinge — force all flap coefficients to zero
if (_type == TAIL) then { _beta0 = 0.0; _a1 = 0.0; _b1 = 0.0; };



private _bladeSpacing = 360.0 / _numBlades;
for "_bladeIndex" from 0 to (_numBlades - 1) do {
	private _psi = _rotorAzimuth + (_bladeSpacing * _bladeIndex);

	private _bladeDir = [_rVec, _uVec, _psi] call bmkhs_fnc_mathVectorRotateAroundAxis;
	private _chordDir = [_fVec, _uVec, _psi] call bmkhs_fnc_mathVectorRotateAroundAxis;
	private _dirSign  = if (_dir == CW) then { _chordDir = _chordDir vectorMultiply -1; -1 } else { 1 };

	// Per-blade flap angle from fixed body-frame coefficients.
	// a1 (longitudinal): blade contribution proportional to its forward component
	// b1 (lateral):      blade contribution proportional to its rightward component
	private _flapAngle    = _beta0 + (_a1 * (_bladeDir vectorDotProduct _fVec)) + (_b1 * (_bladeDir vectorDotProduct _rVec));
	private _featherAngle = _dirSign * (_rollFeather * cos(_psi) + _pitchFeather * sin(_psi) + _collFeather - (_flapAngle * _delta3));

	private _tipTwist = _bladeTwist * _dirSign;

	// ROOT INCIDENCE: reference the commanded collective at 75%R (the standard rotor convention -
	// nominal blade twist = 0 at 75%R), not at the root. Linear twist -> the root sits +0.75*twist
	// ABOVE the 75%R value, the tip 0.25*twist below. So shift every feather anchor UP by
	// -_tipTwist*0.75 (= +6.75 deg for the -9 main twist) - the root becomes collective+6.75, 75%R
	// = collective, tip = collective-2.25. Without this the whole productive span sat ~0.75*twist
	// BELOW the commanded pitch, stalling the root before making enough thrust/power.
	// MAIN ROTOR ONLY: the TAIL's collective range (-15..+27) was already calibrated for ROOT-
	// referenced pitch; adding +6 deg over-pitched it and pushed the right-pedal (more anti-torque)
	// side PAST the 15.75 deg airfoil stall = near-zero right-pedal authority at hover. Tail keeps
	// its original root reference. (_tipTwist already carries _dirSign, so this stays sign-consistent.)
	private _rootIncidence      = if (_type == MAIN) then { -_tipTwist * 0.5 } else { 0.0 };

	private _a_rootPos          = _pos       vectorAdd  ([_bladeDir vectorMultiply _bladeCutout,          _chordDir, _flapAngle]              call bmkhs_fnc_mathVectorRotateAroundAxis);
	private _b_tipPos           = _pos       vectorAdd  ([_bladeDir vectorMultiply _bladeLength,          _chordDir, _flapAngle]              call bmkhs_fnc_mathVectorRotateAroundAxis);
	private _c_rootLeadingEdge  = _a_rootPos vectorAdd  ([_chordDir vectorMultiply (_bladeChord * 0.25), _bladeDir, -(_featherAngle + _rootIncidence)]              call bmkhs_fnc_mathVectorRotateAroundAxis);
	private _d_tipLeadingEdge   = _b_tipPos  vectorAdd  ([_chordDir vectorMultiply (_bladeChord * 0.25), _bladeDir, -(_featherAngle + _rootIncidence + _tipTwist)] call bmkhs_fnc_mathVectorRotateAroundAxis);
	private _e_tipTrailingEdge  = _b_tipPos  vectorDiff ([_chordDir vectorMultiply (_bladeChord * 0.75), _bladeDir, -(_featherAngle + _rootIncidence + _tipTwist)] call bmkhs_fnc_mathVectorRotateAroundAxis);
	private _f_rootTrailingEdge = _a_rootPos vectorDiff ([_chordDir vectorMultiply (_bladeChord * 0.75), _bladeDir, -(_featherAngle + _rootIncidence)]              call bmkhs_fnc_mathVectorRotateAroundAxis);

	// Store this blade's azimuth so the decomposition next frame uses the correct position
	[_heli, "bmkhs_rotorBladeAzimuth", _rotorIndex, _bladeIndex, _psi] call bmkhs_fnc_utilSetMultiArrayVariable;

	[ _heli
	, _bladeIndex
	, _rotorIndex
	, _deltaTime
	, _numElements
	, _numBlades
	, _pos
	, _uVec
	, _omega
	, _airfoilTable
	, _bladeCutout
	, _bladeLength
	, _inflowAlpha
	, _a_rootPos
	, _b_tipPos
	, _c_rootLeadingEdge
	, _d_tipLeadingEdge
	, _e_tipTrailingEdge
	, _f_rootTrailingEdge ] call bmkhs_fnc_rotorBlade;

	if (BMKHS_FM_DEBUG) then {
	[_heli, _a_rootPos,          _b_tipPos,            "blue"]  call bmkhs_fnc_debugDrawLine;
	[_heli, _c_rootLeadingEdge,  _d_tipLeadingEdge,    "red"]   call bmkhs_fnc_debugDrawLine;
	[_heli, _d_tipLeadingEdge,   _e_tipTrailingEdge,   "white"] call bmkhs_fnc_debugDrawLine;
	[_heli, _e_tipTrailingEdge,  _f_rootTrailingEdge,  "white"] call bmkhs_fnc_debugDrawLine;
	[_heli, _f_rootTrailingEdge, _c_rootLeadingEdge,   "white"] call bmkhs_fnc_debugDrawLine;
	};
};

// Sum the per-element inflow contributions across blades, lerp toward it once, publish for
// next frame. DO NOT divide by numBlades: fn_rotorBlade accumulates each blade's (T_elem/denom)
// where denom uses the FULL (all-blade) annulus area, so the blade SUM = T_annulus_total/denom -
// exactly the annulus momentum balance vi = sqrt(T_total/(2 rho A)). Dividing by numBlades here
// balanced only ONE blade's thrust against the whole annulus, giving vi = vi_true/sqrt(numBlades)
// = HALF for the 4-blade rotor -> induced power (and thus shaft power/engine torque) came out ~2x
// too low (OGE hover read ~38%/engine instead of ~84%). Thrust was unaffected (independent of vi
// magnitude), which is why only power/torque was wrong.
private _viAccum = (_heli getVariable "bmkhs_rotorInducedFlowAccum") select _rotorIndex;
for "_ei" from 0 to (_numElements - 1) do {
    private _viRawAvg = _viAccum select _ei;
    private _viPrev   = ((_heli getVariable "bmkhs_rotorInducedFlow") select _rotorIndex) select _ei;
    private _viNext   = [_viPrev, _viRawAvg, _inflowAlpha] call BIS_fnc_lerp;
    [_heli, "bmkhs_rotorInducedFlow", _rotorIndex, _ei, _viNext] call bmkhs_fnc_utilSetMultiArrayVariable;
};

// Convert accumulated blade power to rotor shaft torque (Q = P / omega),
// then refer to engine shaft via gear ratio.
private _totalPower     = (_heli getVariable "bmkhs_rotorReactionTorque") select _rotorIndex;
private _reactionTorque = if (_omega > 0.0) then { _totalPower / _omega } else { 0.0 };
private _reqEngTorque   = if (_gearRatio > 0.0) then { _reactionTorque / _gearRatio } else { 0.0 };

//DIAG (power-vs-collective): main rotor only. kW = total rotor power; thrust = accumulated
//lift (N); vi = induced velocity at outer element. At full collective OGE this should climb
//high enough to exceed 2x engine power (~2100 kW) and droop NR. Remove when fixed.
if (_type == MAIN) then {
    private _thr = (_heli getVariable "bmkhs_rotorThrustAccum") select _rotorIndex;
    private _viArr = (_heli getVariable "bmkhs_rotorInducedFlow") select _rotorIndex;
    systemChat format ["ROTOR kW=%1 thrust=%2N vi=[%3] coll=%4",
        (_totalPower/1000) toFixed 0, _thr toFixed 0,
        (_viArr apply {_x toFixed 1}) joinString ",",
        (_heli getVariable "bmkhs_collectiveOutput") toFixed 2];
};

// Smooth rotor torque demand before publishing — BET produces frame-to-frame
// noise that would otherwise couple directly into the transmission and governor.
// 0.1 s time constant keeps physical transients while killing high-freq noise.
private _tqSmoothed     = (_heli getVariable ["bmkhs_reqEngTorque", [0.0, 0.0]]) select _rotorIndex;
private _tqAlpha        = 1.0 - exp (-_deltaTime / 0.1);
_tqSmoothed             = _tqSmoothed + (_reqEngTorque - _tqSmoothed) * _tqAlpha;
[_heli, "bmkhs_reqEngTorque", _rotorIndex, _tqSmoothed, true] call bmkhs_fnc_utilSetArrayVariable;

private _rotorThrust = (_heli getVariable "bmkhs_rotorThrustAccum") select _rotorIndex;
[_heli, "bmkhs_rtrThrust", _rotorIndex, _rotorThrust, true] call bmkhs_fnc_utilSetArrayVariable;

private _Icm  = (1.0 / 3.0) * _bladeMass * (_bladeLength * _bladeLength);
private _Iy   = (1.0 / 12.0) * _bladeMass * (_bladeChord * _bladeChord);
private _Itot = _Icm;
private _Jtot = (_Iy + _Itot) * _numBlades;
[_heli, "bmkhs_rtrMoi", _rotorIndex, _Jtot, true] call bmkhs_fnc_utilSetArrayVariable;

// Apply rotor drag torque reaction to fuselage — main rotor only.
// Use the smoothed value so BET noise doesn't shake the airframe.
private _reactionMoment = [0,0,0];
if (_type == MAIN) then {
    private _torqueSign = [-1.0, 1.0] select (_dir == CW);
    //BET TORQUE tuning scalar (yaw knob) - multiplies ONLY this fuselage reaction couple, NOT the
    //engine load (_totalPower stays physics-true). Airspeed-banded; 1.0 = pure physics. Lets the
    private _velBet   = vectorMagnitude [(_heli getVariable "bmkhs_velModelSpace" select 0), (_heli getVariable "bmkhs_velModelSpace" select 1)];
    private _torqTbl  = [];
    private _torqueScale = if (_torqTbl isEqualTo []) then { 1.0 } else { [_torqTbl, _velBet] call bmkhs_fnc_mathLinearInterp select 1 };
    _reactionMoment = _uVec vectorMultiply (_tqSmoothed * _gearRatio * _torqueSign * _deltaTime * _torqueScale);
    _heli addTorque (_heli vectorModelToWorld _reactionMoment);
};

}; // end damage check

if (BMKHS_FM_DEBUG) then {
[_heli, _pivot, _pos,                 "white"] call bmkhs_fnc_debugDrawLine;
[_heli, _pos,   _pos vectorAdd _fVec, "green"] call bmkhs_fnc_debugDrawLine;
[_heli, _pos,   _pos vectorAdd _rVec, "red"]   call bmkhs_fnc_debugDrawLine;
[_heli, _pos,   _pos vectorAdd _uVec, "blue"]  call bmkhs_fnc_debugDrawLine;
};
