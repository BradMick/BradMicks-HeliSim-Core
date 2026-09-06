#include "\bmkhs_helisim\functions\core\core.hpp"
params ["_heli", "_bladeIndex", "_rotorIndex", "_deltaTime", "_numElements", "_numBlades", "_pos", "_uVec", "_omega", "_airfoilTable", "_bladeCutout", "_bladeLength", "_inflowAlpha", "_a_rootPos", "_b_tipPos", "_c_rootLeadingEdge", "_d_tipLeadingEdge", "_e_tipTrailingEdge", "_f_rootTrailingEdge"];

private _rho              = _heli getVariable "bmkhs_rho";
private _dissymetryOfLift = false;
private _totalFlapMoment  = 0.0;
private _totalPower       = (_heli getVariable "bmkhs_rotorReactionTorque") select _rotorIndex;

//BET FORCE-OUTPUT TUNING SCALARS (physics-derived forces, multiplied at the output so the master
//(main uses betMainLiftTable, tail uses betTailLiftTable). A SEPARATE _torqueScale (-> reaction
//couple, applied to _totalPower below) lets thrust and yaw tune independently. Both default 1.0
//Looked up once here per blade from the aircraft's fwd speed.
//LIFT scalar only (thrust). The TORQUE scalar is applied to the reaction couple in fn_rotor.
private _velBet   = vectorMagnitude [(_heli getVariable "bmkhs_velModelSpace" select 0), (_heli getVariable "bmkhs_velModelSpace" select 1)];
private _isTail   = _rotorIndex == 1;
private _liftTbl  = _heli getVariable [(["bmkhs_betMainLiftTable", "bmkhs_betTailLiftTable"] select _isTail), []];
private _bladeScale = if (_liftTbl isEqualTo []) then { 1.0 } else { [_liftTbl, _velBet] call bmkhs_fnc_mathLinearInterp select 1 };
[_heli, "bmkhs_rotorFlapMoment", _rotorIndex, _bladeIndex, 0.0] call bmkhs_fnc_utilSetMultiArrayVariable;

for "_i" from 0 to (_numElements - 1) do {
	private _spanInboard  =  _i      / _numElements;
	private _spanOutboard = (_i + 1) / _numElements;

	private _a = _a_rootPos          vectorAdd ((_b_tipPos          vectorDiff _a_rootPos)          vectorMultiply _spanInboard);
	private _b = _a_rootPos          vectorAdd ((_b_tipPos          vectorDiff _a_rootPos)          vectorMultiply _spanOutboard);

	private _c = _c_rootLeadingEdge  vectorAdd ((_d_tipLeadingEdge  vectorDiff _c_rootLeadingEdge)  vectorMultiply _spanInboard);
	private _d = _c_rootLeadingEdge  vectorAdd ((_d_tipLeadingEdge  vectorDiff _c_rootLeadingEdge)  vectorMultiply _spanOutboard);
	private _e = _f_rootTrailingEdge vectorAdd ((_e_tipTrailingEdge vectorDiff _f_rootTrailingEdge) vectorMultiply _spanOutboard);
	private _f = _f_rootTrailingEdge vectorAdd ((_e_tipTrailingEdge vectorDiff _f_rootTrailingEdge) vectorMultiply _spanInboard);

	private _liftFraction = [0.5, 0.75] select (_numElements == 1);
	private _liftPos      = _a vectorAdd ((_b vectorDiff _a) vectorMultiply _liftFraction);

	private _chordLine = (_c vectorAdd ((_d vectorDiff _c) vectorMultiply 0.5)) vectorDiff (_f vectorAdd ((_e vectorDiff _f) vectorMultiply 0.5));
	_chordLine         = vectorNormalized _chordLine;

	if (BMKHS_FM_DEBUG) then {
	[_heli, _liftPos, _liftPos vectorAdd _chordLine, "white"] call bmkhs_fnc_debugDrawLine;
	};

	// Induced inflow from previous frame — breaks the thrust/inflow circular dependency
	private _vi = ((_heli getVariable "bmkhs_rotorInducedFlow") select _rotorIndex) select _i;

	private _velModel = _heli getVariable "bmkhs_velModelSpace";
	private _relWind  = if (_dissymetryOfLift) then {
		_velModel vectorMultiply -1.0
	} else {
		[0.0, 0.0, -(_velModel select 2)]
	};

	private _spanDir    = vectorNormalized (_b vectorDiff _a);

	private _radialVec  = (_liftPos vectorDiff _pos) vectorDiff (_uVec vectorMultiply ((_liftPos vectorDiff _pos) vectorDotProduct _uVec));
	private _r          = vectorMagnitude _radialVec;
	private _rotRelWind = (_uVec vectorCrossProduct _spanDir) vectorMultiply (_r * _omega * -1.0);

	private _up = vectorNormalized (_spanDir vectorCrossProduct _chordLine);
	if ((_up vectorDotProduct _uVec) < 0.0) then { _up = _up vectorMultiply -1.0; };

	// AIRFRAME ANGULAR-RATE contribution to the blade's local wind = the velocity this blade
	// element has because the whole AIRFRAME is rotating (roll/pitch/yaw rate). v = omega_body
	// x r, where r = element position relative to the CoM. This is the physics that gives a
	// rotor its ROLL/PITCH RATE DAMPING: when the airframe rolls, the down-going blade sees
	// MORE upflow and the up-going blade LESS, so lift shifts asymmetrically and the disc flaps
	// to OPPOSE the roll. Omitting it (as before) left the rotor with ZERO rate damping - a ~1.7
	// deg disc tilt rolled unchecked to tens of deg/s. All model space (angVelModelSpace and the
	// positions), so the cross product is frame-consistent. This is real, pure-mechanical damping.
	private _bodyRate     = _heli getVariable ["bmkhs_angVelModelSpace", [0,0,0]];
	//Scalar on the rate-damping term. 1.0 = bare physics. Arma derives the airframe's roll/pitch
	//INERTIA from the p3d mass geometry and it appears too LOW (a ~1deg disc tilt over-rolls), so
	//the bare-physics rotor damping alone doesn't fully tame the pure-mechanical response. Scaling
	//this >1 adds the extra rate damping that the missing inertia would otherwise provide - the
	//honest knob for "compensate for Arma's under-modeled inertia". Tunable; dial for the right
	//pure-mechanical feel (no SAS). Held in a variable so it can be adjusted without a reload.
	private _rateDampScalar = _heli getVariable ["bmkhs_rotorRateDampScalar", 1.0];
	private _bodyRateWind = (_bodyRate vectorCrossProduct (_liftPos vectorDiff (getCenterOfMass _heli))) vectorMultiply _rateDampScalar;

	private _localWind    = _relWind vectorAdd _rotRelWind vectorAdd _bodyRateWind;

	// Freestream axial and lateral for Glauert inflow — freestream only, no rotation.
	private _vc          = _uVec vectorDotProduct _relWind;
	private _localAxial  = _uVec vectorMultiply _vc;
	private _localLatVel = vectorMagnitude (_relWind vectorDiff _localAxial);

	// Apply induced inflow downward through disc (opposes lift direction)
	private _inducedWind = _uVec vectorMultiply (-_vi);
	_relWind = _localWind vectorAdd _inducedWind;

	if (BMKHS_FM_DEBUG) then {
	[_heli, _liftPos vectorDiff (vectorNormalized _relWind), _liftPos, "red"]   call bmkhs_fnc_debugDrawLine;
	[_heli, _liftPos, _liftPos vectorAdd _up,                          "white"] call bmkhs_fnc_debugDrawLine;
	};

	private _relWindY = _chordLine vectorDotProduct _relWind;
	private _relWindZ = _up        vectorDotProduct _relWind;
	_relWind          = (_chordLine vectorMultiply _relWindY) vectorAdd (_up vectorMultiply _relWindZ);

	if (BMKHS_FM_DEBUG) then {
	[_heli, _liftPos vectorDiff (vectorNormalized _relWind), _liftPos, "green"] call bmkhs_fnc_debugDrawLine;
	};

	private _relWindNormalized = vectorNormalized _relWind;
	private _AoA               = _chordLine vectorDotProduct (_relWindNormalized vectorMultiply -1.0);
	_AoA = [_AoA, -1.0, 1.0] call BIS_fnc_clamp;
	_AoA = acos _AoA;

	if ((_up vectorDotProduct _relWindNormalized) < 0.0) then { _AoA = _AoA * -1.0; };

	private _area = [_c, _d, _e, _f] call bmkhs_fnc_mathGetArea;
	private _v    = vectorMagnitude _relWind;
	private _q    = 0.5 * _rho * _area * (_v * _v);

	private _CL = [_airfoilTable, _AoA] call bmkhs_fnc_mathLinearInterp select 1;
	private _CD = [_airfoilTable, _AoA] call bmkhs_fnc_mathLinearInterp select 2;

	private _lift = _CL * _q;
	private _drag = (_CD min 0.15) * _q;

	// Update induced inflow velocity for this element via Glauert inflow model.
	// Accounts for axial (climb/descent) and lateral (forward flight) freestream,
	// unlike the hover-only sqrt(T/2rhoA) formula.
	private _r_in  = _bladeCutout + (_spanInboard  * (_bladeLength - _bladeCutout));
	private _r_out = _bladeCutout + (_spanOutboard  * (_bladeLength - _bladeCutout));
	private _annularArea = pi * ((_r_out * _r_out) - (_r_in * _r_in));
	// Compute raw inflow for this element and accumulate across blades.
	// The lerp toward viRaw is applied once in fn_rotor after averaging, not here.
	private _viRaw = if ((_annularArea > 0.0) && (_rho > 0.0)) then {
		private _T_elem = _lift * _bladeScale;
		private _denom  = 2.0 * _rho * _annularArea * (sqrt (((_vc + _vi) * (_vc + _vi)) + (_localLatVel * _localLatVel)));
		private _raw    = if (_denom > 0.0) then { _T_elem / _denom } else { 0.0 };
		[_raw, -25.0, 25.0] call BIS_fnc_clamp
	} else { 0.0 };
	private _viAccum = ((_heli getVariable "bmkhs_rotorInducedFlowAccum") select _rotorIndex) select _i;
	[_heli, "bmkhs_rotorInducedFlowAccum", _rotorIndex, _i, (_viAccum + _viRaw)] call bmkhs_fnc_utilSetMultiArrayVariable;

	_totalFlapMoment = _totalFlapMoment + (_lift * _r);

	private _liftVector = _relWindNormalized vectorCrossProduct _up;
	_liftVector         = _liftVector vectorCrossProduct _relWindNormalized;
	private _liftDir    = vectorNormalized _liftVector;
	private _dragDir    = vectorNormalized _relWind;

	// Power-based rotor load:
	// Induced power = thrust element * induced inflow velocity (energy spent accelerating air downward)
	// Profile power = drag force * tangential blade speed (energy spent overcoming blade drag)
	// Both oppose rotation; sum gives total shaft power consumed by this element.
	private _vTangential  = _r * _omega;
	private _P_induced    = _lift * _vi;
	private _P_profile    = _drag * _vTangential;
	//Power stays PHYSICS-TRUE here (drives engine load/governor). The TORQUE tuning scalar is
	//applied ONLY to the fuselage reaction couple in fn_rotor (the yaw knob), NOT to engine load.
	_totalPower = _totalPower + (_P_induced + _P_profile);


	private _thrustAccum = (_heli getVariable "bmkhs_rotorThrustAccum") select _rotorIndex;
	[_heli, "bmkhs_rotorThrustAccum", _rotorIndex, (_thrustAccum + (_lift * _bladeScale))] call bmkhs_fnc_utilSetArrayVariable;

	_liftVector = _liftDir vectorMultiply (_lift * _deltaTime * _bladeScale);

	private _dragVector = _dragDir vectorMultiply (_drag * _deltaTime * _bladeScale * -1.0);

	_heli addForce [_heli vectorModelToWorld _liftVector, _liftPos];
	_heli addForce [_heli vectorModelToWorld _dragVector, _liftPos];

	if (BMKHS_FM_DEBUG) then {
	[_heli, _liftPos, _liftPos vectorAdd (_liftVector vectorMultiply (1.0 / 30.0)), "green"] call bmkhs_fnc_debugDrawLine;
	[_heli, _liftPos, _liftPos vectorAdd (_dragVector vectorMultiply (1.0 / 30.0)), "red"]   call bmkhs_fnc_debugDrawLine;
	[_heli, _c, _f, "red"] call bmkhs_fnc_debugDrawLine;
	[_heli, _d, _e, "red"] call bmkhs_fnc_debugDrawLine;
	};
};

[_heli, "bmkhs_rotorFlapMoment", _rotorIndex, _bladeIndex, _totalFlapMoment] call bmkhs_fnc_utilSetMultiArrayVariable;

[_heli, "bmkhs_rotorReactionTorque", _rotorIndex, _totalPower] call bmkhs_fnc_utilSetArrayVariable;
