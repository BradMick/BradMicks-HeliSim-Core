/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_simpleRotorMain

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

params ["_heli"];

if (!local _heli) exitWith {};

private _deltaTime              = _heli getVariable "bmkhs_deltaTime";
private _heliCom                = getCenterOfMass _heli;

private _altitude               = _heli getVariable "bmkhs_PA";
private _temperature            = _heli getVariable "bmkhs_FAT";
private _dryAirDensity          = _heli getVariable "bmkhs_RHO";

private _attHoldCycPitchOut     = _heli getVariable "bmkhs_fmcAttHoldCycPitchOut";
private _collToPitchOut         = _heli getVariable "bmkhs_fmcCollectiveToPitch";
private _yawToPitchOut          = _heli getVariable "bmkhs_fmcYawToPitch";
private _sasPitchOut            = _heli getVariable "bmkhs_fmcSasPitchOut";
private _fmcPitchOut            = _attHoldCycPitchOut + _sasPitchOut + _collToPitchOut + _yawToPitchOut;
//_fmcPitchOut                    = [_fmcPitchOut, -0.15, 0.15] call BIS_fnc_clamp;

private _attHoldCycRollOut      = _heli getVariable "bmkhs_fmcAttHoldCycRollOut";
private _sasRollOut             = _heli getVariable "bmkhs_fmcSasRollOut";
private _collToRollOut          = _heli getVariable "bmkhs_fmcCollectiveToRoll";
private _yawToRollOut           = _heli getVariable "bmkhs_fmcYawToRoll";
private _fmcRollOut             = _attHoldCycRollOut + _sasRollOut + _collToRollOut + _yawToRollOut;
//_fmcRollOut                     = [_fmcRollOut, -0.15, 0.15] call BIS_fnc_clamp;

private _collectiveOut          = _heli getVariable "bmkhs_collectiveOutput";
private _altHoldCollOut         = _heli getVariable "bmkhs_fmcAltHoldCollOut";
private _fmcCollOut             = _collectiveOut + _altHoldCollOut;
//private _isAutorotating         = _heli getVariable "bmkhs_isAutorotating";

private _rtrPos                 = _heli getVariable "bmkhs_mainRtrPos";
private _rtrHeightAGL           = _heli getVariable "bmkhs_mainRtrHeightAgl";
private _rtrDesignRPM           = _heli getVariable "bmkhs_mainRtrDesignRpm";
private _rtrRPMTrimVal          = _heli getVariable "bmkhs_mainRtrRpmTrimVal";
private _rtrGearRatio           = _heli getVariable "bmkhs_mainRotorGearRatio";
private _rtrNumBlades           = _heli getVariable "bmkhs_mainRtrNumBlades";

private _bladeRadius            = _heli getVariable "bmkhs_mainRtrBladeRadius";
private _bladeChord             = _heli getVariable "bmkhs_mainRtrBladeChord";
private _bladeMass              = _heli getVariable "bmkhs_mainRtrBladeMass";
private _bladeHingeOffset       = _heli getVariable "bmkhs_mainRtrBladeHingeOff";
private _bladePitch_min         = _heli getVariable "bmkhs_mainRtrBladePitchMin";
private _bladePitch_max         = _heli getVariable "bmkhs_mainRtrBladePitchMax";

private _rtrGndEffTable =
[
 [ 6804, 0.189]
,[ 7711, 0.170]
,[ 8165, 0.187]
,[ 8618, 0.310]
,[ 9525, 0.509]
];
private _rtrThrustScalarTable_min =
[
 [    0, 0.032]
,[ 2000, 0.052]
,[ 4000, 0.037]
,[ 6000, 0.041]
,[ 8000, 0.045]
];
private _rtrThrustScalarTable_max =
[
 [    0, 1.168]
,[ 2000, 1.422]
,[ 4000, 1.745]
,[ 6000, 2.132]
,[ 8000, 2.561]
];
private _rtrTipLossTable =
[
 [ 6804, 1.108]
,[ 7711, 1.050]
,[ 8165, 1.000]
,[ 8618, 0.958]
,[ 9525, 0.890]
];
private _velocityThrustExponentTable =
[
 [ 0.00, 0.000]
,[10.29, 0.209]
,[20.58, 0.558]
,[36.01, 0.606]
,[46.30, 0.497]
,[51.44, 0.474]
,[61.73, 0.392]
,[66.88, 0.397]
,[72.02, 0.428]
];

private _vrsScalarExponent      = 0.3;
//Main rotor yaw-torque scalar vs airspeed. This source array is the single source
private _rtrTorqueScalarTable =
[
 [ 0.00, 1.00]   // fixed hover-tuned reference, carried across all bands
,[10.29, 1.00]
,[20.58, 1.00]
,[36.01, 1.00]
,[46.30, 1.00]
,[51.44, 1.00]
,[61.73, 1.00]
,[66.88, 1.00]
,[72.02, 1.00]
];
//Published for the force overlay/dump readout only - the array above is the source of truth.
_heli setVariable ["bmkhs_rtrTqTable", _rtrTorqueScalarTable];
//Main-rotor thrust scalar vs airspeed (distinct from the TAIL authority table).
//This source array is the single source of truth for the default: publish it into
private _rtrThrustScalarTable =
[
 [ 0.00, 1.164]   // 0-90 kt tuned; 100-140 extrapolated from that trend
,[10.29, 1.059]
,[20.58, 0.953]
,[36.01, 0.848]
,[46.30, 0.889]
,[51.44, 0.890]
,[61.73, 0.947]
,[66.88, 0.990]
,[72.02, 1.043]
];
//Published for the force overlay/dump readout only - the array above is the source of truth.
_heli setVariable ["bmkhs_mainThrustTable", _rtrThrustScalarTable];

private _isOnGnd                = [_heli] call bmkhs_fnc_stateOnGround;

private _pitchTorqueScalar      = 2.50 * 1.3;
private _rollTorqueScalar       = 0.75 * 1.3;

private _baseThrust             = _heli getVariable "bmkhs_mainRtrBaseThrust";

//Moment of inertia
private _Icm  = (1.0 / 3.0) * _bladeMass * (_bladeRadius * _bladeRadius);
private _Iy   = (1.0 / 12.0) * _bladeMass * (_bladeChord * _bladeChord);
private _md2  = _bladeMass * (_bladeHingeOffset * _bladeHingeOffset);
private _Itot = _Icm + _md2;
private _Jtot = (_Iy + _Itot) * _rtrNumBlades;
[_heli, "bmkhs_rtrMoi", 0, _Jtot, true] call bmkhs_fnc_utilSetArrayVariable;

//Thrust produced
private _bladePitch_cur                = _bladePitch_min + (_bladePitch_max - _bladePitch_min) * _fmcCollOut;
private _rtrThrustScalar_min           = [_rtrThrustScalarTable_min, _altitude] call bmkhs_fnc_mathLinearInterp select 1;
private _bladePitchInducedThrustScalar = _rtrThrustScalar_min + ((1 - _rtrThrustScalar_min) / _bladePitch_max)  * _bladePitch_cur;
//(_heli getVariable "bmkhs_engPctNP")
//    params ["_eng1PctNP", "_eng2PctNp"];
private _inputRPM                      = (_heli getVariable "bmkhs_xmsnOutputRpm") / 20900;//_eng1PctNP max _eng2PctNp;
private _inputRpmPct                   = [_inputRpm / _rtrRPMTrimVal, 0.0, 1.0] call BIS_fnc_clamp;

//Rotor induced thrust as a function of RPM
private _rtrThrustScalar_max       = [_rtrThrustScalarTable_max, _altitude] call bmkhs_fnc_mathLinearInterp select 1;
private _rtrRPMInducedThrustScalar = _inputRpmPct * _rtrThrustScalar_max;

//Thrust scalar as a result of altitude
private _airDensityThrustScalar    = _dryAirDensity / ISA_STD_DAY_AIR_DENSITY;

//Additional thrust gained from increasing forward airspeed
private _velX                      = _heli getVariable "bmkhs_velModelSpace" select 0;
private _velY                      = _heli getVariable "bmkhs_velModelSpace" select 1;
private _velXY                     = vectorMagnitude [_velX, _velY] min VEL_VNE;
if ([_velXY] call bmkhs_fnc_mathIsNAN || [_velXY] call bmkhs_fnc_mathIsINF) then { _velXY = 0.0; };
if (_isOnGnd) then { _velXY = 0.0; };
private _velocityThrustExponent    = [_velocityThrustExponentTable, _velXY] call bmkhs_fnc_mathLinearInterp select 1;
//systemChat format ["_velocityThrustExponent = %1 -- _fmcCollOut = %2", _velocityThrustExponent toFixed 3, _fmcCollOut toFixed 3];
private _airspeedVelocityScalar    = (1 + (_velXY / VEL_VBE)) ^ (_velocityThrustExponent);

//Induced flow handler
private _velZ                      = _heli getVariable "bmkhs_velModelSpace" select 2;
private _inducedVelocityScalar     = 1.0;
private _vrsVelMin                 = _heli getVariable "bmkhs_vrsVelocityMin";
private _vrsVelMax                 = _heli getVariable "bmkhs_vrsVelocityMax";
private _vrsVel = 0.0;
if (_heli getVariable "bmkhs_rtrThrust" select 0 > 0.0) then {
    _vrsVel = linearConversion[0.0, VEL_ETL, _velXY, _vrsVelMax, VEL_VRS, true];
};
if (_velZ < -_vrsVelMin && _velXY < VEL_ETL) then {
    private _vrsScalar = if(_velZ == 0.0) then { 0.0; } else { abs(_vrsVelMin / _velZ)^_vrsScalarExponent; };
    _inducedVelocityScalar   = if(_vrsVelMax == 0.0) then { 1.0; } else { (1 - (_velZ / _vrsVelMax)) * _vrsScalar; };
} else {

    private _denom = linearConversion[-7.62, -19.30, _velZ, _vrsVel, 3.81, true];
    _inducedVelocityScalar = if(_vrsVel == 0.0) then { 1.0; } else { 1 - (_velZ / _denom); };
};

//Finally, multiply all the scalars above to arrive at the final thrust scalar
private _rtrThrustScalar           = _bladePitchInducedThrustScalar * _rtrRPMInducedThrustScalar * _airDensityThrustScalar * _airspeedVelocityScalar * _inducedVelocityScalar;
private _rtrThrust                 = _baseThrust * _rtrThrustScalar;
private _rtrOmega                  = (2.0 * pi) * ((_rtrDesignRPM * _inputRPM) / 60);
private _bladeTipVel               = _rtrOmega * _bladeRadius;
private _rtrArea                   = pi * _bladeRadius^2;

//Calculate the required rotor power
private _profile_min = 0.180;
private _profile_max = 0.407;

private _velXYNoWind = vectorMagnitude [_velX, _velY] min VEL_VNE;
if (_isOnGnd) then { _velXYNoWind = 0.0; };

private _profilePowerCollectiveScalar = [_fmcCollOut / _profile_max, 0.0, 1.0] call BIS_fnc_clamp;
private _profile_cur                  = (_profile_min + (((_profile_max * _profilePowerCollectiveScalar) - _profile_min) / VEL_VNE) * _velXYNoWind);

private _inducedPowerVelocityScalarTable =
[
 [ 0.00, 1.202]
,[10.29, 0.970]
,[20.58, 0.951]
,[36.01, 0.923]
,[46.30, 0.904]
,[51.44, 0.895]
,[61.73, 0.876]
,[66.88, 0.867]
,[69.96, 0.861]
,[72.02, 0.899]
];
private _inducedPowerVelocityScalar = ([_inducedPowerVelocityScalarTable, _velXYNoWind] call bmkhs_fnc_mathLinearInterp) select 1;
_inducedPowerVelocityScalar         = _inducedPowerVelocityScalar * _fmcCollOut;

private _inducedPowerCollectiveCorrectionTable =
[
 [ 0.00, 0.649]
,[10.29, 0.591]
,[20.58, 0.602]
,[36.01, 0.760]
,[46.30, 0.860]
,[51.44, 0.871]
,[61.73, 0.860]
,[66.88, 0.827]
,[69.96, 0.799]
,[72.02, 0.840]
];
private _collectiveTorqueCorrectionTable =
[
 [0.000, 0.000]
,[0.050, 0.760]
,[0.225, 0.798]
,[0.250, 1.000]
,[0.850, 1.000]
,[1.000, 1.500]
];
private _autorotationTorqueTable =
[
 [-20.32,-100.0] //4000fpm
,[-15.24, -50.0] //3000fpm
,[-12.70, -25.0] //2500fpm
,[-10.16, -10.0] //2000fpm
,[ -7.62,  -5.0] //1500fpm
,[  0.00,   0.0]
];
private _inducedPowerCollectiveCorrection = ([_inducedPowerCollectiveCorrectionTable, _velXYNoWind] call bmkhs_fnc_mathLinearInterp) select 1;
private _induced_val                      = [_fmcCollOut / _inducedPowerCollectiveCorrection, 0.0, 2.0] call BIS_fnc_clamp;
private _induced_cur                      = _inducedPowerVelocityScalar * _induced_val;
private _collectiveTorqueCorrection       = ([_collectiveTorqueCorrectionTable, _fmcCollOut] call bmkhs_fnc_mathLinearInterp) select 1;
_collectiveTorqueCorrection               = linearConversion[0.0, VEL_ETL, _velXYNoWind, 1.0, _collectiveTorqueCorrection, true];
private _power_val                        = [(_profile_cur + _induced_cur) * _collectiveTorqueCorrection, -1.0, 2.50] call BIS_fnc_clamp;
private _power_req                        = _power_val * 2133.0;
private _torque_req                       = (_power_req / 0.001) / 0.105 / 21109;
private _autorotationTorque               = ([_autorotationTorqueTable, _velZ] call bmkhs_fnc_mathLinearInterp) select 1;
_torque_req                               = (_torque_req * _inputRpmPct) + _autorotationTorque;

//systemChat format ["_velZ = %1 -- _autorotationTorque = %2", _velZ * 196.85, _autorotationTorque];

private _rtrTorque   = _torque_req * _rtrGearRatio;
[_heli, "bmkhs_reqEngTorque", 0, _torque_req, true] call bmkhs_fnc_utilSetArrayVariable;

private _axisX = [1.0, 0.0, 0.0];
private _axisY = [0.0, 1.0, 0.0];
private _axisZ = [0.0, 0.0, 1.0];

//Ground Effect
private _heightAGL     = _rtrHeightAGL  + (ASLToAGL getPosASL _heli # 2);
private _rtrDiam       = _bladeRadius * 2;

private _rtrGndEffScalar = ([_rtrGndEffTable, _heli getVariable "bmkhs_GWT"] call bmkhs_fnc_mathLinearInterp) select 1;
_rtrGndEffScalar         = _rtrGndEffScalar * (1.0);
private _gndEffScalar  = (1 - (_heightAGL / _rtrDiam)) * _rtrGndEffScalar;
_gndEffScalar          = [_gndEffScalar, 0.0, 1.0] call BIS_fnc_clamp;
private _gndEffThrust  = _rtrThrust * _gndEffScalar;

private _eng1TQ        = _heli getVariable "bmkhs_engPctTQ" select 0;
private _eng2TQ        = _heli getVariable "bmkhs_engPctTQ" select 1;
private _engPctTQ      = _eng1TQ max _eng2TQ;
private _isSingleEng   = _heli getVariable "bmkhs_isSingleEng";

private _cruiseTqTable =
[
 [ 0.00, 0.94]
,[ 2.57, 0.93]
,[ 5.14, 0.90]
,[ 7.72, 0.87]
,[10.29, 0.82]
,[12.86, 0.78]
,[20.58, 0.62]
,[25.72, 0.54]
,[30.87, 0.50]
,[36.01, 0.49]
,[41.16, 0.50]
,[46.30, 0.52]
,[51.44, 0.56]
,[56.59, 0.64]
,[61.73, 0.72]
,[66.88, 0.85]
,[72.02, 1.01]
,[77.17, 1.18]
];
private _cruiseTq   = [_cruiseTqTable, _velXY] call bmkhs_fnc_mathLinearInterp select 1;
if (_isSingleEng) then {
    _cruiseTq = _cruiseTq * 2.0;
};
private _tqChange   = _engPctTq - _cruiseTq;
_tqChange           = [_tqChange, 0.0, 0.8] call BIS_fnc_clamp;
private _tqRoCTable =
[
 [0.0, 0.000]   //0fpm
,[0.1, 0.078]   //400fpm
,[0.2, 0.151]   //800fpm
,[0.3, 0.224]   //1200fpm
,[0.4, 0.297]   //1600fpm
,[0.5, 0.369]   //2000fpm
,[0.6, 0.457]   //2400fpm
,[0.7, 0.517]   //2800fpm
,[0.8, 0.587]   //3200fpm
];
private _RoCScalar       = [_tqRoCTable, _tqChange] call bmkhs_fnc_mathLinearInterp select 1;
private _climbThrust     = _baseThrust * _RoCScalar;
private _tipLossScalar   = [_rtrTipLossTable, _heli getVariable "bmkhs_GWT"] call bmkhs_fnc_mathLinearInterp select 1;
private _totThrust       = (_rtrThrust + _gndEffThrust + _climbThrust) * _tipLossScalar;
if ([_totThrust] call bmkhs_fnc_mathIsNAN || [_totThrust] call bmkhs_fnc_mathIsINF) then { _totThrust = 0.0; };
[_heli, "bmkhs_rtrThrust", 0, _totThrust, true] call bmkhs_fnc_utilSetArrayVariable;
//Main-thrust scalar. In forward flight: the airspeed-banded table. In HOVER (low
//forward speed): blend the IGE and OGE hover thrust values by AGL height (IGE at
//5 ft, OGE at 80 ft) so ground effect gets its own tuned thrust at each height.
private _rtrThrustScalar = [_rtrThrustScalarTable, _velXY] call bmkhs_fnc_mathLinearInterp select 1;
if (_velXY < 2.6) then {   // < ~5 kt = hover
    private _igeThr = 1.0;
    private _ogeThr = 1.0;
    private _aglFt  = (ASLToAGL getPosASL _heli # 2) * 3.28084;
    //Linear blend: 5 ft -> IGE value, 80 ft -> OGE value, clamped outside.
    private _f      = [(_aglFt - 5.0) / 75.0, 0.0, 1.0] call BIS_fnc_clamp;
    _rtrThrustScalar = _igeThr + (_f * (_ogeThr - _igeThr));
};
private _thrustZ         = _axisZ vectorMultiply (_totThrust * _rtrThrustScalar * _deltaTime);
private _inducedVelocity = sqrt(_totThrust / (2 * _dryAirDensity * _rtrArea));
if ([_inducedVelocity] call bmkhs_fnc_mathIsNAN || [_inducedVelocity] call bmkhs_fnc_mathIsINF) then { _inducedVelocity = 0.0; };
_heli setVariable ["bmkhs_vrsVelocityMin", _inducedVelocity * 0.23];
_heli setVariable ["bmkhs_vrsVelocityMax", _inducedVelocity * 1.25];
/////////////////////////////////////////////////////////////////////////////////////////////
// Retreating Blade Stall ///////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
private _retBladeStallSpeedTable =
[
 [  0.00, 0.00, 0.00]   //0ktas
,[ 77.16, 0.00, 0.00]   //150ktas
,[ 82.30, 0.04, 0.20]   //160ktas
,[ 87.45, 0.12, 0.50]   //170ktas
,[ 92.59, 0.25, 0.90]   //180ktas
,[ 97.74, 0.50, 1.40]   //190ktas
,[100.31, 0.70, 1.70]   //195ktas
,[102.88, 1.00, 2.00]   //200ktas
];

private _retBladeStallCollTable =
[
 [0.0, [_retBladeStallSpeedTable, _velXY] call bmkhs_fnc_mathLinearInterp select 1]
,[0.7, [_retBladeStallSpeedTable, _velXY] call bmkhs_fnc_mathLinearInterp select 2]
];

private _retBladeStallInput = [_retBladeStallCollTable, _fmcCollOut] call bmkhs_fnc_mathLinearInterp select 1;
private _retBladeStallVal   = linearConversion [77.16, 102.88, _velXY, 1.0, 0.0, true];
/////////////////////////////////////////////////////////////////////////////////////////////
// Pitch Torque         /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
private _cyclicFwdAft     = _heli getVariable "bmkhs_cyclicFwdAft";
private _cyclicFwdAftTrim = 0.0;
_cyclicFwdAftTrim         = _heli getVariable "bmkhs_forceTrimPosPitch";

private _pitchTorque      = linearConversion [0.0, 1.0, _inputRpmPct, 0.0, 100000 * _pitchTorqueScalar * _deltaTime, true];
private _pitchInput       = ([_cyclicFwdAft, _cyclicFwdAftTrim] call bmkhs_fnc_inputGetInterp) + _fmcPitchOut;
_pitchInput               = [_pitchInput, -1.0, 1.0] call BIS_fnc_clamp;

private _momentX          = _pitchTorque * _pitchInput;
/////////////////////////////////////////////////////////////////////////////////////////////
// Roll Torque          /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
private _cyclicLeftRight     = _heli getVariable "bmkhs_cyclicLeftRight";
private _cyclicLeftRightTrim = 0.0;
_cyclicLeftRightTrim         = _heli getVariable "bmkhs_forceTrimPosRoll";

private _rollTorque          = linearConversion [0.0, 1.0, _inputRpmPct, 0.0, 100000 * _rollTorqueScalar * _deltaTime, true];
private _rollInput           = ([_cyclicLeftRight, _cyclicLeftRightTrim] call bmkhs_fnc_inputGetInterp) + _fmcRollOut;
_rollInput                   = [_rollInput, -1.0, 1.0] call BIS_fnc_clamp;

private _momentY             = _rollTorque * _rollInput;
//systemChat format ["_pitchInput = %1 -- _rollInput = %2", _pitchInput toFixed 3, _rollInput toFixed 3];
/////////////////////////////////////////////////////////////////////////////////////////////
// Yaw Torque           /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
private _rtrTorqueScalar = [_rtrTorqueScalarTable, _velXY] call bmkhs_fnc_mathLinearInterp select 1;
private _momentZ         = _rtrTorque * _rtrTorqueScalar * _deltaTime;
//systemChat format ["main rotor _momentZ = %1", _momentZ toFixed 0];
/////////////////////////////////////////////////////////////////////////////////////////////
// Rotor Forces         /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
if (currentPilot _heli == player) then {
    private _mainRtrDamage = [_heli, "mainRotor"] call bmkhs_fnc_damageGet;

    if (_mainRtrDamage < 0.99) then {
        private _advanceRatio = if (_bladeTipVel > 1.0) then { _velY / _bladeTipVel } else { 0.0 };
        //FLAPBACK. Gains come from the simple-rotor config - the BET model derives its own
        //flapping from blade dynamics and has no use for a gain.
        //TWO OPEN ITEMS, both needing air time rather than a code change:
        //  1. Longitudinal flapback is NOT WIRED UP. _flapLon is computed and discarded -
        //     the pitch argument to mathVectorRotate below is a hardcoded 0.0 - so raising
        //     mainRtrFlapbackLon does nothing until that vector call passes it. This is the
        //     PRIMARY flapback effect (disc tilting nose-up as speed builds); only lateral,
        //     the secondary effect, is running. That is backwards from a real rotor.
        //  2. The lateral SIGN was never verified in the sim; positive tilts the thrust
        //     vector one way and nobody has confirmed it is the right way.
        private _kFlapLon     = _heli getVariable ["bmkhs_mainRtrFlapbackLon", 0.0];
        private _kFlapLat     = _heli getVariable ["bmkhs_mainRtrFlapbackLat", 0.0];
        private _flapLon      = _kFlapLon * _advanceRatio;
        private _flapLat      = _kFlapLat * _advanceRatio;

        //Pitch is 0.0 here - see item 1 above; _flapLon has nowhere to go until it is used.
        private _thrustVector = [_thrustZ, 0.0, (_rollInput * -6.0) - _flapLat, 0.0] call bmkhs_fnc_mathVectorRotate;
        //private _thrustVector = _thrustZ;

        if (BMKHS_FM_DEBUG) then {
        [_heli, _rtrPos, _rtrPos vectorAdd (vectorNormalized _thrustVector), "white"] call bmkhs_fnc_debugDrawLine;
        };

        if ([vectorMagnitude _thrustVector] call bmkhs_fnc_mathIsNAN || [vectorMagnitude _thrustVector] call bmkhs_fnc_mathIsINF) then { _thrustVector = [0.0, 0.0, 0.0]; };

        //Main rotor torque
        private _moment = [0.0, 0.0, 0.0];
        if (bmkhs_helisimRealismSetting == REALISTIC) then {
            //Main rotor thrust
            _heli addForce  [_heli vectorModelToWorld _thrustVector, _rtrPos];
            //Main rotor torque. NO `private` here - it would shadow the outer _moment, so the
            _moment = [_momentX, _momentY, _momentZ];
            if ([vectorMagnitude _moment] call bmkhs_fnc_mathIsNAN || [vectorMagnitude _moment] call bmkhs_fnc_mathIsINF) then { _moment = [0.0, 0.0, 0.0]; };
            _heli addTorque (_heli vectorModelToWorld _moment);
        } else {
            //Main rotor thrust
            _heli addForce  [_heli vectorModelToWorld _thrustVector, _heliCom];
            //Main rotor torque - yaw deliberately zeroed in casual (no torque reaction to fight).
            _moment = [_momentX, _momentY, 0.0];
            if ([vectorMagnitude _moment] call bmkhs_fnc_mathIsNAN || [vectorMagnitude _moment] call bmkhs_fnc_mathIsINF) then { _moment = [0.0, 0.0, 0.0]; };
            _heli addTorque (_heli vectorModelToWorld _moment);
        };
    };
};
/////////////////////////////////////////////////////////////////////////////////////////////
// Rotor Effects        /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
if (cameraView == "INTERNAL") then {
    //Camera shake effect for ETL (16 to 24 knots)
    if (_velXYNoWind > 8.23 && _velXYNoWind < 12.35 && !_isOnGnd) then {
        enableCamShake true;
        setCamShakeParams [0.0, 0.5, 0.0, 0.0, true];
        addCamShake       [0.9, 0.4, 6.2];
        enableCamShake false;

        setCustomSoundController[_heli, "CustomSoundController3", 1.5];
        setCustomSoundController[_heli, "CustomSoundController4", 0.8];
    } else {
        setCustomSoundController[_heli, "CustomSoundController4", 0.0];
    };
    //Camera shake effect 130kts to 140kts
    private _vel2d = (_heli getVariable "bmkhs_vel2D") * KNOTS_TO_MPS;
    if (_vel2d >= 66.87 && _vel2d < 72.02) then {
        enableCamShake true;
        setCamShakeParams [0.0, 0.5, 0.0, 0.0, true];
        addCamShake       [2.5, 1, 5];
        enableCamShake false;

        setCustomSoundController[_heli, "CustomSoundController3", 6.4];
        setCustomSoundController[_heli, "CustomSoundController4", 1.8];
    } else {
        setCustomSoundController[_heli, "CustomSoundController4", 0.0];
    };
    //Camera shake effect 140kts to 150kts
    if (_vel2d >= 72.02 && _vel2d < 77.16) then {
            enableCamShake true;
            setCamShakeParams [0.0, 0.5, 0.0, 0.5, true];
            addCamShake       [3, 1, 5.5];
            enableCamShake false;

            setCustomSoundController[_heli, "CustomSoundController3", 6.4];
            setCustomSoundController[_heli, "CustomSoundController4", 1.8];
    } else {
        setCustomSoundController[_heli, "CustomSoundController4", 0.0];
    };
    //Camera shake effect 150kts to 160kts
    if (_vel2d >= 77.16 && _vel2d < 82.30) then {
            enableCamShake true;
            setCamShakeParams [0.0, 0.75, 0.0, 0.75, true];
            addCamShake       [3.5, 1, 6.0];
            enableCamShake false;

            setCustomSoundController[_heli, "CustomSoundController3", 6.4];
            setCustomSoundController[_heli, "CustomSoundController4", 1.8];
    } else {
        setCustomSoundController[_heli, "CustomSoundController4", 0.0];
    };
    //Camera shake effect >160kts
    if (_vel2d >= 82.30) then {
            enableCamShake true;
            setCamShakeParams [0.0, 1.0, 0.0, 2.0, true];
            addCamShake       [4.0, 1, 6.5];
            enableCamShake false;

            setCustomSoundController[_heli, "CustomSoundController3", 6.4];
            setCustomSoundController[_heli, "CustomSoundController4", 1.8];
    } else {
        setCustomSoundController[_heli, "CustomSoundController4", 0.0];
    };
    //Camera shake effect for vortex ring sate
    if (_velXYNoWind < 12.35 && _inputRPM > EPSILON && !_isOnGnd) then {  //must be less than ETL
        //2000 fpm to 2933fpm
        if (_velZ < -(_vrsVelMax * 0.40) && _velZ > -(_vrsVelMax * 0.60)) then {
            enableCamShake true;
            setCamShakeParams [0.0, 0.5, 0.0, 0.0, true];
            addCamShake       [2.5, 1, 5];
            enableCamShake false;

            setCustomSoundController[_heli, "CustomSoundController3", 6.4];
            setCustomSoundController[_heli, "CustomSoundController4", 1.8];

            if (bmkhs_vrsWarning) then {
                hintSilent parseText format ["<t size='1.5' font='EtelkaMonospacePro' color='#99ffffff'>Entering VRS Condition!</t>"];
            };
        };
        //2933 fpm to 3867
        if (_velZ <= -(_vrsVelMax * 0.60) && _velZ > -(_vrsVelMax * 0.80)) then {
            enableCamShake true;
            setCamShakeParams [0.0, 0.5, 0.0, 0.5, true];
            addCamShake       [3, 1, 5.5];
            enableCamShake false;

            setCustomSoundController[_heli, "CustomSoundController3", 6.4];
            setCustomSoundController[_heli, "CustomSoundController4", 1.8];

            if (bmkhs_vrsWarning) then {
                hintSilent parseText format ["<t size='1.5' font='EtelkaMonospacePro' color='#FFFF00'>Caution! VRS Developing!</t>"];
            };
        };
        //3867fpm to 4800 fpm
        if (_velZ <= -(_vrsVelMax * 0.80) && _velZ > -_vrsVelMax) then {
            enableCamShake true;
            setCamShakeParams [0.0, 0.75, 0.0, 0.75, true];
            addCamShake       [3.5, 1, 6.0];
            enableCamShake false;

            setCustomSoundController[_heli, "CustomSoundController3", 6.4];
            setCustomSoundController[_heli, "CustomSoundController4", 1.8];
            if (bmkhs_vrsWarning) then {
                hintSilent parseText format ["<t size='1.5' font='EtelkaMonospacePro' color='#ff0000'>Warning! Fully Developed VRS Imminent!</t>"];
            };
        };
        //> 4800fpm
        if (_velZ < -_vrsVelMax) then {
            enableCamShake true;
            setCamShakeParams [0.0, 1.0, 0.0, 2.0, true];
            addCamShake       [4.0, 1, 6.5];
            enableCamShake false;

            setCustomSoundController[_heli, "CustomSoundController3", 6.4];
            setCustomSoundController[_heli, "CustomSoundController4", 1.8];

            if (bmkhs_vrsWarning) then {

                hintSilent parseText format ["<t size='1.5' font='EtelkaMonospacePro' color='#ff0000'>Danger! You are in VRS!</t>"];
            };
        };
    } else {
        setCustomSoundController[_heli, "CustomSoundController4", 0.0];
    };
};

if (BMKHS_FM_DEBUG) then {
[_heli, _rtrPos, _rtrPos vectorAdd _axisX,        "red"]   call bmkhs_fnc_debugDrawLine;
[_heli, _rtrPos, _rtrPos vectorAdd _axisY,        "green"] call bmkhs_fnc_debugDrawLine;
[_heli, _rtrPos, _rtrPos vectorAdd _axisZ,        "blue"]  call bmkhs_fnc_debugDrawLine;
[_heli, 24, _rtrPos, _bladeRadius, 2, "white", 0]   call bmkhs_fnc_debugDrawCircle;
};

//[_outThrust, _outTq];

/*
hintsilent format ["v0.7 testing
                    \nRotor Omega = %1
                    \nBlade Tip Vel = %2
                    \nRotor Power Req = %3 kW
                    \nRotor Torque = %4 Nm
                    \nE1 Tq = %5 % E2 Tq = %6 %
                    \nVelZ = %7
                    \nInduced Vel Scalar = %8
                    \nGnd Eff Scalar = %9
                    \nStab = %10
                    \nPitch = %11", _rtrOmega, _bladeTipVel, _rtrPowerReq * 0.001, _reqEngTorque, (_reqEngTorque / 2) / 481, (_reqEngTorque / 2) / 481, _velZ, _inducedVelocityScalar, _gndEffScalar, (_heli getVariable "bmkhs_collectiveOutput"), _heli call BIS_fnc_getPitchBank select 0];
                    */
