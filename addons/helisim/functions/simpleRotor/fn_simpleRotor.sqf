/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_simpleRotor

Description:
    Runs every declared rotor and applies its thrust and torque. Thrust and
    power are independent chains - neither reads the other.

    Field reference: \bmkhs_helisim\simpleRotor.hpp

Parameters:
    _heli - The helicopter to run the rotors on [Object].

Returns:
    Nothing. Writes bmkhs_rtrThrust[] and bmkhs_reqEngTorque[] per rotor and
    applies force and torque to the airframe.

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

private _rotors = _heli getVariable ["bmkhs_simpleRotors", []];
if (_rotors isEqualTo []) exitWith {};

private _deltaTime     = _heli getVariable "bmkhs_deltaTime";
private _heliCom       = getCenterOfMass _heli;
private _dryAirDensity = _heli getVariable "bmkhs_RHO";
private _isOnGnd       = [_heli] call bmkhs_fnc_stateOnGround;
private _isSingleEng   = _heli getVariable "bmkhs_isSingleEng";

//Control inputs, each the pilot's stick plus whatever the FMC is adding.
private _fmcPitchOut = (_heli getVariable "bmkhs_fmcAttHoldCycPitchOut")
                     + (_heli getVariable "bmkhs_fmcSasPitchOut")
                     + (_heli getVariable "bmkhs_fmcCollectiveToPitch")
                     + (_heli getVariable "bmkhs_fmcYawToPitch");

private _fmcRollOut  = (_heli getVariable "bmkhs_fmcAttHoldCycRollOut")
                     + (_heli getVariable "bmkhs_fmcSasRollOut")
                     + (_heli getVariable "bmkhs_fmcCollectiveToRoll")
                     + (_heli getVariable "bmkhs_fmcYawToRoll");

private _fmcCollOut  = (_heli getVariable "bmkhs_collectiveOutput")
                     + (_heli getVariable "bmkhs_fmcAltHoldCollOut");

private _pitchInput = [([_heli getVariable "bmkhs_cyclicFwdAft",   _heli getVariable "bmkhs_forceTrimPosPitch"] call bmkhs_fnc_inputGetInterp) + _fmcPitchOut, -1.0, 1.0] call BIS_fnc_clamp;
private _rollInput  = [([_heli getVariable "bmkhs_cyclicLeftRight", _heli getVariable "bmkhs_forceTrimPosRoll"]  call bmkhs_fnc_inputGetInterp) + _fmcRollOut,  -1.0, 1.0] call BIS_fnc_clamp;
private _pedalInput = [([_heli getVariable "bmkhs_pedalLeftRight",  _heli getVariable "bmkhs_forceTrimPosYaw"]   call bmkhs_fnc_inputGetInterp),               -1.0, 1.0] call BIS_fnc_clamp;

//Already airspeed - fn_stateVelocities subtracts wind. Do not re-add it.
private _vel = _heli getVariable "bmkhs_velModelSpace";

//Drivetrain state - one number, shared by every rotor on the same gearbox.
private _xmsnOutputRpm = _heli getVariable "bmkhs_xmsnOutputRpm";
private _engPctTQ      = (_heli getVariable "bmkhs_engPctTQ" select 0) max (_heli getVariable "bmkhs_engPctTQ" select 1);

private _thrustOut = [];
private _torqueOut = [];
private _vrsMinOut = [];
private _vrsMaxOut = [];
private _moiOut    = [];

{
    private _r      = _x;
    private _idx    = _forEachIndex;
    private _isMain = _r get "isMain";

    private _pos         = _r get "position";
    private _bladeRadius = _r get "bladeRadius";
    private _baseThrust  = _r get "baseThrust";
    private _vne         = _r get "vne";
    private _vbe         = _r get "vbe";
    private _etl         = _r get "etl";

    private _rtrArea = pi * _bladeRadius * _bladeRadius;

    /////////////////////////////////////////////////////////////////////////////////////////
    // RPM - shared drivetrain, per-rotor gearing
    /////////////////////////////////////////////////////////////////////////////////////////
    private _refRpm     = _r get "refRpm";
    private _inputRPM   = if (_refRpm == 0.0) then {0.0} else {_xmsnOutputRpm / _refRpm};
    private _rpmTrim    = _r get "rpmTrimVal";
    private _inputRpmPct= [(if (_rpmTrim == 0.0) then {0.0} else {_inputRPM / _rpmTrim}), 0.0, 1.0] call BIS_fnc_clamp;

    private _rtrOmega = (2.0 * pi) * ((_r get "designRpm") * _inputRPM) / 60.0;

    //Hub airspeed: CG airspeed plus (r x omega) - a yaw rate swings the tail
    //through the air even with the CG still.
    private _leverArm = _pos vectorDiff _heliCom;
    private _velHub   = _vel vectorAdd (_leverArm vectorCrossProduct (_heli getVariable ["bmkhs_angVelModelSpace", [0,0,0]]));

    //Flow comes from the AXIS, not from rotorType - through the disc along the
    //thrust axis, across it perpendicular.
    private _axis           = _r get "axis";
    private _velThroughDisc = _velHub vectorDotProduct _axis;
    private _velAcrossDisc  = vectorMagnitude (_velHub vectorDiff (_axis vectorMultiply _velThroughDisc));
    _velAcrossDisc = _velAcrossDisc min _vne;
    if ([_velAcrossDisc] call bmkhs_fnc_mathIsNAN || [_velAcrossDisc] call bmkhs_fnc_mathIsINF) then { _velAcrossDisc = 0.0; };
    if (_isOnGnd) then { _velAcrossDisc = 0.0; };

    /////////////////////////////////////////////////////////////////////////////////////////
    // THRUST CHAIN - does not read torque
    /////////////////////////////////////////////////////////////////////////////////////////
    //Collective for a main, pedal for a tail.
    private _ctlInput  = if (_isMain) then {_fmcCollOut} else {_pedalInput};
    private _ctlScalar = [_r get "thrustVsCollective", _ctlInput] call bmkhs_fnc_mathLinearInterp select 1;

    //Translational lift.
    private _airspeedScalar = [_r get "thrustVsAirspeed", _velAcrossDisc] call bmkhs_fnc_mathLinearInterp select 1;

    //Density falls off with altitude and heat; the rotor loses thrust with it.
    private _densityScalar = _dryAirDensity / ISA_STD_DAY_AIR_DENSITY;

    //VRS. Band comes from last frame's induced velocity, so it tracks the
    //airframe with no config.
    private _vrsMin = (_heli getVariable ["bmkhs_vrsVelocityMin", []]) param [_idx, 0.0];
    private _vrsMax = (_heli getVariable ["bmkhs_vrsVelocityMax", []]) param [_idx, 0.0];
    private _inducedScalar = 1.0;
    if (_velThroughDisc < -_vrsMin && {_velAcrossDisc < _etl} && {_vrsMax > 0.0}) then {
        private _severity = if (_velThroughDisc == 0.0) then {0.0} else {(abs (_vrsMin / _velThroughDisc)) ^ VRS_SEVERITY_EXP};
        _inducedScalar = (1.0 - (_velThroughDisc / _vrsMax)) * _severity;
    };

    private _thrust = _baseThrust * _ctlScalar * _inputRpmPct * _densityScalar * _airspeedScalar * _inducedScalar;

    //Ground effect: (1 - h/D) shape in Core, strength from config.
    if (_isMain) then {
        private _gain = _r get "groundEffectGain";
        if (_gain > 0.0) then {
            private _heightAGL = (_r get "heightAgl") + (ASLToAGL getPosASL _heli # 2);
            private _cushion   = [(1.0 - (_heightAGL / (_bladeRadius * 2.0))) * _gain, 0.0, 1.0] call BIS_fnc_clamp;
            _thrust = _thrust + (_thrust * _cushion);
        };

        //Climb thrust from torque the engine is making above what cruise needs.
        private _climbGain = _r get "climbGain";
        if (_climbGain > 0.0) then {
            private _cruiseTq = [_r get "powerVsAirspeed", _velAcrossDisc] call bmkhs_fnc_mathLinearInterp select 1;
            if (_isSingleEng) then { _cruiseTq = _cruiseTq * 2.0; };
            private _tqChange = [_engPctTQ - _cruiseTq, 0.0, 0.8] call BIS_fnc_clamp;
            _thrust = _thrust + (_baseThrust * _tqChange * _climbGain);
        };
    };

    if ([_thrust] call bmkhs_fnc_mathIsNAN || [_thrust] call bmkhs_fnc_mathIsINF) then { _thrust = 0.0; };

    //Induced velocity for NEXT frame's VRS band. Momentum theory - no config.
    private _inducedVelocity = if (_rtrArea <= 0.0 || {_dryAirDensity <= 0.0}) then {0.0} else {
        sqrt ((abs _thrust) / (2.0 * _dryAirDensity * _rtrArea))
    };
    if ([_inducedVelocity] call bmkhs_fnc_mathIsNAN || [_inducedVelocity] call bmkhs_fnc_mathIsINF) then { _inducedVelocity = 0.0; };

    /////////////////////////////////////////////////////////////////////////////////////////
    // POWER CHAIN - does not read thrust
    /////////////////////////////////////////////////////////////////////////////////////////
    private _torqueReq = 0.0;
    if (_isMain) then {
        private _powerFrac = [_r get "powerVsAirspeed",   _velAcrossDisc] call bmkhs_fnc_mathLinearInterp select 1;
        private _collCorr  = [_r get "powerVsCollective", _fmcCollOut]    call bmkhs_fnc_mathLinearInterp select 1;

        //Fades in above ETL.
        _collCorr = linearConversion [0.0, _etl, _velAcrossDisc, 1.0, _collCorr, true];

        private _powerVal = [_powerFrac * _collCorr, -1.0, 2.50] call BIS_fnc_clamp;
        private _powerReq = _powerVal * (_r get "maxPower");

        //kW -> Nm at the transmission output shaft.
        _torqueReq = (_powerReq * 1000.0) / ((2.0 * pi / 60.0) * _refRpm);
        _torqueReq = _torqueReq * _inputRpmPct;

        //Autorotation: descending air drives the rotor.
        private _autoro = _r get "autoroTorque";
        if (_autoro > 0.0 && {_velThroughDisc < 0.0}) then {
            _torqueReq = _torqueReq + (_velThroughDisc * _autoro);
        };
    } else {
        //Costs power, and more on a pedal input - falls out of its own thrust.
        _torqueReq = (abs _thrust) * (_r get "torqueScalar");
    };

    if ([_torqueReq] call bmkhs_fnc_mathIsNAN || [_torqueReq] call bmkhs_fnc_mathIsINF) then { _torqueReq = 0.0; };

    /////////////////////////////////////////////////////////////////////////////////////////
    // APPLY
    /////////////////////////////////////////////////////////////////////////////////////////
    private _damageRole = if (_isMain) then {"mainRotor"} else {"tailRotor"};
    private _damage     = [_heli, _damageRole] call bmkhs_fnc_damageGet;
    private _damageThr  = if (_isMain) then {MAIN_RTR_DMG_THRESH} else {TAIL_RTR_DMG_THRESH};

    private _driven = _damage < _damageThr;
    if (!_isMain) then {
        //Tail also stops if its drive does.
        private _igb = [_heli, "intermediateGearbox"] call bmkhs_fnc_damageGet;
        private _tgb = [_heli, "tailRotorGearbox"]    call bmkhs_fnc_damageGet;
        _driven = _driven && {_igb < SYS_IGB_DMG_THRESH} && {_tgb < SYS_TGB_DMG_THRESH};
    };

    if (_driven && {currentPilot _heli == player}) then {
        private _realistic = bmkhs_helisimRealismSetting == REALISTIC;

        //Tilted by roll input and flapback on a main rotor.
        private _thrustVector = (_r get "axis") vectorMultiply (_thrust * _deltaTime);
        if (_isMain) then {
            private _tipVel  = _rtrOmega * _bladeRadius;
            private _advance = if (_tipVel > 1.0) then {(_vel select 1) / _tipVel} else {0.0};
            private _flapLat = (_r get "flapbackLat") * _advance;
            private _tiltRoll = (_rollInput * (_r get "thrustTiltRoll")) - _flapLat;
            _thrustVector = [_thrustVector, 0.0, _tiltRoll, 0.0] call bmkhs_fnc_mathVectorRotate;
        };
        if ([vectorMagnitude _thrustVector] call bmkhs_fnc_mathIsNAN || [vectorMagnitude _thrustVector] call bmkhs_fnc_mathIsINF) then { _thrustVector = [0.0, 0.0, 0.0]; };

        //Main gets cyclic; tail gets the yaw its thrust makes.
        private _moment = [0.0, 0.0, 0.0];
        if (_isMain) then {
            //Control authority scales with what the rotor can actually DO, not
            //with a fixed Nm figure: baseThrust is the reference, the authority
            //fields are multiples of it, and both ramp with Nr. A dying rotor
            //loses control power because it loses thrust.
            private _pitchTq = linearConversion [0.0, 1.0, _inputRpmPct, 0.0, _baseThrust * (_r get "pitchAuthority") * _deltaTime, true];
            private _rollTq  = linearConversion [0.0, 1.0, _inputRpmPct, 0.0, _baseThrust * (_r get "rollAuthority")  * _deltaTime, true];
            //Reaction to driving the rotor. Zeroed in casual.
            //Yaw is the REACTION to driving the rotor - it comes out of the
            //torque actually being demanded, scaled only by gearing. Zeroed in
            //casual: nothing to fight.
            private _yawTq = if (_realistic) then {
                _torqueReq * (_r get "gearRatio") * (_r get "dirSign") * (_r get "yawAuthority") * _deltaTime
            } else {0.0};
            _moment = [_pitchTq * _pitchInput, _rollTq * _rollInput, _yawTq];
        } else {
            private _moment2 = _thrustVector vectorCrossProduct (_pos vectorDiff _heliCom);
            //Tail thrust acts above the roll axis, so it rolls as well as yaws.
            _moment2 set [1, (_moment2 select 1) * (_r get "rollCouple")];
            _moment = _moment2;
        };
        if ([vectorMagnitude _moment] call bmkhs_fnc_mathIsNAN || [vectorMagnitude _moment] call bmkhs_fnc_mathIsINF) then { _moment = [0.0, 0.0, 0.0]; };

        private _applyAt = if (_realistic) then {_pos} else {_heliCom};
        _heli addForce  [_heli vectorModelToWorld _thrustVector, _applyAt];
        _heli addTorque (_heli vectorModelToWorld _moment);

        if (BMKHS_FM_DEBUG) then {
            [_heli, _pos, _pos vectorAdd (vectorNormalized _thrustVector), "white"] call bmkhs_fnc_debugDrawLine;
        };
    };

    _thrustOut pushBack _thrust;
    _torqueOut pushBack _torqueReq;
    _vrsMinOut pushBack (_inducedVelocity * VRS_VEL_MIN_FRAC);
    _vrsMaxOut pushBack (_inducedVelocity * VRS_VEL_MAX_FRAC);
    _moiOut    pushBack (_r get "rotorInertia");
} forEach _rotors;

_heli setVariable ["bmkhs_rtrThrust",      _thrustOut];
_heli setVariable ["bmkhs_reqEngTorque",   _torqueOut];
_heli setVariable ["bmkhs_vrsVelocityMin", _vrsMinOut];
_heli setVariable ["bmkhs_vrsVelocityMax", _vrsMaxOut];
_heli setVariable ["bmkhs_rtrMoi",         _moiOut];
