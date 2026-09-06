/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_simpleRotorTail

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
params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\systems\systems.hpp"

if (!local _heli) exitWith {};

private _deltaTime              = _heli getVariable "bmkhs_deltaTime";
private _heliCom                = getCenterOfMass _heli;

private _altitude               = _heli getVariable "bmkhs_PA";
private _temperature            = _heli getVariable "bmkhs_FAT";
private _dryAirDensity          = _heli getVariable "bmkhs_rho";

private _hdgHoldPedalYawOut     = _heli getVariable "bmkhs_fmcHdgHoldPedalYawOut";
private _sasYawOut              = _heli getVariable "bmkhs_fmcSasYawOut";
private _fmcYawOut              = _hdgHoldPedalYawOut + _sasYawOut;
//_fmcYawOut                      = [_fmcYawOut, -0.15, 0.15] call BIS_fnc_clamp;

private _rtrPos                 = _heli getVariable "bmkhs_tailRtrPos";

private _rtrDesignRPM           = _heli getVariable "bmkhs_tailRtrDesignRpm";
private _rtrRPMTrimVal          = _heli getVariable "bmkhs_tailRtrRpmTrimVal";
private _rtrGearRatio           = _heli getVariable "bmkhs_tailRtrGearRatio";
private _rtrNumBlades           = _heli getVariable "bmkhs_tailRtrNumBlades";

private _bladeRadius            = _heli getVariable "bmkhs_tailRtrBladeRadius";
private _bladeChord             = _heli getVariable "bmkhs_tailRtrBladeChord";

private _velVne                 = _heli getVariable "bmkhs_tailRtrVne";
private _velVrs                 = _heli getVariable "bmkhs_tailRtrVrs";
private _velEtl                 = _heli getVariable "bmkhs_tailRtrEtl";

private _bladePitchInducedThrustTable = _heli getVariable "bmkhs_tailRtrPitchThrustTable";

private _thrustVsAirspeedTable = _heli getVariable "bmkhs_tailRtrThrustVsAirspeed";

private _baseThrust             = _heli getVariable "bmkhs_tailRtrBaseThrust";

//Thrust produced
private _pedalLeftRight     = _heli getVariable "bmkhs_pedalLeftRight";
private _pedalLeftRightTrim = 0.0;
_pedalLeftRightTrim         = _heli getVariable "bmkhs_forceTrimPosYaw";

private _pedalInput         = ([_pedalLeftRight, _pedalLeftRightTrim] call bmkhs_fnc_inputGetInterp) + _fmcYawOut;
_pedalInput                 = [_pedalInput, -1.0, 1.0] call BIS_fnc_clamp;
//Publish the total tail-rotor yaw input (manual pedal + trim + FMC) so the
private _bladePitchInducedThrustScalar = [_bladePitchInducedThrustTable, _pedalInput] call bmkhs_fnc_mathLinearInterp select 1;//linearConversion [_bladePitch_min, _bladePitch_max, _bladePitch_cur, _rtrThrustScalar_min, _rtrThrustScalar_max, true];
//systemChat format ["_bladePitchInducedThrustScalar = %1 -- _pedalInput = %2", _bladePitchInducedThrustScalar toFixed 3, _pedalInput];
(_heli getVariable "bmkhs_engPctNP")
    params ["_eng1PctNP", "_eng2PctNp"];
private _inputRPM                  = _eng1PctNP max _eng2PctNp;
//Rotor induced thrust as a function of RPM
private _rtrRPMInducedThrustScalar = _inputRPM / _rtrRPMTrimVal;

//Thrust scalar as a result of altitude
private _airDensityThrustScalar    = _dryAirDensity / ISA_STD_DAY_AIR_DENSITY;
//Additional thrust gained from increasing forward airspeed
//THE ROTOR'S FRAME, from tailRtrRotation. Built every frame so it can be
//retuned live. discRight/discFwd span the disc, axis is up the mast.
(_heli getVariable "bmkhs_tailRtrRotation") params [["_rotP",0],["_rotR",0],["_rotY",0]];
private _axis      = vectorNormalized ([[0.0, 0.0, 1.0], _rotP, _rotR, _rotY] call bmkhs_fnc_mathVectorRotate);
private _discRight = vectorNormalized ([[1.0, 0.0, 0.0], _rotP, _rotR, _rotY] call bmkhs_fnc_mathVectorRotate);
private _discFwd   = vectorNormalized ([[0.0, 1.0, 0.0], _rotP, _rotR, _rotY] call bmkhs_fnc_mathVectorRotate);

private _deltaPos                  = _rtrPos vectorDiff _heliCom;
private _angVel                    = _heli getVariable ["bmkhs_angVelModelSpace", [0,0,0]];
private _velRot                    = _deltaPos vectorCrossProduct _angVel;
private _velHub                    = (_heli getVariable "bmkhs_velModelSpace") vectorAdd _velRot;

//Wind is added to the hub flow, as before, then resolved into the frame.
private _velHubWind                = _velHub;
private _velWindY                  = _heli getVariable "bmkhs_velWindModelSpace" select 1;
private _velWindX                  = _heli getVariable "bmkhs_velWindModelSpace" select 0;
if (_velWindY < 0.0) then {
    _velWindY = 0.0;
};
_velHubWind = _velHubWind vectorAdd [_velWindX, _velWindY, 0.0];
//ACROSS the disc: what is left once the through-disc component is removed.
private _velThroughDisc            = _velHubWind vectorDotProduct _axis;
private _velYZ                     = (vectorMagnitude (_velHubWind vectorDiff (_axis vectorMultiply _velThroughDisc))) min _velVne;
private _airspeedVelocityScalar    = [_thrustVsAirspeedTable, _velYZ] call bmkhs_fnc_mathLinearInterp select 1;
//Induced flow handler - lateral flow through the disk, at the HUB.
//THROUGH the disc, along the mast.
private _velX                      = _velThroughDisc;

private _inducedVelocityScalar     = 1.0;
if (_velX < -_velVrs && _velYZ < _velEtl) then {
    _inducedVelocityScalar = 0.0;
} else {
    _inducedVelocityScalar = 1 - (_velX / _velVrs);
};
//Finally, multiply all the scalars above to arrive at the final thrust scalar
private _rtrThrustScalar   = _bladePitchInducedThrustScalar * _rtrRPMInducedThrustScalar * _airDensityThrustScalar * _airspeedVelocityScalar * _inducedVelocityScalar;
private _rtrThrust         = _baseThrust * _rtrThrustScalar;


//Airspeed authority is folded into thrustVsAirspeed above - one curve, not two.
private _totThrust       = _rtrThrust;
//systemChat format ["_totThrust %1", _totThrust toFixed 0];

private _thrustVector  = _axis vectorMultiply (_totThrust * _deltaTime);
private _moment        = _thrustVector vectorCrossProduct _deltaPos;
//rollCouple trims how much of the moment reaches the airframe's ROLL axis.
//Decomposed about that axis rather than an array index, so it holds however
//the rotor is mounted.
private _rollAx  = [0.0, 1.0, 0.0];
private _rollAmt = _moment vectorDotProduct _rollAx;
_moment = _moment vectorAdd (_rollAx vectorMultiply (_rollAmt * ((_heli getVariable "bmkhs_tailRtrRollCouple") - 1.0)));

private _tailRtrDamage = [_heli, "tailRotor"] call bmkhs_fnc_damageGet;
private _IGBDamage     = [_heli, "intermediateGearbox"] call bmkhs_fnc_damageGet;
private _TGBDamage     = [_heli, "tailRotorGearbox"] call bmkhs_fnc_damageGet;

private _outThrust = [0.0, 0.0, 0.0];
private _outTq     = [0.0, 0.0, 0.0];

if ([vectorMagnitude _thrustVector] call bmkhs_fnc_mathIsNAN || [vectorMagnitude _thrustVector] call bmkhs_fnc_mathIsINF) then { _thrustVector = [0.0, 0.0, 0.0]; };

if (_tailRtrDamage < (_heli getVariable "bmkhs_tailRtrDamageThresh") && _IGBDamage < SYS_IGB_DMG_THRESH && _TGBDamage < SYS_TGB_DMG_THRESH) then {
    if (currentPilot _heli == player) then {
        if ( bmkhs_helisimRealismSetting == REALISTIC) then {
            //Tail rotor thrust
            _heli addForce [_heli vectorModelToWorld _thrustVector, _rtrPos];
            //Tail rotor torque
            _heli addTorque (_heli vectorModelToWorld _moment);
        } else {
            //Tail rotor thrust
            _heli addForce [_heli vectorModelToWorld _thrustVector, _heliCom];
            //Tail rotor torque
            _heli addTorque (_heli vectorModelToWorld _moment);
        };
    };
};

if (BMKHS_FM_DEBUG) then {
[_heli, _rtrPos, _rtrPos vectorAdd _discRight, "red"]   call bmkhs_fnc_debugDrawLine;
[_heli, _rtrPos, _rtrPos vectorAdd _discFwd,   "green"] call bmkhs_fnc_debugDrawLine;
[_heli, _rtrPos, _rtrPos vectorAdd _axis,      "blue"]  call bmkhs_fnc_debugDrawLine;
[_heli, 24, _rtrPos, _bladeRadius, 0, "white", 0]   call bmkhs_fnc_debugDrawCircle;
};

[_outThrust, _outTq];

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
