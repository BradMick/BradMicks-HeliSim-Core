/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_coreUpdate

Description:
    Updates all of the modules core functions.

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

if (isGamePaused || CBA_missionTime < 0.1) exitWith {
    _heli setVariable ["bmkhs_previousTime",  diag_tickTime];
    _heli setVariable ["bmkhs_deltaTime_avg", [bmkhs_movingAverageSize] call bmkhs_fnc_smoothAverageInit];
};

if (isAutoHoverOn _heli) then {
    _heli action ["AutoHoverCancel", _heli];
};

[_heli] call bmkhs_fnc_stateDeltaTime;

//Environment
[_heli] call bmkhs_fnc_environment;

//Velocities
[_heli] call bmkhs_fnc_stateVelocities;
[_heli] call bmkhs_fnc_stateAccelerations;

//Input
[_heli] call bmkhs_fnc_fmc;
[_heli] call bmkhs_fnc_inputUpdate;

//Preston AI Pilot
[_heli] call bmkhs_fnc_stateAeroValues;

//Fuel
[_heli] call bmkhs_fnc_fuelUpdate;
[_heli] call bmkhs_fnc_fuelMgmtUpdate;

//Mass and Balance
[_heli] call bmkhs_fnc_massUpdate;

//Performance
[_heli] call bmkhs_fnc_perfData;

//Engines
[_heli] call bmkhs_fnc_engineController;

//Transmission
[_heli] call bmkhs_fnc_transmissionUpdate;

//Damage - stub, see fn_damageApply
//[_heli] call bmkhs_fnc_damageApply;

//The systems display owns the hint while it is up - both use hintSilent, and this one
//runs last, so it would simply overwrite the other.
if (bmkhs_fmDebug && {!bmkhs_sysDebug}) then {
    hintSilent format [
    "_cyclicFwdAft = %1
    \n_cyclicLeftRight = %2
    \n_pedalYaw = %3
    \n_collectiveOuput = %4
    \n_stabilatorPosition = %5
    \n--------------------
    \n_springlessCyclic = %6
    \n_springlessPedals = %7
    \n_stickyPitch = %8
    \n_stickyRoll = %9
    \n_stickyYaw = %10
    \n_autoPedal = %11
    \n_cyclicCenterTrimMode = %12
    \n_pedalCenterTrimMode = %13
    \n--------------------
    \n_mouseAsJoystick = %14
    \n_kbStickyInterupt = %15
    \n_forceTrimInterupted = %16
    \n--------------------
    \n_attHoldCycPitchOut = %17
    \n_sasPitchOut = %18
    \n_attHoldCycRollOut = %19
    \n_sasRollOut = %20
    \n_hdgHoldPedalYawOut = %21
    \n_sasYawOut = %22
    \n_altHoldCollOut = %23
    \n--------------------
    \n_forceTrimPosPitch = %24
    \n_forceTrimPosRoll = %25
    \n_forceTrimPosPedal = %26
    \n--------------------
    \n_centerOfMass = [%27, %28, %29]
    \n_grossWeight = %30 lbs
    \n--------------------
    \n_attHoldActive = %31
    \n_attHoldSubMode = %32
    \n--------------------
    \n_altHoldActive = %33
    \n_altHoldSubMode = %34
    \n--------------------
    \n_hdgHoldActive = %35
    \n_hdgHoldSubMode = %36
    \n--------------------
    \n_flightControlLockOut = %37
    \n--------------------
    \nPitch = %38 Roll = %39
    \nYaw = %41  Sideslip = %40
    \n--------------------
    \nAutoAtt = %42
    \nPitchActive = %43 APTarget = %44
    \nRollActive = %45  ARTarget = %46
    ",
    _heli getVariable "bmkhs_cyclicFwdAft" toFixed 3,                    //1
    _heli getVariable "bmkhs_cyclicLeftRight" toFixed 3,                 //2
    _heli getVariable "bmkhs_pedalLeftRight" toFixed 3,                  //3
    _heli getVariable "bmkhs_collectiveOutput" toFixed 3,                //4
    _heli getVariable "bmkhs_stabilatorPosition" toFixed 3,                 //5
    bmkhs_springlessCyclic,                                          //6
    bmkhs_springlessPedals,                                          //7
    bmkhs_keyboardStickyPitch,                                       //8
    bmkhs_keyboardStickyRoll,                                        //9
    bmkhs_keyboardStickyYaw,                                         //10
    bmkhs_autoPedal,                                                 //11
    bmkhs_cyclicCenterTrimMode,                                          //12
    bmkhs_pedalCenterTrimMode,                                           //13
    bmkhs_mouseAsJoystick,                                           //14
    _heli getVariable "bmkhs_kbStickyInterupt",                          //15
    _heli getVariable "bmkhs_forceTrimInterupted",                          //16
    _heli getVariable "bmkhs_fmcAttHoldCycPitchOut" toFixed 3,          //17
    _heli getVariable "bmkhs_fmcSasPitchOut" toFixed 3,                 //18
    _heli getVariable "bmkhs_fmcAttHoldCycRollOut" toFixed 3,           //19
    _heli getVariable "bmkhs_fmcSasRollOut" toFixed 3,                  //20
    _heli getVariable "bmkhs_fmcHdgHoldPedalYawOut" toFixed 3,          //21
    _heli getVariable "bmkhs_fmcSasYawOut" toFixed 3,                   //22
    _heli getVariable "bmkhs_fmcAltHoldCollOut" toFixed 3,              //23
    _heli getVariable "bmkhs_forceTrimPosPitch" toFixed 3,                 //24
    _heli getVariable "bmkhs_forceTrimPosRoll" toFixed 3,                  //25
    _heli getVariable "bmkhs_forceTrimPosYaw" toFixed 3,                   //26
    //Report the CoM in SURVEYED space - the frame the user measured in Object Builder and typed
    //into the arms - not the engine's shifted frame. setCenterOfMass was handed the surveyed CG
    //with boundingCenter subtracted, so adding it back here undoes that and the readout matches
    //what the user expects to see (e.g. 1.201 against surveyed CG limits of 1.117 / 0.944, rather
    //than the shifted 1.926 which means nothing to them).
    ((getCenterOfMass _heli) vectorAdd (boundingCenter _heli)) select 0 toFixed 3,   //27
    ((getCenterOfMass _heli) vectorAdd (boundingCenter _heli)) select 1 toFixed 3,   //28
    ((getCenterOfMass _heli) vectorAdd (boundingCenter _heli)) select 2 toFixed 3,   //29
    ((_heli getVariable "bmkhs_GWT") * 2.20462) toFixed 0,              //30
    _heli getVariable "bmkhs_attHoldActive",                               //31
    _heli getVariable "bmkhs_attHoldSubMode",                              //32
    _heli getVariable "bmkhs_altHoldActive",                               //33
    _heli getVariable "bmkhs_altHoldSubMode",                              //34
    _heli getVariable "bmkhs_hdgHoldActive",                               //35
    _heli getVariable "bmkhs_hdgHoldSubMode",                              //36
    _heli getVariable "bmkhs_flightControlLockOut",                     //37
    _heli call BIS_fnc_getPitchBank select 0 toFixed 2,                       //38
    _heli call BIS_fnc_getPitchBank select 1 toFixed 2,                       //39
    _heli getVariable "bmkhs_aero_beta_deg" toFixed 2,                  //40
    ([player getRelDir _heli] call CBA_fnc_simplifyAngle180) toFixed 2,       //41
    bmkhs_helisimRealismSetting != REALISTIC,                              //42
    _heli getVariable "bmkhs_prestonPitchActive",                          //43
    _heli getVariable "bmkhs_prestonPitchTarget" toFixed 1,                //44
    _heli getVariable "bmkhs_prestonRollActive",                           //45
    _heli getVariable "bmkhs_prestonRollTarget" toFixed 1                  //46
    ];
};
