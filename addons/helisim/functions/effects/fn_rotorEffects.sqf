/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_rotorEffects

Description:
    Camera shake and the sound controllers that go with it - translational lift,
    the high speed bands, and vortex ring state.

    Everything here is read from published state rather than passed in, so both
    rotor models call it the same way and neither owns the effects.

    Sound controllers:
        CustomSoundController3 - intensity
        CustomSoundController4 - blend; zeroed whenever no band is active

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    Nothing.

Examples:
    [_heli] call bmkhs_fnc_rotorEffects;

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

if (!local _heli) exitWith {};
if (cameraView != "INTERNAL") exitWith {};

private _velNoWind    = _heli getVariable "bmkhs_velModelSpaceNoWind";
private _velXYNoWind  = vectorMagnitude [_velNoWind select 0, _velNoWind select 1];
private _velZ         = _velNoWind select 2;
private _vel2d        = (_heli getVariable "bmkhs_vel2D") * KNOTS_TO_MPS;
private _isOnGnd      = [_heli] call bmkhs_fnc_stateOnGround;
private _inputRPM     = _heli getVariable "bmkhs_rtrRPM";

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
    //2000 fpm to 2933 fpm
    if (_velZ < -VRS_BAND_ENTERING && _velZ > -VRS_BAND_DEVELOPING) then {
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
    //2933 fpm to 3867 fpm
    if (_velZ <= -VRS_BAND_DEVELOPING && _velZ > -VRS_BAND_IMMINENT) then {
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
    //3867 fpm to 4800 fpm
    if (_velZ <= -VRS_BAND_IMMINENT && _velZ > -VEL_VRS) then {
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
    //> 4800 fpm
    if (_velZ < -VEL_VRS) then {
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
