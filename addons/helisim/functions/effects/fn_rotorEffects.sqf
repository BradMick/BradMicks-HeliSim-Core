/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_rotorEffects

Description:
    Camera shake and the sound controllers that go with it - translational lift,
    the high speed bands, and vortex ring state.

    Everything here is read from published state rather than passed in, so both
    rotor models call it the same way and neither owns the effects.

    Sound controllers:
        CustomSoundController64 - intensity
        CustomSoundController63 - blend; zero whenever no band is active

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
#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

params ["_heli"];

if (!local _heli) exitWith {};
if (cameraView != "INTERNAL") exitWith {};

private _velNoWind    = _heli getVariable "bmkhs_velModelSpaceNoWind";
private _velXYNoWind  = vectorMagnitude [_velNoWind select 0, _velNoWind select 1];
private _velZ         = _velNoWind select 2;
private _isOnGnd      = [_heli] call bmkhs_fnc_stateOnGround;
private _inputRPM     = _heli getVariable "bmkhs_rtrRpm";

//Set once, from whichever band is active - the last active band wins
private _intensity = 0;
private _blend     = 0;

//Camera shake effect for ETL (16 to 24 knots)
if (_velXYNoWind > 8.23 && _velXYNoWind < 12.35 && !_isOnGnd) then {
    enableCamShake true;
    setCamShakeParams [0.0, 0.5, 0.0, 0.0, true];
    addCamShake       [0.9, 0.4, 6.2];
    enableCamShake false;

    _intensity = 1.5;
    _blend     = 0.8;
};
//Camera shake effect for vortex ring sate
if (_velXYNoWind < 12.35 && _inputRPM > EPSILON && !_isOnGnd) then {  //must be less than ETL
    //2000 fpm to 2933 fpm
    if (_velZ < -VRS_BAND_ENTERING && _velZ > -VRS_BAND_DEVELOPING) then {
        enableCamShake true;
        setCamShakeParams [0.0, 0.5, 0.0, 0.0, true];
        addCamShake       [2.5, 1, 5];
        enableCamShake false;

        _intensity = 6.4;
        _blend     = 1.8;

        if (bmkhs_vrsWarning && {vehicle player == _heli}) then {
            hintSilent parseText format ["<t size='1.5' font='EtelkaMonospacePro' color='#99ffffff'>Entering VRS Condition!</t>"];
        };
    };
    //2933 fpm to 3867 fpm
    if (_velZ <= -VRS_BAND_DEVELOPING && _velZ > -VRS_BAND_IMMINENT) then {
        enableCamShake true;
        setCamShakeParams [0.0, 0.5, 0.0, 0.5, true];
        addCamShake       [3, 1, 5.5];
        enableCamShake false;

        _intensity = 6.4;
        _blend     = 1.8;

        if (bmkhs_vrsWarning && {vehicle player == _heli}) then {
            hintSilent parseText format ["<t size='1.5' font='EtelkaMonospacePro' color='#FFFF00'>Caution! VRS Developing!</t>"];
        };
    };
    //3867 fpm to 4800 fpm
    if (_velZ <= -VRS_BAND_IMMINENT && _velZ > -VEL_VRS) then {
        enableCamShake true;
        setCamShakeParams [0.0, 0.75, 0.0, 0.75, true];
        addCamShake       [3.5, 1, 6.0];
        enableCamShake false;

        _intensity = 6.4;
        _blend     = 1.8;
        if (bmkhs_vrsWarning && {vehicle player == _heli}) then {
            hintSilent parseText format ["<t size='1.5' font='EtelkaMonospacePro' color='#ff0000'>Warning! Fully Developed VRS Imminent!</t>"];
        };
    };
    //> 4800 fpm
    if (_velZ < -VEL_VRS) then {
        enableCamShake true;
        setCamShakeParams [0.0, 1.0, 0.0, 2.0, true];
        addCamShake       [4.0, 1, 6.5];
        enableCamShake false;

        _intensity = 6.4;
        _blend     = 1.8;

        if (bmkhs_vrsWarning && {vehicle player == _heli}) then {
            hintSilent parseText format ["<t size='1.5' font='EtelkaMonospacePro' color='#ff0000'>Danger! You are in VRS!</t>"];
        };
    };
};
//Retreating blade stall warning - an N-per-rev vibration building from 5/6 Vne to Vne
private _main = (_heli getVariable "bmkhs_simpleRotors") select {(_x get "type") == MAIN};
if (_main isNotEqualTo []) then {
    _main = _main select 0;
    private _vne      = _main get "vne";
    private _onset    = _vne * 5 / 6;
    private _airspeed = (_heli getVariable "bmkhs_velModelSpace") select 1;
    if (_airspeed > _onset) then {
        private _deltaTime = _heli getVariable "bmkhs_deltaTime";
        private _severity  = (((_airspeed - _onset) / (_vne - _onset)) min 1) ^ 2;
        //Blade passing frequency, capped at half the frame rate
        private _freq = ((_main get "numBlades") * ((_heli getVariable "bmkhs_xmsnOutputRpm") / (_main get "gearRatio")) / 60)
                        min (0.5 / (_deltaTime max 0.001));
        enableCamShake true;
        setCamShakeParams [0.0, 1.0, 0.0, 0.5, true];
        addCamShake       [_severity * 1.5, 1, _freq];
        enableCamShake false;

        _intensity = _severity * 6.4;
        _blend     = _severity * 1.8;
    };
};

setCustomSoundController [_heli, "CustomSoundController64", _intensity];
setCustomSoundController [_heli, "CustomSoundController63", _blend];
