/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_damageEffects

Description:
    Camera shake for g-stress failures - the 1/rev of a failed blade, and a jolt
    when a blade or the tail structure comes off. Runs on every crew machine.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\rotor\rotor.hpp"
#include "\bmkhs_helisim\functions\damage\gStress.hpp"

params ["_heli"];

if (vehicle player != _heli || {cameraView != "INTERNAL"}) exitWith {};

private _rotors   = _heli getVariable "bmkhs_simpleRotors";
private _xmsnRpm  = _heli getVariable "bmkhs_xmsnOutputRpm";
private _severity = 0.0;
private _freq     = 0.0;
private _gone     = count (_heli getVariable "bmkhs_partsLost");
{
    private _rotor = _rotors select _forEachIndex;
    if ((_rotor get "type") != MAIN) then { continue };
    {
        _x params ["_mode", "_track"];
        _severity = _severity + ((abs _track) * 25.0) + ([0.0, 0.0, 1.0, 2.5, 4.0] select _mode);
        if (_mode == BLADE_SEPARATED) then { _gone = _gone + 1 };
    } forEach _x;
    _freq = _freq max (_xmsnRpm / (_rotor get "gearRatio") / 60.0);
} forEach (_heli getVariable "bmkhs_bladeState");

//1/rev
if (_severity > 0.0 && {_freq > 0.5}) then {
    enableCamShake true;
    setCamShakeParams [0.0, 1.0, 0.0, 1.0, true];
    addCamShake       [_severity, 1, _freq];
    enableCamShake false;
};

//Something came off
if (_gone > (_heli getVariable ["bmkhs_damageEffectsGone", 0])) then {
    enableCamShake true;
    addCamShake [8.0, 1.5, 12.0];
    enableCamShake false;
};
_heli setVariable ["bmkhs_damageEffectsGone", _gone];
