/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stateRtrRPM

Description:
    Returns the rotor RPM depending on the simulation being used

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    Rotor rpm

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];


private _deltaTime = _heli getVariable "bmkhs_deltaTime";
private _rtrRPM    = _heli getVariable "bmkhs_rtrRPM";

private _mainRtrDamage  = [_heli, "mainRotor"] call bmkhs_fnc_damageGet;
if (_mainRtrDamage == 1.0) then {
    _rtrRPM = 0.0;
} else {
    //(_heli getVariable "bmkhs_engPctNP")
    //params ["_e1Np", "_e2Np"];
//
    //if (_e1Np == 0.0 && _e2Np == 0.0) then {
    //    _rtrRPM = [_rtrRPM, 0.0, (1.0 / 10.0) * _deltaTime] call BIS_fnc_lerp;
    //} else {
    //    _rtrRPM = _e1Np max _e2Np;
    //};
    _rtrRPM = (_heli getVariable "bmkhs_xmsnOutputRpm") / 20900;
};
_heli setVariable ["bmkhs_rtrRPM", _rtrRPM];

_rtrRPM;
