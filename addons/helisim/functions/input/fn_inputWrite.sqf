/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputWrite

Description:
    Turns the player's input (bmkhs_input*) into the control positions and what the rotors get.

Parameters:
    _heli    - The helicopter to get information from [Unit].
    _piloted - This player is flying the aircraft [Bool].

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_piloted"];
#include "\bmkhs_helisim\functions\core\core.hpp"

/////////////////////////////////////////////////////////////////////////////////////////////
// Hydraulics           /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//Hydraulic boost is mechanical control, it never freezes. Unsupplied, everything else does.
private _boost      = _heli getVariable "bmkhs_hydBoost";
private _frozen     = !_boost && {!(_heli getVariable ["bmkhs_fltCtrlsSupplied", true])};
//The tail rotor, lost by having no hydraulic authority to move it or no drive turning it
private _tailFrozen = _frozen
    || {!_boost && {!(_heli getVariable ["bmkhs_tailRtrSupplied", true])}}
    || {!(_heli getVariable ["bmkhs_tailRtrDriven", true])};

//The rate command, per axis: the mouse's, and the keyboard's unless sticky has it
private _ratePitch  = _piloted && {!_frozen} && {bmkhs_pitchScheme == "mouse" || {bmkhs_pitchScheme == "kb" && {!bmkhs_keyboardStickyPitch}}};
private _rateRoll   = _piloted && {!_frozen} && {bmkhs_rollScheme  == "mouse" || {bmkhs_rollScheme  == "kb" && {!bmkhs_keyboardStickyRoll}}};
if (!_ratePitch) then { _heli setVariable ["bmkhs_rateCommandPitchOn", false] };
if (!_rateRoll)  then { _heli setVariable ["bmkhs_rateCommandRollOn",  false] };

//Frozen controls stay where they were
if (_frozen) exitWith {};

if (_piloted) then {
    private _deltaTime         = _heli getVariable "bmkhs_deltaTime";
    private _inputLagValue     = getNumber (configOf _heli >> "BMKHS_HeliSim" >> "inputLagValue");
    private _fltControlLockout = _heli getVariable "bmkhs_flightControlLockOut";

    private _cyclicFwdAft      = _heli getVariable "bmkhs_inputPitch";
    private _cyclicLeftRight   = _heli getVariable "bmkhs_inputRoll";
    private _pedalLeftRight    = _heli getVariable "bmkhs_inputPedal";

    /////////////////////////////////////////////////////////////////////////////////////////////
    // Auto Pedal          /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    if (bmkhs_autoPedal) then {
        _pedalLeftRight = [_heli, _deltaTime, _pedalLeftRight] call bmkhs_fnc_inputAutoPedal;
    };
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Rate Command        /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    if (_ratePitch || _rateRoll) then {
        (_heli call BIS_fnc_getPitchBank) params ["_pitch", "_bank"];
        //Body rates: + nose down, + roll left - the cyclic's own sense
        (_heli getVariable "bmkhs_angVelModelSpace") params ["_pitchRate", "_rollRate"];
        private _onGround = [_heli] call bmkhs_fnc_stateOnGround;
        if (_ratePitch) then { _cyclicFwdAft    = [_heli, _deltaTime, "Pitch", _cyclicFwdAft,    bmkhs_pitchScheme == "kb", deg _pitchRate, _pitch, _onGround] call bmkhs_fnc_inputRateCommand };
        if (_rateRoll)  then { _cyclicLeftRight = [_heli, _deltaTime, "Roll",  _cyclicLeftRight, bmkhs_rollScheme  == "kb", deg _rollRate,  _bank,  _onGround] call bmkhs_fnc_inputRateCommand };
    };
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Flight Ctrl Lockout  /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    if (_fltControlLockout) then {
        if (
            (_cyclicFwdAft    < CENTER_TRIM_VAL && _cyclicFwdAft    > -CENTER_TRIM_VAL) &&
            (_cyclicLeftRight < CENTER_TRIM_VAL && _cyclicLeftRight > -CENTER_TRIM_VAL) &&
            (_pedalLeftRight  < CENTER_TRIM_VAL && _pedalLeftRight  > -CENTER_TRIM_VAL)
           ) then {
                _heli setVariable ["bmkhs_flightControlLockOut", false];
           };
           if (bmkhs_cyclicCenterTrimMode) then {
                _cyclicFwdAft    = 0.0;
                _cyclicLeftRight = 0.0;
           };

           if (bmkhs_pedalCenterTrimMode) then {
                _pedalLeftRight  = 0.0;
           };
    };
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Cyclic and Pedals    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    _cyclicFwdAft    = [_heli, "pitch", _cyclicFwdAft,    _inputLagValue] call bmkhs_fnc_actuator;
    _cyclicLeftRight = [_heli, "roll",  _cyclicLeftRight, _inputLagValue] call bmkhs_fnc_actuator;
    _pedalLeftRight  = [_heli, "yaw",   _pedalLeftRight,  _inputLagValue] call bmkhs_fnc_actuator;
    //systemChat format ["_cyclicFwdAft = %1 -- _cyclicLeftRight = %2 -- _pedalLeftRight = %3", _cyclicFwdAft, _cyclicLeftRight, _pedalLeftRight];

    _heli setVariable ["bmkhs_cyclicFwdAft",    _cyclicFwdAft];
    _heli setVariable ["bmkhs_cyclicLeftRight", _cyclicLeftRight];
    if (!_tailFrozen) then {
        _heli setVariable ["bmkhs_pedalLeftRight", _pedalLeftRight];
    };
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Collective           /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    private _collectiveValue = _heli getVariable "bmkhs_collectiveOutput";
    private _collectiveIn    = _heli getVariable "bmkhs_inputCollective";
    //A flight director vertical mode drives the lever itself (fn_fmc). Moving the collective takes
    //it back and uncouples the mode - an axis has to meet the lever first, so nothing jumps.
    private _fdModes = ((_heli getVariable "bmkhs_fmc") getOrDefault ["FlightDirector", createHashMap]) getOrDefault ["modes", []];
    private _fdColl  = ["ralt", "alt", "altp"] select {(_x in _fdModes) && {_heli getVariable ["bmkhs_fd_" + _x, false]}};
    //However the mode let go
    if (_fdColl isNotEqualTo []) then { _heli setVariable ["bmkhs_collectivePickup", true] };
    if ((_heli getVariable ["bmkhs_collectivePickup", false])
        && {_heli getVariable "bmkhs_inputCollectiveMoved"}
        && {bmkhs_keyboardCollective || {abs (_collectiveIn - _collectiveValue) < 0.03}}) then {
        { [_heli, _x] call bmkhs_fnc_fmcFdMode } forEach _fdColl;
        _heli setVariable ["bmkhs_collectivePickup", false];
    };
    if !(_heli getVariable ["bmkhs_collectivePickup", false]) then { _collectiveValue = _collectiveIn };
    _collectiveValue = [_heli, "collective", _collectiveValue, _inputLagValue] call bmkhs_fnc_actuator;
    _heli setVariable ["bmkhs_collectiveOutput", _collectiveValue];
};

/////////////////////////////////////////////////////////////////////////////////////////////
// To the rotors        /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//Pilot and trim, SAS, the holds and mixing
private _pitch = ([_heli getVariable "bmkhs_cyclicFwdAft", _heli getVariable "bmkhs_forceTrimPosPitch"] call bmkhs_fnc_inputGetInterp)
    + (_heli getVariable "bmkhs_fmcSasPitchOut") + (_heli getVariable "bmkhs_fmcAttHoldCycPitchOut") + (_heli getVariable "bmkhs_mixPitchOut");
private _roll  = ([_heli getVariable "bmkhs_cyclicLeftRight", _heli getVariable "bmkhs_forceTrimPosRoll"] call bmkhs_fnc_inputGetInterp)
    + (_heli getVariable "bmkhs_fmcSasRollOut") + (_heli getVariable "bmkhs_fmcAttHoldCycRollOut") + (_heli getVariable "bmkhs_mixRollOut");
private _yaw   = ([_heli getVariable "bmkhs_pedalLeftRight", _heli getVariable "bmkhs_forceTrimPosYaw"] call bmkhs_fnc_inputGetInterp)
    + (_heli getVariable "bmkhs_fmcSasYawOut") + (_heli getVariable "bmkhs_fmcHdgHoldPedalYawOut") + (_heli getVariable "bmkhs_mixYawOut");
private _coll  = (_heli getVariable "bmkhs_collectiveOutput") + (_heli getVariable "bmkhs_fmcAltHoldCollOut") + (_heli getVariable "bmkhs_mixCollOut");

_heli setVariable ["bmkhs_fltCtrlPitchOut", [_pitch, -1.0, 1.0] call BIS_fnc_clamp];
_heli setVariable ["bmkhs_fltCtrlRollOut",  [_roll,  -1.0, 1.0] call BIS_fnc_clamp];
_heli setVariable ["bmkhs_fltCtrlCollOut",  _coll];
if (!_tailFrozen) then {
    _heli setVariable ["bmkhs_fltCtrlYawOut", [_yaw, -1.0, 1.0] call BIS_fnc_clamp];
};
