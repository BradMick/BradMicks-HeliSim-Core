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
private _ratePitch  = _piloted && {!_frozen} && {bmkhs_pitchScheme == "mouseGamepad" || {bmkhs_pitchScheme == "kb" && {!bmkhs_keyboardStickyPitch}}};
private _rateRoll   = _piloted && {!_frozen} && {bmkhs_rollScheme  == "mouseGamepad" || {bmkhs_rollScheme  == "kb" && {!bmkhs_keyboardStickyRoll}}};
if (!_ratePitch) then { _heli setVariable ["bmkhs_rateCommandPitchOn", false] };
if (!_rateRoll)  then { _heli setVariable ["bmkhs_rateCommandRollOn",  false] };

//Frozen controls stay where they were
if (_frozen) exitWith {};

//Auto pedal is the feet of a player without pedals - HOTAS pedals in use, it is ignored
private _autoPedal = bmkhs_autoPedal && {bmkhs_pedalScheme != "hotas"};

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
    if (_autoPedal) then {
        _pedalLeftRight = [_heli, _deltaTime, _pedalLeftRight] call bmkhs_fnc_inputAutoPedal;
    };
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Auto Taxi           /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    private _onGround = [_heli] call bmkhs_fnc_stateOnGround;
    private _kbTaxi   = _ratePitch && {bmkhs_pitchScheme == "kb"} && {bmkhs_autoTaxi} && {(_heli getVariable "bmkhs_numWheels") > 0};
    private _regime   = _heli getVariable "bmkhs_autoTaxiRegime";
    private _radAltFt = ((_heli getVariable "bmkhs_radAlt") * METERS_TO_FEET) max 0.0;
    //Leaving the wheels with the forward key held is a rolling takeoff. Up to AUTOTAXI_BLEND_FT the key blends into the rate command.
    private _climb    = _kbTaxi && {!_onGround} && {_radAltFt < AUTOTAXI_BLEND_FT}
        && {_regime == "climb" || {_regime == "takeoff" && {_cyclicFwdAft > 0.0}}};
    //On the wheels the pitch key is a taxi speed. Any other key held from the wheels keeps it until released.
    private _taxi     = _kbTaxi && {!_climb} && {_onGround || {_cyclicFwdAft != 0.0 && {_regime in ["taxi", "takeoff"]}}};
    //On the ground the mouse or gamepad is the stick itself
    private _stick = _ratePitch && {bmkhs_pitchScheme == "mouseGamepad"} && {_onGround};
    if (_taxi) then {
        _cyclicFwdAft = [_heli, _deltaTime, _cyclicFwdAft] call bmkhs_fnc_inputAutoTaxi;
    } else {
        _heli setVariable ["bmkhs_autoTaxiRegime", ["off", "climb"] select _climb];
    };
    if (_taxi || _stick) then {
        //The rate command takes the axis afresh at lift-off. Trim is neutral on the ground, as it leaves it.
        _ratePitch = false;
        _heli setVariable ["bmkhs_rateCommandPitchOn",   false];
        _heli setVariable ["bmkhs_rateCommandPitchHeld", 0.0];
        if (_onGround && {_heli getVariable "bmkhs_fmcTrimOn"}) then {
            _heli setVariable ["bmkhs_forceTrimPosPitch", [_heli getVariable "bmkhs_forceTrimPosPitch", 0.0, _deltaTime, RATECMD_TRIM_TIME] call bmkhs_fnc_utilSmooth];
        };
    };
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Rate Command        /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    if (_ratePitch || _rateRoll) then {
        (_heli call BIS_fnc_getPitchBank) params ["_pitch", "_bank"];
        //Body rates: + nose down, + roll left - the cyclic's own sense
        (_heli getVariable "bmkhs_angVelModelSpace") params ["_pitchRate", "_rollRate"];
        //Climbing out, the forward key's share of its rate grows with height; the rest holds the level-flight attitude
        private _share = [1.0, (_radAltFt / AUTOTAXI_BLEND_FT) ^ 2] select _climb;
        if (_ratePitch) then { _cyclicFwdAft    = [_heli, _deltaTime, "Pitch", _cyclicFwdAft,    bmkhs_pitchScheme == "kb", deg _pitchRate, _pitch, _onGround, _share, _heli getVariable "bmkhs_autoAttLevelPitch"] call bmkhs_fnc_inputRateCommand };
        if (_rateRoll)  then { _cyclicLeftRight = [_heli, _deltaTime, "Roll",  _cyclicLeftRight, bmkhs_rollScheme  == "kb", deg _rollRate,  _bank,  _onGround] call bmkhs_fnc_inputRateCommand };
    };
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Flight Ctrl Lockout  /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    if (_fltControlLockout) then {
        //The lockout is for the pilot's own controls - the feet are not locked out
        private _pedalOwn = [_pedalLeftRight, 0.0] select _autoPedal;
        if (
            (_cyclicFwdAft    < CENTER_TRIM_VAL && _cyclicFwdAft    > -CENTER_TRIM_VAL) &&
            (_cyclicLeftRight < CENTER_TRIM_VAL && _cyclicLeftRight > -CENTER_TRIM_VAL) &&
            (_pedalOwn        < CENTER_TRIM_VAL && _pedalOwn        > -CENTER_TRIM_VAL)
           ) then {
                _heli setVariable ["bmkhs_flightControlLockOut", false];
           };
           if (bmkhs_cyclicCenterTrimMode) then {
                _cyclicFwdAft    = 0.0;
                _cyclicLeftRight = 0.0;
           };

           if (bmkhs_pedalCenterTrimMode && {!_autoPedal}) then {
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
//While the feet are not flying, the foot follows the pedal - they take it from where it is
if (!_piloted || !_autoPedal) then {
    _heli setVariable ["bmkhs_autoPedalPos", [_heli getVariable "bmkhs_pedalLeftRight", _heli getVariable "bmkhs_forceTrimPosYaw"] call bmkhs_fnc_inputGetInterp];
};
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
