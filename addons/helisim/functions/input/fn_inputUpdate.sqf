/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputUpdate

Description:
    Handles keyboard and HOTAS input for the simulation.

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

if (currentPilot _heli != player || !local _heli) exitWith {};

private _paused             = isNull findDisplay 49;
private _chatting           = isNull findDisplay 24;
private _inDialog           = !dialog;
private _isZeus             = isNull findDisplay 312;
private _inMap              = !visibleMap;
private _inInventory        = isNull findDisplay 602;
//Set by whatever is applying a loadout, to suppress pilot input while it works
private _mplannerApplying   = _heli getVariable ["bmkhs_inputLockout", false];

private _isPlaying          = isGameFocused && _paused && _chatting && _inDialog && _isZeus && _inMap && _inInventory && !bmkhs_lastFrameGetIn && !_mplannerApplying;

private _config             = configOf _heli >> "BMKHS_HeliSim";
private _configVehicles     = configOf _heli;
private _inputLagValue      = getNumber (_config >> "inputLagValue");

private _hydFailure         = false;
private _tailRtrFixed       = false;

private _deltaTime          = _heli getVariable "bmkhs_deltaTime";

//Keyboard
private _kbStickyInterupt   = _heli getVariable "bmkhs_kbStickyInterupt";
private _fltControlLockout  = _heli getVariable "bmkhs_flightControlLockOut";

//Auto-pedal hover->nose-to-tail handover speed: 12.35 m/s = ~24kts GS (5.14444 m/s = 10kts, x2.4).
//Below this the pedals hold heading; above it they hold the velocity vector on the nose.
private _kbYawSwitchVel     = 5.14444 * 2.4;
private _yawBreakout        = false;
private _kbPedalLeftRight   = _heli getVariable "bmkhs_kbPedalLeftRight";

private _priHydPumpDamage   = [_heli, "priPump"] call bmkhs_fnc_damageGet;
//No systems modelled -> full pressure, not the failure threshold
private _priHydPSI          = _heli getVariable ["bmkhs_priHydPsi", 3000];

private _utilHydPumpDamage  = [_heli, "utilPump"] call bmkhs_fnc_damageGet;
private _utilHydPSI         = _heli getVariable ["bmkhs_utilHydPsi", 3000];
private _utilLevel_pct      = _heli getVariable ["bmkhs_utilLevel_pct", 1.0];

private _emerHydOn          = _heli getVariable ["bmkhs_emerHydOn", false];
private _apuOn              = _heli getVariable ["bmkhs_apuOn", true];

/////////////////////////////////////////////////////////////////////////////////////////////
// Cyclic & Pedal Input /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
private _heliCyclicFwdOut   = _heli getVariable "bmkhs_heliCyclicForwardOut";
private _heliCyclicBackOut  = _heli getVariable "bmkhs_heliCyclicBackwardOut";
private _cyclicFwdAft       = _heliCyclicFwdOut - _heliCyclicBackOut;
_cyclicFwdAft               = [_cyclicFwdAft, -1.0, 1.0] call BIS_fnc_clamp;
//systemChat format ["_cyclicFwdAft = %1", _cyclicFwdAft toFixed 2];

private _heliCyclicLeftOut  = _heli getVariable "bmkhs_heliCyclicLeftOut";
private _heliCyclicRightOut = _heli getVariable "bmkhs_heliCyclicRightOut";
private _cyclicLeftRight    = _heliCyclicLeftOut - _heliCyclicRightOut;
_cyclicLeftRight            = [_cyclicLeftRight, -1.0, 1.0] call BIS_fnc_clamp;
//systemChat format ["_cyclicLeftRight = %1", _cyclicLeftRight toFixed 2];

private _heliRudderLeftOut  = _heli getVariable "bmkhs_heliRudderLeftOut";
private _heliRudderRightOut = _heli getVariable "bmkhs_heliRudderRightOut";
private _pedalLeftRight     = _heliRudderRightOut - _heliRudderLeftOut;
_pedalLeftRight             = [_pedalLeftRight, -1.0, 1.0] call BIS_fnc_clamp;
//systemChat format ["_pedalLeftRight = %1", _pedalLeftRight toFixed 2];

if (!_isPlaying || (freeLook && bmkhs_mouseAsJoystick)) then {
    _cyclicFwdAft      = 0.0;
    _cyclicLeftRight   = 0.0;
};

//Cyclic Pitch
if (bmkhs_keyboardStickyPitch) then {
    private _cyclicPitchValue     = _heli getVariable "bmkhs_cyclicPitchValue";
    private _prevCyclicPitchValue = _heli getVariable "bmkhs_prevCyclicPitchValue";

    if (_kbStickyInterupt) then {
        _cyclicFwdAft         = [_cyclicFwdAft, _prevCyclicPitchValue] call bmkhs_fnc_inputGetInterp;
    } else {
        if (_cyclicFwdAft > 0.1) then {
            _cyclicPitchValue = _cyclicPitchValue + 0.01;
        };
        if (_cyclicFwdAft < -0.1) then {
            _cyclicPitchValue = _cyclicPitchValue - 0.01;
        };

       //_cyclicPitcValue =

        _cyclicFwdAft         = [_cyclicPitchValue, -1.0, 1.0] call BIS_fnc_clamp;
        _heli setVariable ["bmkhs_cyclicPitchValue",    [_cyclicPitchValue, -1.0, 1.0] call BIS_fnc_clamp];
        _heli setVariable ["bmkhs_prevCyclicPitchValue", _cyclicPitchValue];
    };
};
//Cyclic Roll
if (bmkhs_keyboardStickyRoll) then {
    private _cyclicRollValue     = _heli getVariable "bmkhs_cyclicRollValue";
    private _prevCyclicRollValue = _heli getVariable "bmkhs_prevCyclicRollValue";

    if (_kbStickyInterupt) then {
        _cyclicLeftRight     = [_cyclicLeftRight, _prevCyclicRollValue] call bmkhs_fnc_inputGetInterp;
    } else {
        if (_cyclicLeftRight > 0.1) then {
            _cyclicRollValue = _cyclicRollValue + 0.01;
        };
        if (_cyclicLeftRight < -0.1) then {
            _cyclicRollValue = _cyclicRollValue - 0.01;
        };

        _cyclicLeftRight     = [_cyclicRollValue, -1.0, 1.0] call BIS_fnc_clamp;
        _heli setVariable ["bmkhs_cyclicRollValue",    [_cyclicRollValue, -1.0, 1.0] call BIS_fnc_clamp];
        _heli setVariable ["bmkhs_prevCyclicRollValue", _cyclicRollValue];
    };
};
//Pedal yaw
if (bmkhs_keyboardStickyYaw && !bmkhs_autoPedal) then {
    private _pedalYawValue     = _heli getVariable "bmkhs_pedalYawValue";
    private _prevPedalYawValue = _heli getVariable "bmkhs_prevPedalYawValue";

    if (_kbStickyInterupt) then {
        _pedalLeftRight    = [_pedalLeftRight, _prevPedalYawValue] call bmkhs_fnc_inputGetInterp;
    } else {
        if (_pedalLeftRight > 0.1) then {
            _pedalYawValue = _pedalYawValue + 0.01;
        };
        if (_pedalLeftRight < -0.1) then {
            _pedalYawValue = _pedalYawValue - 0.01;
        };

        _pedalLeftRight        = [_pedalYawValue, -1.0, 1.0] call BIS_fnc_clamp;
        _heli setVariable ["bmkhs_pedalYawValue", [_pedalYawValue, -1.0, 1.0] call BIS_fnc_clamp];
        _heli setVariable ["bmkhs_prevPedalYawValue", _pedalYawValue];
    };
};
/////////////////////////////////////////////////////////////////////////////////////////////
// Auto Pedal          /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//Player accommodation: the machine pilot's FEET, available to a HUMAN pilot independently of
//whether Preston is flying. Same function fn_preston calls - see prestonAi/fn_prestonPedal.sqf.
if (bmkhs_autoPedal) then {
    ([_heli, _deltaTime, _pedalLeftRight, _kbPedalLeftRight, _kbYawSwitchVel]
        call bmkhs_fnc_prestonPedal) params ["_pedalLeftRight", "_yawBreakout"];
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
_cyclicFwdAft    = [_heli, "pitch", _cyclicFwdAft,    _inputLagValue] call bmkhs_fnc_actuator;
_cyclicLeftRight = [_heli, "roll",  _cyclicLeftRight, _inputLagValue] call bmkhs_fnc_actuator;
_pedalLeftRight  = [_heli, "yaw",   _pedalLeftRight,  _inputLagValue] call bmkhs_fnc_actuator;

//systemChat format ["_cyclicFwdAft = %1 -- _cyclicLeftRight = %2 -- _pedalLeftRight = %3", _cyclicFwdAft, _cyclicLeftRight, _pedalLeftRight];
/////////////////////////////////////////////////////////////////////////////////////////////
// Collective           /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//Keyboard collective
private _keyCollectiveUp = _heli getVariable "bmkhs_kbHeliCollectiveRaiseOut";
private _keyCollectiveDn = _heli getVariable "bmkhs_kbHeliCollectiveLowerOut";
//Joystick collective
private _joyCollectiveUp = _heli getVariable "bmkhs_heliCollectiveRaiseOut";
private _joyCollectiveDn = _heli getVariable "bmkhs_heliCollectiveLowerOut";

//What has supply is the aircraft's declaration to make, not this function's - it names
//the circuits, Core answers. Both default true so an airframe declaring no hydraulics
//flies rather than locking up.
if !(_heli getVariable ["bmkhs_fltCtrlsSupplied", true]) then {
    _hydFailure = true;
};

//Lost either by having no hydraulic authority to move it or no drive turning it.
private _tailHyd    = _heli getVariable ["bmkhs_tailRtrSupplied", true];
private _tailDriven = _heli getVariable ["bmkhs_tailRtrDriven",   true];
if (!_tailHyd || !_tailDriven) then {
    _tailRtrFixed = true;
};

if (!_hydFailure || _emerHydOn) then {
    private _collectiveValue = _heli getVariable "bmkhs_collectiveOutput";
    if (bmkhs_keyboardCollective) then {
        if (_keyCollectiveUp > 0.1) then { _collectiveValue = _collectiveValue + ((1.0 / 4.0) * _deltaTime); };
        if (_keyCollectiveDn > 0.1) then { _collectiveValue = _collectiveValue - ((1.0 / 4.0) * _deltaTime); };
        _collectiveValue = (round (_collectiveValue / 0.005)) * 0.005;
        _collectiveValue = [_collectiveValue, 0.0, 1.0] call bis_fnc_clamp;
        //systemChat format ["KB collective! -- %1", (_heli getVariable "bmkhs_collectiveOutput") toFixed 3];
    } else {
        _collectiveValue = _joyCollectiveUp - _joyCollectiveDn;
        _collectiveValue = [_collectiveValue, -1.0, 1.0] call BIS_fnc_clamp;
        _collectiveValue = linearConversion[ -1.0, 1.0, _collectiveValue, 0.0, 1.0];
        //systemChat format ["HOTAS collective! -- %1", (_heli getVariable "bmkhs_collectiveOutput") toFixed 3];
    };
    if (_isPlaying) then {
        //Hydraulic actuator lag on collective (crisp with FMC/hydraulics good, lagged when not) -
        //same treatment as cyclic/pedal above.
        _collectiveValue = [_heli, "collective", _collectiveValue, _inputLagValue] call bmkhs_fnc_actuator;
        _heli setVariable ["bmkhs_collectiveOutput", _collectiveValue];
    };
};

/////////////////////////////////////////////////////////////////////////////////////////////
// Cyclic and Pedals    /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
if (_isZeus && (!_hydFailure || _emerHydOn)) then {
    if (bmkhs_mouseAsJoystick) then {
        _heli setVariable ["bmkhs_cyclicFwdAft",    _cyclicFwdAft    * bmkhs_mouseSense];
        _heli setVariable ["bmkhs_cyclicLeftRight", _cyclicLeftRight * bmkhs_mouseSense];
    } else {
        _heli setVariable ["bmkhs_cyclicFwdAft",    _cyclicFwdAft];
        _heli setVariable ["bmkhs_cyclicLeftRight", _cyclicLeftRight];
    };
    if (!_tailRtrFixed) then {
        _heli setVariable ["bmkhs_pedalLeftRight", _pedalLeftRight];
    };
} else {
    _heli setVariable ["bmkhs_cyclicFwdAft",     0.0];
    _heli setVariable ["bmkhs_cyclicLeftRight",  0.0];
    _heli setVariable ["bmkhs_pedalLeftRight",   0.0];
};

if (bmkhs_lastFrameGetIn) then {
    bmkhs_lastFrameGetIn = false;
};
