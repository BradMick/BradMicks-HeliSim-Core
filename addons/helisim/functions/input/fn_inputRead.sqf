/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputRead

Description:
    Reads the player's cyclic, pedals and collective and publishes them as bmkhs_input*.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    True when this player is flying the aircraft [Bool]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

if (currentPilot _heli != player || !local _heli) exitWith {false};

private _paused             = isNull findDisplay 49;
private _chatting           = isNull findDisplay 24;
private _inDialog           = !dialog;
private _isZeus             = isNull findDisplay 312;
private _inMap              = !visibleMap;
private _inInventory        = isNull findDisplay 602;
//Set by whatever is applying a loadout, to suppress pilot input while it works
private _mplannerApplying   = _heli getVariable ["bmkhs_inputLockout", false];

private _isPlaying          = isGameFocused && _paused && _chatting && _inDialog && _isZeus && _inMap && _inInventory && !bmkhs_lastFrameGetIn && !_mplannerApplying;

private _deltaTime          = _heli getVariable "bmkhs_deltaTime";

//Keyboard sticky
private _kbStickyInterupt   = _heli getVariable "bmkhs_kbStickyInterupt";

/////////////////////////////////////////////////////////////////////////////////////////////
// Cyclic & Pedal Input /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//Each axis reads the control scheme last used on it
private _pitchIn            = "bmkhs_" + bmkhs_pitchScheme + "Pitch";
private _rollIn             = "bmkhs_" + bmkhs_rollScheme  + "Roll";
private _pedalIn            = "bmkhs_" + bmkhs_pedalScheme + "Pedal";

private _heliCyclicFwdOut   = _heli getVariable (_pitchIn + "ForwardOut");
private _heliCyclicBackOut  = _heli getVariable (_pitchIn + "BackwardOut");
private _cyclicFwdAft       = _heliCyclicFwdOut - _heliCyclicBackOut;
_cyclicFwdAft               = [_cyclicFwdAft, -1.0, 1.0] call BIS_fnc_clamp;
//systemChat format ["_cyclicFwdAft = %1", _cyclicFwdAft toFixed 2];

private _heliCyclicLeftOut  = _heli getVariable (_rollIn + "LeftOut");
private _heliCyclicRightOut = _heli getVariable (_rollIn + "RightOut");
private _cyclicLeftRight    = _heliCyclicLeftOut - _heliCyclicRightOut;
_cyclicLeftRight            = [_cyclicLeftRight, -1.0, 1.0] call BIS_fnc_clamp;
//systemChat format ["_cyclicLeftRight = %1", _cyclicLeftRight toFixed 2];

private _heliRudderLeftOut  = _heli getVariable (_pedalIn + "LeftOut");
private _heliRudderRightOut = _heli getVariable (_pedalIn + "RightOut");
private _pedalLeftRight     = _heliRudderRightOut - _heliRudderLeftOut;
_pedalLeftRight             = [_pedalLeftRight, -1.0, 1.0] call BIS_fnc_clamp;
//systemChat format ["_pedalLeftRight = %1", _pedalLeftRight toFixed 2];

if (!_isPlaying) then {
    _cyclicFwdAft      = 0.0;
    _cyclicLeftRight   = 0.0;
};
//Freelook has the mouse
if (freeLook) then {
    if (bmkhs_pitchScheme == "mouseGamepad") then { _cyclicFwdAft    = 0.0 };
    if (bmkhs_rollScheme  == "mouseGamepad") then { _cyclicLeftRight = 0.0 };
    if (bmkhs_pedalScheme == "mouseGamepad") then { _pedalLeftRight  = 0.0 };
};

//Cyclic Pitch
if (bmkhs_keyboardStickyPitch && {bmkhs_pitchScheme == "kb"}) then {
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
if (bmkhs_keyboardStickyRoll && {bmkhs_rollScheme == "kb"}) then {
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
if (bmkhs_keyboardStickyYaw && {bmkhs_pedalScheme == "kb"} && {!bmkhs_autoPedal}) then {
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

//Mouse sensitivity, on the axes the mouse has
if (bmkhs_pitchScheme == "mouseGamepad") then { _cyclicFwdAft    = _cyclicFwdAft    * bmkhs_mouseGamepadSense };
if (bmkhs_rollScheme  == "mouseGamepad") then { _cyclicLeftRight = _cyclicLeftRight * bmkhs_mouseGamepadSense };
if (bmkhs_pedalScheme == "mouseGamepad") then { _pedalLeftRight  = _pedalLeftRight  * bmkhs_mouseGamepadSense };

//Zeus has the screen
if (!_isZeus) then {
    _cyclicFwdAft    = 0.0;
    _cyclicLeftRight = 0.0;
    _pedalLeftRight  = 0.0;
};

_heli setVariable ["bmkhs_inputPitch", _cyclicFwdAft];
_heli setVariable ["bmkhs_inputRoll",  _cyclicLeftRight];
_heli setVariable ["bmkhs_inputPedal", _pedalLeftRight];

/////////////////////////////////////////////////////////////////////////////////////////////
// Collective           /////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//Not playing, the lever is asked to stay where it is
private _collectiveValue = _heli getVariable "bmkhs_collectiveOutput";
private _collectiveMoved = false;
if (bmkhs_keyboardCollective) then {
    private _keyCollectiveUp = _heli getVariable "bmkhs_kbHeliCollectiveRaiseOut";
    private _keyCollectiveDn = _heli getVariable "bmkhs_kbHeliCollectiveLowerOut";
    if (_isPlaying) then {
        //The keys move the lever from where it is
        if (_keyCollectiveUp > 0.1) then { _collectiveValue = _collectiveValue + ((1.0 / 4.0) * _deltaTime); };
        if (_keyCollectiveDn > 0.1) then { _collectiveValue = _collectiveValue - ((1.0 / 4.0) * _deltaTime); };
        _collectiveValue = (round (_collectiveValue / 0.005)) * 0.005;
        _collectiveValue = [_collectiveValue, 0.0, 1.0] call bis_fnc_clamp;
        _collectiveMoved = _keyCollectiveUp > 0.1 || _keyCollectiveDn > 0.1;
    };
    //systemChat format ["KB collective! -- %1", (_heli getVariable "bmkhs_collectiveOutput") toFixed 3];
} else {
    private _joyCollectiveUp = _heli getVariable "bmkhs_heliCollectiveRaiseOut";
    private _joyCollectiveDn = _heli getVariable "bmkhs_heliCollectiveLowerOut";
    private _axis = _joyCollectiveUp - _joyCollectiveDn;
    _axis = [_axis, -1.0, 1.0] call BIS_fnc_clamp;
    _axis = linearConversion[ -1.0, 1.0, _axis, 0.0, 1.0];
    private _axisPrev = _heli getVariable ["bmkhs_collectiveAxisPrev", _axis];
    _heli setVariable ["bmkhs_collectiveAxisPrev", _axis];
    if (_isPlaying) then {
        _collectiveValue = _axis;
        _collectiveMoved = abs (_axis - _axisPrev) > 0.0005;
    };
    //systemChat format ["HOTAS collective! -- %1", (_heli getVariable "bmkhs_collectiveOutput") toFixed 3];
};
_heli setVariable ["bmkhs_inputCollective",      _collectiveValue];
_heli setVariable ["bmkhs_inputCollectiveMoved", _collectiveMoved];

if (bmkhs_lastFrameGetIn) then {
    bmkhs_lastFrameGetIn = false;
};

true
