/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputVariables

Description:
    Initialises the pilot input state - cyclic, pedal and collective.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

//Cyclic and pedal input, per control scheme
{
    private _scheme = _x;
    {
        _heli setVariable ["bmkhs_" + _scheme + _x + "Out", 0.0];
    } forEach ["PitchForward", "PitchBackward", "RollLeft", "RollRight", "PedalLeft", "PedalRight"];
} forEach ["kb", "hotas", "mouse"];
//Collective input
_heli setVariable ["bmkhs_heliCollectiveRaiseOut", 0.0];
_heli setVariable ["bmkhs_heliCollectiveLowerOut", 0.0];

//What the player is asking for
_heli setVariable ["bmkhs_inputPitch",            0.0];
_heli setVariable ["bmkhs_inputRoll",             0.0];
_heli setVariable ["bmkhs_inputPedal",            0.0];
_heli setVariable ["bmkhs_inputCollective",       0.0];
_heli setVariable ["bmkhs_inputCollectiveMoved",  false];
//What the rotors get
_heli setVariable ["bmkhs_fltCtrlPitchOut",       0.0];
_heli setVariable ["bmkhs_fltCtrlRollOut",        0.0];
_heli setVariable ["bmkhs_fltCtrlYawOut",         0.0];
_heli setVariable ["bmkhs_fltCtrlCollOut",        0.0];

_heli setVariable ["bmkhs_cyclicFwdAft",          0.0];
_heli setVariable ["bmkhs_cyclicPitchValue",      0.0];
_heli setVariable ["bmkhs_prevCyclicPitchValue",  0.0];

_heli setVariable ["bmkhs_cyclicLeftRight",       0.0];
_heli setVariable ["bmkhs_cyclicRollValue",       0.0];
_heli setVariable ["bmkhs_prevCyclicRollValue",   0.0];

_heli setVariable ["bmkhs_pedalLeftRight",        0.0];
_heli setVariable ["bmkhs_pedalYawValue",         0.0];
_heli setVariable ["bmkhs_prevPedalYawValue",     0.0];

_heli setVariable ["bmkhs_collectiveOutput",      0.0];

//Input state
_heli setVariable ["bmkhs_kbStickyInterupt",     false];
_heli setVariable ["bmkhs_flightControlLockOut", false];
_heli setVariable ["bmkhs_kbHeliCollectiveRaiseOut", 0.0];
_heli setVariable ["bmkhs_kbHeliCollectiveLowerOut", 0.0];

//Keyboard rate command: the attitude held, how long the key has been down, the rate asked for, the gains
{
    _heli setVariable ["bmkhs_rateCommand" + _x + "Ref",  0.0];
    _heli setVariable ["bmkhs_rateCommand" + _x + "Held", 0.0];
    _heli setVariable ["bmkhs_rateCommand" + _x + "On",   false];
    _heli setVariable ["bmkhs_rateCommand" + _x + "Rate", 0.0];
    private _gains = getArray (_config >> ("pidRateCommand" + _x));
    if (count _gains != 4) then {
        format ["[BMKHS] INPUT CONFIG ERROR: pidRateCommand%1[] {kp, ki, kd, limit} is missing. The keyboard cyclic does nothing on that axis.", _x] call bmkhs_fnc_utilConfigError;
        _gains = [0, 0, 0, 0];
    };
    _heli setVariable ["bmkhs_pid_rateCommand" + _x, _gains call bmkhs_fnc_pidCreate];
} forEach ["Pitch", "Roll"];

//Auto pedal
_heli setVariable ["bmkhs_autoPedalHdg",       getDir _heli];
_heli setVariable ["bmkhs_autoPedalRegime",    "hdg"];   //hdg | ntt | aero | taxi (live regime)
_heli setVariable ["bmkhs_autoPedalPos",       0.0];     //where the feet have the pedals
_heli setVariable ["bmkhs_autoPedalTrim",      0.0];     //pedal before the pilot's turn
_heli setVariable ["bmkhs_autoPedalHdgSet",    true];    //hover heading captured
_heli setVariable ["bmkhs_autoPedalRawPrev",   0.0];     //pilot's raw pedal, last frame
_heli setVariable ["bmkhs_autoPedalSprung",    false];   //analog pedal springing back - released
