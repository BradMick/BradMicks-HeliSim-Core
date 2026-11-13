params ["_name", "_value", ["_heli", vehicle player]];
//Gate on HeliSim being initialised, not on an airframe class - Core is airframe-agnostic
if !(_heli getVariable ["bmkhs_initialised", false]) exitWith {};

//Flight director targets. The value is a fraction, 0..1, of the target's declared range - so
//an axis and a cockpit knob are the same input.
private _fdTarget = createHashMapFromArray [
    ["bmkhs_fdRaltTarget", "ralt"], ["bmkhs_fdAltTarget", "alt"], ["bmkhs_fdAltpTarget", "altp"],
    ["bmkhs_fdIasTarget", "ias"], ["bmkhs_fdHdgTarget", "hdg"]
];
if (_name in _fdTarget) exitWith {
    private _t = ((((_heli getVariable "bmkhs_fmc") getOrDefault ["FlightDirector", createHashMap]) getOrDefault ["targets", createHashMap])
        getOrDefault [_fdTarget get _name, []]);
    if (_t isNotEqualTo []) then {
        [_heli, _fdTarget get _name, linearConversion [0, 1, _value, _t select 0, _t select 1, true]] call bmkhs_fnc_fmcFdTarget;
    };
};

//Cyclic and pedal binds: [scheme, axis]
private _binds = createHashMapFromArray [
     ["bmkhs_hotasGamepadPitchForward",  ["hotasGamepad", "pitch"]]
    ,["bmkhs_hotasGamepadPitchBackward", ["hotasGamepad", "pitch"]]
    ,["bmkhs_hotasGamepadRollLeft",      ["hotasGamepad", "roll"]]
    ,["bmkhs_hotasGamepadRollRight",     ["hotasGamepad", "roll"]]
    ,["bmkhs_hotasGamepadPedalLeft",     ["hotasGamepad", "pedal"]]
    ,["bmkhs_hotasGamepadPedalRight",    ["hotasGamepad", "pedal"]]
    ,["bmkhs_mousePitchForward",  ["mouse", "pitch"]]
    ,["bmkhs_mousePitchBackward", ["mouse", "pitch"]]
    ,["bmkhs_mouseRollLeft",      ["mouse", "roll"]]
    ,["bmkhs_mouseRollRight",     ["mouse", "roll"]]
    ,["bmkhs_mousePedalLeft",     ["mouse", "pedal"]]
    ,["bmkhs_mousePedalRight",    ["mouse", "pedal"]]
];
if (_name in _binds) exitWith {
    (_binds get _name) params ["_scheme", "_axis"];
    //Freelook has the mouse
    if (_scheme == "mouse" && {freeLook}) exitWith { _heli setVariable [_name + "Out", 0.0] };

    private _devices = [];
    {
        if ("JOYSTICK_AXIS" in (_x select 0 select 1)) then {
            _devices pushBackUnique _x;
        };
    } forEach actionKeysEx _name;

    private _out = if (_devices isEqualTo []) then { 1.0; } else {1.0 / (count _devices); };
    _out         = linearConversion [0.0, _out, _value, 0.0, 1.0, true];
    _heli setVariable [_name + "Out", _out];
    //Last used has the axis
    missionNamespace setVariable ["bmkhs_" + _axis + "Scheme", _scheme];
};

private _heliCollectiveRaiseOut = _heli getVariable "bmkhs_heliCollectiveRaiseOut";
private _heliCollectiveLowerOut = _heli getVariable "bmkhs_heliCollectiveLowerOut";

//When button pressed
switch (_name) do {
    case "bmkhs_collectiveUp": {
        bmkhs_keyboardCollective = false;

        private _heliCollectiveRaiseDevices = [];
        {
            if ("JOYSTICK_AXIS" in (_x select 0 select 1)) then {
                _heliCollectiveRaiseDevices pushBackUnique _x;
            };
        } forEach actionKeysEx "bmkhs_collectiveUp";

        _heliCollectiveRaiseOut = if (_heliCollectiveRaiseDevices isEqualTo []) then { 1.0; } else {1.0 / (count _heliCollectiveRaiseDevices); };
        _heliCollectiveRaiseOut = linearConversion [0.0, _heliCollectiveRaiseOut, _value, 0.0, 1.0, true];

        _heli setVariable ["bmkhs_heliCollectiveRaiseOut", _heliCollectiveRaiseOut];
    };
    case "bmkhs_collectiveDn": {
        bmkhs_keyboardCollective = false;

        private _heliCollectiveLowerDevices = [];
        {
            if ("JOYSTICK_AXIS" in (_x select 0 select 1)) then {
                _heliCollectiveLowerDevices pushBackUnique _x;
            };
        } forEach actionKeysEx "bmkhs_collectiveDn";

        _heliCollectiveLowerOut = if (_heliCollectiveLowerDevices isEqualTo []) then { 1.0; } else {1.0 / (count _heliCollectiveLowerDevices); };
        _heliCollectiveLowerOut = linearConversion [0.0, _heliCollectiveLowerOut, _value, 0.0, 1.0, true];

        _heli setVariable ["bmkhs_heliCollectiveLowerOut", _heliCollectiveLowerOut];
    };
};
