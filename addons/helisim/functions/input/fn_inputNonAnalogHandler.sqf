params["_name", "_value"];
private _heli = vehicle player;
//Gate on HeliSim being initialised, not on an airframe class - Core is airframe-agnostic
if !(_heli getVariable ["bmkhs_initialised", false]) exitWith {};

//Keyboard cyclic and keyboard/gamepad pedal binds: [scheme, axis]
private _binds = createHashMapFromArray [
     ["bmkhs_kbPitchForward",       ["kb", "pitch"]]
    ,["bmkhs_kbPitchBackward",      ["kb", "pitch"]]
    ,["bmkhs_kbRollLeft",           ["kb", "roll"]]
    ,["bmkhs_kbRollRight",          ["kb", "roll"]]
    ,["bmkhs_kbGamepadPedalLeft",   ["kbGamepad", "pedal"]]
    ,["bmkhs_kbGamepadPedalRight",  ["kbGamepad", "pedal"]]
];
if (_name in _binds) exitWith {
    (_binds get _name) params ["_scheme", "_axis"];
    _heli setVariable [_name + "Out", [0.0, 1.0] select _value];
    //Last used has the axis
    missionNamespace setVariable ["bmkhs_" + _axis + "Scheme", _scheme];
};

if (_value) then {
    //When button pressed
    switch (_name) do {
        case "bmkhs_kbCollectiveUp": {
            bmkhs_keyboardCollective = true;
            _heli setVariable ["bmkhs_kbHeliCollectiveRaiseOut", 1.0];
            //systemChat format ["increasing collective!"];
        };
        case "bmkhs_kbCollectiveDn": {
            bmkhs_keyboardCollective = true;
            _heli setVariable ["bmkhs_kbHeliCollectiveLowerOut", 1.0];
            //systemChat format ["decreasing collective!"];
        };
    };
};

if !(_value) then {
    //When button releassed
    switch (_name) do {
        case "bmkhs_kbCollectiveUp": {
            bmkhs_keyboardCollective = true;
            _heli setVariable ["bmkhs_kbHeliCollectiveRaiseOut", 0.0];
            //systemChat format ["no longer increasing collective!"];
        };
        case "bmkhs_kbCollectiveDn": {
            bmkhs_keyboardCollective = true;
            _heli setVariable ["bmkhs_kbHeliCollectiveLowerOut", 0.0];
            //systemChat format ["no longer decreasing collective!"];
        };
    };
};
