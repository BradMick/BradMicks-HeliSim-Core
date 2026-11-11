params["_name", "_value"];
private _heli = vehicle player;
//Gate on HeliSim being initialised, not on an airframe class - Core is airframe-agnostic
if !(_heli getVariable ["bmkhs_initialised", false]) exitWith {};

//Keyboard cyclic and pedal binds: axis
private _binds = createHashMapFromArray [
     ["bmkhs_kbPitchForward",  "pitch"]
    ,["bmkhs_kbPitchBackward", "pitch"]
    ,["bmkhs_kbRollLeft",      "roll"]
    ,["bmkhs_kbRollRight",     "roll"]
    ,["bmkhs_kbPedalLeft",     "pedal"]
    ,["bmkhs_kbPedalRight",    "pedal"]
];
if (_name in _binds) exitWith {
    _heli setVariable [_name + "Out", [0.0, 1.0] select _value];
    //Last used has the axis
    missionNamespace setVariable ["bmkhs_" + (_binds get _name) + "Scheme", "kb"];
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
