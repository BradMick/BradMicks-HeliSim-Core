params ["_heli"];

//An always-on hold is not the pilot's to release
private _fmc   = _heli getVariable "bmkhs_fmc";
private _attOn = ((_fmc getOrDefault ["AttitudeHold", createHashMap]) getOrDefault ["alwaysOn", 0]) == 1;

if (_heli getVariable "bmkhs_altHoldActive" || {!_attOn && {_heli getVariable "bmkhs_attHoldActive"}}) then {
    [_heli, "holdModeDisengaged"] call bmkhs_fnc_utilNotify;
};

//De-activate attitude hold and set the reference back to 0
if (!_attOn) then {
    [_heli, "bmkhs_attHoldActive", false] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "bmkhs_attHoldDesiredVel", [0.0, 0.0]] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "bmkhs_attHoldDesiredAtt", [0.0, 0.0]] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "bmkhs_attHoldDesiredSpd", 0.0] call bmkhs_fnc_utilUpdateNetworkGlobal;
};

//De-activate altitude hold and set the reference back to 0
[_heli, "bmkhs_altHoldActive", false] call bmkhs_fnc_utilUpdateNetworkGlobal;
[_heli, "bmkhs_altHoldDesiredAlt", 0.0] call bmkhs_fnc_utilUpdateNetworkGlobal;
