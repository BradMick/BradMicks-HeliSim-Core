params ["_heli"];

if (_heli getVariable "bmkhs_altHoldActive" || _heli getVariable "bmkhs_attHoldActive") then {
    [_heli, "holdModeDisengaged"] call bmkhs_fnc_utilNotify;
};

//De-activate attitude hold and set the reference back to 0
[_heli, "bmkhs_attHoldActive", false] call bmkhs_fnc_utilUpdateNetworkGlobal;
[_heli, "bmkhs_attHoldDesiredVel", [0.0, 0.0]] call bmkhs_fnc_utilUpdateNetworkGlobal;
[_heli, "bmkhs_attHoldDesiredAtt", [0.0, 0.0]] call bmkhs_fnc_utilUpdateNetworkGlobal;

//De-activate altitude hold and set the reference back to 0
[_heli, "bmkhs_altHoldActive", false] call bmkhs_fnc_utilUpdateNetworkGlobal;
[_heli, "bmkhs_altHoldDesiredAlt", 0.0] call bmkhs_fnc_utilUpdateNetworkGlobal;
