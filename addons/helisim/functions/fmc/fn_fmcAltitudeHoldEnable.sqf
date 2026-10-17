params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

private _alt = (_heli getVariable "bmkhs_fmc") getOrDefault ["AltitudeHold", createHashMap];
if (count _alt == 0) exitWith {};

private _gndSpeed  = _heli getVariable "bmkhs_gndSpeed";
private _velClimb  = _heli getVariable "bmkhs_velClimb";
private _engage    = (_alt get "engageFpm") * FPM_TO_MPS;

if (_heli getVariable "bmkhs_altHoldActive" == false) then {
    //Collect required inputs
    private _curAltAGL = ASLToAGL getPosASL _heli # 2;
    if (_curAltAGL > 50) then {
        _curAltAGL = round ((_curAltAGL / 10) * 10);
    };
    private _curAltASL = round(((getPosASL _heli # 2) / 10) * 10);
    //Engages only near level flight
    if (abs _velClimb <= _engage) then {
        //The collective reference is required to determine when to deactivate alt hold. If the
        //pilot moves the collective > 0.25 inches up/down from the ref, alt hold will be
        //de-activate.
        [_heli, "bmkhs_altHoldCollRef", (_heli getVariable "bmkhs_collectiveOutput")] call bmkhs_fnc_utilUpdateNetworkGlobal;
        //Radar below its height and speed, barometric otherwise
        if (_curAltAGL < ((_alt get "radBelowFt") * FEET_TO_METERS) && _gndSpeed < ((_alt get "radBelowKts") * KNOTS_TO_MPS)) then {
            [_heli, "bmkhs_altHoldDesiredAlt", _curAltAGL] call bmkhs_fnc_utilUpdateNetworkGlobal;
            [_heli, "bmkhs_altHoldSubMode", "rad"] call bmkhs_fnc_utilUpdateNetworkGlobal;
        } else {
            [_heli, "bmkhs_altHoldDesiredAlt", _curAltASL] call bmkhs_fnc_utilUpdateNetworkGlobal;
            [_heli, "bmkhs_altHoldSubMode", "bar"] call bmkhs_fnc_utilUpdateNetworkGlobal;
        };
        //Activate altitude hold
        [_heli, "bmkhs_altHoldActive", true] call bmkhs_fnc_utilUpdateNetworkGlobal;
    };
} else {
    //Reset the desired altitude and de-activate altitude hold
    [_heli, "bmkhs_altHoldDesiredAlt", 0.0] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "bmkhs_altHoldActive", false] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "bmkhs_altHoldCollRef", 0.0] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "holdModeDisengaged"] call bmkhs_fnc_utilNotify;
};
