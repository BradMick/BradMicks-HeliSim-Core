params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

private _gndSpeed  = (_heli getVariable "bmkhs_gndSpeed") * KNOTS_TO_MPS;
private _velClimb  = (_heli getVariable "bmkhs_velClimb") * FPM_TO_MPS;

if (_heli getVariable "bmkhs_altHoldActive" == false) then {
    //Collect required inputs
    private _curAltAGL = ASLToAGL getPosASL _heli # 2;
    if (_curAltAGL > 50) then {
        _curAltAGL = round ((_curAltAGL / 10) * 10);
    };
    private _curAltASL = round(((getPosASL _heli # 2) / 10) * 10);
    //If the vertical velocity is <= 200fpm and >= -200fpm, altitude hold can be engaged
    if (_velClimb <= 1.016 && _velClimb >= -1.016) then {
        //The collective reference is required to determine when to deactivate alt hold. If the
        //pilot moves the collective > 0.25 inches up/down from the ref, alt hold will be
        //de-activate.
        [_heli, "bmkhs_altHoldCollRef", (_heli getVariable "bmkhs_collectiveOutput")] call bmkhs_fnc_utilUpdateNetworkGlobal;
        //If the helicopters radar altitude is < 1428ft (435.25m) and current velocity is < 40kts
        //then set the desired altitude to the current AGL altitude, otherwise set it to the
        //current ASL altitude.
        if (_curAltAGL < 435.254 && _gndSpeed < 20.577) then {
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
