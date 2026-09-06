params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

private _gndSpeed = (_heli getVariable "bmkhs_gndSpeed") * KNOTS_TO_MPS;

if (_heli getVariable "bmkhs_attHoldActive" == false) then {
    //Position hold
    if (_gndSpeed <= POS_HOLD_SPEED_SWITCH) then {
        _heli setVariable ["bmkhs_attHoldSubMode",   "pos",         true];
        _heli setVariable ["bmkhs_attHoldDesiredPos", getPos _heli, true];
    };
    //Velocity hold
    //This needs to check if accelerating or decelerating...really it's
    //5 to 40 knots accelerating, 30 to 5 knots decelerating
    if (_gndSpeed > POS_HOLD_SPEED_SWITCH && _gndSpeed <= VEL_HOLD_SPEED_SWITCH_ACCEL) then {
        _heli setVariable ["bmkhs_attHoldSubMode", "vel", true];

        private _velX  = ((_heli getVariable "bmkhs_velModelSpaceNoWind") # 0) * -1.0;
        private _velY  =  (_heli getVariable "bmkhs_velModelSpaceNoWind") # 1;
        _heli setVariable ["bmkhs_attHoldDesiredVel", [_velX, _velY], true];
    };
    //Attitude hold
    if (_gndSpeed > VEL_HOLD_SPEED_SWITCH_ACCEL) then {
        _heli setVariable ["bmkhs_attHoldSubMode", "att", true];

        (_heli call BIS_fnc_getPitchBank)
            params ["_curPitch", "_curRoll"];
        _heli setVariable ["bmkhs_attHoldDesiredAtt", [_curPitch, _curRoll], true];
    };

    _heli setVariable ["bmkhs_attHoldActive", true, true];
} else {
    _heli setVariable ["bmkhs_attHoldActive", false, true];
    [_heli, "holdModeDisengaged"] call bmkhs_fnc_utilNotify;
};
