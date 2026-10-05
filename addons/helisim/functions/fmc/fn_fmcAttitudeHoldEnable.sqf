params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

private _att = (_heli getVariable "bmkhs_fmc") getOrDefault ["AttitudeHold", createHashMap];
if (count _att == 0) exitWith {};

if (_heli getVariable "bmkhs_attHoldActive" == false) then {
    //References for the sub-mode fn_fmcAttitudeHold has already chosen - one rule for both
    switch (_heli getVariable "bmkhs_attHoldSubMode") do {
        case "pos": {
            _heli setVariable ["bmkhs_attHoldDesiredPos", getPos _heli, true];
        };
        case "vel": {
            private _velX  = ((_heli getVariable "bmkhs_velModelSpaceNoWind") # 0) * -1.0;
            private _velY  =  (_heli getVariable "bmkhs_velModelSpaceNoWind") # 1;
            _heli setVariable ["bmkhs_attHoldDesiredVel", [_velX, _velY], true];
        };
        case "att": {
            (_heli call BIS_fnc_getPitchBank)
                params ["_curPitch", "_curRoll"];
            _heli setVariable ["bmkhs_attHoldDesiredAtt", [_curPitch, _curRoll], true];
        };
    };

    _heli setVariable ["bmkhs_attHoldActive", true, true];
} else {
    _heli setVariable ["bmkhs_attHoldActive", false, true];
    [_heli, "holdModeDisengaged"] call bmkhs_fnc_utilNotify;
};
