#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

if (currentPilot _heli != player || !local _heli) exitWith {};

private _cyclicFwdAft    = _heli getVariable "bmkhs_cyclicFwdAft";
private _cyclicLeftRight = _heli getVariable "bmkhs_cyclicLeftRight";
private _pedalLeftRight  = _heli getVariable "bmkhs_pedalLeftRight";

if (bmkhs_cyclicCenterTrimMode) then {
    while {_cyclicFwdAft > CENTER_TRIM_VAL || _cyclicFwdAft < -CENTER_TRIM_VAL} do {
        //systemChat format ["Pitch Locked out! Return Pitch to Center!"];
        _heli setVariable ["bmkhs_flightControlLockOut", true];
    };

    while {_cyclicLeftRight > CENTER_TRIM_VAL || _cyclicLeftRight < -CENTER_TRIM_VAL} do {
        //systemChat format ["Roll Locked out! Return Roll to Center!"];
        _heli setVariable ["bmkhs_flightControlLockOut", true];
    };
};

if (bmkhs_pedalCenterTrimMode) then {
    while {_pedalLeftRight > CENTER_TRIM_VAL || _pedalLeftRight < -CENTER_TRIM_VAL} do {
        //systemChat format ["Yaw Locked out! Return Yaw to Center!"];
        _heli setVariable ["bmkhs_flightControlLockOut", true];
    };
};
