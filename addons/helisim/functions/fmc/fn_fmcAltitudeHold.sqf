params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

private _deltaTime  = _heli getVariable "bmkhs_deltaTime";
private _gndSpeed   = (_heli getVariable "bmkhs_gndSpeed") * KNOTS_TO_MPS;
private _pidRadAlt  = _heli getVariable "bmkhs_pid_radHold";
private _pidBarAlt  = _heli getVariable "bmkhs_pid_barHold";
private _curAltAGL  = ASLToAGL getPosASL _heli # 2;
private _subMode    = _heli getVariable "bmkhs_altHoldSubMode";
private _desiredAlt = _heli getVariable "bmkhs_altHoldDesiredAlt";
private _curAltMSL  = getPosASL _heli # 2;
private _collRef    = _heli getVariable  "bmkhs_altHoldCollRef";
private _e1tq       = _heli getVariable "bmkhs_engPctTQ" select 0;
private _e2tq       = _heli getVariable "bmkhs_engPctTQ" select 1;
private _tq         = _e1tq max _e2tq;
private _output     = 0.0;

//If the total torque exceeds 98%, de-activate altitude hold and don't allow its
//activation until it it is below 98%
if (_tq >= 0.98) then {
    [_heli, "bmkhs_altHoldActive", false] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_pidRadAlt] call bmkhs_fnc_pidReset;
    [_pidBarAlt] call bmkhs_fnc_pidReset;
};

if ( _heli getVariable "bmkhs_altHoldActive") then {
    //If the pilot is intentionally trying to change altitude, de-activate altitude
    //hold and allow them to do so
    private _collRef_low = _collRef * 0.95;
    private _collRef_hi  = _collRef * 1.05;
    if ((_heli getVariable "bmkhs_collectiveOutput") >= _collRef_hi || (_heli getVariable "bmkhs_collectiveOutput") <= _collRef_low) then {
        [_heli, "bmkhs_altHoldActive", false] call bmkhs_fnc_utilUpdateNetworkGlobal;
        [_heli, "holdModeDisengaged"] call bmkhs_fnc_utilNotify;
    };

    //If the helicopters radar altitude is < 1428ft (435.25m) and current velocity is < 40kts
    //then set the desired altitude to the current AGL altitude, otherwise set it to the
    //current ASL altitude.
    if (_curAltAGL < RAD_ALT_MAX_ALT && _gndSpeed < ALT_HOLD_SPEED_SWITCH) then {
        [_heli, "bmkhs_altHoldSubMode", "rad"] call bmkhs_fnc_utilUpdateNetworkGlobal;
    } else {
        [_heli, "bmkhs_altHoldSubMode", "bar"] call bmkhs_fnc_utilUpdateNetworkGlobal;
    };

    if (_subMode == "rad") then {
        //Radar altitude hold uses AGL altitude
        private _altError = _curAltAGL - _desiredAlt;
        _output = [_pidRadAlt, _deltaTime, 0.0, _altError] call bmkhs_fnc_pidRun;
    } else {
        //Barometric altitude hold uses the ASL altitude
        private _altError = _curAltMSL - _desiredAlt;
        _output = [_pidBarAlt, _deltaTime, 0.0, _altError] call bmkhs_fnc_pidRun;
    };
} else {
    [_pidRadAlt] call bmkhs_fnc_pidReset;
    [_pidBarAlt] call bmkhs_fnc_pidReset;
};

_output;
