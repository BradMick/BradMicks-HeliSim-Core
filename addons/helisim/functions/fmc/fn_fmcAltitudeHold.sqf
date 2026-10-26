params ["_heli", "_alt", "_on"];
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\fmc.hpp"

if (count _alt == 0) exitWith {0.0};

private _deltaTime  = _heli getVariable "bmkhs_deltaTime";
private _gndSpeed   = _heli getVariable "bmkhs_gndSpeed";
private _pidRadAlt  = _alt get "rad";
private _pidBarAlt  = _alt get "bar";
private _curAltAGL  = ASLToAGL getPosASL _heli # 2;
private _subMode    = _heli getVariable "bmkhs_altHoldSubMode";
private _desiredAlt = _heli getVariable "bmkhs_altHoldDesiredAlt";
private _curAltMSL  = getPosASL _heli # 2;
private _collRef    = _heli getVariable  "bmkhs_altHoldCollRef";
private _tq         = selectMax (_heli getVariable "bmkhs_engPctTq");
private _output     = 0.0;

//At the torque limit, de-activate altitude hold and don't allow its activation until below it
if (_tq >= (_alt get "dropAboveTq")) then {
    [_heli, "bmkhs_altHoldActive", false] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_pidRadAlt] call bmkhs_fnc_pidReset;
    [_pidBarAlt] call bmkhs_fnc_pidReset;
};

if (_on && {_heli getVariable "bmkhs_altHoldActive"}) then {
    //If the pilot is intentionally trying to change altitude, de-activate altitude
    //hold and allow them to do so
    private _collRef_low = _collRef * (1 - (_alt get "collBand"));
    private _collRef_hi  = _collRef * (1 + (_alt get "collBand"));
    if ((_heli getVariable "bmkhs_collectiveOutput") >= _collRef_hi || (_heli getVariable "bmkhs_collectiveOutput") <= _collRef_low) then {
        [_heli, "bmkhs_altHoldActive", false] call bmkhs_fnc_utilUpdateNetworkGlobal;
        [_heli, "holdModeDisengaged"] call bmkhs_fnc_utilNotify;
    };

    //Radar below its height and speed, barometric otherwise
    if (_curAltAGL < ((_alt get "radBelowFt") * FEET_TO_METERS) && _gndSpeed < ((_alt get "radBelowKts") * KNOTS_TO_MPS)) then {
        [_heli, "bmkhs_altHoldSubMode", "rad"] call bmkhs_fnc_utilUpdateNetworkGlobal;
    } else {
        [_heli, "bmkhs_altHoldSubMode", "bar"] call bmkhs_fnc_utilUpdateNetworkGlobal;
    };

    //Beep - a tap steps, held moves at the rate, then the fast rate
    private _beep = _alt get "beep";
    private _held = _heli getVariable ["bmkhs_altBeepHeld", []];
    if (count _beep >= 4 && {count _held > 0}) then {
        _beep params ["_stepFt", "_rateFps", "_fastFps", "_fastAfterS"];
        _held params ["_sign", "_t0", "_stepped"];
        private _heldFor = CBA_missionTime - _t0 - FMC_ALT_BEEP_HOLD_S;
        private _moveFt  = 0.0;
        if (!_stepped) then { _moveFt = _stepFt; _held set [2, true] };
        if (_heldFor > 0) then { _moveFt = _moveFt + ([_rateFps, _fastFps] select (_heldFor > _fastAfterS)) * _deltaTime };
        if (_moveFt != 0) then {
            _desiredAlt = _desiredAlt + _sign * _moveFt * FEET_TO_METERS;
            _heli setVariable ["bmkhs_altHoldDesiredAlt", _desiredAlt];
        };
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
