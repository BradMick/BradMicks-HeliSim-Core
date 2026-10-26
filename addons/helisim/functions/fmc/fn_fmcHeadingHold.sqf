params ["_heli", "_hh", "_on"];
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\fmc.hpp"

if (count _hh == 0) exitWith {0.0};

private _pidHdg        = _hh get "hdg";
private _pidTrn        = _hh get "trn";
private _hdgBelow      = (_hh get "hdgBelowKts") * KNOTS_TO_MPS;
private _blendTo       = (_hh get "blendToKts") * KNOTS_TO_MPS;
private _authority     = _hh get "authority";
private _subModes      = _hh get "subModes";
private _enterBank     = _hh get "trnEnterBankDeg";
(_hh get "trnExit") params ["_exitBank", "_exitRate", "_exitSec"];
//The turn sub-mode - pedals on the ball; none = heading on the pedals at every speed
private _turn          = ["hdg", ["yaw", "trn"] select ("trn" in _subModes)] select ("trn" in _subModes || {"yaw" in _subModes});

//The SAS yaw loop is cleared alongside, where the aircraft has one
private _pidYaw        = ((_heli getVariable "bmkhs_fmc") getOrDefault ["Sas", createHashMap]) getOrDefault ["yaw", createHashMap];
private _resetYaw      = { if (count _pidYaw > 0) then { [_pidYaw] call bmkhs_fnc_pidReset } };
private _cycBreakout   = ((_heli getVariable "bmkhs_fmc") getOrDefault ["AttitudeHold", createHashMap]) getOrDefault ["breakout", FMC_ATT_BREAKOUT];

private _deltaTime     = _heli getVariable "bmkhs_deltaTime";
private _gndSpeed      = _heli getVariable "bmkhs_gndSpeed";
private _angVelZ       = (_heli getVariable "bmkhs_angVelModelSpace") # 2;
private _curBank       = (_heli call BIS_fnc_getPitchBank) # 1;
private _curHdg        = getDir _heli;
private _desiredHdg    = _heli getVariable "bmkhs_hdgHoldDesiredHdg";
private _hdgError      = [_curHdg - _desiredHdg] call CBA_fnc_simplifyAngle180;
//SIDESLIP ERROR - drive the TRIM BALL to centre. The ball is bmkhs_aero_beta_g (lateral
//specific force in g, from fn_calculateAeroValues via bodyAccel); when it is zero the aircraft is
//in aerodynamic trim, which is exactly what this loop is for.
//
//This previously read the GLOBAL gauge sideslip signal, which and wrong here on
//two counts:
//  1. it is CLAMPED to +-1 at 0.15 g, so past that the controller goes blind - it sees a constant
//     maxed-out error however hard the aircraft is actually skidding, and the loop just pins its
//     output (measured: _hdgHoldPedalYawOut sat at exactly -0.100, the clamp, in cruise)
//  2. fn_avionicsSlipIndicator exits early unless the PLAYER is aboard, so for an AI aircraft it is
//     whatever stale value was left there by another aircraft
//The auto-pedal was already moved off the gauge onto beta_g for these same reasons.
//
//No CBA_fnc_simplifyAngle180 either: beta_g is a g-load, not an angle in degrees, so wrapping it
//to +-180 is meaningless (the gauge value was being wrapped as though it were an angle).
//ERROR SENSE: (actual - desired), passed as the setpoint against measurement 0.
private _desiredSlip   = _heli getVariable "bmkhs_hdgHoldDesiredSideslip";
private _sideslipError = (_heli getVariable ["bmkhs_aero_beta_g", 0.0]) - _desiredSlip;
private _subMode       = _heli getVariable "bmkhs_hdgHoldSubMode";
private _hdgOutput     = 0.0;
private _trnOutput     = 0.0;
private _output        = 0.0;

private _onGnd         = [_heli] call bmkhs_fnc_stateOnGround;
//Breakout values expand as the aircraft goes faster to provide good pedal response
//at a hover. The expanded range is meant to de-sensitize the pedals in order to
//prevent disengaging the heading hold mode during cruise flight. By the attitude hold's sub-mode,
//so both holds change band together, on its hysteresis: {pos, vel, att}, spd as att.
private _breakoutValue = (_hh get "breakout") select ((["pos", "vel", "att"] find ((_heli getVariable "bmkhs_attHoldSubMode") regexReplace ["spd", "att"])) max 0);
//If we are on the ground, or if the force trim is interupted, or the pilot has exceeded
//the breakout values for the pedals, then heading hold is not active (doing work)
//otherwise, heading hold is ALWAYS active
private _breakout = false;
if ((_heli getVariable "bmkhs_pedalLeftRight") <= -_breakoutValue && (_heli getVariable "bmkhs_pedalLeftRight") < 0.0) then {
    _breakout = true;
};

if ((_heli getVariable "bmkhs_pedalLeftRight") >= _breakoutValue && (_heli getVariable "bmkhs_pedalLeftRight") > 0.0) then {
    _breakout = true;
};
//Pilot action - pedals, force trim, roll stick, lateral beep, bank
private _action = _breakout
    || {_heli getVariable "bmkhs_forceTrimInterupted"}
    || {abs (_heli getVariable "bmkhs_cyclicLeftRight") > _cycBreakout}
    || {((_heli getVariable ["bmkhs_trimBeepDelta", [0.0, 0.0]]) # 1) != 0}
    || {_enterBank > 0 && {abs _curBank > _enterBank}};
//systemChat format ["_breakoutValue = %1 -- bmkhs_pedalLeftRight = %2", _breakoutValue, (_heli getVariable "bmkhs_pedalLeftRight") toFixed 2];
if (   !_on
    || _onGnd
    || _heli getVariable "bmkhs_forceTrimInterupted"
    || _breakout
    ) then {
        if (_heli getVariable "bmkhs_hdgHoldActive" isNotEqualTo false) then {
            _heli setVariable ["bmkhs_hdgHoldActive", false, true];
        };
        //Pedals or force trim in flight - yaw SAS; auto pedal - aut
        private _idle = ["yaw", "aut"] select bmkhs_autoPedal;
        if (_on && {!_onGnd} && {_turn != "hdg" || {bmkhs_autoPedal}} && {_subMode != _idle}) then {
            _heli setVariable ["bmkhs_hdgHoldSubMode", _idle, true];
        };
} else {
    if (_heli getVariable "bmkhs_hdgHoldActive" isNotEqualTo true) then {
        _heli setVariable ["bmkhs_hdgHoldActive", true, true];
        _heli setVariable ["bmkhs_hdgHoldDesiredHdg", getDir _heli, true];
        //Clear the PIDs on ENGAGE. Capturing the heading alone is not enough: the integrators
        //still hold whatever they accumulated before the mode dropped out (a pedal breakout, a
        //force-trim interrupt, sitting on the ground), and that lands on the pedals as a kick the
        //instant the hold re-engages.
        [_pidHdg] call bmkhs_fnc_pidReset;
        [_pidTrn] call bmkhs_fnc_pidReset;
        call _resetYaw;
        //Heading captured once settled
        if (_turn != "hdg" && {!bmkhs_autoPedal}) then {
            _heli setVariable ["bmkhs_hdgHoldSubMode", "yaw", true];
            _heli setVariable ["bmkhs_hdgHoldTrnSettle", 0.0];
            _subMode = "yaw";
        };
    };
};
private _bankDeg = 0.0;
//Finally, if the heading hold is active, perform the required functions
if (_heli getVariable "bmkhs_hdgHoldActive") then {
    //Sub-mode. Below blendToKts heading hold; out of it is yaw SAS.
    //At and above it: attitude hold on - hdg, trn on pilot action; off - trn always.
    //  auto pedal                          -> aut, PIDs idle, any speed
    //  no turn mode                        -> hdg, pedals
    //  attitude hold off, fast             -> trn
    //  hdg, pilot action (attitude hold)   -> yaw slow, trn fast
    //  yaw / trn / aut, settled for the exit time -> hdg
    private _attHoldOn = _heli getVariable "bmkhs_attHoldActive";
    private _fast      = _gndSpeed >= _blendTo;
    private _out       = ["yaw", _turn] select _fast;
    private _yawRateOk = abs (_angVelZ * RAD_TO_DEG) < _exitRate;
    private _targetSubMode = switch (true) do {
        case (bmkhs_autoPedal):                                     { "aut" };
        case (_turn == "hdg"):                                      { "hdg" };
        case (!_attHoldOn && {_fast}):                              { _turn };
        case (_subMode == "hdg"):                                   { ["hdg", _out] select (_attHoldOn && _action) };
        default {
            //Attitude hold off: yaw rate and 1 s
            private _settled = [_yawRateOk, !_action && {abs _curBank < _exitBank} && {_yawRateOk}] select _attHoldOn;
            private _t = [0.0, (_heli getVariable ["bmkhs_hdgHoldTrnSettle", 0.0]) + _deltaTime] select _settled;
            _heli setVariable ["bmkhs_hdgHoldTrnSettle", _t];
            ["hdg", _out] select (!_settled || {_t < ([_exitSec max FMC_HDG_SAS_REENGAGE_S, _exitSec] select _attHoldOn)})
        };
    };

    if (_subMode != _targetSubMode) then {
        //trn PID kept through hdg / yaw / trn
        if (_subMode == "aut" || {_targetSubMode == "aut"}) then { [_pidTrn] call bmkhs_fnc_pidReset; call _resetYaw };
        if (_targetSubMode == "hdg") then {
            [_pidHdg] call bmkhs_fnc_pidReset;
            _heli setVariable ["bmkhs_hdgHoldDesiredHdg", getDir _heli, true];
            //Roll under 3 deg - reference to zero
            if (_attHoldOn && {_subMode != "aut"}) then {
                private _desiredAtt = _heli getVariable "bmkhs_attHoldDesiredAtt";
                _heli setVariable ["bmkhs_attHoldDesiredAtt", [_desiredAtt # 0, 0.0], true];
            };
        };
        if (_subMode == "hdg") then { _heli setVariable ["bmkhs_hdgHoldTrnSettle", 0.0] };
        _subMode = _targetSubMode;
        _heli setVariable ["bmkhs_hdgHoldSubMode", _subMode, true];
    };

    //Ball share - 0 at hdgBelowKts, 1 at blendToKts
    private _w = [linearConversion [_hdgBelow, _blendTo, _gndSpeed, 0, 1, true], 0] select (_turn == "hdg");

    //Heading on the pedals
    if (_subMode == "hdg" && {_w < 1}) then {
        _hdgOutput = [_pidHdg, _deltaTime, 0.0, _hdgError] call bmkhs_fnc_pidRun;
        _hdgOutput = [_hdgOutput, -1.0, 1.0] call BIS_fnc_clamp;
    } else {
        [_pidHdg] call bmkhs_fnc_pidReset;
    };
    //Ball on the pedals
    if (_subMode != "aut" && {_w > 0}) then {
        //Right pedal moves the ball negative
        _trnOutput = [_pidTrn, _deltaTime, _sideslipError, 0.0] call bmkhs_fnc_pidRun;
        _trnOutput = [_trnOutput, -1.0, 1.0] call BIS_fnc_clamp;
    } else {
        [_pidTrn] call bmkhs_fnc_pidReset;
    };
    //"aut": auto pedal owns the yaw axis; fn_fmc.sqf zeroes the output

    //Heading at speed - a coordinated bank on the attitude hold's roll reference
    if (_subMode == "hdg" && {_w > 0} && {_attHoldOn}) then {
        _bankDeg = _w * (((-_hdgError * FMC_HDG_BANK_PER_DEG) max -FMC_HDG_MAX_BANK_DEG) min FMC_HDG_MAX_BANK_DEG);
    };

    _output = ([0.0, (1 - _w) * _hdgOutput] select (_subMode == "hdg")) + ([0.0, _w * _trnOutput] select (_subMode != "aut"));
} else {
    [_pidHdg] call bmkhs_fnc_pidReset;
    [_pidTrn] call bmkhs_fnc_pidReset;
    call _resetYaw;
};
_heli setVariable ["bmkhs_hdgHoldBankDeg", _bankDeg];

_output = [_output, -_authority, _authority] call BIS_fnc_clamp;

_output;
