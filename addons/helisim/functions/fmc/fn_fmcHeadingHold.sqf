params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

private _pidHdg        = _heli getVariable "bmkhs_pid_hdgHold";

private _pidTrn        = _heli getVariable "bmkhs_pid_trnCoord";
//_pidTrn set ["kp", T_KP];
//_pidTrn set ["ki", T_KI];
//_pidTrn set ["kd", T_KD];

private _pidYaw        = _heli getVariable "bmkhs_pid_sas_yaw";

private _deltaTime     = _heli getVariable "bmkhs_deltaTime";
private _gndSpeed      = (_heli getVariable "bmkhs_gndSpeed") * KNOTS_TO_MPS;
private _angVelZ       = (_heli getVariable "bmkhs_angVelModelSpace") # 2;
private _pedalTrim     = _heli getVariable "bmkhs_forceTrimPosYaw";
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
//  2. fn_avionicsSlipIndicator exits early unless the PLAYER is aboard, so for an AI Apache it is
//     whatever stale value was left there by another aircraft
//The auto-pedal was already moved off the gauge onto beta_g for these same reasons.
//
//No CBA_fnc_simplifyAngle180 either: beta_g is a g-load, not an angle in degrees, so wrapping it
//to +-180 is meaningless (the gauge value was being wrapped as though it were an angle).
//ERROR SENSE: (actual - desired), the SAME form this always used. The two consumers below then
//apply their own sense - "trn" passes it as the measurement against setpoint 0, "yaw" passes it as
//the setpoint against measurement 0, which negates it. Those two branch signs are VALIDATED; do
//not change them, and do not negate here either, or the "yaw" branch double-negates.
private _desiredSlip   = _heli getVariable "bmkhs_hdgHoldDesiredSideslip";
private _sideslipError = (_heli getVariable ["bmkhs_aero_beta_g", 0.0]) - _desiredSlip;
private _subMode       = _heli getVariable "bmkhs_hdgHoldSubMode";
private _attSubMode    = _heli getVariable "bmkhs_attHoldSubMode";
private _hdgOutput     = 0.0;
private _trnOutput     = 0.0;
private _yawOutput     = 0.0;
private _output        = 0.0;

private _onGnd         = [_heli] call bmkhs_fnc_stateOnGround;
//Breakout values expand as the aircraft goes faster to provide good pedal response
//at a hover. The expanded range is meant to de-sensitize the pedals in order to
//prevent disengaging the heading hold mode during cruise flight
private _breakoutValue = 0.0;
if (_attSubMode == "pos") then {
    _breakoutValue = HDG_HOLD_BREAKOUT_VALUE;
};
if (_attSubMode == "vel") then {
    _breakoutValue = VEL_HOLD_BREAKOUT_VALUE;
};
if (_attSubMode == "att") then {
    _breakoutValue = ATT_HOLD_BREAKOUT_VALUE;
};
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
//systemChat format ["_breakoutValue = %1 -- bmkhs_pedalLeftRight = %2", _breakoutValue, (_heli getVariable "bmkhs_pedalLeftRight") toFixed 2];
if (   _onGnd
    || _heli getVariable "bmkhs_forceTrimInterupted"
    || _breakout
    ) then {
        if (_heli getVariable "bmkhs_hdgHoldActive" isNotEqualTo false) then {
            _heli setVariable ["bmkhs_hdgHoldActive", false, true];
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
        [_pidYaw] call bmkhs_fnc_pidReset;
    };
};
//Finally, if the heading hold is active, perform the required functions
if (_heli getVariable "bmkhs_hdgHoldActive") then {
    //Compute target sub-mode from current state — one authoritative decision per frame.
    //  < 5 kts                              → hdg  (always, regardless of auto pedal)
    //  >= 5 kts, auto pedal on              → aut  (auto pedal owns axis via force trim, PIDs idle)
    //  >= 5 kts, auto pedal off, trn active → trn
    //  >= 5 kts, auto pedal off, trn off    → yaw
    private _targetSubMode = if (_gndSpeed < POS_HOLD_SPEED_SWITCH) then {
        "hdg"
    } else {
        if (bmkhs_autoPedal) then {
            "aut"
        } else {
            private _attHoldActive = _heli getVariable "bmkhs_attHoldActive";
            private _curBank       = (_heli call BIS_fnc_getPitchBank) # 1;

            //Turn coord: engages at > 7 deg bank from level, disengages when back within 3 deg.
            //When disengaging, reset att hold roll reference to 0 so yaw SAS holds level cleanly.
            private _trnCoordActive = _attHoldActive && (if (_subMode == "trn") then { abs _curBank > 3.0 } else { abs _curBank > 7.0 });
            if (_subMode == "trn" && !_trnCoordActive) then {
                private _desiredAtt = _heli getVariable "bmkhs_attHoldDesiredAtt";
                _heli setVariable ["bmkhs_attHoldDesiredAtt", [_desiredAtt # 0, 0.0], true];
            };

            ["yaw", "trn"] select (_trnCoordActive)
        }
    };

    //On transition: reset outgoing PID; capture heading when returning to hdg.
    //NOTE "trn" and "yaw" SHARE _pidTrn, so switching between those two must also clear it -
    //otherwise the incoming mode inherits the outgoing one's wound-up integrator and jumps on
    //the first frame. Resetting the OUTGOING mode covers that (both branches hit _pidTrn), but
    //the incoming one is reset explicitly too so a first-ever entry starts clean rather than
    //from whatever the PID was seeded with.
    if (_subMode != _targetSubMode) then {
        if (_subMode == "hdg") then { [_pidHdg] call bmkhs_fnc_pidReset; };
        if (_subMode == "trn" || _subMode == "yaw") then { [_pidTrn] call bmkhs_fnc_pidReset; };
        if (_subMode == "aut") then { [_pidYaw] call bmkhs_fnc_pidReset; };
        if (_targetSubMode == "hdg") then { [_pidHdg] call bmkhs_fnc_pidReset; };
        if (_targetSubMode == "trn" || _targetSubMode == "yaw") then { [_pidTrn] call bmkhs_fnc_pidReset; };
        if (_targetSubMode == "hdg") then {
            _heli setVariable ["bmkhs_hdgHoldDesiredHdg", getDir _heli, true];
        };
        _subMode = _targetSubMode;
        _heli setVariable ["bmkhs_hdgHoldSubMode", _subMode, true];
    };

    //Run exactly one PID per frame based on current sub-mode
    if (_subMode == "hdg") then {
        _hdgOutput = [_pidHdg, _deltaTime, 0.0, _hdgError] call bmkhs_fnc_pidRun;
        _hdgOutput = [_hdgOutput, -1.0, 1.0] call BIS_fnc_clamp;
    };
    if (_subMode == "trn") then {
        _trnOutput = [_pidTrn, _deltaTime, 0.0, _sideslipError] call bmkhs_fnc_pidRun;
        _trnOutput = [_trnOutput, -1.0, 1.0] call BIS_fnc_clamp;
    };
    if (_subMode == "yaw") then {
        //SIGN IS INTENTIONAL AND VALIDATED - do not "unify" it with the "trn" branch above.
        //The two branches deliberately use opposite error senses.
        _yawOutput = [_pidTrn, _deltaTime, _sideslipError, 0.0] call bmkhs_fnc_pidRun;
        _yawOutput = [_yawOutput, -1.0, 1.0] call BIS_fnc_clamp;
    };
    //"aut": auto pedal owns the yaw axis via bmkhs_forceTrimPosYaw (fn_getInput.sqf).
    //  fn_fmc.sqf zeroes _hdgHoldPedalYawOut when auto pedal is active, so no PID runs here.
    //  This sub-mode exists only to block "yaw" and "trn" from interfering.

    //Blend hdg → high-speed output across the 5-40 kt transition band.
    //Below 5 kts: pure heading hold. Above 40 kts: pure yaw SAS, turn coord, or auto pedal.
    private _highSpeedOutput = switch (_subMode) do {
        case "trn": { _trnOutput };
        case "yaw": { _yawOutput };
        case "aut": { 0.0 };        // auto pedal owns the axis; fn_fmc.sqf zeroes this anyway
        default     { 0.0 };
    };
    _output = linearConversion[POS_HOLD_SPEED_SWITCH, HDG_HOLD_SPEED_SWITCH_ACCEL, _gndSpeed, _hdgOutput, _highSpeedOutput, true];
} else {
    [_pidHdg] call bmkhs_fnc_pidReset;
    [_pidTrn] call bmkhs_fnc_pidReset;
    [_pidYaw] call bmkhs_fnc_pidReset;
};

_output = [_output,  -0.1, 0.1] call BIS_fnc_clamp;

_output;
