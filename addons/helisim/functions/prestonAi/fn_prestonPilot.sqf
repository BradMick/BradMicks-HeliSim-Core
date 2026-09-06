/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_prestonPilot

Description:
    Preston Pilot AI - the machine pilot's HANDS on the cyclic, as the auto-pedal
    is its feet. Owns the cyclic and positions it every frame; the pilot's keys
    say what to ACHIEVE, not where to put the stick.

    Three regimes, blended by weight and never switched:
        POS (hover)      - hold position over the ground
        VEL (transition) - roll nulls lateral drift, pitch holds forward velocity
        ATT (cruise)     - hold the swept attitude target

    Independent of the FMC holds, which are realistic-only. Constants are the
    AUTO_ATT_* block in core.hpp; state is seeded by fn_prestonVariables.

Parameters:
    _heli             - The helicopter [Object].
    _deltaTime        - Frame time [Number].
    _cyclicFwdAft     - Pilot pitch input [Number].
    _cyclicLeftRight  - Pilot roll input [Number].
    _kbStickyInterupt - Expanded-envelope modifier [Bool].

Returns:
    [_cyclicFwdAft, _cyclicLeftRight] - Preston ZEROES both, so force-trim is the
    only path to the rotor and the envelope limiter is real.

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli", "_deltaTime", "_cyclicFwdAft", "_cyclicLeftRight", "_kbStickyInterupt"];

//fn_preston has already decided Preston is flying, so the internal gates are always true.
private _active = true;

//  wPos : 1 below 5kt -> 0 by 15kt.                    "stopped, so hold position"
//  wFast: 0 below 45kt -> 1 above 55kt.                "fast enough for attitude"
//  wHigh: 0 below 40ft -> 1 above 60ft.                "high enough for attitude"
//ATT requires BOTH fast AND high (min = logical AND) - the same LOW-OR-SLOW rule the auto-pedal
//uses. Whatever is left after POS and ATT have taken their share belongs to VEL.
private _wPos = 0.0;
private _wVel = 0.0;
private _wAtt = 0.0;
//Ground velocity in MODEL space (X = right+, Y = forward+), wind excluded - this is what the POS
//and VEL regimes null. velModelSpaceNoWind is the same signal the FMC position hold uses.
private _velX = 0.0;
private _velY = 0.0;
private _gndSpd   = 0.0;
private _agl  = 0.0;
if (_active) then {
    _gndSpd = (_heli getVariable "bmkhs_gndSpeed") * KNOTS_TO_MPS;
    //RAW radar altitude (3rd return), not the displayed one - the displayed value is rounded to
    //10ft above 50ft, which would turn this blend into a staircase as the aircraft bobs.
    ([_heli] call bmkhs_fnc_stateAltitude) params ["", "", "_aaRadAlt"];
    _agl = _aaRadAlt;

    (_heli getVariable "bmkhs_velModelSpaceNoWind") params ["_aaVelX", "_aaVelY"];
    _velX = _aaVelX;
    _velY = _aaVelY;

    _wPos = 1.0 - (linearConversion [AUTO_ATT_POS_SPD_LO, AUTO_ATT_POS_SPD_HI, _gndSpd, 0.0, 1.0, true]);
    private _wFast = linearConversion [AUTO_ATT_ATT_SPD_LO, AUTO_ATT_ATT_SPD_HI, _gndSpd,  0.0, 1.0, true];
    private _wHigh = linearConversion [AUTO_ATT_ATT_AGL_LO, AUTO_ATT_ATT_AGL_HI, _agl, 0.0, 1.0, true];
    //ATT takes its share of whatever POS has left; VEL gets the remainder.
    _wAtt = (1.0 - _wPos) * (_wFast min _wHigh);
    _wVel = 1.0 - _wPos - _wAtt;

    _heli setVariable ["bmkhs_prestonWPos", _wPos];
    _heli setVariable ["bmkhs_prestonWVel", _wVel];
    _heli setVariable ["bmkhs_prestonWAtt", _wAtt];
};

if (_active) then {
    //PILOT AUTHORITY - same contract as the auto-pedal: the auto modes never fight an input, they
    //only restore equilibrium once the pilot lets go. A live key commands the regime directly,
    //suspends the hover position loop with its integral frozen, and on release re-captures the
    //datum wherever the aircraft now is.
    private _pitchKey = 0.0;
    private _rollKey  = 0.0;
    if ((abs _cyclicFwdAft)    > AUTO_ATT_KEY_DEADBAND) then { _pitchKey = _cyclicFwdAft;    };
    if ((abs _cyclicLeftRight) > AUTO_ATT_KEY_DEADBAND) then { _rollKey  = _cyclicLeftRight; };
    private _cyclicBreakout     = (_pitchKey != 0.0) || (_rollKey != 0.0);
    private _prevCyclicBreakout = _heli getVariable ["bmkhs_prestonBreakout", false];
    //Release capture: on the falling edge of the breakout, the datum becomes wherever we are now.
    if (_prevCyclicBreakout && !_cyclicBreakout) then {
        _heli setVariable ["bmkhs_prestonHoverDatum", getPos _heli];
        _heli setVariable ["bmkhs_prestonHoverIntX",  0.0];
        _heli setVariable ["bmkhs_prestonHoverIntY",  0.0];
    };
    //HANDS RE-SEED, same as the auto-pedal does for the feet: while the pilot is on the cyclic,
    //FREEZE the hands filter at the current trim position. Without this the filter state goes
    //stale during the breakout and, on release, the machine pilot rate-limits back from wherever
    //it was left rather than moving on from where the stick actually is - a visible lurch at
    //exactly the moment the pilot hands control back.
    if (_cyclicBreakout) then {
        _heli setVariable ["bmkhs_prestonPrevPitch", _heli getVariable ["bmkhs_forceTrimPosPitch", 0.0]];
        _heli setVariable ["bmkhs_prestonPrevRoll",  _heli getVariable ["bmkhs_forceTrimPosRoll",  0.0]];
    };
    _heli setVariable ["bmkhs_prestonBreakout", _cyclicBreakout];

    /////////////////////////////////////////////////////////////////////////////////////////
    // ATTITUDE REGIME - hold the swept pitch/roll target. Runs every frame regardless of the
    // blend weight so its PIDs never see a discontinuity; the weight only decides how much of
    // its output reaches the cyclic.
    /////////////////////////////////////////////////////////////////////////////////////////
    private _pidAutoPitch    = _heli getVariable "bmkhs_pid_prestonPitch";
    private _pidAutoRoll     = _heli getVariable "bmkhs_pid_prestonRoll";
    private _pitchTarget = _heli getVariable "bmkhs_prestonPitchTarget";
    private _rollTarget  = _heli getVariable "bmkhs_prestonRollTarget";
    (_heli call BIS_fnc_getPitchBank) params ["_curPitch", "_curRoll"];

    //ENVELOPE. The keyboard technique is to MASH the key until the target hits the limit, so these
    //limits ARE the commanded attitude in normal use. Sticky-interrupt selects the EXPANDED
    //envelope (reused from the KeyboardSticky branches - a player will not have both).
    //
    //PITCH is asymmetric and scheduled: nose-down authority is earned with height and speed, nose
    //up is not gated. 5 deg at a 3ft hover, 10 with speed while low, 15 above 50ft. Override
    //expands both to 30 and bypasses the nose-down gates.
    private _pitchUpLimit = AUTO_ATT_PITCH_UP_LIMIT;
    //Nose-down: speed schedule first (5 -> 10 deg), then the height gate opens it toward 15.
    private _dnBySpeed    = linearConversion [AUTO_ATT_PITCH_DN_SPD_LO, AUTO_ATT_PITCH_DN_SPD_HI, _gndSpd, AUTO_ATT_PITCH_DN_LIMIT_GND, AUTO_ATT_PITCH_DN_LIMIT_LOW, true];
    private _dnByHeight   = linearConversion [AUTO_ATT_PITCH_GND_AGL, AUTO_ATT_ATT_AGL_HI, _agl, 0.0, 1.0, true];
    private _pitchDnLimit = _dnBySpeed + ((AUTO_ATT_PITCH_DN_LIMIT_HI - _dnBySpeed) * _dnByHeight);
    //Within GND_AGL of the ground the ground clamp is absolute (no speed credit) - this is what
    //stops a fast, low pass from unlocking nose-down authority right above the deck.
    if (_agl <= AUTO_ATT_PITCH_GND_AGL) then { _pitchDnLimit = AUTO_ATT_PITCH_DN_LIMIT_GND; };
    if (_kbStickyInterupt) then {
        _pitchUpLimit = AUTO_ATT_PITCH_LIMIT_EXP;
        _pitchDnLimit = AUTO_ATT_PITCH_LIMIT_EXP;
    };

    private _rollCap    = [AUTO_ATT_ROLL_LIMIT,  AUTO_ATT_ROLL_LIMIT_EXP]  select (_kbStickyInterupt);
    private _turnRate   = [AUTO_ATT_TURN_RATE,   AUTO_ATT_TURN_RATE_EXP]   select (_kbStickyInterupt);
    //Bank for the target turn rate at the current speed: bank = atan(V * omega / g), omega in
    //rad/s. Below ETL this collapses toward zero (no meaningful turn at a hover), so a floor keeps
    //some roll authority for repositioning; the hard cap bounds the top end at high speed.
    private _rateBank   = atan ((_gndSpd * (_turnRate * (pi / 180.0))) / 9.806);
    private _rollLimit  = [(_rateBank max AUTO_ATT_TURN_MIN_BANK), 0.0, _rollCap] call BIS_fnc_clamp;

    //PITCH target sweep - PERSISTS on release (pitch attitude selects airspeed). The target is
    //only ever MOVED, never snapped, so it stays continuous across press and release alike.
    if (_pitchKey != 0.0) then {
        //Forward cyclic (+) commands nose DOWN, so the target moves negative.
        _pitchTarget = _pitchTarget - (_pitchKey * AUTO_ATT_PITCH_RATE * _deltaTime);
        _heli setVariable ["bmkhs_prestonPitchTarget", _pitchTarget];
    };
    //Clamp EVERY frame, not only while the key is down: the nose-down limit MOVES with height and
    //speed, so a 15 deg dive that was legal at 200ft has to be walked back as the aircraft
    //descends through 50ft - and releasing the override has to bring an expanded target back
    //inside. Asymmetric: -down for nose low, +up for nose high.
    _pitchTarget = [_pitchTarget, -_pitchDnLimit, _pitchUpLimit] call BIS_fnc_clamp;
    _heli setVariable ["bmkhs_prestonPitchTarget", _pitchTarget];

    //ROLL target sweep - DECAYS to wings-level on release (kills the standing right roll).
    //SIGN: _cyclicLeftRight is built as (heliCyclicLeftOut - heliCyclicRightOut) - but that name
    //is misleading. In this codebase that expression is the RIGHT-positive stick convention (the
    //same one _rollInput uses downstream in fn_simpleRotorMain), and BIS_fnc_getPitchBank also
    //reports bank RIGHT-positive. The two senses AGREE, so the key is ADDED, not subtracted.
    //Subtracting it inverted the commanded bank in the ATT regime while pitch stayed correct.
    if (_rollKey != 0.0) then {
        _rollTarget = _rollTarget + (_rollKey * AUTO_ATT_ROLL_RATE * _deltaTime);
    } else {
        //A RATE decay (not a snap, not an exponential) keeps the roll-out predictable and
        //identical from any bank.
        private _levelStep = AUTO_ATT_ROLL_LEVEL_RATE * _deltaTime;
        if ((abs _rollTarget) <= _levelStep) then {
            _rollTarget = 0.0;
        } else {
            _rollTarget = _rollTarget - (_levelStep * ([1, -1] select (_rollTarget < 0.0)));
        };
    };
    //Clamp EVERY frame (as with pitch). The roll limit MOVES with airspeed, so a bank that was
    //legal at 140kt is outside the envelope once the aircraft slows - holding the key must not
    //keep it there. The decay path is clamped too so releasing the expand key while banked hard
    //walks the target back inside rather than stranding it.
    _rollTarget = [_rollTarget, -_rollLimit, _rollLimit] call BIS_fnc_clamp;
    _heli setVariable ["bmkhs_prestonRollTarget", _rollTarget];

    //SIGN - matches fn_fmcAttitudeHold's "att" branch: pidRun gives (setpoint - measurement), so
    //passing (target, current) then negating makes the command (current - target). Without the
    //negation this is positive feedback.
    //
    //A hover has no correct attitude - it is whatever balances the forces, an OUTPUT of holding
    //position rather than a target. So while hover owns the axis the attitude target is slaved to
    //actual and the attitude PIDs are held reset. Slaving also makes the handover to the attitude
    //regime seamless, since it picks up from the attitude the aircraft already has.
    private _attPitchOut = 0.0;
    private _attRollOut  = 0.0;
    if (_wAtt > 0.0) then {
        _attPitchOut = [_pidAutoPitch, _deltaTime, _pitchTarget, _curPitch] call bmkhs_fnc_pidRun;
        _attPitchOut = -([_attPitchOut, -1.0, 1.0] call BIS_fnc_clamp);
        _attRollOut  = [_pidAutoRoll,  _deltaTime, _rollTarget,  _curRoll]  call bmkhs_fnc_pidRun;
        _attRollOut  = -([_attRollOut,  -1.0, 1.0] call BIS_fnc_clamp);
    } else {
        //Not in cruise: slave the target to actual and hold the PIDs reset.
        _pitchTarget = _curPitch;
        _rollTarget  = _curRoll;
        _heli setVariable ["bmkhs_prestonPitchTarget", _pitchTarget];
        _heli setVariable ["bmkhs_prestonRollTarget",  _rollTarget];
        [_pidAutoPitch] call bmkhs_fnc_pidReset;
        [_pidAutoRoll]  call bmkhs_fnc_pidReset;
    };

    //VEL REGIME (transition) - ROLL nulls lateral ground velocity, PITCH holds forward velocity.
    //Roll is the critical half: the pedals are holding nose-to-tail trim here, so lateral drift
    //makes them chase a sideways velocity vector - hands and feet end up fighting. The hands kill
    //the drift so the feet have something sane to trim to.
    //SIGN: from fn_fmcAttitudeHold "vel" - roll measures -velX, pitch +velY, output AS-IS.
    private _velPitchOut = 0.0;
    private _velRollOut  = 0.0;
    if (_wVel > 0.0) then {
        private _pidVelX = _heli getVariable "bmkhs_pid_prestonVelX";
        private _pidVelY = _heli getVariable "bmkhs_pid_prestonVelY";

        //Keys command a ground velocity. Lateral setpoint is ZERO unless the pilot is actively
        //asking for drift - the whole point of this regime is that sideward velocity is the
        //enemy. Forward setpoint is swept by the pitch key and PERSISTS, so the machine pilot
        //accelerates to a commanded speed and holds it.
        private _velCmdFwd = _heli getVariable ["bmkhs_prestonVelCmdFwd", 0.0];
        if (_pitchKey != 0.0) then {
            //Forward cyclic (+) = accelerate forward.
            _velCmdFwd = _velCmdFwd + (_pitchKey * AUTO_ATT_VEL_ACCEL_RATE * _deltaTime);
            _velCmdFwd = [_velCmdFwd, -AUTO_ATT_VEL_CMD_LIMIT, AUTO_ATT_VEL_CMD_LIMIT] call BIS_fnc_clamp;
        };
        _heli setVariable ["bmkhs_prestonVelCmdFwd", _velCmdFwd];
        //Lateral setpoint lives in the NEGATED frame (matching fn_fmcAttitudeHoldEnable, which
        //stores velX * -1.0) and is passed plain against the -velX measurement.
        //key +LEFT -> drift left -> velX negative -> negated frame: positive.
        private _velCmdLat = _rollKey * AUTO_ATT_HOVER_VEL_RATE;
        _velCmdLat = [_velCmdLat, -AUTO_ATT_HOVER_VEL_LIMIT, AUTO_ATT_HOVER_VEL_LIMIT] call BIS_fnc_clamp;

        //ACCELERATION LEAD - as in the POS regime: bank as the aircraft STARTS to slide.
        private _vLeadX = [(_heli getVariable ["bmkhs_accelX", 0.0]) * AUTO_ATT_ACCEL_LEAD,
                           -AUTO_ATT_ACCEL_LEAD_CLAMP, AUTO_ATT_ACCEL_LEAD_CLAMP] call BIS_fnc_clamp;
        private _vLeadY = [(_heli getVariable ["bmkhs_accelY", 0.0]) * AUTO_ATT_ACCEL_LEAD,
                           -AUTO_ATT_ACCEL_LEAD_CLAMP, AUTO_ATT_ACCEL_LEAD_CLAMP] call BIS_fnc_clamp;

        _velRollOut  = [_pidVelX, _deltaTime, ( _velCmdLat), (-(_velX + _vLeadX))] call bmkhs_fnc_pidRun;
        _velRollOut  = [_velRollOut,  -1.0, 1.0] call BIS_fnc_clamp;
        _velPitchOut = [_pidVelY, _deltaTime, ( _velCmdFwd), ( _velY + _vLeadY)] call bmkhs_fnc_pidRun;
        _velPitchOut = [_velPitchOut, -1.0, 1.0] call BIS_fnc_clamp;
    } else {
        [_heli getVariable "bmkhs_pid_prestonVelX"] call bmkhs_fnc_pidReset;
        [_heli getVariable "bmkhs_pid_prestonVelY"] call bmkhs_fnc_pidReset;
        //Seed the commanded forward velocity to actual, so entering VEL from either side starts
        //from what the aircraft is already doing instead of snapping to a stale command.
        _heli setVariable ["bmkhs_prestonVelCmdFwd", _velY];
    };

    /////////////////////////////////////////////////////////////////////////////////////////
    // POS REGIME (hover) - hold POSITION over the ground. Structure follows the FMC position
    // hold (fn_fmcAttitudeHold "pos"): a velocity-null PID per axis, plus a slow, tightly-
    // clamped position-error integral that biases the velocity SETPOINT rather than the output.
    // The bias works THROUGH the velocity loop, so velocity does all the actuation and the
    // integral only re-aims it - it cannot fight the loop.
    //
    // The PID's OWN ki is what finds the standing cyclic offset that balances the forces on the
    // aircraft (tail thrust, CG, wind) - that is the thing a real pilot's hand does at a hover,
    // and kp alone cannot do it (zero velocity error commands zero cyclic and it drifts again).
    /////////////////////////////////////////////////////////////////////////////////////////
    private _hovPitchOut = 0.0;
    private _hovRollOut  = 0.0;
    if (_wPos > 0.0) then {
        private _pidHovX = _heli getVariable "bmkhs_pid_prestonHoverX";
        private _pidHovY = _heli getVariable "bmkhs_pid_prestonHoverY";

        //Commanded reposition velocity. A hovering pilot does not command an attitude, they
        //command a DRIFT - "slide left a bit". Key -> ground velocity setpoint, in m/s.
        //SIGN: the lateral setpoint lives in the NEGATED-X frame, because that is the frame the
        //roll channel works in throughout (measurement is -velX, and the reference stores its
        //captured setpoint pre-negated). Build it negated here, pass it PLAIN below - negating
        //again at the call site is what inverted the roll axis. Pitch is negated nowhere.
        private _setVelX = _rollKey  * AUTO_ATT_HOVER_VEL_RATE;   //key +LEFT, negated frame -> +
        private _setVelY = _pitchKey * AUTO_ATT_HOVER_VEL_RATE;   //key +FWD  -> velocity +Y (fwd)
        _setVelX = [_setVelX, -AUTO_ATT_HOVER_VEL_LIMIT, AUTO_ATT_HOVER_VEL_LIMIT] call BIS_fnc_clamp;
        _setVelY = [_setVelY, -AUTO_ATT_HOVER_VEL_LIMIT, AUTO_ATT_HOVER_VEL_LIMIT] call BIS_fnc_clamp;

        private _iX = _heli getVariable ["bmkhs_prestonHoverIntX", 0.0];
        private _iY = _heli getVariable ["bmkhs_prestonHoverIntY", 0.0];

        //Position integral runs ONLY when the pilot is off the cyclic. While they are flying it
        //the datum is meaningless (they are deliberately leaving it), so the integral is frozen -
        //not reset - and the datum is re-captured on release by the breakout handler above.
        if (!_cyclicBreakout) then {
            private _datum = _heli getVariable ["bmkhs_prestonHoverDatum", getPos _heli];
            private _dPos  = _datum vectorDiff (getPos _heli);
            private _hdg   = direction _heli;
            //World -> model space position error: X = right+, Y = forward+.
            private _posErrX = ((_dPos # 0) * cos _hdg) - ((_dPos # 1) * sin _hdg);
            private _posErrY = ((_dPos # 0) * sin _hdg) + ((_dPos # 1) * cos _hdg);
            //Deadband: do not chase sub-metre error. Subtracting (rather than zeroing inside the
            //band) keeps the response continuous - no step at the edge of the band.
            if ((abs _posErrX) <= AUTO_ATT_HOVER_POS_DEADBAND) then {
                _posErrX = 0.0;
            } else {
                _posErrX = _posErrX - (AUTO_ATT_HOVER_POS_DEADBAND * ([1, -1] select (_posErrX < 0.0)));
            };
            if ((abs _posErrY) <= AUTO_ATT_HOVER_POS_DEADBAND) then {
                _posErrY = 0.0;
            } else {
                _posErrY = _posErrY - (AUTO_ATT_HOVER_POS_DEADBAND * ([1, -1] select (_posErrY < 0.0)));
            };

            //PROPORTIONAL position recovery: metres of error -> m/s of commanded return.
            //This replaced an accumulating integral that SATURATED - measured in a real hover it
            //sat pinned at its clamp on both axes (setpoint 0.400, integral 0.300) while the
            //aircraft drifted aft at 3-5 m/s. It was asking to come home at 0.4 m/s against a
            //drift ten times faster, so the error grew without bound and it could never unwind:
            //the loop looked like it "never put in enough cyclic" because it was never ASKING
            //for enough. A proportional term cannot rail like that - the further off datum, the
            //faster the commanded return, up to the cap.
            _iX = [_posErrX * AUTO_ATT_HOVER_POS_PKP, -AUTO_ATT_HOVER_POS_PCLAMP, AUTO_ATT_HOVER_POS_PCLAMP] call BIS_fnc_clamp;
            _iY = [_posErrY * AUTO_ATT_HOVER_POS_PKP, -AUTO_ATT_HOVER_POS_PCLAMP, AUTO_ATT_HOVER_POS_PCLAMP] call BIS_fnc_clamp;
            _heli setVariable ["bmkhs_prestonHoverIntX", _iX];
            _heli setVariable ["bmkhs_prestonHoverIntY", _iY];

            //Off the cyclic the setpoint is purely the position-recovery bias. NEGATE the lateral
            //one: _iX comes from _posErrX in RAW model space (+X right), and the roll channel
            //works in the negated-X frame (see the setpoint note above). Pitch needs no negation.
            _setVelX = -_iX;
            _setVelY =  _iY;
        };

        //Velocity-null PIDs, copied from the FMC position hold (fn_fmcAttitudeHold "pos"): roll
        //measures -velX, pitch measures +velY, output used AS-IS.
        //DO NOT add the attitude branch's output negation - "pos"/"vel" do not negate, only "att"
        //does. Negating here inverts both axes.
        //
        //ACCELERATION LEAD: the PID measures vel + accel * horizon, not raw velocity, so it
        //commands cyclic as drift BEGINS rather than chasing it. Clamped so a spike cannot rail
        //the command.
        private _leadX = [(_heli getVariable ["bmkhs_accelX", 0.0]) * AUTO_ATT_ACCEL_LEAD,
                          -AUTO_ATT_ACCEL_LEAD_CLAMP, AUTO_ATT_ACCEL_LEAD_CLAMP] call BIS_fnc_clamp;
        private _leadY = [(_heli getVariable ["bmkhs_accelY", 0.0]) * AUTO_ATT_ACCEL_LEAD,
                          -AUTO_ATT_ACCEL_LEAD_CLAMP, AUTO_ATT_ACCEL_LEAD_CLAMP] call BIS_fnc_clamp;
        private _predVelX = _velX + _leadX;
        private _predVelY = _velY + _leadY;

        _hovRollOut  = [_pidHovX, _deltaTime, ( _setVelX), (-_predVelX)] call bmkhs_fnc_pidRun;
        _hovRollOut  = [_hovRollOut,  -1.0, 1.0] call BIS_fnc_clamp;
        _hovPitchOut = [_pidHovY, _deltaTime, ( _setVelY), ( _predVelY)] call bmkhs_fnc_pidRun;
        _hovPitchOut = [_hovPitchOut, -1.0, 1.0] call BIS_fnc_clamp;

        //LEARN the converged integral while the hover is genuinely settled, so the next entry (and
        //crucially the next PICKUP) can seed straight to it instead of climbing from near zero.
        //Only trusted when the aircraft is actually holding station on BOTH axes - otherwise the
        //integral is mid-transient and remembering it would bake in the error.
        if ((abs _velX) < AUTO_ATT_HOVER_SEED_LEARN_VEL
         && (abs _velY) < AUTO_ATT_HOVER_SEED_LEARN_VEL) then {
            _heli setVariable ["bmkhs_prestonLearnedIntX", _pidHovX get "integral"];
            _heli setVariable ["bmkhs_prestonLearnedIntY", _pidHovY get "integral"];
        };

        //DIAGNOSTIC - publish the hover loop's internals so the actual numbers can be read in the
        //debug hint instead of inferred from behaviour. Remove once this loop is settled.
        _heli setVariable ["bmkhs_dbgHovSetX",  _setVelX];
        _heli setVariable ["bmkhs_dbgHovSetY",  _setVelY];
        _heli setVariable ["bmkhs_dbgHovVelX",  _velX];
        _heli setVariable ["bmkhs_dbgHovVelY",  _velY];
        _heli setVariable ["bmkhs_dbgHovOutR",  _hovRollOut];
        _heli setVariable ["bmkhs_dbgHovOutP",  _hovPitchOut];
        _heli setVariable ["bmkhs_dbgHovIntR",  _pidHovX get "integral"];
        _heli setVariable ["bmkhs_dbgHovIntP",  _pidHovY get "integral"];
    } else {
        //Regime not in play. Keep the datum under the aircraft so the first frame back in hover
        //does not lurch, and clear the POSITION-recovery bias (that is a function of where the
        //datum is, and the datum just moved).
        _heli setVariable ["bmkhs_prestonHoverDatum", getPos _heli];
        _heli setVariable ["bmkhs_prestonHoverIntX",  0.0];
        _heli setVariable ["bmkhs_prestonHoverIntY",  0.0];

        //SEED the velocity PIDs' integrators rather than zeroing them. A hover needs a standing
        //cyclic offset and the integral carries it, so starting from zero means winding up from
        //scratch while the aircraft drifts - which rings on entry.
        //Prefer the value the integral converged to during the last steady hover; fall back to a
        //fraction of current trim before anything has been learned. Divided by ki because pidRun
        //multiplies it back. SEED_FRACTION < 1 because kp supplies most of the settled trim, not
        //the integral.
        private _pidHovXr = _heli getVariable "bmkhs_pid_prestonHoverX";
        private _pidHovYr = _heli getVariable "bmkhs_pid_prestonHoverY";
        [_pidHovXr] call bmkhs_fnc_pidReset;
        [_pidHovYr] call bmkhs_fnc_pidReset;

        private _kiX = _pidHovXr get "ki";
        private _kiY = _pidHovYr get "ki";
        //Roll trim is in the negated-X frame (see the setpoint notes above); pitch is not.
        private _seedFrac = AUTO_ATT_HOVER_SEED_FRAC;
        private _seedX = if (_kiX > 0.0) then { -(_heli getVariable ["bmkhs_forceTrimPosRoll",  0.0]) * _seedFrac / _kiX } else { 0.0 };
        private _seedY = if (_kiY > 0.0) then {  (_heli getVariable ["bmkhs_forceTrimPosPitch", 0.0]) * _seedFrac / _kiY } else { 0.0 };
        private _learnedX = _heli getVariable ["bmkhs_prestonLearnedIntX", -9999];
        private _learnedY = _heli getVariable ["bmkhs_prestonLearnedIntY", -9999];
        if (_learnedX != -9999) then { _seedX = _learnedX; };
        if (_learnedY != -9999) then { _seedY = _learnedY; };
        //Never seed outside the anti-windup clamp, or the first frame would start already railed.
        private _clX = _pidHovXr get "ki_clamp";
        private _clY = _pidHovYr get "ki_clamp";
        _pidHovXr set ["integral", [_seedX, -_clX, _clX] call BIS_fnc_clamp];
        _pidHovYr set ["integral", [_seedY, -_clY, _clY] call BIS_fnc_clamp];
    };

    /////////////////////////////////////////////////////////////////////////////////////////
    // BLEND + OUTPUT. Weighted mix of the three regimes (the weights sum to 1), then slewed
    // into force-trim so even a step in PID output reaches the swashplate as a ramp.
    /////////////////////////////////////////////////////////////////////////////////////////
    private _pitchGoal = (_hovPitchOut * _wPos) + (_velPitchOut * _wVel) + (_attPitchOut * _wAtt);
    private _rollGoal  = (_hovRollOut  * _wPos) + (_velRollOut  * _wVel) + (_attRollOut  * _wAtt);

    //PILOT HANDS: lag THEN rate-limit, exactly as the auto-pedal models the feet (see the
    //AUTO_ATT_CYCLIC_* note in core.hpp for why that order and not the reverse). This replaces the
    //old plain lerp-into-trim, which was a single smoothing stage with no rate limit.
    private _pitchPrev = _heli getVariable ["bmkhs_prestonPrevPitch", 0.0];
    private _rollPrev  = _heli getVariable ["bmkhs_prestonPrevRoll",  0.0];
    private _lagCoef   = [(_deltaTime / AUTO_ATT_CYCLIC_TAU), 0.0, 1.0] call BIS_fnc_clamp;
    private _maxStep   = AUTO_ATT_CYCLIC_RATE * _deltaTime;

    private _pitchLag  = _pitchPrev + ((_pitchGoal - _pitchPrev) * _lagCoef);
    private _pitchStep = [_pitchLag - _pitchPrev, -_maxStep, _maxStep] call BIS_fnc_clamp;
    private _pitchTrim = [_pitchPrev + _pitchStep, -1.0, 1.0] call BIS_fnc_clamp;

    private _rollLag   = _rollPrev + ((_rollGoal - _rollPrev) * _lagCoef);
    private _rollStep  = [_rollLag - _rollPrev, -_maxStep, _maxStep] call BIS_fnc_clamp;
    private _rollTrim  = [_rollPrev + _rollStep, -1.0, 1.0] call BIS_fnc_clamp;

    _heli setVariable ["bmkhs_prestonPrevPitch", _pitchTrim];
    _heli setVariable ["bmkhs_prestonPrevRoll",  _rollTrim];
    _heli setVariable ["bmkhs_forceTrimPosPitch",   _pitchTrim, true];
    _heli setVariable ["bmkhs_forceTrimPosRoll",    _rollTrim,  true];

    _heli setVariable ["bmkhs_prestonPitchActive", true];
    _heli setVariable ["bmkhs_prestonRollActive",  true];
};

//Preston owns the cyclic, so the raw stick is ZEROED - otherwise a held key adds unbounded control
//on top of the clamped target and pitch/bank run past every envelope limit. Force-trim, which is
//clamped, becomes the only path to the rotor.
[0.0, 0.0]
