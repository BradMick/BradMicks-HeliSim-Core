#ifndef BMKHS_HELISIM_FMC_HPP
#define BMKHS_HELISIM_FMC_HPP

//BMKHS flight management computer - the augmentation and holds an aircraft declares.
//
//Declared in class FMC, beside ControlMixing in helisim_flightControls.hpp. Each feature is
//a class; a feature not declared does not exist - its outputs are 0 and its keys do nothing.
//
//    class FMC {
//        class Sas {...};
//        class AttitudeHold {...};
//        class AltitudeHold {...};
//        class HeadingHold {...};
//    };
//
//Every feature takes a gate[] - the same form as a component gate: a variable name, or
//{circuit, threshold}, all must hold. A closed gate is the feature off: no output, its loops
//reset. The FMC channels (bmkhs_fnc_fmcSetChannel) still switch individual axes on top.
//
//Gains are {kp, ki, kd, limit}, per pilot unit of error (deg, deg/s, kt, ft, fpm, g as
//listed); Core converts them once at init. limit = the most the integral can push, in the
//loop's output (control fraction; deg where the output is an attitude), whatever ki is. The gains for
//what the aircraft runs are required - a feature missing one is skipped and logged; gains for
//what it does not run may be left out. Every other field is optional and defaults to the value
//shown - the FMC_ / TRIM_ defines at the end.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// Trim - where the controls sit with no force on them
/////////////////////////////////////////////////////////////////////////////////////////////
//
//Declared in class Trim, beside class FMC. It needs no FMC. Every aircraft has force trim -
//centre-sprung sticks need it - and the force trim button sets the trim position to where the
//controls are. No class Trim is force trim with no beep.
//
//    class Trim {
//        class Beep {...};
//    };
//
//The force trim switch is bmkhs_fmcTrimOn (bmkhs_fnc_fmcSetChannel, "trim"). Off, the trim
//positions are held at 0 and the force trim button and beep do nothing; back on, the trim is
//set to where the controls are. A casual assist that owns an axis keeps writing it.
//
//Beep - trim moved by a switch, at a rate while held. Undeclared, the aircraft has no beep.
//  gate[]       when it works - the trim actuators' power. Component gate form
//  rate[]       {pitch, roll} trim travel per second held, control fraction. {0.10, 0.10}
//It moves 0.1 s after the press. A press made on a flight director axis moves that director's
//target instead, until it is released. Beep does nothing on an axis a casual assist owns, or
//with a springless cyclic or sticky keyboard on that axis.
//
//Inputs, through bmkhs_fnc_inputControlHandle:
//    bmkhs_beepTrimForward / Backward / Left / Right    held
//    bmkhs_forceTrim                                    held, release sets the trim
//    bmkhs_forceTrimPanic                               zero the trim and keyboard input
//
//Published: bmkhs_forceTrimPosPitch / Roll / Yaw (-1..1, + = forward / left / right, in the
//packed state), bmkhs_trimOn (force trim working, local).
//
/////////////////////////////////////////////////////////////////////////////////////////////
// Sas - rate damping
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  gate[]       when it works - the hydraulics and power its servos need
//  authority[]  {pitch, roll, yaw} output limit, control fraction. {0.2, 0.1, 0.1}
//  pitch[] roll[] yaw[]   gains, body rate (deg/s) to control
//
//With SAS available the pilot's command is unlagged (the electrical servo path); without it
//the mechanical actuator lag applies.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// AttitudeHold - position, velocity or attitude, by ground speed; airspeed, by KTAS
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  gate[]
//  subModes[]   any of "pos", "vel", "att", "spd". Where speed picks one not declared, the
//               nearest declared flies. {"att"} = attitude hold at all speeds. "spd" = airspeed
//               hold above spdAboveKts KTAS, to 10 kt below it: pitch holds the airspeed at which
//               the trim is set, within 10 deg of level; roll holds bank as att. Not under FD
//               HVR. {"pos", "vel", "att"}
//  posBelowKts  position hold at or below. 5
//  velBelowKts  accelerating, velocity hold up to this, attitude hold above. 40
//  attBelowKts  decelerating, attitude hold down to this, velocity hold below. 30
//  spdAboveKts  airspeed hold above this KTAS. 40
//  authority    output limit, cyclic fraction. 0.1
//  breakout     cyclic displacement from trim, per axis, past which that axis is not active - no
//               output; back inside, it returns to its reference. No sub-mode change meanwhile. 0.10
//  alwaysOn     1 = engaged whenever it can be, re-engaged after force trim; the keys do not
//               engage or release it. 0
//  trimGain[]   {pitch, roll} deg of attitude reference per unit of beep trim travel. With it,
//               beep (class Trim) moves the engaged hold's attitude reference with the
//               stick; in spd the speed once settled, 0.1 s at most, is the new reference. {0, 0}:
//               beep moves the stick only.
//  posPitch[] posRoll[]   gains, ground velocity (kt) to cyclic - pos and vel, and FD HVR
//  attPitch[] attRoll[]   gains, attitude (deg) to cyclic - att and spd
//  spdPitch[]   gains, airspeed (kt) to pitch attitude (deg) - spd
//
/////////////////////////////////////////////////////////////////////////////////////////////
// AltitudeHold - radar or barometric
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  gate[]
//  radBelowFt   radar hold below this height AGL... 1428
//  radBelowKts  ...and this ground speed; barometric otherwise. 40
//  engageFpm    engages only within this vertical speed. 200
//  collBand     drops out when the collective moves this fraction from where it engaged. 0.05
//  dropAboveTq  drops out at this engine torque fraction. 0.98
//  rad[] bar[]  gains, altitude (ft) to collective
//  beep[]       altitude beep: {step ft, rate ft/s, fast rate ft/s, fast after s}. A tap steps;
//               held, it moves at the rate, then the fast rate. None = no beep. {}
//
/////////////////////////////////////////////////////////////////////////////////////////////
// HeadingHold - heading on the pedals below, on the roll axis above; pedals on the ball above
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  gate[]
//  subModes[]   any of "hdg", "yaw", "trn". With "trn" (or "yaw") the pedals centre the ball at
//               speed and heading is held by bank; with neither, pedals hold heading at every
//               speed. {"hdg", "yaw", "trn"}
//  hdgBelowKts  heading on the pedals below this ground speed... 5
//  blendToKts   ...handed to bank and ball by this one. trn only at or above it; below it a
//               pilot action is yaw SAS until settled. 40
//  trnEnterBankDeg  bank that alone starts a turn; 0 = only pilot action does. 7
//  trnExit[]    turn ends, heading captured: {bank deg, yaw rate deg/s, held s}. {3, 3, 0}
//  breakout[]   pedal that drops the hold, by the attitude hold's sub-mode - {pos, vel, att},
//               so it changes band on the attitude hold's speeds and hysteresis. {0.05, 0.10, 0.20}
//  authority    output limit, pedal fraction. 0.1
//  hdg[]        gains, heading (deg) to pedal - hdg
//  trn[]        gains, yaw damping and turn coordination (on lateral g) - yaw and trn
//
/////////////////////////////////////////////////////////////////////////////////////////////
// FlightDirector - coupled modes
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  gate[]
//  modes[]      the modes the aircraft has, of "ralt" "alt" "altp" "ias" "hdg" "nav" "hvr"
//  class Targets  {min, max, step, wraps} for each target the aircraft has:
//                 ralt[] / alt[] / altp[] ft, ias[] kt, hdg[] deg (wraps = 1)
//  floorKts     every mode but RALT and HVR engages at or above this KTAS and drops 2 kt below
//               it. 0 = none
//  altGain      climb rate per foot of altitude error, fpm/ft. 10
//  vsMaxFpm     climb rate limit. 1000
//  vsAccelFpm   how fast the climb rate asked for may change, fpm per second. 200
//  tqBand       torque fraction below continuous inside which the limit acts. 0.1
//  tqFpmGain    climb allowed per unit of torque headroom, fpm. 1000 (10 fpm per 1 %)
//               The vertical modes never take the engines past continuous torque - each
//               engine's first tqLimits[] tier, or tqLimitsSe[]'s single-engine. Inside tqBand
//               of it the climb allowed is the present one plus headroom x tqFpmGain - a little
//               more with room, the present climb at the limit, less over it - so torque eases
//               onto the limit and stops there, in any mode. Too high a gain and it bounces off
//               the limit. The climb rate then eases onto that at vsAccelFpm, like any other change.
//               ALT / ALTP fly pressure altitude (bmkhs_barAlt, with the environment's base
//               altitude) - what the barometric altimeter reads; RALT flies radar height.
//  captureFt    ALTP hands over to ALT inside this. 50
//  iasAccelKts  how fast the airspeed asked for may change, kt per second. 2
//  maxPitchDeg  IAS / HVR pitch limit. 15
//  pitchRateDps how fast the pitch asked for may change, deg/s. 3
//  maxBankDeg   HDG / NAV / HVR bank limit. 30
//  turnRateDps  turn rate HDG / NAV bank for, deg/s - standard rate. The bank comes from the
//               airspeed: tan(bank) = V * rate / g. 3
//  bankPerDeg   bank per degree of heading error, so it rolls out onto the heading. 1
//  rollRateDps  how fast the bank asked for may change, deg/s. 5
//  bankAboveKts turns by bank above this ground speed; below it, by pedal with the wings level. 20
//  hvrDecelKts  HVR's deceleration to the hover, kt per second. 2
//  collAuthority / cycAuthority / pedAuthority   output limits. 1.0 / 0.1 / 0.1 - the
//               collective is full travel, continuous torque limits it
//  vs[]         gains, climb rate (fpm) to collective - ralt, alt, altp
//  ias[] pitch[]  gains, airspeed (kt) to pitch attitude (deg), attitude to cyclic - ias
//  roll[] yaw[]   gains, bank (deg) to cyclic, heading (deg) to pedal - hdg, nav
//  hvrPitch[] hvrRoll[]   gains, ground velocity (kt) to pitch / bank (deg), slowing to the
//               hover - hvr, flown by pitch[] / roll[]. Stopped, the AttitudeHold's position
//               hold holds the spot (posPitch[] / posRoll[]). Flown through, force trimmed or
//               beeped, HVR slows again and anchors where it stops.
//
//Each command is eased onto at its rate from where the aircraft is, so a step in a target is
//never a step in a control. Fields are in pilot units; the director works, and publishes its
//targets, in m, m/s and deg.
//
//An axis the director holds replaces that axis's hold. Engaging a mode cancels the others on
//its axis: vertical {ralt, alt, altp}, longitudinal {ias, hvr}, lateral {hdg, nav}. HVR takes
//the ground velocity it is engaged at down to nothing at hvrDecelKts, then engages the
//AttitudeHold - in position hold by then - over the spot, and drops if that drops. HVR is a
//low speed mode: it engages, and stays, at or below 40 kt ground speed. HVR has the cyclic, so
//HDG / NAV turn by pedal under it. Only the vertical modes engage on the ground; NAV needs a
//waypoint. Taking or giving up an axis, the director blends with the hold over 1 s.
//
//Force trim pressed and the director stands aside on the cyclic - the pilot flies it. Released,
//IAS and HDG resume onto their targets; HVR takes the ground velocity the aircraft has and
//slows to a hover over the new spot. A cyclic axis past the AttitudeHold breakout stands aside
//on that axis only; back inside, it resumes on its reference.
//
//RALT, ALT, IAS and HDG engage on where the aircraft is - that is the target; ALTP flies to its
//preselect. A beep pressed on a director axis moves its target, not the stick: fore/aft the IAS
//target at iasAccelKts, lateral the HDG target at turnRateDps. In HVR both axes move the ground
//velocity it holds at 1 kt/s - beep away from the hover, and back to nothing to hold the new
//spot. The press stays on the target until released, even if the mode drops.
//
//Inputs are Core's own actions, through the established dispatchers - the cockpit calls them
//by the same names:
//    bmkhs_fd<Mode>                  toggle, bmkhs_fnc_inputControlHandle
//    bmkhs_fd<Tgt>Up / Dn / Sync     step or sync a target, bmkhs_fnc_inputControlHandle
//    bmkhs_fd<Tgt>Target             set a target, bmkhs_fnc_inputAnalogHandler - the value is a
//                                    fraction 0..1 of its range, so an axis and a knob are alike
//    bmkhs_fdWaypoint                the active waypoint, posASL or [] - the aircraft writes it
//
//Published: bmkhs_fd_<mode> (engaged), bmkhs_fdTgt_<target> (m, m/s, deg), bmkhs_fdWptBearing and
//bmkhs_fdWptDistance (-1 with none), all networked. Event fdModeChanged [mode, engaged].
//
//Published: bmkhs_fmcSasAvail, bmkhs_fmcAttHoldAvail, bmkhs_fmcAltHoldAvail,
//bmkhs_fmcHdgHoldAvail, bmkhs_fmcFdAvail - declared and its gate holding, networked.

/////////////////////////////////////////////////////////////////////////////////////////////
// Defaults - a field an aircraft leaves out
/////////////////////////////////////////////////////////////////////////////////////////////
//Trim
#define TRIM_BEEP_RATE              [0.10, 0.10]
#define TRIM_BEEP_LAG               0.1         //s
//Sas
#define FMC_SAS_AUTHORITY           [0.2, 0.1, 0.1]
//AttitudeHold
#define FMC_ATT_SUB_MODES           ["pos", "vel", "att"]
#define FMC_ATT_POS_BELOW_KTS       5
#define FMC_ATT_VEL_BELOW_KTS       40
#define FMC_ATT_ATT_BELOW_KTS       30
#define FMC_ATT_SPD_ABOVE_KTS       40
#define FMC_ATT_AUTHORITY           0.1
#define FMC_ATT_ALWAYS_ON           0
#define FMC_ATT_TRIM_GAIN           [0, 0]
#define FMC_ATT_SPD_BAND_KTS        10          //spd out this far below spdAboveKts
#define FMC_ATT_SPD_MAX_PITCH_DEG   10          //from level
#define FMC_ATT_SPD_SETTLE_KTS      0.5         //kt/s
#define FMC_ATT_SPD_DELAY_S         0.1        //beep release to capture, max
#define FMC_ATT_BREAKOUT            0.10        //cyclic throw
#define FMC_ATT_POS_GAIN            0.2         //closing speed per m off the spot, 1/s
#define FMC_ATT_POS_MAX_KTS         3           //closing speed, max
//AltitudeHold
#define FMC_ALT_RAD_BELOW_FT        1428
#define FMC_ALT_RAD_BELOW_KTS       40
#define FMC_ALT_ENGAGE_FPM          200
#define FMC_ALT_COLL_BAND           0.05
#define FMC_ALT_DROP_ABOVE_TQ       0.98
#define FMC_ALT_BEEP_HOLD_S         0.5         //held this long, the beep moves at its rate
//HeadingHold
#define FMC_HDG_SUB_MODES           ["hdg", "yaw", "trn"]
#define FMC_HDG_BELOW_KTS           5
#define FMC_HDG_BLEND_TO_KTS        40
#define FMC_HDG_AUTHORITY           0.1
#define FMC_HDG_BREAKOUT            [0.05, 0.10, 0.20]
#define FMC_HDG_TRN_ENTER_BANK_DEG  7
#define FMC_HDG_TRN_EXIT            [3, 3, 0]   //deg, deg/s, s
#define FMC_HDG_BANK_PER_DEG        1           //heading error to bank, at speed
#define FMC_HDG_MAX_BANK_DEG        3
#define FMC_HDG_SAS_REENGAGE_S      1           //attitude hold off
//FlightDirector
#define FMC_FD_ALT_GAIN             10
#define FMC_FD_VS_MAX_FPM           1000
#define FMC_FD_VS_ACCEL_FPM         200
#define FMC_FD_TQ_BAND              0.1
#define FMC_FD_TQ_FPM_GAIN          1000
#define FMC_FD_CAPTURE_FT           50
#define FMC_FD_IAS_ACCEL_KTS        2
#define FMC_FD_MAX_PITCH_DEG        15
#define FMC_FD_PITCH_RATE_DPS       3
#define FMC_FD_MAX_BANK_DEG         30
#define FMC_FD_TURN_RATE_DPS        3
#define FMC_FD_BANK_PER_DEG         1
#define FMC_FD_ROLL_RATE_DPS        5
#define FMC_FD_BANK_ABOVE_KTS       20
#define FMC_FD_HVR_DECEL_KTS        2
#define FMC_FD_FLOOR_KTS            0
#define FMC_FD_COLL_AUTHORITY       1.0
#define FMC_FD_CYC_AUTHORITY        0.1
#define FMC_FD_PED_AUTHORITY        0.1
#define FMC_FD_FLOOR_DROP_KTS       2           //modes drop this far below floorKts
#define FMC_FD_HVR_BELOW_KTS        40          //ground speed
#define FMC_FD_HVR_BEEP_KTS         1           //kt/s
#define FMC_FD_ACCEL_FILTER_S       0.5         //ground acceleration filter, HVR base attitude
#define FMC_FD_HVR_HOLD_KTS        0.5         //stopped - slowed or beeped below this, holds the spot
#define FMC_FD_HANDOFF_SEC          1.0

#endif
