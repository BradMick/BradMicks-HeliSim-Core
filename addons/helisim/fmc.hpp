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
//Gains are {kp, ki, kd, ki_clamp}. A feature missing any of its gain arrays is skipped and
//logged. Every other field is optional and defaults to the value shown.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// Sas - rate damping
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  gate[]       when it works - the hydraulics and power its servos need
//  authority[]  {pitch, roll, yaw} output limit, control fraction. {0.2, 0.1, 0.1}
//  pitch[] roll[] yaw[]   gains, on body rate
//
//With SAS available the pilot's command is unlagged (the electrical servo path); without it
//the mechanical actuator lag applies.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// AttitudeHold - position, velocity or attitude, by ground speed
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  gate[]
//  posBelowKts  position hold at or below. 5
//  velBelowKts  accelerating, velocity hold up to this, attitude hold above. 40
//  attBelowKts  decelerating, attitude hold down to this, velocity hold below. 30
//  authority    output limit, cyclic fraction. 0.1
//  posPitch[] posRoll[]   gains, position and velocity hold
//  attPitch[] attRoll[]   gains, attitude hold
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
//  rad[] bar[]  gains
//
/////////////////////////////////////////////////////////////////////////////////////////////
// HeadingHold - heading below, yaw damping and turn coordination above
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  gate[]
//  hdgBelowKts  heading hold below this ground speed... 5
//  blendToKts   ...blending to yaw damping / turn coordination by this one. 40
//  breakout[]   pedal that drops the hold, by the attitude hold's sub-mode - {pos, vel, att},
//               so it changes band on the attitude hold's speeds and hysteresis. {0.05, 0.10, 0.20}
//  authority    output limit, pedal fraction. 0.1
//  hdg[]        gains, heading
//  trn[]        gains, yaw damping and turn coordination (on lateral g)
//
/////////////////////////////////////////////////////////////////////////////////////////////
// FlightDirector - coupled modes
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  gate[]
//  modes[]      the modes the aircraft has, of "ralt" "alt" "altp" "ias" "hdg" "nav" "hvr"
//  class Targets  {min, max, step, wraps} for each target the aircraft has, in its units:
//                 ralt[] / alt[] / altp[] ft, ias[] kt, hdg[] deg (wraps = 1)
//  altGain      climb rate per foot of altitude error, fpm/ft. 10
//  vsMaxFpm     climb rate limit. 1000
//  tqBand       torque fraction below continuous over which the climb tapers. 0.05
//  tqFpmGain    climb rate taken off per unit of torque over continuous, fpm. 20000
//               The vertical modes never take the engines past continuous torque - each
//               engine's first tqLimits[] tier, or tqLimitsSe[]'s single-engine. Inside tqBand
//               of it, the climb asked for beyond the present one tapers with the headroom left,
//               to none at the limit; over it, a little less than the present one.
//  captureFt    ALTP hands over to ALT inside this. 50
//  maxPitchDeg  IAS pitch limit. 15
//  maxBankDeg   HDG / NAV bank limit. 30
//  turnRateDps  turn rate HDG / NAV bank for, deg/s - standard rate. The bank comes from the
//               speed: tan(bank) = V * rate / g. 3
//  bankPerDeg   bank per degree of heading error, so it rolls out onto the heading. 1
//  bankAboveKts turns by bank above this; below it, by pedal with the wings level. 20
//  collAuthority / cycAuthority / pedAuthority   output limits. 1.0 / 0.1 / 0.1 - the
//               collective is full travel, continuous torque limits it
//  vs[]         gains, climb rate to collective
//  ias[]        gains, airspeed to pitch attitude
//  pitch[] roll[]   gains, attitude to cyclic
//  yaw[]        gains, heading to pedal
//
//An axis the director holds replaces that axis's hold. Engaging a mode cancels the others on
//its axis: vertical {ralt, alt, altp}, longitudinal {ias, hvr}, lateral {hdg, nav}. HVR is the
//aircraft's AttitudeHold. Only the vertical modes engage on the ground; NAV needs a waypoint.
//
//Inputs are Core's own actions, through the established dispatchers - the cockpit calls them
//by the same names:
//    bmkhs_fd<Mode>                  toggle, bmkhs_fnc_inputControlHandle
//    bmkhs_fd<Tgt>Up / Dn / Sync     step or sync a target, bmkhs_fnc_inputControlHandle
//    bmkhs_fd<Tgt>Target             set a target, bmkhs_fnc_inputAnalogHandler - the value is a
//                                    fraction 0..1 of its range, so an axis and a knob are alike
//    bmkhs_fdWaypoint                the active waypoint, posASL or [] - the aircraft writes it
//
//Published: bmkhs_fd_<mode> (engaged), bmkhs_fdTgt_<target>, bmkhs_fdWptBearing and
//bmkhs_fdWptDistance (-1 with none), all networked. Event fdModeChanged [mode, engaged].
//
//Published: bmkhs_fmcSasAvail, bmkhs_fmcAttHoldAvail, bmkhs_fmcAltHoldAvail,
//bmkhs_fmcHdgHoldAvail, bmkhs_fmcFdAvail - declared and its gate holding, networked.

#endif
