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
//  class Targets  {min, max, step, wraps} for each target the aircraft has:
//                 ralt[] / alt[] / altp[] ft, ias[] kt, hdg[] deg (wraps = 1)
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
//  maxPitchDeg  IAS pitch limit. 15
//  pitchRateDps how fast the pitch asked for may change, deg/s. 3
//  maxBankDeg   HDG / NAV bank limit. 30
//  turnRateDps  turn rate HDG / NAV bank for, deg/s - standard rate. The bank comes from the
//               airspeed: tan(bank) = V * rate / g. 3
//  bankPerDeg   bank per degree of heading error, so it rolls out onto the heading. 1
//  rollRateDps  how fast the bank asked for may change, deg/s. 5
//  bankAboveKts turns by bank above this ground speed; below it, by pedal with the wings level. 20
//  hvrDecelKts  HVR's deceleration to the hover, kt per second. 2
//  collAuthority / cycAuthority / pedAuthority   output limits. 1.0 / 0.1 / 0.1 - the
//               collective is full travel, continuous torque limits it
//  vs[]         gains, climb rate (m/s) to collective
//  ias[]        gains, airspeed (m/s) to pitch attitude (deg)
//  pitch[] roll[]   gains, attitude (deg) to cyclic
//  yaw[]        gains, heading (deg) to pedal
//  hvrPitch[] hvrRoll[]   gains, ground velocity (m/s) to cyclic, slowing to the hover
//
//Each command is eased onto at its rate from where the aircraft is, so a step in a target is
//never a step in a control. Fields are in pilot units; the director works, and publishes its
//targets, in m, m/s and deg.
//
//An axis the director holds replaces that axis's hold. Engaging a mode cancels the others on
//its axis: vertical {ralt, alt, altp}, longitudinal {ias, hvr}, lateral {hdg, nav}. HVR takes
//the ground velocity it is engaged at down to nothing at hvrDecelKts, then engages the
//AttitudeHold - in position hold by then - over the spot, and drops if that drops. HVR has the
//cyclic, so HDG / NAV turn by pedal under it. Only the vertical modes engage on the ground; NAV
//needs a waypoint.
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

#endif
