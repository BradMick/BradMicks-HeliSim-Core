#ifndef BMKHS_HELISIM_ENGINE_HPP
#define BMKHS_HELISIM_ENGINE_HPP

//Limiter approach bands. The limiter closes over a margin rather than switching at the line,
//so it does not chatter frame to frame once the engine is sitting on a limit. Core's, not the
//airframe's: how a limiter approaches its setpoint is a property of the control law.
#define GT_TGT_LIMIT_BAND   40.0    //deg C below maxTgt over which fuel authority closes
#define GT_NG_LIMIT_BAND    0.030   //Ng fraction below the ceiling over which it closes
#define GT_LIMIT_GAIN       16.0    //fuel per second per unit of limiter error
#define GT_LIMIT_TRACK      1.02    //how far above the governor's demand the allowance rides

//Oil pressure at 100% Ng, as a gauge fraction. The pump is on the gas generator shaft, so
//pressure tracks Ng rather than torque.
#define GT_OIL_PSI_SCALE    0.90

//Single engine - contingency - when an engine's torque is below this fraction of another's.
#define GT_SINGLE_ENG_TQ_RATIO  0.51

//No systems: the lever goes to FLY once Ng holds at this fraction of idle Ng for this long.
#define GT_IDLE_STABLE_FRAC     0.99
#define GT_IDLE_TO_FLY_SEC      1.0

#endif
