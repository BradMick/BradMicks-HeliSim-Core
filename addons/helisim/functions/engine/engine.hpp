#ifndef BMKHS_HELISIM_ENGINE_HPP
#define BMKHS_HELISIM_ENGINE_HPP

//Limiter approach bands. The limiter closes over a margin rather than switching at the line,
//so it does not chatter frame to frame once the engine is sitting on a limit. Core's, not the
//airframe's: how a limiter approaches its setpoint is a property of the control law.
#define GT_TGT_LIMIT_BAND   40.0    //deg C below maxTgt over which fuel authority closes
#define GT_NG_LIMIT_BAND    0.030   //Ng fraction below the ceiling over which it closes

//Oil pressure at 100% Ng, as a gauge fraction. The pump is on the gas generator shaft, so
//pressure tracks Ng rather than torque.
#define GT_OIL_PSI_SCALE    0.90

//Seconds for the power lever to travel idle to fly. Coming back to idle is instant.
#define GT_LEVER_TRAVEL_SEC 12.0

#endif
