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

//Seconds an engine's tank may run dry before it is starved.
#define FUEL_STARVE_GRACE_SEC   2

//Gas properties - cold air through the compressor, hot gas through the turbines.
#define GT_GAMMA_COLD       1.40
#define GT_CP_COLD          1.005   //kJ/kg K
#define GT_GAMMA_HOT        1.33
#define GT_CP_HOT           1.148   //kJ/kg K
#define GT_R_AIR            0.28705 //kJ/kg K
#define GT_STD_TEMP_K       288.15
#define GT_STD_PRESSURE_KPA 101.325

//TGT gauge lag - how fast the reading chases the gas at station 4.5.
#define GT_TGT_HEAT_RATE    0.30
#define GT_TGT_COOL_RATE    0.70
#define GT_TGT_STILL_AIR    0.0012
#define GT_TGT_RAM_AIR      0.00065

//Compressor power at Ng 1.0, ISA, in spool units - the spool's torque scale.
#define GT_SPOOL_UNIT_LOAD  2.91244

#endif
