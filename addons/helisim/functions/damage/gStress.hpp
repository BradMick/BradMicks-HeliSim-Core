//G-stress fatigue - Core's, the same for every aircraft
#ifndef BMKHS_HELISIM_GSTRESS_HPP
#define BMKHS_HELISIM_GSTRESS_HPP

//Each limit passed adds (g - limit) / divisor per second
#define GSTRESS_LIMITS          [3.5, 4.0, 4.5, 5.0, 6.0]
#define GSTRESS_DIVISORS        [40.0, 30.0, 20.0, 10.0, 5.0]
#define GSTRESS_G_CAP           7.0

//Rate multiplier by fatigue - the steps replace each other
#define GSTRESS_PROGRESS_STEPS  [0.25, 0.50, 0.75]
#define GSTRESS_PROGRESS_MULT   [1.5, 2.0, 3.0]

//Failure chance per unit of fatigue gained
#define GSTRESS_EVENT_SCALE     4.0

#define BLADE_OK                0
#define BLADE_OUT_OF_TRACK      1
#define BLADE_TIP_CAP           2
#define BLADE_PC_LINK           3
#define BLADE_SEPARATED         4

#define BLADE_TRACK_STEP        0.04
#define BLADE_TIP_CAP_TRACK     0.08
#define BLADE_TIP_CAP_LIFT      0.95
#define BLADE_CG_FRACTION       0.5

//Rotor imbalance the transmission mounts take, N, and the damage per N.s past it
#define GSTRESS_MOUNT_RATING    50000.0
#define GSTRESS_MOUNT_DIVISOR   200000.0
//Chance per unit of that damage that another blade lets go
#define GSTRESS_SHED_SCALE      0.5

//Fatigue at which the tail structure fails
#define GSTRESS_STAB_FAIL       0.60
#define GSTRESS_FIN_FAIL        0.75
#define GSTRESS_BOOM_FAIL       0.90

//CG shift forward, m - the boom's is the total
#define GSTRESS_CG_BOOM         3.00
#define GSTRESS_CG_FIN          0.10
#define GSTRESS_CG_STAB         0.05

#endif
