#ifndef BMKHS_HELISIM_SYSTEMS_HPP
#define BMKHS_HELISIM_SYSTEMS_HPP

//Backstop on the dirty walk. A change propagates along its own chain and stops, so this is
//only ever hit by a component pair that keeps dirtying each other - a declaration bug, not
//a deep graph. Generous enough that no real airframe reaches it.
#define SYS_WALK_LIMIT  256

//Damage threshold for any declared component, whatever kind or domain.
#define SYS_COMP_DMG_THRESH  0.85

#define SYS_APU_DMG_THRESH   0.85
#define SYS_BATT_DMG_THRESH  0.85
#define SYS_GEN_DMG_THRESH   0.85
#define SYS_RECT_DMG_THRESH  0.85
#define SYS_ENG_DMG_THRESH   0.85
#define SYS_XMSN_DMG_THRESH  0.85
#define SYS_NGB_DMG_THRESH   0.85
#define SYS_IGB_DMG_THRESH   0.85
#define SYS_TGB_DMG_THRESH   0.85
#define SYS_FCR_DMG_THRESH   0.85
#define SYS_STAB_DMG_THRESH  0.85
#define SYS_HYD_DMG_THRESH   0.85
#define SYS_ASE_DMG_THRESH   0.85
#define SYS_SIGHT_DMG_THRESH 0.85
#define SYS_WPN_DMG_THRESH   0.85

#define SYS_HYD_RES_MIN_DMG 0.50
#define SYS_HYD_RES_MOD_DMG 0.67
#define SYS_HYD_RES_HVY_DMG 0.83
#define SYS_HYD_MIN_RTR_RPM 0.45

#define SYS_MIN_RPM       0.85

#define SYS_MIN_HYD_PSI   1260
#define SYS_MIN_ACC_PSI   1650
//How quickly a store refills once whatever it started is turning.
#define SYS_START_RECHARGE_SEC  1.0
#define SYS_HYD_MIN_LVL   0.1

#define SYS_BATT_TIMER    12.0  //min
#define SYS_ACC_TIMER     1.5   //min
#define SYS_LEAK_TIMER    2.0   //min

//Damage timers
#define DMG_PER_SEC       0.003 //5 minutes total time

#endif
