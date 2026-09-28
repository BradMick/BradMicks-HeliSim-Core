#ifndef BMKHS_HELISIM_ENGDISPLAY_HPP
#define BMKHS_HELISIM_ENGDISPLAY_HPP

//Bar control pool size. Must match BMKHS_ENGDISP_MAX_ENG in ui\RscEngDisplay.hpp.
#define ED_MAX_ENG          4

//Ng below which a running engine reads as out
#define ED_ENG_OUT_NG       0.63

//Nr annunciation bands, as a fraction of 100%
#define ED_NR_LOW           0.95
#define ED_NR_HIGH          1.05

//Fuel remaining, as a fraction of capacity, below which the low caution shows
#define ED_FUEL_LOW         0.10

//Drift from the position hold point, in metres, that raises the hover drift advisory
#define ED_HOVER_DRIFT_M    14.630

//Torque tape full scale, as a fraction of rated torque
#define ED_TQ_FULL_SCALE    1.50

#endif
