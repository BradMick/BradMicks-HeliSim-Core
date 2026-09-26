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

#endif
