//Wheels and suspension - Core's constants, the same for every aircraft. Geometry, friction and
//the brake and lock controls are the aircraft's: class Wheels, AIRCRAFT_GUIDE.md.
#ifndef BMKHS_HELISIM_SUSPENSION_HPP
#define BMKHS_HELISIM_SUSPENSION_HPP

//The strut: load = static * (compression / sag) ^ exp
#define SUSP_SPRING_EXP         1.5
#define SUSP_ZETA_COMPRESS      0.5
#define SUSP_ZETA_REBOUND       0.7

//Stability limits per unit of the wheel's static load: stiffness * dt^2 and damping * dt
#define SUSP_SPRING_DT_LIMIT    0.05
#define SUSP_DAMP_DT_LIMIT      0.2
#define SUSP_MAX_RATE           5.0     //m/s, compression rate
#define SUSP_MAX_LOAD           5.0     //multiples of the wheel's static load
//Least share of the mass a wheel carries in the static split
#define SUSP_MIN_LOAD_FRAC      0.05

//How far friction acts from the height of the centre of mass toward the wheel, 0 to 1
#define SUSP_ROLL_INFLUENCE     0.25

//The tyre's grip before it slips, per unit of the wheel's load: rad/s and damping
#define SUSP_GRIP_OMEGA         10.0
#define SUSP_GRIP_ZETA          1.0

#define SUSP_CASTOR_ANIM_STEP   0.5     //deg, change needed to update the animation
#define SUSP_STROKE_ANIM_STEP   0.01    //of sag, change needed to update the animation

//The ray starts this far above the wheel's lowest point
#define SUSP_RAY_ABOVE          1.5
//Placing at load-in: ground further than the range is airborne, faster than the speed is rolling
#define SUSP_PLACE_RANGE        5.0     //m
#define SUSP_PLACE_MAX_SPEED    3.0     //m/s

//Load-in: held until frames have stayed under the long frame time for this long, or the limit
#define SUSP_LOAD_LONG_FRAME    0.09    //s
#define SUSP_LOAD_STEADY_TIME   1.0     //s
#define SUSP_LOAD_MAX_TIME      5.0     //s

//Holding an aircraft at rest, once it has been at rest this long
#define SUSP_REST_SPEED         0.3     //m/s
#define SUSP_REST_RATE          0.05    //rad/s
#define SUSP_REST_TIME          0.25    //s
//A crewed aircraft sleeps with the brake on and the collective at or below this
#define SUSP_SLEEP_COLLECTIVE   0.1
#define SUSP_WAKE_MASS_FRAC     0.01
#define SUSP_WAKE_SPEED         0.5     //m/s

#endif
