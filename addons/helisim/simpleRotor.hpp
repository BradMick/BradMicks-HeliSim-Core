#ifndef BMKHS_HELISIM_SIMPLEROTOR_HPP
#define BMKHS_HELISIM_SIMPLEROTOR_HPP

//BMKHS simple rotor - a force generator an aircraft declares in tables.
//
//The simple rotor produces exactly two things: a THRUST vector and a TORQUE. Everything
//it declares exists to shape one of those two. It is not a blade model and does not try
//to be - it is a curve reader, and the curves are the aircraft's.
//
//THE TEST FOR A TABLE. If a designer cannot read it off a flight manual or sketch it by
//hand, it does not belong here. Four tables clear that bar. Nothing else needs to.
//
//THRUST AND TORQUE ARE INDEPENDENT. Both read collective and airspeed; NEITHER READS THE
//OTHER. Thrust never enters the power calculation and torque never scales thrust. That
//separation is deliberate - it is what lets collective feel and engine loading be tuned
//without one dragging the other around. Do not connect them.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// ONE MODEL, N ROTORS
/////////////////////////////////////////////////////////////////////////////////////////////
//
//A tail rotor is NOT a different model. It is the same force generator pointed sideways:
//
//    thrustAxis = "X"    thrust acts laterally instead of up
//    no ground effect    it is not near the ground in the way a main rotor is
//    no cyclic           pedal drives thrust, not disc tilt
//    no climb term       excess torque does not become tail thrust
//
//So both are declared the same way, in one class each, and Core loops them. A NOTAR, a
//coaxial or a tandem is a declaration, not a code change.
//
//    class Rotors {
//        class MainRotor { rotorType = "MAIN"; thrustAxis = "Z"; ... };
//        class TailRotor { rotorType = "TAIL"; thrustAxis = "X"; ... };
//    };
//
//Declaration order is the rotor index, and the index is what bmkhs_rtrThrust[] and
//bmkhs_reqEngTorque[] are keyed on. Reordering the classes reindexes both arrays.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// UNITS ARE DECLARED, NOT ASSUMED
/////////////////////////////////////////////////////////////////////////////////////////////
//
//Airspeed in KNOTS. Power in KILOWATTS. Thrust in NEWTONS. Torque in NEWTON-METRES.
//Angles in DEGREES. Mass in KILOGRAMS. Length in METRES.
//
//Core converts once at init and works in SI internally. Tables are authored in the units
//a flight manual uses, because that is where the numbers come from.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// DEFAULTS ARE NEUTRAL, NOT BORROWED
/////////////////////////////////////////////////////////////////////////////////////////////
//
//Every table defaults to a flat 1.0 and every gain to 0.0. An aircraft that declares
//nothing flies BADLY BUT HONESTLY - it does not quietly inherit another airframe's
//handling. "Flies like a brick" is diagnosable; "flies suspiciously well" is not.
//
//Core knows no aircraft, including the one it was born from.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// THE FOUR TABLES
/////////////////////////////////////////////////////////////////////////////////////////////
//
//Every table is {x, y} pairs, linearly interpolated, CLAMPED at both ends. Points must be
//in ascending x order. Off the end of a table you get the endpoint value, not an
//extrapolation - so the first and last points are the model's behaviour beyond them.
//
//----------------------------------------------------------------------------------------
//thrustVsAirspeed[]      airspeed (kt) -> thrust multiplier
//----------------------------------------------------------------------------------------
//Translational lift. The dip as the rotor outruns its own downwash, and the recovery as
//clean air arrives. A hover-tuned aircraft has its peak at 0 and its trough around 60-70.
//
//    thrustVsAirspeed[] = {{0, 1.164}, {40, 0.953}, {70, 0.848}, {140, 1.043}};
//
//----------------------------------------------------------------------------------------
//thrustVsCollective[]    control input -> thrust multiplier
//----------------------------------------------------------------------------------------
//Collective (main) or pedal (tail) to thrust. x runs 0..1 for a main rotor and -1..1 for
//a tail, because a tail rotor pushes both ways.
//
//A TAIL ROTOR IS USUALLY ASYMMETRIC. Left pedal and right pedal do not have the same
//authority, because the blade pitch range is not symmetric about zero. Declare the real
//curve; do not assume a straight line through the origin.
//
//    thrustVsCollective[] = {{-1.0, 2.0}, {0.0, 0.0}, {1.0, -1.0}};   //2:1 asymmetric tail
//
//----------------------------------------------------------------------------------------
//powerVsAirspeed[]       airspeed (kt) -> fraction of maxPower
//----------------------------------------------------------------------------------------
//THE POWER-REQUIRED CURVE, STRAIGHT OUT OF THE -10. High in the hover, falling to a
//minimum near best-endurance speed, rising again toward Vne. The power bucket.
//
//This one is transcribed, not tuned. If the aircraft has a published power-required
//chart, that chart IS this table.
//
//    powerVsAirspeed[] = {{0, 0.94}, {70, 0.49}, {150, 1.18}};
//
//----------------------------------------------------------------------------------------
//powerVsCollective[]     collective (0..1) -> power multiplier
//----------------------------------------------------------------------------------------
//How power demand grows with collective, INDEPENDENTLY of how thrust grows with it. This
//is the tuning knob: it is what makes pulling collective cost the engine something
//without changing what that collective does to lift.
//
//    powerVsCollective[] = {{0.0, 0.0}, {0.25, 1.0}, {0.85, 1.0}, {1.0, 1.5}};
//
/////////////////////////////////////////////////////////////////////////////////////////////
// FIELD REFERENCE
/////////////////////////////////////////////////////////////////////////////////////////////
//
//IDENTITY
//  rotorType           "MAIN" | "TAIL". MAIN gets ground effect, cyclic and climb thrust;
//                      TAIL gets none of them and drives yaw instead.
//  thrustAxis          "X" | "Y" | "Z" in model space. Z is up (main), X is lateral (tail).
//  direction           "CCW" | "CW". Sets the sign of the torque reaction on the fuselage.
//  position[]          {x, y, z} m in model space - where the force is applied.
//
//GEOMETRY
//  bladeRadius         m. Drives disc area (pi*r^2), tip speed, and the diameter the
//                      ground-effect falloff is measured in. NOT optional - induced
//                      velocity and the whole VRS band come off disc area.
//  rotorInertia        kg.m^2 about the mast. Resists RPM change: spin-up, coastdown and
//                      Nr droop under load. The spreadsheet computes it from blade mass,
//                      radius and blade count as (1/3)*m*r^2*Nb - which is 99.9% of the
//                      true figure for any conventional rotor.
//  bladePitchMin/Max   deg. Collective range, used to map input to blade pitch.
//  designRpm           Rotor RPM at 100% Nr.
//  gearRatio           Rotor to engine shaft. Shared with the transmission model.
//  rpmTrimVal          Nr trim reference, typically ~1.01.
//  heightAgl           m. Hub height above ground with the aircraft on its wheels.
//
//FORCE SCALE
//  baseThrust          N at a thrust multiplier of 1.0. Max gross weight * g is the
//                      sensible starting point - it makes "1.0" mean "hovers at max
//                      weight" and every table entry read as a fraction of that.
//  maxPower            kW at a power fraction of 1.0. Installed power, both engines.
//  refRpm              Transmission output RPM at 100% Nr. Normalises the RPM term.
//
//TUNING
//  groundEffectGain    Thrust gain at zero height, falling off over one rotor diameter as
//                      (1 - h/D). The SHAPE is physics and lives in Core; this is the
//                      STRENGTH, which is the airframe's. 0.0 disables ground effect.
//  climbGain           Excess torque -> climb thrust. MAIN only.
//  autoroTorque        Driving torque per m/s of descent in autorotation. MAIN only.
//  torqueScalar        TAIL only. Nm of engine torque per N of tail thrust. A TAIL ROTOR
//                      COSTS POWER - typically 5-15% of total - and costs MORE when you
//                      stomp a pedal. Scaling off the tail's own thrust means that falls
//                      out of thrustVsCollective[] for free.
//
//                      DECLARE THIS. The default is 0.0 because Core defaults everything
//                      to neutral, but a tail rotor that costs nothing to drive is wrong:
//                      there is no pedal-induced Nr droop and the engine never sees the
//                      yaw demand. Leaving it at 0.0 reproduces a bug the model had
//                      before the field existed, it is not a valid configuration.
//
//CONTROL AUTHORITY
//  cyclicPitchTorque   Nm of pitch moment at full cyclic. MAIN only.
//  cyclicRollTorque    Nm of roll moment at full cyclic. MAIN only.
//  pedalYawTorque      Nm of yaw moment at full pedal.
//  rollCouple          Fraction of this rotor's thrust moment that couples into roll.
//                      A tail rotor sits above the roll axis, so its thrust rolls the
//                      airframe as well as yawing it. 0.0 = no coupling.
//  thrustTiltRoll      deg of thrust-vector tilt per unit roll input.
//  flapbackLon/Lat     deg of disc tilt per unit advance ratio. The advancing blade lifts
//                      more than the retreating one, so the disc tilts as speed builds.
//
//ENVELOPE
//  vne                 kt. Never-exceed - used as a clamp ceiling on airspeed terms.
//  vbe                 kt. Best endurance. Where the power bucket bottoms out.
//  etl                 kt. Effective translational lift onset.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// WHAT CORE COMPUTES AND YOU NEVER DECLARE
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  induced velocity    sqrt(T / 2*rho*A) - momentum theory, from this frame's thrust
//  VRS band            scales with induced velocity, so it tracks the airframe for free
//  ground effect shape (1 - h/D) - dimensionless, universal
//  disc area           pi*r^2
//  tip speed           omega*r
//  air density ratio   rho / rho_ISA
//
//VORTEX RING STATE IS NOT DECLARED. It is derived every frame from thrust, air density
//and disc area - all of which the aircraft already declares or Core already knows. A
//bigger disc gets a wider VRS band automatically. There is no knob and there should not
//be one.
//
/////////////////////////////////////////////////////////////////////////////////////////////

#endif
