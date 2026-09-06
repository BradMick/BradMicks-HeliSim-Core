/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_rotorVariables

Description:
    Defines core rotor variables.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

//CONFIG - what the aircraft IS. Per-rotor arrays, index 0 = main, 1 = tail.
_heli setVariable ["bmkhs_numRotors",          getNumber (_config >> "numRotors")];
_heli setVariable ["bmkhs_rotorType",          getArray  (_config >> "rotorType")];
_heli setVariable ["bmkhs_rotorDirection",     getArray  (_config >> "rotorDirection")];
_heli setVariable ["bmkhs_rotorNumBlades",     getArray  (_config >> "rotorNumBlades")];
_heli setVariable ["bmkhs_rotorNumElements",   getArray  (_config >> "rotorNumElements")];
_heli setVariable ["bmkhs_rotorMastLength",    getArray  (_config >> "rotorMastLength")];
_heli setVariable ["bmkhs_rotorGearRatioArr",  getArray  (_config >> "rotorGearRatio")];
_heli setVariable ["bmkhs_rotorPivot",         getArray  (_config >> "rotorPivot")];
_heli setVariable ["bmkhs_rotorRotation",      getArray  (_config >> "rotorRotation")];
_heli setVariable ["bmkhs_rotorFlapTimeConst", getArray  (_config >> "rotorFlapTimeConst")];
_heli setVariable ["bmkhs_rotorAirfoil",       getArray  (_config >> "rotorAirfoil")];
_heli setVariable ["bmkhs_rotorBladeCutout",   getArray  (_config >> "rotorBladeCutout")];
_heli setVariable ["bmkhs_rotorBladeLength",   getArray  (_config >> "rotorBladeLength")];
_heli setVariable ["bmkhs_rotorBladeChordArr", getArray  (_config >> "rotorBladeChord")];
_heli setVariable ["bmkhs_rotorBladeTwist",    getArray  (_config >> "rotorBladeTwist")];
_heli setVariable ["bmkhs_rotorBladeMassArr",  getArray  (_config >> "rotorBladeMass")];
_heli setVariable ["bmkhs_rotorDelta3",        getArray  (_config >> "rotorDelta3")];
_heli setVariable ["bmkhs_rotorPitchMin",      getArray  (_config >> "rotorPitchMin")];
_heli setVariable ["bmkhs_rotorPitchMid",      getArray  (_config >> "rotorPitchMid")];
_heli setVariable ["bmkhs_rotorPitchMax",      getArray  (_config >> "rotorPitchMax")];
_heli setVariable ["bmkhs_rotorRollMin",       getArray  (_config >> "rotorRollMin")];
_heli setVariable ["bmkhs_rotorRollMid",       getArray  (_config >> "rotorRollMid")];
_heli setVariable ["bmkhs_rotorRollMax",       getArray  (_config >> "rotorRollMax")];
_heli setVariable ["bmkhs_rotorCollMin",       getArray  (_config >> "rotorCollMin")];
_heli setVariable ["bmkhs_rotorCollMid",       getArray  (_config >> "rotorCollMid")];
_heli setVariable ["bmkhs_rotorCollMax",       getArray  (_config >> "rotorCollMax")];
_heli setVariable ["bmkhs_rotorAnimSource",    getArray  (_config >> "rotorAnimSource")];
_heli setVariable ["bmkhs_rotorHitPoint",      getArray  (_config >> "rotorHitPoint")];

//RUNTIME STATE - accumulators and filter state the model carries frame to frame.
//Static arrays sized for up to 6 rotors, indexed by rotor index.
// Per-blade accumulated aerodynamic flap moments (N·m), one 4-element array per rotor
_heli setVariable ["bmkhs_rotorFlapMoment",  [[0.0,0.0,0.0,0.0]
                                                   ,[0.0,0.0,0.0,0.0]
                                                   ,[0.0,0.0,0.0,0.0]
                                                   ,[0.0,0.0,0.0,0.0]
                                                   ,[0.0,0.0,0.0,0.0]
                                                   ,[0.0,0.0,0.0,0.0]]];
// Azimuth (degrees) at which each virtual blade's moment was computed
_heli setVariable ["bmkhs_rotorBladeAzimuth",[[0.0,0.0,0.0,0.0]
                                                   ,[0.0,0.0,0.0,0.0]
                                                   ,[0.0,0.0,0.0,0.0]
                                                   ,[0.0,0.0,0.0,0.0]
                                                   ,[0.0,0.0,0.0,0.0]
                                                   ,[0.0,0.0,0.0,0.0]]];
// Per-element induced inflow velocity (m/s) from previous frame — read during blade loop
_heli setVariable ["bmkhs_rotorInducedFlow",      [[0.0,0.0,0.0,0.0]
                                                        ,[0.0,0.0,0.0,0.0]
                                                        ,[0.0,0.0,0.0,0.0]
                                                        ,[0.0,0.0,0.0,0.0]
                                                        ,[0.0,0.0,0.0,0.0]
                                                        ,[0.0,0.0,0.0,0.0]]];
// Accumulator reset each frame; averaged after blade loop then copied to rotorInducedFlow
_heli setVariable ["bmkhs_rotorInducedFlowAccum", [[0.0,0.0,0.0,0.0]
                                                        ,[0.0,0.0,0.0,0.0]
                                                        ,[0.0,0.0,0.0,0.0]
                                                        ,[0.0,0.0,0.0,0.0]
                                                        ,[0.0,0.0,0.0,0.0]
                                                        ,[0.0,0.0,0.0,0.0]]];
// Accumulated aerodynamic drag torque reaction on fuselage (N·m), one scalar per rotor
_heli setVariable ["bmkhs_rotorReactionTorque", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
// Accumulated rotor thrust (N) per rotor — sum of all element lift across all blades this frame
_heli setVariable ["bmkhs_rotorThrustAccum",    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
// Rotor rate-damping scalar (roll/pitch RATE damping from the airframe angular-velocity term
// in fn_rotorBlade). 0.1 tuned in-sim: anything much higher caused an aggressive, noticeable
// snap-back (the rotor over-damps and fights the return to center). Tunable live.
_heli setVariable ["bmkhs_rotorRateDampScalar", 0.1];
// BET FORCE-OUTPUT TUNING SCALARS (per airspeed band). BET forces are physics-derived; these
// tables. 1.0 = pure physics. LIFT scalar -> thrust; TORQUE scalar -> the reaction couple (yaw),
// SPLIT so thrust and yaw tune independently. Applied in fn_rotorBlade (lift) / fn_rotor (torque).
private _betBands = [0.00, 10.29, 20.58, 36.01, 46.30, 51.44, 61.73, 66.88, 72.02];
_heli setVariable ["bmkhs_betMainLiftTable",   _betBands apply {[_x, 1.0]}];  // main thrust
_heli setVariable ["bmkhs_betMainTorqueTable", _betBands apply {[_x, 1.0]}];  // main yaw torque
_heli setVariable ["bmkhs_betTailLiftTable",   _betBands apply {[_x, 1.0]}];  // tail thrust
// Fixed-frame flap coefficients (degrees) — updated each frame from decomposed blade moments
_heli setVariable ["bmkhs_rotorBeta0",       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]; // collective coning
_heli setVariable ["bmkhs_rotorA1",          [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]; // longitudinal disc tilt
_heli setVariable ["bmkhs_rotorB1",          [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]; // lateral disc tilt
// Pre-filter targets — smoothed over one rotor revolution before the main disc tilt filter
_heli setVariable ["bmkhs_rotorBeta0Target", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
_heli setVariable ["bmkhs_rotorA1Target",    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
_heli setVariable ["bmkhs_rotorB1Target",    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
