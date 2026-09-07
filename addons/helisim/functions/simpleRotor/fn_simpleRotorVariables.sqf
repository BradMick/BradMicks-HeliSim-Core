/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_simpleRotorVariables

Description:
    Defines required simple rotor variables and initializes them.

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


_heli setVariable ["bmkhs_numSimpleRotors",          2];//getNumber (_config >> "numRotors")];
_heli setVariable ["bmkhs_simpleRotorType",          [0,1]];//getArray  (_config >> "rotorType")];
_heli setVariable ["bmkhs_simpleRotorDir",           [0,0]];//getArray  (_config >> "rotorDirection")];
_heli setVariable ["bmkhs_simpleRotorNumBlades",     [4,4]];//getArray  (_config >> "rotorNumBlades")];
_heli setVariable ["bmkhs_simpleRotorMastLength",    [0.70,   -0.87]];//getArray  (_config >> "rotorMastLength")];
_heli setVariable ["bmkhs_simpleRotorGearRatio",     [72.291, 14.90]];//getArray  (_config >> "rotorGearRatio")];
_heli setVariable ["bmkhs_simpleRotorPivot",         [[0.00,2.06,0.00],[0.00,-6.98,-0.075]]];//getArray  (_config >> "rotorPivot")];
_heli setVariable ["bmkhs_simpleRotorRotation",      [[0.00,0.00,0.00],[0.00,90.00,0.00]]];//getArray  (_config >> "rotorRotation")];
_heli setVariable ["bmkhs_simpleRotorBladeRadius",   [7.315,  1.402]];//getArray  (_config >> "rotorBladeCutout")];
_heli setVariable ["bmkhs_simpleRotorBladeChord",    [0.533,  0.253]];//getArray  (_config >> "rotorBladeChord")];
_heli setVariable ["bmkhs_simpleRotorBladeMass",     [72.108, 5.131]];//getArray  (_config >> "rotorBladeMass")];
//Main rotor min thrust coef = (gwt * 15% * 9.806) / (0.5 * rho * area * tipVel^2)
//Main rotor max thrust coef = (gwt * 9.806) / (0.5 * rho * area * tipVel^2)
//Tail rotor min thrust coef = (gwt * 5% * 9.806) / (0.5 * rho * area * tipVel^2)
//Tail rotor max thrust coef = (gwt * -10% * 9.806) / (0.5 * rho * area * tipVel^2)
_heli setVariable ["bmkhs_simpleRotorThrustCoefMin", [0.0030, 0.2040]];//getArray  (_config >> "rotorThrustCoefMin")];
_heli setVariable ["bmkhs_simpleRotorThrustCoefMid", [0.0,    0.0]];//getArray  (_config >> "rotorThrustCoefMid")];
_heli setVariable ["bmkhs_simpleRotorThrustCoefMax", [0.0226, -0.4081]];//getArray  (_config >> "rotorThrustCoefMax")];
_heli setVariable ["bmkhs_simpleRotorPitchFlapMin",  [-10.0,   0.0]];//getArray  (_config >> "rotorPitchFlapMin")];
_heli setVariable ["bmkhs_simpleRotorPitchFlapMid",  [  0.0,   0.0]];//getArray  (_config >> "rotorPitchFlapMid")];
_heli setVariable ["bmkhs_simpleRotorPitchFlapMax",  [ 20.0,   0.0]];//getArray  (_config >> "rotorPitchFlapMax")];
_heli setVariable ["bmkhs_simpleRotorRollFlapMin",   [-10.5,   0.0]];//getArray  (_config >> "rotorRollFlapMin")];
_heli setVariable ["bmkhs_simpleRotorRollFlapMid",   [  0.0,   0.0]];//getArray  (_config >> "rotorRollFlapMid")];
_heli setVariable ["bmkhs_simpleRotorRollFlapMax",   [  7.0,   0.0]];//getArray  (_config >> "rotorRollFlapMax")];
_heli setVariable ["bmkhs_simpleRotorHitPoint",      [0,     0]];//getArray  (_config >> "rotorHitPoint")];
//[pitch, roll] disc tilt lag time constants (s)
_heli setVariable ["bmkhs_simpleRotorFlapTimeConst", [[0.15, 0.15], [0.15, 0.15]]];//getArray  (_config >> "rotorFlapTimeConst")];
//Cyclic gain - scales the lift asymmetry the disc tilt produces across the rotor
_heli setVariable ["bmkhs_simpleRotorCyclicGain",    [5.0,    0.0]];//getArray  (_config >> "rotorCyclicGain")];
//Blade profile drag coefficient and induced power correction factor
_heli setVariable ["bmkhs_simpleRotorBladeCd0",      [0.0095, 0.0110]];//getArray  (_config >> "rotorBladeCd0")];
_heli setVariable ["bmkhs_simpleRotorInducedKappa",  [1.15,   1.20]];//getArray  (_config >> "rotorInducedKappa")];
//RUNTIME STATE - current disc tilt (deg), longitudinal and lateral
_heli setVariable ["bmkhs_simpleRotorFlapLon",       [0.0,    0.0]];
_heli setVariable ["bmkhs_simpleRotorFlapLat",       [0.0,    0.0]];











//CONFIG - what the aircraft IS.
//Rotor geometry
_heli setVariable ["bmkhs_mainRtrPos",          getArray  (_config >> "mainRtrPos")];
_heli setVariable ["bmkhs_mainRtrHeightAgl",    getNumber (_config >> "mainRtrHeightAgl")];
_heli setVariable ["bmkhs_mainRtrDesignRpm",    getNumber (_config >> "mainRtrDesignRpm")];
_heli setVariable ["bmkhs_mainRtrRpmTrimVal",   getNumber (_config >> "mainRtrRpmTrimVal")];
_heli setVariable ["bmkhs_mainRtrNumBlades",    getNumber (_config >> "mainRtrNumBlades")];
_heli setVariable ["bmkhs_mainRtrBladeRadius",  getNumber (_config >> "mainRtrBladeRadius")];
_heli setVariable ["bmkhs_mainRtrBladeChord",   getNumber (_config >> "mainRtrBladeChord")];
_heli setVariable ["bmkhs_mainRtrBladeMass",    getNumber (_config >> "mainRtrBladeMass")];
_heli setVariable ["bmkhs_mainRtrBladeHingeOff",getNumber (_config >> "mainRtrBladeHingeOff")];
_heli setVariable ["bmkhs_mainRtrBladePitchMin",getNumber (_config >> "mainRtrBladePitchMin")];
_heli setVariable ["bmkhs_mainRtrBladePitchMax",getNumber (_config >> "mainRtrBladePitchMax")];
_heli setVariable ["bmkhs_mainRtrBaseThrust",   getNumber (_config >> "mainRtrBaseThrust")];
_heli setVariable ["bmkhs_mainRotorGearRatio",  getNumber (_config >> "mainRtrGearRatio")];
_heli setVariable ["bmkhs_mainRtrFlapbackLon",  getNumber (_config >> "mainRtrFlapbackLon")];
_heli setVariable ["bmkhs_mainRtrFlapbackLat",  getNumber (_config >> "mainRtrFlapbackLat")];
_heli setVariable ["bmkhs_tailRtrPos",          getArray  (_config >> "tailRtrPos")];
_heli setVariable ["bmkhs_tailRtrDesignRpm",    getNumber (_config >> "tailRtrDesignRpm")];
_heli setVariable ["bmkhs_tailRtrRpmTrimVal",   getNumber (_config >> "tailRtrRpmTrimVal")];
_heli setVariable ["bmkhs_tailRtrGearRatio",    getNumber (_config >> "tailRtrGearRatio")];
_heli setVariable ["bmkhs_tailRtrNumBlades",    getNumber (_config >> "tailRtrNumBlades")];
_heli setVariable ["bmkhs_tailRtrBladeRadius",  getNumber (_config >> "tailRtrBladeRadius")];
_heli setVariable ["bmkhs_tailRtrBladeChord",   getNumber (_config >> "tailRtrBladeChord")];
_heli setVariable ["bmkhs_tailRtrBaseThrust",   getNumber (_config >> "tailRtrBaseThrust")];

//RUNTIME STATE - what the model carries frame to frame.
_heli setVariable ["bmkhs_reqEngTorque",   [0.0, 0.0]];
_heli setVariable ["bmkhs_vrsVelocityMin", 0.0];
_heli setVariable ["bmkhs_vrsVelocityMax", 0.0];
_heli setVariable ["bmkhs_rtrThrust",      [0.0, 0.0]];
_heli setVariable ["bmkhs_rtrRPM",         0.0];
_heli setVariable ["bmkhs_rtrMoi",         [0.0, 0.0]];
