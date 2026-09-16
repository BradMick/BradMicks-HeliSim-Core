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
//Main rotor min thrust coef = flat pitch, no lift and no induced torque at idle
//Main rotor max thrust coef = (gwt * 9.806) / (rho * area * tipVel^2)
_heli setVariable ["bmkhs_simpleRotorThrustCoefMin", [0.0,    0.03660]];//getArray  (_config >> "rotorThrustCoefMin")];
_heli setVariable ["bmkhs_simpleRotorThrustCoefMid", [0.0,    0.0]];//getArray  (_config >> "rotorThrustCoefMid")];
_heli setVariable ["bmkhs_simpleRotorThrustCoefMax", [0.0113,-0.2040]];//getArray  (_config >> "rotorThrustCoefMax")];
_heli setVariable ["bmkhs_simpleRotorPitchFlapMin",  [-10.0,   0.0]];//getArray  (_config >> "rotorPitchFlapMin")];
_heli setVariable ["bmkhs_simpleRotorPitchFlapMid",  [  0.0,   0.0]];//getArray  (_config >> "rotorPitchFlapMid")];
_heli setVariable ["bmkhs_simpleRotorPitchFlapMax",  [ 20.0,   0.0]];//getArray  (_config >> "rotorPitchFlapMax")];
_heli setVariable ["bmkhs_simpleRotorRollFlapMin",   [-10.5,   0.0]];//getArray  (_config >> "rotorRollFlapMin")];
_heli setVariable ["bmkhs_simpleRotorRollFlapMid",   [  0.0,   0.0]];//getArray  (_config >> "rotorRollFlapMid")];
_heli setVariable ["bmkhs_simpleRotorRollFlapMax",   [  7.0,   0.0]];//getArray  (_config >> "rotorRollFlapMax")];
_heli setVariable ["bmkhs_simpleRotorHitPoint",      [0,     0]];//getArray  (_config >> "rotorHitPoint")];
_heli setVariable ["bmkhs_simpleRotorFlapTimeConst", [[0.15, 0.15], [0.15, 0.15]]];//getArray  (_config >> "rotorFlapTimeConst")];
_heli setVariable ["bmkhs_simpleRotorCyclicPitchGain", [5.0,    0.0]];//getArray  (_config >> "rotorCyclicGain")];
_heli setVariable ["bmkhs_simpleRotorCyclicRollGain",  [2.5,    0.0]];//getArray  (_config >> "rotorCyclicGain")];
_heli setVariable ["bmkhs_simpleRotorRollGain",        [1.0,    0.25]];//getArray  (_config >> "rotorRollGain")];
//Profile drag coef - solved from the 36% idle point, Cd0 = 8*CQ0/sigma
_heli setVariable ["bmkhs_simpleRotorDragCoefMin",     [0.0145, 0.0110]];//getArray  (_config >> "rotorDragCoefMin")];
_heli setVariable ["bmkhs_simpleRotorDragCoefMid",     [0.0,    0.0110]];//getArray  (_config >> "rotorDragCoefMid")];
_heli setVariable ["bmkhs_simpleRotorDragCoefMax",     [0.0145, 0.0110]];//getArray  (_config >> "rotorDragCoefMax")];
//Induced factor - solved from the 176% OGE hover at 18000 lb; absorbs hover download
_heli setVariable ["bmkhs_simpleRotorInducedKappa",    [1.347,  1.20]];//getArray  (_config >> "rotorInducedKappa")];
_heli setVariable ["bmkhs_simpleRotorThrustVsAirspeedTable",
[
    [
     [ 0.00, 1.000]
    ,[10.29, 1.039]
    ,[20.58, 1.215]
    ,[36.01, 1.316]
    ,[46.30, 1.237]
    ,[51.44, 1.166]
    ,[61.73, 0.997]
    ,[66.88, 0.920]
    ,[72.02, 0.837]
    ]
   ,[
     [ 0.00, 1.000]
    ,[72.02, 1.000]
    ]
]];
//Torque smoothing time constant (s) and omega floor as a fraction of design speed
_heli setVariable ["bmkhs_simpleRotorTorqueTau",     [0.10,   0.10]];
_heli setVariable ["bmkhs_simpleRotorMinOmegaFrac",  [0.05,   0.05]];
//Equivalent flat plate area (m^2) for parasite power
_heli setVariable ["bmkhs_simpleRotorFlatPlateArea", [2.17,   0.00]];
//Hub height above ground on the wheels (m)
_heli setVariable ["bmkhs_simpleRotorHeightAgl",     [3.606,  0.00]];//getNumber (_config >> "mainRtrHeightAgl")];
//RUNTIME STATE - current disc tilt (deg), longitudinal and lateral
_heli setVariable ["bmkhs_simpleRotorFlapLon",       [0.0,    0.0]];
_heli setVariable ["bmkhs_simpleRotorFlapLat",       [0.0,    0.0]];

//RUNTIME STATE - what the model carries frame to frame.
_heli setVariable ["bmkhs_reqEngTorque",   [0.0, 0.0]];
_heli setVariable ["bmkhs_rtrThrust",      [0.0, 0.0]];
_heli setVariable ["bmkhs_rtrRPM",         0.0];
_heli setVariable ["bmkhs_rtrMoi",         [0.0, 0.0]];
