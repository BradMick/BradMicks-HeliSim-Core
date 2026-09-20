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


_heli setVariable ["bmkhs_numSimpleRotors",            2];//getNumber (_config >> "numRotors")];
_heli setVariable ["bmkhs_simpleRotorType",            [0,1]];//getArray  (_config >> "rotorType")];
_heli setVariable ["bmkhs_simpleRotorDir",             [0,0]];//getArray  (_config >> "rotorDirection")];
_heli setVariable ["bmkhs_simpleRotorNumBlades",       [4,4]];//getArray  (_config >> "rotorNumBlades")];
_heli setVariable ["bmkhs_simpleRotorMastLength",      [0.70,   -0.87]];//getArray  (_config >> "rotorMastLength")];
_heli setVariable ["bmkhs_simpleRotorGearRatio",       [72.291, 14.90]];//getArray  (_config >> "rotorGearRatio")];
_heli setVariable ["bmkhs_simpleRotorPivot",           [[0.00,2.06,0.00],[0.00,-6.98,-0.075]]];//getArray  (_config >> "rotorPivot")];
_heli setVariable ["bmkhs_simpleRotorRotation",        [[0.00,0.00,0.00],[0.00, 90.00,0.00]]];//getArray  (_config >> "rotorRotation")];
_heli setVariable ["bmkhs_simpleRotorBladeRadius",     [7.315,  1.402]];//getArray  (_config >> "rotorBladeCutout")];
_heli setVariable ["bmkhs_simpleRotorBladeChord",      [0.533,  0.253]];//getArray  (_config >> "rotorBladeChord")];
_heli setVariable ["bmkhs_simpleRotorBladeMass",       [72.108, 5.131]];//getArray  (_config >> "rotorBladeMass")];
_heli setVariable ["bmkhs_simpleRotorPitchFlapMin",    [-10.0,   0.0]];//getArray  (_config >> "rotorPitchFlapMin")];
_heli setVariable ["bmkhs_simpleRotorPitchFlapMid",    [  0.0,   0.0]];//getArray  (_config >> "rotorPitchFlapMid")];
_heli setVariable ["bmkhs_simpleRotorPitchFlapMax",    [ 20.0,   0.0]];//getArray  (_config >> "rotorPitchFlapMax")];
_heli setVariable ["bmkhs_simpleRotorRollFlapMin",     [-10.5,   0.0]];//getArray  (_config >> "rotorRollFlapMin")];
_heli setVariable ["bmkhs_simpleRotorRollFlapMid",     [  0.0,   0.0]];//getArray  (_config >> "rotorRollFlapMid")];
_heli setVariable ["bmkhs_simpleRotorRollFlapMax",     [  7.0,   0.0]];//getArray  (_config >> "rotorRollFlapMax")];
_heli setVariable ["bmkhs_simpleRotorFlapTimeConst",   [[0.15, 0.15], [0.15, 0.15]]];//getArray  (_config >> "rotorFlapTimeConst")];
_heli setVariable ["bmkhs_simpleRotorCyclicPitchGain", [5.0,    0.0]];//getArray  (_config >> "rotorCyclicGain")];
_heli setVariable ["bmkhs_simpleRotorCyclicRollGain",  [2.5,    0.0]];//getArray  (_config >> "rotorCyclicGain")];
_heli setVariable ["bmkhs_simpleRotorRollGain",        [1.0,    0.25]];//getArray  (_config >> "rotorRollGain")];
_heli setVariable ["bmkhs_simpleRotorFlapGain",        [6.0,    0.0]];
_heli setVariable ["bmkhs_simpleRotorTorqueTau",       [0.10,   0.10]];
private _liftCoefTables =
[
    //MAIN - collective down the side, airspeed (m/s) across the top
    [
    //  Coll \ A/S    0.00    10.29    20.58    36.01    46.30    51.44    61.73    66.88    72.02
         ["A/S",    0.00,   10.29,   20.58,   36.01,   46.30,   51.44,   61.73,   66.88,   72.02]
        ,[ 0.00,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000]
        ,[ 0.20,  0.1666,  0.1695,  0.1826,  0.1904,  0.1841,  0.1786,  0.1666,  0.1541,  0.1252]
        ,[ 0.40,  0.2331,  0.2389,  0.2653,  0.2809,  0.2681,  0.2571,  0.2331,  0.2081,  0.1503]
        ,[ 0.64,  0.2997,  0.3084,  0.3479,  0.3713,  0.3522,  0.3357,  0.2997,  0.2622,  0.1755]
        ,[ 0.80,  0.3330,  0.4100,  0.5340,  0.6100,  0.5790,  0.5713,  0.4860,  0.3928,  0.2915]
        ,[ 1.00,  0.4120,  0.4120,  0.4120,  0.4120,  0.4120,  0.4120,  0.4120,  0.4120,  0.4120]
    ]
    //TAIL - pedal down the side, airspeed (m/s) across the top
   ,[
    //  Pedal \ A/S   0.00    10.29    20.58    36.01    46.30    51.44    61.73    66.88    72.02
         ["A/S",    0.00,   10.29,   20.58,   36.01,   46.30,   51.44,   61.73,   66.88,   72.02]
        ,[-1.00,  0.5336,  0.5865,  0.6331,  0.6946,  0.7314,  0.7488,  0.7820,  0.7978,  0.8131]
        ,[-0.50,  0.3335,  0.3666,  0.3957,  0.4341,  0.4571,  0.4680,  0.4887,  0.4986,  0.5082]
        ,[ 0.00,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000]
        ,[ 0.50, -0.1868, -0.2053, -0.2216, -0.2431, -0.2560, -0.2621, -0.2737, -0.2792, -0.2846]
        ,[ 1.00, -0.2668, -0.2932, -0.3165, -0.3473, -0.3657, -0.3744, -0.3910, -0.3989, -0.4066]
    ]
];

private _dragCoefTables =
[
    //MAIN - collective down the side, airspeed (m/s) across the top
    [
    //  Coll \ A/S    0.00    10.29    20.58    36.01    46.30    51.44    61.73    66.88    72.02
         ["A/S",    0.00,   10.29,   20.58,   36.01,   46.30,   51.44,   61.73,   66.88,   72.02]
        ,[ 0.00,  0.0085,  0.0085,  0.0065,  0.0005,  0.0005,  0.0005,  0.0005,  0.0005,  0.0005]
        ,[ 0.20,  0.0206,  0.0190,  0.0160,  0.0106,  0.0107,  0.0108,  0.0117,  0.0117,  0.0117]
        ,[ 0.40,  0.0326,  0.0296,  0.0255,  0.0206,  0.0208,  0.0211,  0.0229,  0.0229,  0.0229]
        ,[ 0.64,  0.0447,  0.0401,  0.0350,  0.0307,  0.0310,  0.0314,  0.0341,  0.0341,  0.0341]
        ,[ 0.80,  0.0474,  0.0474,  0.0474,  0.0474,  0.0474,  0.0474,  0.0474,  0.0474,  0.0474]
        ,[ 1.00,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000]
    ]
    //TAIL - pedal down the side, airspeed (m/s) across the top
   ,[
    //  Pedal \ A/S   0.00    10.29    20.58    36.01    46.30    51.44    61.73    66.88    72.02
         ["A/S",    0.00,   10.29,   20.58,   36.01,   46.30,   51.44,   61.73,   66.88,   72.02]
        ,[-1.00,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110]
        ,[ 0.00,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110]
        ,[ 1.00,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110,  0.0110]
    ]
];

private _liftTables = [];
private _dragTables = [];
{
    _liftTables pushBack ([_x, format ["rotor %1 liftCoefTable", _forEachIndex]] call bmkhs_fnc_mathBuildInterpGrid);
} forEach _liftCoefTables;
{
    _dragTables pushBack ([_x, format ["rotor %1 dragCoefTable", _forEachIndex]] call bmkhs_fnc_mathBuildInterpGrid);
} forEach _dragCoefTables;

_heli setVariable ["bmkhs_simpleRotorLiftCoefTable", _liftTables];
_heli setVariable ["bmkhs_simpleRotorDragCoefTable", _dragTables];

_heli setVariable ["bmkhs_simpleRotorFlapLon",       [0.0,    0.0]];
_heli setVariable ["bmkhs_simpleRotorFlapLat",       [0.0,    0.0]];

_heli setVariable ["bmkhs_reqEngTorque",   [0.0, 0.0]];
_heli setVariable ["bmkhs_rtrThrust",      [0.0, 0.0]];
_heli setVariable ["bmkhs_rtrRPM",         0.0];
_heli setVariable ["bmkhs_rtrMoi",         [0.0, 0.0]];
