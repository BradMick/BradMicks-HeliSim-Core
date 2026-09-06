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
_heli setVariable ["bmkhs_mainRtrBaseThrust",   getNumber (_config >> "mainRtrBaseThrust")];
_heli setVariable ["bmkhs_mainRotorGearRatio",  getNumber (_config >> "mainRtrGearRatio")];
_heli setVariable ["bmkhs_mainRtrFlapbackLon",  getNumber (_config >> "mainRtrFlapbackLon")];
_heli setVariable ["bmkhs_mainRtrFlapbackLat",  getNumber (_config >> "mainRtrFlapbackLat")];

_heli setVariable ["bmkhs_mainRtrGndEffTable", getArray (_config >> "mainRtrGndEffTable")];
_heli setVariable ["bmkhs_mainRtrThrustVsCollective", getArray (_config >> "mainRtrThrustVsCollective")];
_heli setVariable ["bmkhs_mainRtrTipLossTable", getArray (_config >> "mainRtrTipLossTable")];
_heli setVariable ["bmkhs_mainRtrVelExponentTable", getArray (_config >> "mainRtrVelExponentTable")];
_heli setVariable ["bmkhs_mainRtrTorqueScalarTable", getArray (_config >> "mainRtrTorqueScalarTable")];
_heli setVariable ["bmkhs_mainRtrThrustVsAirspeed", getArray (_config >> "mainRtrThrustVsAirspeed")];
_heli setVariable ["bmkhs_mainRtrInducedPwrVelTable", getArray (_config >> "mainRtrInducedPwrVelTable")];
_heli setVariable ["bmkhs_mainRtrInducedPwrCollTable", getArray (_config >> "mainRtrInducedPwrCollTable")];
_heli setVariable ["bmkhs_mainRtrCollTorqueCorrTable", getArray (_config >> "mainRtrCollTorqueCorrTable")];
_heli setVariable ["bmkhs_mainRtrAutoroTorqueTable", getArray (_config >> "mainRtrAutoroTorqueTable")];
_heli setVariable ["bmkhs_mainRtrCruiseTqTable", getArray (_config >> "mainRtrCruiseTqTable")];
_heli setVariable ["bmkhs_mainRtrTqRoCTable", getArray (_config >> "mainRtrTqRoCTable")];
_heli setVariable ["bmkhs_tailRtrPos",          getArray  (_config >> "tailRtrPos")];
_heli setVariable ["bmkhs_tailRtrRotation",     getArray  (_config >> "tailRtrRotation"), true];
_heli setVariable ["bmkhs_tailRtrDesignRpm",    getNumber (_config >> "tailRtrDesignRpm")];
_heli setVariable ["bmkhs_tailRtrRpmTrimVal",   getNumber (_config >> "tailRtrRpmTrimVal")];
_heli setVariable ["bmkhs_tailRtrGearRatio",    getNumber (_config >> "tailRtrGearRatio")];
_heli setVariable ["bmkhs_tailRtrNumBlades",    getNumber (_config >> "tailRtrNumBlades")];
_heli setVariable ["bmkhs_tailRtrBladeRadius",  getNumber (_config >> "tailRtrBladeRadius")];
_heli setVariable ["bmkhs_tailRtrBladeChord",   getNumber (_config >> "tailRtrBladeChord")];
_heli setVariable ["bmkhs_tailRtrBaseThrust",   getNumber (_config >> "tailRtrBaseThrust")];
_heli setVariable ["bmkhs_tailRtrPitchThrustTable", getArray (_config >> "tailRtrPitchThrustTable")];
_heli setVariable ["bmkhs_tailRtrThrustVsAirspeed", getArray (_config >> "tailRtrThrustVsAirspeed")];
_heli setVariable ["bmkhs_tailRtrRollCouple",     getNumber (_config >> "tailRtrRollCouple")];

//RUNTIME STATE - what the model carries frame to frame.
_heli setVariable ["bmkhs_reqEngTorque",   [0.0, 0.0]];
_heli setVariable ["bmkhs_vrsVelocityMin", 0.0];
_heli setVariable ["bmkhs_vrsVelocityMax", 0.0];
_heli setVariable ["bmkhs_rtrThrust",      [0.0, 0.0]];
_heli setVariable ["bmkhs_rtrRPM",         0.0];
_heli setVariable ["bmkhs_rtrMoi",         [0.0, 0.0]];
