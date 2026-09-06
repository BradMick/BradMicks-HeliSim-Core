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
_heli setVariable ["bmkhs_tailRtrPitchThrustTable", getArray (_config >> "tailRtrPitchThrustTable")];
_heli setVariable ["bmkhs_tailRtrAuthorityTable",   getArray (_config >> "tailRtrAuthorityTable")];
_heli setVariable ["bmkhs_tailRtrAirspeedMod",    getNumber (_config >> "tailRtrAirspeedMod")];
_heli setVariable ["bmkhs_tailRtrRollCouple",     getNumber (_config >> "tailRtrRollCouple")];
_heli setVariable ["bmkhs_tailRtrDamageThresh",   getNumber (_config >> "tailRtrDamageThresh")];
_heli setVariable ["bmkhs_tailRtrVne",            getNumber (_config >> "tailRtrVne")];
_heli setVariable ["bmkhs_tailRtrVbe",            getNumber (_config >> "tailRtrVbe")];
_heli setVariable ["bmkhs_tailRtrVrs",            getNumber (_config >> "tailRtrVrs")];
_heli setVariable ["bmkhs_tailRtrEtl",            getNumber (_config >> "tailRtrEtl")];

//RUNTIME STATE - what the model carries frame to frame.
_heli setVariable ["bmkhs_reqEngTorque",   [0.0, 0.0]];
_heli setVariable ["bmkhs_vrsVelocityMin", 0.0];
_heli setVariable ["bmkhs_vrsVelocityMax", 0.0];
_heli setVariable ["bmkhs_rtrThrust",      [0.0, 0.0]];
_heli setVariable ["bmkhs_rtrRPM",         0.0];
_heli setVariable ["bmkhs_rtrMoi",         [0.0, 0.0]];
