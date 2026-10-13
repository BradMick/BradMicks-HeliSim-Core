params ["_heli", "_config"];

//Hydraulic boost - an aircraft without an FMC declares it to keep its controls crisp. Undeclared,
//the controls are mechanical. bmkhs_hydBoostOn is the boost switch.
private _boost = _config >> "HydraulicBoost";
_heli setVariable ["bmkhs_hydBoost",     isClass _boost];
_heli setVariable ["bmkhs_hydBoostGate", getArray (_boost >> "gate")];
[_heli, "bmkhs_hydBoostOn", true, true] call bmkhs_fnc_utilSeed;

_heli setVariable ["bmkhs_prevLagInputPitch",  0.0];
_heli setVariable ["bmkhs_prevLagOutputPitch", 0.0];

_heli setVariable ["bmkhs_prevLagInputYaw",    0.0];
_heli setVariable ["bmkhs_prevLagOutputYaw",   0.0];

_heli setVariable ["bmkhs_prevLagInputRoll",   0.0];
_heli setVariable ["bmkhs_prevLagOutputRoll",  0.0];

_heli setVariable ["bmkhs_prevLagInputColl",   0.0];
_heli setVariable ["bmkhs_prevLagOutputColl",  0.0];
