/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engine

Description:
    Sourced from JSBSim.

Parameters:
    ...

Returns:
    Lag coeffient required for actuator simulation.

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_inputAxis", "_input", "_lagVal"];
#include "\bmkhs_helisim\functions\systems\systems.hpp"

//SAS SERVO = a FAST ELECTRICAL path. When SCAS is available (the aircraft's declared SAS with
//its gate[] holding, AND that axis's FMC channel on), the pilot command reaches the swashplate
//through this electrical servo essentially INSTANTLY ("speed of light" - the real aircraft's lag
//is 100% invisible with SCAS on), and the slow mechanical linkage just follows behind. So: SCAS
//available -> output = input (CRISP, unlagged). Otherwise the fast electrical path is gone and
//the command falls back to the raw MECHANICAL LAG (push/pull tubes, bell cranks). An aircraft
//with no SAS declared always flies the mechanical path. (The SCAS rate augmentation itself is
//summed on top downstream in fn_rotorControl; this function is the pilot-command path only.)
private _sasOk = _heli getVariable ["bmkhs_fmcSasAvail", false];
private _scasAvail = switch (_inputAxis) do {
    case "pitch"      : { _sasOk && (_heli getVariable "bmkhs_fmcPitchOn") };
    case "roll"       : { _sasOk && (_heli getVariable "bmkhs_fmcRollOn")  };
    case "yaw"        : { _sasOk && (_heli getVariable "bmkhs_fmcYawOn")   };
    case "collective" : { _sasOk && (_heli getVariable "bmkhs_fmcCollOn")  };
    default { false };
};

if (_scasAvail) then {
    _input   // fast electrical path -> instant, no lag
} else {
    [_heli, _inputAxis, _input, _lagVal] call bmkhs_fnc_actuatorLag   // mechanical lag
};
