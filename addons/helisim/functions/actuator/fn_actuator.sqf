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

//SAS SERVO = a FAST ELECTRICAL path. When SCAS is available (that axis's FMC channel on AND
//primary hydraulics good), the pilot command reaches the swashplate through this electrical
//servo essentially INSTANTLY ("speed of light" - the real aircraft's lag is 100% invisible
//with SCAS on), and the slow mechanical linkage just follows behind. So: SCAS available ->
//output = input (CRISP, unlagged). When SCAS is NOT available (FMC axis off OR primary
//hydraulics lost - the FMC/SAS servo runs THROUGH primary hydraulics), the fast electrical
//path is gone and the command falls back to the raw MECHANICAL LAG (push/pull tubes, bell
//cranks). The lag is therefore ONLY felt with SCAS off. (The SCAS rate augmentation itself is
//summed on top downstream in fn_rotorControl; this function is the pilot-command path only.)
private _priHydOk = ([_heli, "priPump"] call bmkhs_fnc_damageGet) < SYS_HYD_DMG_THRESH;
private _scasAvail = switch (_inputAxis) do {
    case "pitch"      : { _priHydOk && (_heli getVariable "bmkhs_fmcPitchOn") };
    case "roll"       : { _priHydOk && (_heli getVariable "bmkhs_fmcRollOn")  };
    case "yaw"        : { _priHydOk && (_heli getVariable "bmkhs_fmcYawOn")   };
    case "collective" : { _priHydOk && (_heli getVariable "bmkhs_fmcCollOn")  };
    default { false };
};

if (_scasAvail) then {
    _input   // fast electrical path -> instant, no lag
} else {
    [_heli, _inputAxis, _input, _lagVal] call bmkhs_fnc_actuatorLag   // mechanical lag
};
