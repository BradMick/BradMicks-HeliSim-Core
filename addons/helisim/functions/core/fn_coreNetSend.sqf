/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_coreNetSend

Description:
    Publishes the aircraft's running state as one packed variable, so the
    network carries a single message rather than one per value. Called by
    the owner only; see netState.hpp for what is sent and why.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

//Copies of the arrays, not the live ones - the model keeps writing to those after this has gone out.
//The list is Core's plus the aircraft's own, as fn_coreConfig built it.
_heli setVariable ["bmkhs_netState", [
    CBA_missionTime,
    (_heli getVariable "bmkhs_netStateVars") apply {
        private _v = _heli getVariable _x;
        if (isNil "_v") then { nil } else { if (_v isEqualType []) then { +_v } else { _v } }
    },
    (_heli getVariable "bmkhs_pid_engine") apply { [_x get "integral", _x getOrDefault ["prevError", 0.0]] }
], true];
