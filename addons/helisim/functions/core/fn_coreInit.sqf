/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_coreInit

Description:
    Initialize public variables on mission startup
    To set up information accessible by all crew members

Parameters:
    _heli - the helicopter upon which to execute the code

Returns:
    Nothing

Examples:
    [_heli] call bmkhs_fnc_coreInit

Author:
    Snow(Dryden)
---------------------------------------------------------------------------- */
params ["_heli"];

if !(_heli getVariable ["bmkhs_initialised", false]) then {
    [_heli, "bmkhs_initialised", true, true] call bmkhs_fnc_utilSeed;
};
