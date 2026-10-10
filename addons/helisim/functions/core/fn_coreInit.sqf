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

    //Repair is an event, not something to poll for. HandleDamage fires whenever a
    //hitpoint changes, including downward, so a repair announces itself - and "Repaired"
    //alone would miss a single component being fixed.
    _heli addEventHandler ["HandleDamage", {
        params ["_unit", "_selection", "_damage"];
        //Only a reduction is a repair; anything else is the damage model doing its job.
        if (_selection != "" && {_damage < (_unit getHitPointDamage _selection)}) then {
            _unit setVariable ["bmkhs_repairPending", true];
        };
        _damage
    }];
};
