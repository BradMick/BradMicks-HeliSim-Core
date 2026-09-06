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

if (!(_heli getVariable ["bmkhs_initialised", false]) && local _heli) then {
    _heli setVariable ["bmkhs_initialised", true, true];

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

    //FMC
    _heli setVariable ["bmkhs_fmcPitchOn",                true,  true];
    _heli setVariable ["bmkhs_fmcRollOn",                 true,  true];
    _heli setVariable ["bmkhs_fmcYawOn",                  true,  true];
    _heli setVariable ["bmkhs_fmcCollOn",                 true,  true];
    _heli setVariable ["bmkhs_fmcTrimOn",                 true,  true];
    //Force Trim
    _heli setVariable ["bmkhs_forceTrimInterupted",       false, true];
    _heli setVariable ["bmkhs_forceTrimPosPitch",         0.0,   true];
    _heli setVariable ["bmkhs_forceTrimPosRoll",          0.0,   true];
    _heli setVariable ["bmkhs_forceTrimPosYaw",           0.0,   true];
    //Attitude Hold
    _heli setVariable ["bmkhs_attHoldActive",             false, true];
    _heli setVariable ["bmkhs_attHoldDesiredPos",         getPos _heli, true];
    _heli setVariable ["bmkhs_attHoldDesiredVel",         [0.0, 0.0], true];
    _heli setVariable ["bmkhs_attHoldDesiredAtt",         [0.0, 0.0], true];
    _heli setVariable ["bmkhs_attHoldSubMode",            "pos", true];   //pos, vel, att
    //Altitude Hold
    _heli setVariable ["bmkhs_altHoldActive",             false, true];
    _heli setVariable ["bmkhs_altHoldDesiredAlt",         0.0,   true];
    _heli setVariable ["bmkhs_altHoldSubMode",            "rad", true];   //rad, bar
    _heli setVariable ["bmkhs_altHoldCollRef",            0.0,   true];
    //Heading Hold
    _heli setVariable ["bmkhs_hdgHoldActive",             false, true];
    _heli setVariable ["bmkhs_hdgHoldDesiredHdg",         0.0,   true];
    _heli setVariable ["bmkhs_hdgHoldDesiredSideslip",    0.0,   true];
    _heli setVariable ["bmkhs_hdgHoldSubMode",            "hdg", true];    //hdg, trn, yaw, aut
    _heli setVariable ["bmkhs_hdgHoldPedalRef",           0.0,   true];    //<-- probably not needed, kept just in case...

    _heli setVariable ["bmkhs_stabilatorPosition",        0.0,   true];
};
