/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_CoreUpdate

Description:
    Updates all of the modules core functions.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

private _deltaTime = _heli getVariable "bmkhs_deltaTime";

//The component graph - whatever this airframe declared. Hydraulics live here now,
//as producers and storage rather than as functions Core wrote for them.
[_heli, _deltaTime] call bmkhs_fnc_systemsSolve;

[_heli] call bmkhs_fnc_systemsDebug;

//Torque limits apply whether or not the aircraft models systems - an airframe does not
//get to ignore what its drivetrain is rated for by declining to simulate the rest. This
//needs no circuits, only a torque and a limit, so it runs outside the solve. An airframe
//that declares no drive components has nothing rated and nothing happens.
[_heli, _deltaTime] call bmkhs_fnc_systemTorque;

//The APU's fuel burn and state notify, which are not supply. The solve above has already
//exited if this aircraft models no systems.
if !(_heli getVariable ["bmkhs_useSystems", false]) exitWith {};

[_heli, _deltaTime] call bmkhs_fnc_apu;
