/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_preston

Description:
    Preston Pilot AI module entry. Preston is the PILOT for aircraft no human is
    flying - Arma's AI cannot drive this flight model (it moves aircraft
    kinematically and exposes no control demand to SQF), so for an AI Apache
    Preston is the only thing on the controls.

    Called from fn_coreUpdate, NOT fn_getInput - that function exits early unless
    the player is flying, so an AI aircraft would never reach it.

    Currently the HANDS (cyclic, fn_prestonPilot). The FEET (auto-pedal) still
    live in fn_getInput and will move here.

    STILL TO BUILD: the outer guidance loop. Preston is an INNER loop - it takes a
    desired state (position / velocity / attitude) and works the controls to hold
    it. Something must turn Arma's navigation goals (expectedDestination,
    waypointPosition, flyInHeight, waypointSpeed) into those setpoints, or an AI
    Apache simply holds station where it spawned.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    Nothing. Outputs are published as object variables; fn_prestonPilot writes
    force-trim directly.

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

//DISABLED while the FMC holds are tuned. Remove the `false && ` to re-enable.
//
//WHEN RE-ENABLED this becomes "no human is flying": AI aircraft always, and a CPG-crewed aircraft
//until the CPG takes the controls. It is NOT the casual-realism check it started as.
private _active = false && {!isPlayer (currentPilot _heli)};

if (!_active) exitWith {
    //Idle: clear the flags so the readouts do not report Preston as flying.
    _heli setVariable ["bmkhs_prestonActive",      false];
    _heli setVariable ["bmkhs_prestonPitchActive", false];
    _heli setVariable ["bmkhs_prestonRollActive",  false];
    _heli setVariable ["bmkhs_prestonWPos",        0.0];
    _heli setVariable ["bmkhs_prestonWVel",        0.0];
    _heli setVariable ["bmkhs_prestonWAtt",        0.0];
};

_heli setVariable ["bmkhs_prestonActive", true];

private _deltaTime = _heli getVariable "bmkhs_deltaTime";

//HANDS - the cyclic. Writes force-trim, which is the only path to the rotor when Preston flies.
//The three input arguments are the COMMAND channel (what to achieve, not where to put the stick).
//With no human aboard they are zero; the guidance loop will drive them once it exists.
[_heli, _deltaTime, 0.0, 0.0, false] call bmkhs_fnc_prestonPilot;

//FEET - the pedals. Skipped when the PLAYER already has the auto-pedal option on, because
//fn_getInput has run it for this frame already and running it twice would double-integrate.
if (!bmkhs_autoPedal) then {
    private _kbPedal      = _heli getVariable ["bmkhs_kbPedalLeftRight", 0.0];
    private _pedal        = _heli getVariable ["bmkhs_pedalLeftRight",   0.0];
    //Same hover -> nose-to-tail handover speed fn_getInput uses: ~24kts GS.
    private _yawSwitchVel = 5.14444 * 2.4;
    [_heli, _deltaTime, _pedal, _kbPedal, _yawSwitchVel] call bmkhs_fnc_prestonPedal;
};
