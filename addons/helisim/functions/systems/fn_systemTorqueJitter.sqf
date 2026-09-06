/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemTorqueJitter

Description:
    What a damaged drivetrain is doing to one engine's torque needle.

    Each drive component publishes its OWN wander while it is damaged - the
    component owns its state, as every other kind does. This sums whatever
    reaches the engine asked about: a component carrying that one engine
    contributes its own, and a component carrying all of them - a transmission
    summing both - contributes to every engine it turns.

    So an airframe with three engines and two transmissions needs no new code
    and no shared array with a layout baked into it.

Parameters:
    _heli   - The helicopter [Object]
    _engNum - Which engine's needle [Number]

Returns:
    Torque to add to that engine, as a fraction of rated [Number]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_engNum"];

private _total = 0;
{
    //A summed component turns every engine, so its wander reaches all of them. Otherwise
    //it is per-member, and only its own engine feels it.
    if ((_x get "torqueSum") || {(_x get "index") == _engNum}) then {
        _total = _total + (_heli getVariable [(_x get "varName") + "TqJitter", 0]);
    };
} forEach ((_heli getVariable ["bmkhs_sysTorqued", []]) select {_x get "jitters"});

_total
