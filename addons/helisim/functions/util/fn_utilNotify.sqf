/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_utilNotify

Description:
    Raises a HeliSim event for the aircraft pack to react to. Core has no
    opinion on what an event should look or sound like - it only reports that
    something happened.

    The event goes to the handler registered for this aircraft's pack, found by
    base class - so with two packs installed, each only hears its own aircraft.
    A pack registers with bmkhs_fnc_utilNotifyRegister.

    With no handler registered for the aircraft the call is a no-op, so Core
    runs standalone.

Parameters:
    _heli  - The helicopter [Object]
    _event - Event name [String], e.g. "holdModeDisengaged"
    _data  - Optional event payload [Array]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_event", ["_data", []]];

{
    if (_heli isKindOf _x) exitWith {
        [_heli, _event, _data] call _y;
    };
} forEach (missionNamespace getVariable ["bmkhs_notifyHandlers", createHashMap]);
