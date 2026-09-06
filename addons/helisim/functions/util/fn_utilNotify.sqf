/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_utilNotify

Description:
    Raises a HeliSim event for the aircraft pack to react to. Core has no
    opinion on what an event should look or sound like - it only reports that
    something happened.

    A pack registers a handler by setting bmkhs_notifyHandler to a code block:

        bmkhs_notifyHandler = { params ["_heli", "_event", ["_data", []]]; ... };

    If no handler is registered the call is a no-op, so Core runs standalone.

Parameters:
    _heli  - The helicopter [Object]
    _event - Event name [String], e.g. "holdModeDisengaged"
    _data  - Optional event payload [Array]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_event", ["_data", []]];

if (isNil "bmkhs_notifyHandler") exitWith {};

[_heli, _event, _data] call bmkhs_notifyHandler;
