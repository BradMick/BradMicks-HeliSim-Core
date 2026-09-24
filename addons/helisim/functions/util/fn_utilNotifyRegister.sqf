/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_utilNotifyRegister

Description:
    Registers a pack's event handler for its own aircraft. Core raises events
    through bmkhs_fnc_utilNotify, which hands each one to the handler whose base
    class the aircraft is kind of - so every installed pack hears only its own
    airframe, whatever order the packs load in.

    Call it from the pack's XEH_preInit:

        [
            "yourAircraftBase",
            { params ["_heli", "_event", ["_data", []]]; ... }
        ] call bmkhs_fnc_utilNotifyRegister;

Parameters:
    _baseClass - The pack's bmkhsBaseClass [String]
    _handler   - Called with [_heli, _event, _data] [Code]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_baseClass", "_handler"];

if (isNil "bmkhs_notifyHandlers") then {
    bmkhs_notifyHandlers = createHashMap;
};
bmkhs_notifyHandlers set [_baseClass, _handler];
