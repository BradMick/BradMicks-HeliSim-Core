/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemConsumer

Description:
    Answers whether each consumer has supply. suppliedBy is an OR, so a
    consumer naming two circuits survives losing one and a consumer naming one
    dies with it - selective failure without Core knowing the plumbing.

    Core publishes the state; what it MEANS is the aircraft's business.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

private _consumers = _heli getVariable ["bmkhs_sysConsumers", []];
if (_consumers isEqualTo []) exitWith {};

{
    //_comp, not _x: the inner forEach rebinds it to the circuit pair.
    private _comp     = _x;
    private _needsAll = _comp get "needsAll";

    //Any one is enough, unless it needs all of them.
    private _supplied = _needsAll;
    {
        private _up = ([_heli, _x select 0] call bmkhs_fnc_systemCircuit) >= (_x select 1);
        if (_needsAll) then {
            if (!_up) exitWith { _supplied = false };
        } else {
            if (_up) exitWith { _supplied = true };
        };
    } forEach (_comp get "circuits");

    if (_comp get "networked") then {
        [_heli, _comp get "varName", _supplied] call bmkhs_fnc_utilUpdateNetworkGlobal;
    } else {
        _heli setVariable [_comp get "varName", _supplied];
    };
} forEach _consumers;
