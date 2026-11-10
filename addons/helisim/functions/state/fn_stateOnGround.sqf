/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stateOnGround

Description:
    Returns true or false based on being in contact with the ground. With wheels
    declared it is weight on wheels: a squat-switch wheel compressed.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    On the ground [Bool]

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

private _onGround = false;

if ((_heli getVariable "bmkhs_numWheels") > 0) then {
    private _compression = _heli getVariable "bmkhs_wheelCompression";
    private _wow         = _heli getVariable "bmkhs_wheelWow";
    _onGround = if (_wow >= 0) then { (_compression select _wow) >= 0.0 } else { (_compression findIf {_x >= 0.0}) >= 0 };
} else {
    private _radAlt = _heli getVariable "bmkhs_radAlt";

    //Metres. Was 0.5 ft when the raw value was published in feet.
    if (isTouchingGround _heli || _radAlt < 0.15) then {
        _onGround = true;
    };
};

_onGround;
