/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stateOnGround

Description:
    Returns true or false based on being in contact with the ground.

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

private _onGround = false;
private _radAlt   = _heli getVariable "bmkhs_radAltRaw";

//Metres. Was 0.5 ft when the raw value was published in feet.
if (isTouchingGround _heli || _radAlt < 0.15) then {
    _onGround = true;
};

_onGround;
