/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputUpdate

Description:
    Reads the player's input, then turns it into the control positions.

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

private _piloted = [_heli] call bmkhs_fnc_inputRead;
[_heli, _piloted] call bmkhs_fnc_inputWrite;
