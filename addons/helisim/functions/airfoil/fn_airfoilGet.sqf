/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_airfoilGet

Description:
    Returns the lift/drag table for a named airfoil section.

    A name that matches nothing is a config error, not a silent zero: an empty
    table interpolates to no lift at all, which reads as a broken airframe
    rather than a broken config. The error is logged once per lookup site and
    an empty table returned, so the aircraft still runs.

Parameters:
    _heli - The helicopter [Object]
    _name - The airfoil name, as declared in the Airfoils config [String]
    _for  - What is asking, for the error text, e.g. "rotor 1" [String]

Returns:
    The {AoA, CL, CD} table, or [] if the name is unknown [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_name", ["_for", "?"]];

private _airfoils = _heli getVariable ["bmkhs_airfoils", createHashMap];

if (_name in _airfoils) exitWith { _airfoils get _name };

diag_log text format [
    "[BMKHS] AIRFOIL CONFIG ERROR: %1 asks for airfoil '%2', which is not declared. Known: %3.",
    _for, _name, keys _airfoils
];
[]
