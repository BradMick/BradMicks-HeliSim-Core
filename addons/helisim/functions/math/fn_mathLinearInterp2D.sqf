/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_mathLinearInterp2D

Description:
    Interpolates a value from a table that varies along TWO axes - a thrust
    curve that changes with altitude, a torque curve that changes with
    temperature, and so on.

    It does no arithmetic of its own. Both passes are bmkhs_fnc_mathLinearInterp:
    every inner table is interpolated on the inner key, the results are stacked
    against their outer keys, and that stack is interpolated on the outer key.
    Clamping at the ends of both axes is therefore whatever mathLinearInterp
    does, which is what every other table in the model already does.

    The shape is the same as mathLinearInterp, one level deeper: each entry is
    an outer key followed by a table whose rows start with the inner key.

        _arr = [
            [   0, [[0.00, 0.032], [1.00, 1.000]] ],   //outer key 0
            [2000, [[0.00, 0.052], [1.00, 1.000]] ],   //outer key 2000
            [4000, [[0.00, 0.037], [1.00, 1.000]] ]
        ];

    Inner tables may carry any number of columns, as mathLinearInterp allows,
    and every inner table must have the same number. Outer keys ascending.

Parameters:
    _arr      - the table, as above
    _outerKey - value on the outer axis (altitude, temperature)
    _innerKey - value on the inner axis (collective, airspeed)

Returns:
    Array, as mathLinearInterp: the outer key followed by the interpolated
    columns. Take `select 1` for a single-column table.

Examples:
    private _scalar = [_thrustTable, _pressureAlt, _collective]
                        call bmkhs_fnc_mathLinearInterp2D select 1;

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_arr", "_outerKey", "_innerKey"];

if (_arr isEqualTo []) exitWith {[_outerKey, 0]};

//Interpolate every inner table on the inner key, then rebuild each result as a
//row keyed by its outer key - which is exactly the shape mathLinearInterp
//wants for the second pass.
private _stack = _arr apply {
    _x params ["_key", "_table"];
    private _row = [_table, _innerKey] call bmkhs_fnc_mathLinearInterp;
    //Drop the inner key mathLinearInterp puts at index 0, put the outer key
    //in its place, and keep every interpolated column after it.
    _row set [0, _key];
    _row
};

[_stack, _outerKey] call bmkhs_fnc_mathLinearInterp
