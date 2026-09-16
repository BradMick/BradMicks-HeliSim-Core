/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_mathLinearInterp2D

Description:
    Interpolates a value from a table that varies along TWO axes - a lift
    coefficient that changes with collective and airspeed, a torque curve that
    changes with temperature, and so on.

    Takes a grid already built and validated by bmkhs_fnc_mathBuildInterpGrid:
    the column axis and the data rows, header stripped. Authoring, validation
    and error reporting all live there; this does nothing but interpolate.

    Both passes are bmkhs_fnc_mathLinearInterp: the rows are interpolated on the
    row key, which yields one value per column, and those are then interpolated
    against the column keys. Clamping at the ends of both axes is therefore
    whatever mathLinearInterp does, which is what every other table in the model
    already does.

Parameters:
    _grid   - [_colKeys, _rows] from bmkhs_fnc_mathBuildInterpGrid
    _rowKey - value on the row axis (collective, pedal)
    _colKey - value on the column axis (airspeed)

Returns:
    Number - the interpolated value. 0 if the grid is empty.

Examples:
    private _cl = [_liftCoefGrid, _collective, _velXY] call bmkhs_fnc_mathLinearInterp2D;

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_grid", "_rowKey", "_colKey"];

if (_grid isEqualTo []) exitWith {0};

_grid params ["_colKeys", "_rows"];

//Interpolate on the row key - mathLinearInterp carries every column through, so
//this is one value per column of the grid. Index 0 is the row key it echoes back.
private _row = [_rows, _rowKey] call bmkhs_fnc_mathLinearInterp;

//Pair each interpolated column with its key, which is the shape mathLinearInterp
//wants for the second pass.
private _stack = [];
{
    _stack pushBack [_x, _row select (_forEachIndex + 1)];
} forEach _colKeys;

([_stack, _colKey] call bmkhs_fnc_mathLinearInterp) select 1
