/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_mathBuildInterpGrid

Description:
    Validates a 2D coefficient grid and converts it to the form
    bmkhs_fnc_mathLinearInterp2D reads. Called once at init, never per frame.

    The AUTHORED grid is written the way it comes off a spreadsheet. Row 0 is the
    header: its first cell is the STRING naming the column axis, and the rest are
    the column keys. Every row after it starts with its row key and carries one
    value per column.

        [
        //  Coll \ A/S    0.00    10.29    20.58
             ["A/S",      0.00,   10.29,   20.58]
            ,[ 0.00,    0.1000,  0.1000,  0.1000]
            ,[ 0.50,    0.2331,  0.2389,  0.2653]
            ,[ 1.00,    0.4120,  0.4120,  0.4120]
        ];

    The header is found BY POSITION and its first cell is never read as a number,
    so no row key can collide with it - a collective grid (0..1) and a pedal grid
    (-1..1) are written identically.

    Every failure is reported once, here, naming the table - a malformed grid
    cannot reach the flight model and read as quietly wrong numbers.

Parameters:
    _arr  - the authored grid, as above
    _name - table name, used in error messages [String]

Returns:
    [_colKeys, _rows] - the column axis, and the data rows with the header
    stripped. [] if the grid is invalid.

Examples:
    private _t = [_liftCoefTable, "liftCoefTable"] call bmkhs_fnc_mathBuildInterpGrid;

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_arr", ["_name", "interp grid"]];

private _fail = {
    diag_log text format ["[BMKHS] %1: %2", _name, _this];
    []
};

if (!(_arr isEqualType []) || {count _arr < 2}) exitWith {
    "needs a header row and at least one data row" call _fail
};

private _header = _arr select 0;
if (!(_header isEqualType []) || {!((_header select 0) isEqualType "")}) exitWith {
    "row 0 must be the header, first cell a string naming the column axis" call _fail
};

private _colKeys = _header select [1];
private _rows    = _arr select [1];
private _width   = count _colKeys;

if (_width < 2) exitWith { "header needs at least two column keys" call _fail };

//Column keys ascending - mathLinearInterp walks them in order.
private _prev = _colKeys select 0;
{
    if (!(_x isEqualType 0)) exitWith { _prev = "BAD"; };
    if (_forEachIndex > 0 && {_x <= _prev}) exitWith { _prev = "BAD"; };
    _prev = _x;
} forEach _colKeys;
if (_prev isEqualType "") exitWith { "column keys must be numbers in ascending order" call _fail };

//Every row: right width, numeric, row keys ascending.
private _err = "";
private _lastKey = 0;
{
    private _row = _x;
    private _i   = _forEachIndex;
    if (!(_row isEqualType [])) then { _err = format ["row %1 is not an array", _i]; };
    if (_err isEqualTo "" && {count _row != (_width + 1)}) then {
        _err = format ["row %1 has %2 values, header declares %3 columns (+1 row key)", _i, count _row, _width];
    };
    if (_err isEqualTo "" && {(_row findIf {!(_x isEqualType 0)}) > -1}) then {
        _err = format ["row %1 has a non-numeric value", _i];
    };
    if (_err isEqualTo "" && {_i > 0 && {(_row select 0) <= _lastKey}}) then {
        _err = format ["row %1 key %2 is not above the row above it", _i, _row select 0];
    };
    if (_err isEqualTo "") then { _lastKey = _row select 0; };
} forEach _rows;

if (_err isNotEqualTo "") exitWith { _err call _fail };

[_colKeys, _rows]
