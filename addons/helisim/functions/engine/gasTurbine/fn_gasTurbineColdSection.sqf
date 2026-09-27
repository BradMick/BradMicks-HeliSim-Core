/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_gasTurbineColdSection

Description:
    Compressor and spool. Ng comes from a torque balance, not from seeking a
    target at a declared rate, so acceleration tails off on its own as the
    compressor eats the surplus.

Parameters:
    _engine    - That engine's config [HashMap]
    _ng        - Spool speed at the top of the frame [Number]
    _fuelCmd   - Commanded fuel, normalised [Number]
    _starterTq - Starter torque on the spool [Number]
    _shaft     - The free turbine's share of the gas [Number]
    _dens      - Air density as a fraction of a standard day [Number]
    _running   - Burning, as opposed to cranked [Boolean]
    _coasting  - Running down, unfired and uncranked [Boolean]
    _deltaTime - Frame time [Number]

Returns:
    [_ng, _gasPower, _absorbed] - the stepped speed, and the two terms the power
    turbine needs for its surplus [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_engine", "_ng", "_fuelCmd", "_starterTq", "_shaft", "_dens", "_running", "_coasting", "_deltaTime"];

private _gasPower = [0.0, _fuelCmd * _dens] select _running;

//A compressor pumping against no combustion absorbs far more than a fired one, and that -
//not bearing friction - is what stops the spool. COASTING, not merely unfired: a cold spool
//being cranked is unfired too, and applying these to a start makes the starter fight them.
private _drag     = (_engine get "compressorLoad") * ([1.0, _engine get "unfiredDragMult"] select _coasting);
private _absorbed = (_drag * _ng * _ng) + ([0.0, _engine get "unfiredFriction"] select _coasting);

private _ngDot = (_gasPower + _starterTq - _absorbed - _shaft) / (_engine get "spoolInertia");
_ng = [_ng + (_ngDot * _deltaTime), 0.0, 1.1] call BIS_fnc_clamp;

[_ng, _gasPower, _absorbed]
