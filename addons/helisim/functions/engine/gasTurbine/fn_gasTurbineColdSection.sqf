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
    _spooling  - Spooling down, unfired and uncranked [Boolean]
    _deltaTime - Frame time [Number]

Returns:
    [_ng, _fuelGas, _absorbed, _airGas] - the stepped speed, and the terms the
    power turbine needs for its share [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_engine", "_ng", "_fuelCmd", "_starterTq", "_shaft", "_dens", "_running", "_spooling", "_deltaTime"];

//Heat accelerates the spool - the compressor turbine runs on combustion.
private _fuelGas = [0.0, _fuelCmd * _dens] select _running;

//Cold air the compressor is moving right now, lit or not. It reaches the power turbine, so
//it does part of the extraction and the spool is only charged for the remainder. No floor -
//a stopped compressor moves no air.
private _airGas = ((_ng ^ (_engine get "massFlowExp")) * _dens) * (_engine get "airCoef");

//Spooling down, the compressor is pure load and that is what stops it. The floor finishes the
//stop, since ng^2 alone only asymptotes.
private _drag     = (_engine get "compressorLoad") * ([1.0, _engine get "compDragMult"] select _spooling);
private _absorbed = (_drag * _ng * _ng) + ([0.0, _engine get "compDragFloor"] select _spooling);

//The air covers part of what the power turbine extracts, but it never drives the compressor -
//heat does that. So it offsets the load and no further.
private _ngDot = (_fuelGas + _starterTq - _absorbed - ((_shaft - _airGas) max 0.0)) / (_engine get "compressorInertia");
_ng = [_ng + (_ngDot * _deltaTime), 0.0, 1.1] call BIS_fnc_clamp;

[_ng, _fuelGas, _absorbed, _airGas]
