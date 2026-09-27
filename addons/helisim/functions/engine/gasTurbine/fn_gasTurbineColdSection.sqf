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
    _dens      - Air density as a fraction of a standard day [Number]
    _running   - Burning, as opposed to cranked [Boolean]
    _spooling  - Spooling down, unfired and uncranked [Boolean]
    _deltaTime - Frame time [Number]

Returns:
    [_ng, _fuelGas, _compWork, _airGas] - the stepped speed, and the terms the
    power turbine needs [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_engine", "_ng", "_fuelCmd", "_starterTq", "_dens", "_running", "_spooling", "_deltaTime"];

//Heat accelerates the spool - the compressor turbine runs on combustion.
private _fuelGas = [0.0, _fuelCmd * _dens] select _running;

//Cold air the compressor is moving right now, lit or not, on its way to the power turbine.
//No floor - a stopped compressor moves no air.
private _airGas = ((_ng ^ (_engine get "massFlowExp")) * _dens) * (_engine get "airCoef");

//Running, the load rises steeply with Ng. Spooling down, the compressor is pure load and that
//is what stops it; the floor finishes the stop, since ng^2 alone only asymptotes.
private _compLoad = _engine get "compressorLoad";
private _absorbed = if (_running) then {
    _compLoad * (_engine get "compRunMult") * (_ng ^ (_engine get "compRunExp"))
} else {
    (_compLoad * ([1.0, _engine get "compDragMult"] select _spooling) * _ng * _ng)
        + ([0.0, _engine get "compDragFloor"] select _spooling)
};

//What the compressor turbine takes out of the gas before it reaches the power turbine.
private _compWork = _compLoad * _ng * _ng;

//Heat against the compressor, and nothing else. The free turbine is FREE - rotor load reaches
//it and stops there, so it cannot drag the gas generator down.
private _ngDot = (_fuelGas + _starterTq - _absorbed) / (_engine get "compressorInertia");
_ng = [_ng + (_ngDot * _deltaTime), 0.0, 1.1] call BIS_fnc_clamp;

[_ng, _fuelGas, _compWork, _airGas]
