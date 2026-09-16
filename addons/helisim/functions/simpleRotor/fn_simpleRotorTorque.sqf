/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_simpleRotorTorque

Description:
    Refers a rotor's torque to the engine shaft and publishes what the
    transmission and the torque gauge read.

    The rotor's drag torque IS what the drivetrain feels - the drag table
    carries induced and profile together, and its airspeed columns carry how
    they vary with speed. Nothing is derived here; the torque arrives already
    computed.

Parameters:
    _heli        - The helicopter [Unit].
    _rotorIndex  - Which rotor [Number].
    _rotorTorque - Torque at the rotor shaft (Nm) [Number].
    _gearRatio   - Rotor to engine shaft [Number].
    _numBlades   - Blade count [Number].
    _bladeMass   - Mass of one blade (kg) [Number].
    _bladeRadius - Blade radius (m) [Number].
    _deltaTime   - Frame time (s) [Number].

Returns:
    Nothing. Publishes bmkhs_reqEngTorque and bmkhs_rtrMoi.

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_rotorIndex", "_rotorTorque", "_gearRatio", "_numBlades", "_bladeMass", "_bladeRadius", "_deltaTime"];

if (!local _heli) exitWith {};

//The transmission divides by it, so zero means Nr never moves.
private _rtrMoi = _numBlades * ((1.0 / 3.0) * _bladeMass * (_bladeRadius * _bladeRadius));
[_heli, "bmkhs_rtrMoi", _rotorIndex, _rtrMoi, true] call bmkhs_fnc_utilSetArrayVariable;

private _reqEngTorque = if (_gearRatio > 0.0) then { _rotorTorque / _gearRatio } else { 0.0 };

//1 - exp(-dt/tau), so the filter is the same at 30 fps and 144.
private _tqSmoothed = (_heli getVariable "bmkhs_reqEngTorque") select _rotorIndex;
private _tqAlpha    = 1.0 - (exp (-_deltaTime / ((_heli getVariable "bmkhs_simpleRotorTorqueTau") select _rotorIndex)));
_tqSmoothed         = _tqSmoothed + ((_reqEngTorque - _tqSmoothed) * _tqAlpha);
if ([_tqSmoothed] call bmkhs_fnc_mathIsNAN || [_tqSmoothed] call bmkhs_fnc_mathIsINF) then { _tqSmoothed = 0.0; };

[_heli, "bmkhs_reqEngTorque", _rotorIndex, _tqSmoothed, true] call bmkhs_fnc_utilSetArrayVariable;
