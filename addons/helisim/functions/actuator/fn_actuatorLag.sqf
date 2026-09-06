/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engine

Description:
    Sourced from JSBSim.

Parameters:
    ...

Returns:
    Lag coeffient required for actuator simulation.

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_inputAxis", "_input", "_lagVal"];

private _output = 0.0;
switch (_inputAxis) do {
    case "pitch" : {
        private _prevLagInput  = _heli getVariable "bmkhs_prevLagInputPitch";
        private _prevLagOutput = _heli getVariable "bmkhs_prevLagOutputPitch";

        private _ca = [_heli, _lagVal] call bmkhs_fnc_actuatorGetLagCoefA;
        private _cb = [_heli, _lagVal] call bmkhs_fnc_actuatorGetLagCoefB;

        _output     = _ca * (_input + _prevLagInput) + _prevLagOutput * _cb;

        _heli setVariable ["bmkhs_prevLagInputPitch",  _input];
        _heli setVariable ["bmkhs_prevLagOutputPitch", _output];
    };
    case "roll" : {
        private _prevLagInput  = _heli getVariable "bmkhs_prevLagInputRoll";
        private _prevLagOutput = _heli getVariable "bmkhs_prevLagOutputRoll";

        private _ca = [_heli, _lagVal] call bmkhs_fnc_actuatorGetLagCoefA;
        private _cb = [_heli, _lagVal] call bmkhs_fnc_actuatorGetLagCoefB;

        _output     = _ca * (_input + _prevLagInput) + _prevLagOutput * _cb;

        _heli setVariable ["bmkhs_prevLagInputRoll",  _input];
        _heli setVariable ["bmkhs_prevLagOutputRoll", _output];
    };
    case "yaw" : {
        private _prevLagInput  = _heli getVariable "bmkhs_prevLagInputYaw";
        private _prevLagOutput = _heli getVariable "bmkhs_prevLagOutputYaw";

        private _ca = [_heli, _lagVal] call bmkhs_fnc_actuatorGetLagCoefA;
        private _cb = [_heli, _lagVal] call bmkhs_fnc_actuatorGetLagCoefB;

        _output     = _ca * (_input + _prevLagInput) + _prevLagOutput * _cb;

        _heli setVariable ["bmkhs_prevLagInputYaw",  _input];
        _heli setVariable ["bmkhs_prevLagOutputYaw", _output];
    };
    case "collective" : {
        private _prevLagInput  = _heli getVariable "bmkhs_prevLagInputColl";
        private _prevLagOutput = _heli getVariable "bmkhs_prevLagOutputColl";

        private _ca = [_heli, _lagVal] call bmkhs_fnc_actuatorGetLagCoefA;
        private _cb = [_heli, _lagVal] call bmkhs_fnc_actuatorGetLagCoefB;

        _output     = _ca * (_input + _prevLagInput) + _prevLagOutput * _cb;

        _heli setVariable ["bmkhs_prevLagInputColl",  _input];
        _heli setVariable ["bmkhs_prevLagOutputColl", _output];
    };
};

_output;
