/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engine

Description:
    Provides a visually accurate simulation of a turbine engine based on table
    interpolation and collective input.

Parameters:
    _heli      - The helicopter to get information from [Unit].
    _engNum    - The engine to simulate

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_engNum"];

private _cfg           = configOf _heli;
private _sfmPlusConfig = _cfg >> "BMKHS_HeliSim";

private _deltaTime          = _heli getVariable "bmkhs_deltaTime";
private _engState           = _heli getVariable "bmkhs_engState" select _engNum;
private _isSingleEng        = _heli getVariable "bmkhs_isSingleEng";
//private _isAutorotating     = _heli getVariable "bmkhs_isAutorotating";
private _engPowerLeverState = _heli getVariable "bmkhs_engPowerLeverState" select _engNum;
private _engPctNG           = _heli getVariable "bmkhs_engPctNG" select _engNum;
private _engPctTQ           = _heli getVariable "bmkhs_engPctTQ" select _engNum;
private _engTGT             = _heli getVariable "bmkhs_engTGT" select _engNum;
private _engOilPSI          = _heli getVariable "bmkhs_engOilPSI" select _engNum;
private _engFF              = _heli getVariable "bmkhs_engFF" select _engNum;
private _collectiveOutput   = _heli getVariable "bmkhs_collectiveOutput";
private _engThrottle        = 0.0;
private _engSimTime         = getNumber (_sfmPlusConfig >> "engSimTime");
//With no start procedure the whole spool is the startup, so it runs longer - the rotor
//comes up over that window rather than snapping to speed.
if !(_heli getVariable ["bmkhs_useSystems", false]) then {
    _engSimTime = _engSimTime * 1.25;
};

//Torque - TQ
private _engIdleTQ  = getNumber (_sfmPlusConfig >> "engIdleTQ");
private _engFlyTQ   = getNumber (_sfmPlusConfig >> "engFlyTQ");
private _engBaseTQ  = 0.0;
private _engSetTQ   = 0.0;
private _engLimitTQ = 0.0;

private _hvrIGE      = _heli getVariable "bmkhs_hvrTQ_IGE";
private _hvrOGE      = _heli getVariable "bmkhs_hvrTQ_OGE";
private _maxTQ_CONT  = _heli getVariable "bmkhs_maxTQ_CONT";
private _maxTQ_DE    = _heli getVariable "bmkhs_maxTQ_DE";
private _maxTQ_SE    = _heli getVariable "bmkhs_maxTQ_SE";
private _maxTQ       = getNumber (_sfmPlusConfig >> "engMaxTQ");
private _ovrspdTQ    = getNumber (_sfmPlusConfig >> "engOvrspdTQ");
//Gas producer - Ng
private _engStartNG  = getNumber (_sfmPlusConfig >> "engStartNG");
private _engIdleNG   = getNumber (_sfmPlusConfig >> "engIdleNG");
private _engFlyNG    = getNumber (_sfmPlusConfig >> "engFlyNG");
private _engMaxNG    = getNumber (_sfmPlusConfig >> "engMaxNG");
private _engBaseNG   = 0.0;
private _engSetNG    = 0.0;

//Throttle
if (_engPowerLeverState in ["OFF", "IDLE"]) then {
    _engThrottle = 0.0;
} else { _engThrottle = 1.0; };

//Tq

if (_engPowerLeverState != "OFF") then {
    _engBaseTQ = _engIdleTQ + (_engFlyTQ - _engIdleTQ) * _engThrottle;
} else {
    _engBaseTQ = 0.0;
};

//Ng
_engBaseNG = _engIdleNG + (_engFlyNG - _engIdleNG) * _engThrottle;

switch (_engState) do {
	case "OFF": {
		//Ng
		_engPctNG = [_engPctNG, 0.0, _deltaTime] call BIS_fnc_lerp;
	};
	case "STARTING": {
		if (_engPowerLeverState == "OFF") then {
			//Ng
			_engPctNG = [_engPctNG, _engStartNG, (1.0 / (_engSimTime / 2.0)) * _deltaTime] call BIS_fnc_lerp;
		} else {
			//Ng
			_engPctNG = [_engPctNG, _engBaseNG, (1.0 / _engSimTime) * _deltaTime] call BIS_fnc_lerp;
		};

		//Transition state to ON
		if (_engPctNG > (_heli getVariable "bmkhs_engRunNG")) then {
			_engState = "ON";
			[_heli, "bmkhs_engState", _engNum, "ON", true] call bmkhs_fnc_utilSetArrayVariable;
		};
	};
	case "ON": {
		if (_engPowerLeverState == "OFF") then {
			_engState = "OFF";
			[_heli, "bmkhs_engState", _engNum, "ON", true] call bmkhs_fnc_utilSetArrayVariable;
		};
		//Ng
		_engSetNG = _engBaseNG + (_engMaxNG - _engBaseNG) * _engThrottle * _collectiveOutput;
		_engPctNG = [_engPctNG, _engSetNG, _deltaTime] call BIS_fnc_lerp;
	};
};

private _intEngBaseTable = [getArray (_sfmPlusConfig >> "engBaseTable"), _engPctNG] call bmkhs_fnc_mathLinearInterp;
//Base TGT
private _engBaseTGT      = _intEngBaseTable select 1;
//Base Oil
private _engBaseOilPSI   = _intEngBaseTable select 4;
//Torque
private _heightAGL  = ASLToAGL getPosASL _heli # 2;
private _hvrTQ      = linearConversion [15.24, 1.52, _heightAGL, _hvrOGE, _hvrIGE, true];

private _engTable = [[  _engBaseTQ, _engBaseTGT, _engBaseNG, _engBaseOilPSI],
                     [ _maxTQ_CONT,         810,      0.950,           0.91],   //30 min
                     [   _maxTQ_DE,         867,      0.990,           0.94],   //10 min
                     [   _maxTQ_SE,         896,      0.997,           0.99]];  //2.5 Min

_engTGT    = [_engTable,   _engPctTQ] call bmkhs_fnc_mathLinearInterp select 1;
if (_isSingleEng) then {
    private _tgtMax = _heli getVariable "bmkhs_engMaxTGT_SE";
    if (_engTGT > _tgtMax) then { _engTGT = _tgtMax; };
} else {
    private _tgtMax = _heli getVariable "bmkhs_engMaxTGT_DE";
    if (_engTGT > _tgtMax) then { _engTGT = _tgtMax; };
};

_engOilPSI = [_engTable,   _engPctTQ] call bmkhs_fnc_mathLinearInterp select 3;
_engFF     = [getArray (_sfmPlusConfig >> "engFFTable"), _engPctTQ] call bmkhs_fnc_mathLinearInterp select 1;


//Update variables
[_heli, "bmkhs_engPctNG",      _engNum, _engPctNG] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engBaseTGT",    _engNum, _engBaseTGT] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engBaseOilPSI", _engNum, _engBaseOilPSI] call bmkhs_fnc_utilSetArrayVariable;

[_heli, "bmkhs_engTGT",        _engNum, _engTGT] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engOilPSI",     _engNum, _engOilPSI] call bmkhs_fnc_utilSetArrayVariable;
[_heli, "bmkhs_engFF",         _engNum, _engFF] call bmkhs_fnc_utilSetArrayVariable;
