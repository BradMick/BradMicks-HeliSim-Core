#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

private _numRotor = _heli getVariable "bmkhs_numSimpleRotors";

for "_rotorIndex" from 0 to (_numRotor - 1) do {
    [ _heli
    , _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorType")            select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorDir")             select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorNumBlades")       select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorGearRatio")       select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorPivot")           select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorMastLength")      select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorRotation")        select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorPitchFlapMin")    select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorPitchFlapMid")    select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorPitchFlapMax")    select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorCyclicPitchGain") select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorRollFlapMin")     select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorRollFlapMid")     select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorRollFlapMax")     select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorCyclicRollGain")  select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorFlapGain")        select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorFlapTimeConst")   select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorBladeRadius")     select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorBladeChord")      select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorBladeMass")       select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorLiftCoefTable")   select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorDragCoefTable")   select _rotorIndex
    ] call bmkhs_fnc_simpleRotor;
};

/*
private _rVec = [1,0,0];
private _fVec = [0,1,0];
private _uVec = [0,0,1];

private _hubR = [_rVec, P, R, Y] call bmkhs_fnc_mathVectorRotate;
private _hubF = [_fVec, P, R, Y] call bmkhs_fnc_mathVectorRotate;
private _hubU = [_uVec, P, R, Y] call bmkhs_fnc_mathVectorRotate;

private _hub  = _hubR vectorAdd _hubF vectorAdd _hubU;

systemChat format ["_hubR [%1, %2, %3]", (_hubR select 0) toFixed 3, (_hubR select 1) toFixed 3, (_hubR select 2) toFixed 3];
systemChat format ["_hubF [%1, %2, %3]", (_hubF select 0) toFixed 3, (_hubF select 1) toFixed 3, (_hubF select 2) toFixed 3];
systemChat format ["_hubU [%1, %2, %3]", (_hubU select 0) toFixed 3, (_hubU select 1) toFixed 3, (_hubU select 2) toFixed 3];

private _force = [0,0,1796];
private _forceVec = 
[
  [_hubR # 0, _hubF # 0, _hubU # 0]
, [_hubR # 1, _hubF # 1, _hubU # 1]
, [_hubR # 2, _hubF # 2, _hubU # 2]
] matrixMultiply [[_force # 0], [_force # 1], [_force # 2]];
_forceVec = [_forceVec # 0 # 0, _forceVec # 1 # 0, _forceVec # 2 # 0];

private _heliCom = getCenterOfMass _heli;
systemChat format ["_heliCom [%1, %2, %3]", (_heliCom select 0) toFixed 3, (_heliCom select 1) toFixed 3,(_heliCom select 2) toFixed 3];

private _deltaPos = [0,-6.98,-0.075] vectorDiff _heliCom;
private _torque   = _deltaPos vectorCrossProduct _forceVec;

systemChat format ["_forceVec [%1, %2, %3]", (_forceVec select 0) toFixed 3, (_forceVec select 1) toFixed 3, (_forceVec select 2) toFixed 3];
systemChat format ["_torque [%1, %2, %3]", (_torque select 0) toFixed 3, (_torque select 1) toFixed 3, (_torque select 2) toFixed 3];
*/
