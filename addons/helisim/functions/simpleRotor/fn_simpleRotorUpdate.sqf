#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

private _numRotor = _heli getVariable "bmkhs_numSimpleRotors";

for "_rotorIndex" from 0 to (_numRotor - 1) do {
    [ _heli
    , _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorPivot")    	     select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorRotation") 	     select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorType")     	     select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorDir")	  	     select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorNumBlades")	     select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorMastLength")      select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorGearRatio")       select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorBladeRadius")     select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorBladeChord")      select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorBladeMass")       select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorThrustCoefMin")   select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorThrustCoefMid")   select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorThrustCoefMax")   select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorPitchFlapMin")    select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorPitchFlapMid")    select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorPitchFlapMax")    select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorRollFlapMin")     select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorRollFlapMid")     select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorRollFlapMax")     select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorFlapTimeConst")   select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorDragCoefMin")     select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorDragCoefMid")     select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorDragCoefMax")     select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorInducedKappa")    select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorCyclicPitchGain") select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorCyclicRollGain")  select _rotorIndex
	, (_heli getVariable "bmkhs_simpleRotorRollGain")        select _rotorIndex
    , (_heli getVariable "bmkhs_simpleRotorHitPoint")        select _rotorIndex
    ] call bmkhs_fnc_simpleRotor;
};
