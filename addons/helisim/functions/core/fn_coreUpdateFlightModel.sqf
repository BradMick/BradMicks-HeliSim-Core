#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

if (isGamePaused || CBA_missionTime < 0.1) exitWith {};

if (bmkhs_rotorModel == 1) then {
    [_heli] call bmkhs_fnc_rotorUpdate;
} else {
    [_heli] call bmkhs_fnc_simpleRotorUpdate;
};

//Fuselage
[_heli] call bmkhs_fnc_fuselageUpdate;

//Wings
[_heli] call bmkhs_fnc_wingUpdate;
