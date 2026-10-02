#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

if (bmkhs_rotorModel == 1) then {
    [_heli] call bmkhs_fnc_rotorUpdate;
} else {
    [_heli] call bmkhs_fnc_simpleRotorUpdate;
};

//Fuselage
[_heli] call bmkhs_fnc_fuselageUpdate;

//Wings
[_heli] call bmkhs_fnc_wingUpdate;
