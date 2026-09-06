#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

[_heli] call bmkhs_fnc_fuselageFront;
[_heli] call bmkhs_fnc_fuselageTop;
[_heli] call bmkhs_fnc_fuselageSide;
