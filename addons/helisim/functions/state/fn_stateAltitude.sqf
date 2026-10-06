/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_stateAltitude

Description:
    Publishes the helicopter's height above the ground, exact.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

//Height above the ground, METRES, exact - the flight model works in metres. An instrument's
//steps and range are the reader's; the barometric altitude is fn_environment's.
_heli setVariable ["bmkhs_radAlt", getPos _heli # 2];
