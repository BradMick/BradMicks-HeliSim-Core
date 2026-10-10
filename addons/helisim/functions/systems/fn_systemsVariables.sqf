/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_variables

Description:
    ...

Parameters:
    _heli      - The helicopter to get information from [Unit].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

#include "\bmkhs_helisim\functions\systems\systems.hpp"

if !(_heli getVariable ["bmkhs_systemsInitialised", false]) then {
    [_heli, "bmkhs_systemsInitialised", true, true] call bmkhs_fnc_utilSeed;

    //Electrical and APU are only modelled when the aircraft asks for them. Without
    //them the aircraft behaves like vanilla Arma: powered up and running, with no
    //start procedure - so everything downstream reads as already on.
    private _sys = _heli getVariable ["bmkhs_useSystems", false];

    //Switch states - cold and dark either way, the crew turns it on.
    [_heli, "bmkhs_battSwitchOn", false, true] call bmkhs_fnc_utilSeed;

    //Electrical - cold and dark. With systems the crew brings the buses up; without them
    //they come on with everything else when the aircraft wakes.
    [_heli, "bmkhs_battBusOn", false, true] call bmkhs_fnc_utilSeed;
    [_heli, "bmkhs_acBusOn",   false, true] call bmkhs_fnc_utilSeed;
    [_heli, "bmkhs_dcBusOn",   false, true] call bmkhs_fnc_utilSeed;

    //APU - an aircraft with no systems has no APU to be on, so it reads OFF. Anything
    //that needed it, like an engine start, is not gated on it either.
    [_heli, "bmkhs_apuBtnOn",   false, true] call bmkhs_fnc_utilSeed;
    [_heli, "bmkhs_apuRpm_pct", 0.0,   true] call bmkhs_fnc_utilSeed;
    [_heli, "bmkhs_apuOn",      false, true] call bmkhs_fnc_utilSeed;
    //Bleed air defaults available without systems, so nothing that needs it is blocked.
    [_heli, "bmkhs_pneuAvail",  !_sys, true] call bmkhs_fnc_utilSeed;


    //Hydraulics - reservoirs and the accumulator start full.

    //Pressure comes up with the aircraft too, so it reads zero until then.
    [_heli, "bmkhs_priHydPsi",  0.0,    true] call bmkhs_fnc_utilSeed;
    [_heli, "bmkhs_utilHydPsi", 0.0,    true] call bmkhs_fnc_utilSeed;
    [_heli, "bmkhs_accHydPsi",  3000.0, true] call bmkhs_fnc_utilSeed;
};

_heli setVariable ["bmkhs_apuFuelAvail",      true];
_heli setVariable ["bmkhs_repairPending",     false];
_heli setVariable ["bmkhs_sysWalkCost",       0];
_heli setVariable ["bmkhs_sysWalkPeak",       0];
_heli setVariable ["bmkhs_sysWatchedLast",    createHashMap];

[_heli, "bmkhs_emerHydOn",        false, true] call bmkhs_fnc_utilSeed;
//Latched by a start begun with the rotor brake set; cleared only by the brake coming off.
[_heli, "bmkhs_rtrBrkStartLatch", 0,     true] call bmkhs_fnc_utilSeed;
//A running engine is a bleed air source, alongside the APU.
[_heli, "bmkhs_engBleedAvail",    false, true] call bmkhs_fnc_utilSeed;

//Systems tuning - the aircraft supplies these, Core keeps damage thresholds fixed
