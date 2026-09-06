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

if (!(_heli getVariable ["bmkhs_systemsInitialised", false]) && local _heli) then {
    _heli setVariable ["bmkhs_systemsInitialised", true, true];

    //Electrical and APU are only modelled when the aircraft asks for them. Without
    //them the aircraft behaves like vanilla Arma: powered up and running, with no
    //start procedure - so everything downstream reads as already on.
    private _sys = _heli getVariable ["bmkhs_useSystems", false];

    //Switch states - cold and dark either way, the crew turns it on.
    _heli setVariable ["bmkhs_battSwitchOn",      false, true];

    //Electrical - cold and dark. With systems the crew brings the buses up; without them
    //they come on with everything else when the aircraft wakes.
    _heli setVariable ["bmkhs_battPower_pctCharge", 1.0, true];
    _heli setVariable ["bmkhs_battBusOn",         false, true];
    _heli setVariable ["bmkhs_acBusOn",           false, true];
    _heli setVariable ["bmkhs_dcBusOn",           false, true];

    //APU - an aircraft with no systems has no APU to be on, so it reads OFF. Anything
    //that needed it, like an engine start, is not gated on it either.
    _heli setVariable ["bmkhs_apuBtnOn",          false, true];
    _heli setVariable ["bmkhs_apuRPM_pct",        0.0,   true];
    _heli setVariable ["bmkhs_apuOn",             false, true];
    //Bleed air defaults available without systems, so nothing that needs it is blocked.
    _heli setVariable ["bmkhs_pneuAvail",         !_sys, true];


    //Hydraulics - reservoirs and the accumulator start full.
    _heli setVariable ["bmkhs_priLevel_pctCharge",  1.0, true];
    _heli setVariable ["bmkhs_utilLevel_pctCharge", 1.0, true];
    _heli setVariable ["bmkhs_accHydPsiCharge",     1.0, true];

    //Pressure comes up with the aircraft too, so it reads zero until then.
    _heli setVariable ["bmkhs_priHydPsi",  0.0,    true];
    _heli setVariable ["bmkhs_utilHydPsi", 0.0,    true];
    _heli setVariable ["bmkhs_accHydPsi",  3000.0, true];
};

_heli setVariable ["bmkhs_apuFF_kgs",         0.0];
_heli setVariable ["bmkhs_dmgTimerCont",      0.0];
_heli setVariable ["bmkhs_dmgTimerTrans",     0.0];

_heli setVariable ["bmkhs_emerHydOn",         false, true];
//Latched by a start begun with the rotor brake set; cleared only by the brake coming off.
_heli setVariable ["bmkhs_rtrBrkStartLatch",  0, true];
//A running engine is a bleed air source, alongside the APU.
_heli setVariable ["bmkhs_engBleedAvail",     false, true];
_heli setVariable ["bmkhs_engineOverspeed",   [false, false], true];

//Systems tuning - the aircraft supplies these, Core keeps damage thresholds fixed
