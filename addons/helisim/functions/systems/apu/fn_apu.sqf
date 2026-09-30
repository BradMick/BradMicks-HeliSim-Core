/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_apu

Description:
    What the APU does BESIDES supplying power. Whether it is running, and how
    fast it is turning, is the component graph's answer - this burns the fuel
    that costs and tells the aircraft when the state changed.

Parameters:
    _heli      - The helicopter [Object]
    _deltaTime - Frame time [Number]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_deltaTime"];
#include "\bmkhs_helisim\functions\core\core.hpp"

private _apuOn = _heli getVariable ["bmkhs_apuOn", false];

//Draws its own fuel from its own tank; no tanks modelled is always fuel.
private _apu = (_heli getVariable "bmkhs_sysProducers") select {(_x get "damageRole") == "apu"};
if (_apu isNotEqualTo [] && {(_heli getVariable "bmkhs_numFuelTanks") > 0}) then {
    private _tank = (_apu select 0) get "fuelTank";
    //An armed fire handle closes the APU's fuel while DC is up.
    private _shut = (_heli getVariable "bmkhs_dcBusOn") && {"apuFireHandle" in (_heli getVariable "bmkhs_ctrlIndex")}
        && {_heli getVariable "bmkhs_apuFireHandleOn"};
    private _mass = if (_tank == "" || _shut) then {0} else {_heli getVariable (_tank + "Mass")};
    if (_apuOn && {_mass > EPSILON}) then {
        _heli setVariable [_tank + "Mass", (_mass - (((_apu select 0) get "fuelFlow") * _deltaTime)) max 0];
    };
    [_heli, "bmkhs_apuFuelAvail", _mass > EPSILON] call bmkhs_fnc_utilUpdateNetworkGlobal;
};

//Stopped, or no fuel: the APU drops out and needs a fresh press to restart.
if (local _heli && {_heli getVariable "bmkhs_apuBtnOn"}
        && {!(_heli getVariable "bmkhs_apuFuelAvail") || {!_apuOn && {_heli getVariable ["bmkhs_apuOnLast", false]}}}) then {
    ["apuBtn", 0, _heli] call bmkhs_fnc_controlSet;
};

//Cockpit indication is the aircraft's business - Core only reports the state, and only
//when it actually changes. The default here has to be something apuOn can DIFFER from, or
//the first transition compares equal to itself and the notify never fires at all.
if (_apuOn isNotEqualTo (_heli getVariable ["bmkhs_apuOnLast", !_apuOn])) then {
    _heli setVariable ["bmkhs_apuOnLast", _apuOn];
    [_heli, "apuStateChanged"] call bmkhs_fnc_utilNotify;
};
