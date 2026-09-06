/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_apu

Description:
    What the APU does BESIDES supplying power. Whether it is running, and how
    fast it is turning, is the component graph's answer - this burns the fuel
    that costs and tells the aircraft when the state changed.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

private _apuOn = _heli getVariable ["bmkhs_apuOn", false];

//175 pph while it runs.
_heli setVariable ["bmkhs_apuFF_kgs", [0.0, 0.0220] select _apuOn];

//Cockpit indication is the aircraft's business - Core only reports the state, and only
//when it actually changes. The default here has to be something apuOn can DIFFER from, or
//the first transition compares equal to itself and the notify never fires at all.
if (_apuOn isNotEqualTo (_heli getVariable ["bmkhs_apuOnLast", !_apuOn])) then {
    _heli setVariable ["bmkhs_apuOnLast", _apuOn];
    [_heli, "apuStateChanged"] call bmkhs_fnc_utilNotify;
};
