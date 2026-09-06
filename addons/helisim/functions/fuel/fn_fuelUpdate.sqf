/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fuelUpdate

Description:
    Runs the fuel system for one frame and publishes the results.

    The work itself is split by concern:
      fuelDraw         engine and APU consumption out of the mains
      fuelTransfer     XFER pump between mains, transfer cells feeding them
      fuelTransferAux  auxiliary tanks feeding the internal tanks
      fuelLeak         damaged tanks draining

    Tank state is held in two arrays for the duration of the frame - internal
    and auxiliary - and written back at the end. The helpers mutate those
    arrays in place.

Parameters:
    _heli - The helicopter to update [Unit].

Returns:
    None

Author:
    BradMick / FZA Development Team
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\fuel\fuel.hpp"
params ["_heli"];

private _deltaTime = _heli getVariable "bmkhs_deltaTime";
if (_deltaTime <= 0) exitWith {};

private _maxTotFuelMass = _heli getVariable "bmkhs_maxTotFuelMass";
if (_maxTotFuelMass <= 0) exitWith {};

//If Arma fuel was changed externally (editor, trigger, refuel script), resync so the
//simulation and the engine agree from here on.
private _storedTotFuelMass = _heli getVariable ["bmkhs_totFuelMass", 0];
private _storedFuelFrac    = _storedTotFuelMass / _maxTotFuelMass;
if (abs ((fuel _heli) - _storedFuelFrac) > 0.01) then {
    [_heli] call bmkhs_fnc_fuelSet;
    _maxTotFuelMass = _heli getVariable "bmkhs_maxTotFuelMass";
};

private _fuelTanks = _heli getVariable ["bmkhs_fuelTanks", []];
private _auxTanks  = _heli getVariable ["bmkhs_auxTanks",  []];
private _mains     = _heli getVariable ["bmkhs_fuelMains",     []];
private _transfers = _heli getVariable ["bmkhs_fuelTransfers", []];

private _fuelMass = [];
private _fuelMax  = [];
private _fuelLow  = [];
for "_i" from 1 to (count _fuelTanks) do {
    private _v = (_fuelTanks select (_i - 1)) get "varName";
    _fuelMass pushBack (_heli getVariable [_v + "Mass", 0]);
    _fuelMax  pushBack (_heli getVariable [_v + "Max",  0]);
    _fuelLow  pushBack (_heli getVariable [_v + "Low",  0]);
};

private _auxMass = [];
private _auxMax  = [];
for "_i" from 1 to (count _auxTanks) do {
    private _v = (_auxTanks select (_i - 1)) get "varName";
    _auxMass pushBack (_heli getVariable [_v + "Mass", 0]);
    _auxMax  pushBack (_heli getVariable [_v + "Max",  0]);
};

//Which aux switch groups are armed. The switch variable per group is the aircraft's;
//Core reads bmkhs_<group>AuxOn for whatever groups the tanks declare.
private _groupOn = createHashMap;
{
    private _g = _x get "group";
    if !(_g in _groupOn) then {
        _groupOn set [_g, _heli getVariable [format ["bmkhs_%1AuxOn", toLower _g], false]];
    };
} forEach _auxTanks;

//An armed aux tank with fuel left inhibits the transfer cells, so aux empties first.
//findIf does NOT provide _forEachIndex, so the index comes from a plain counter.
private _auxArmed = false;
{
    if ((_groupOn getOrDefault [_x get "group", false])
            && {(_auxMass param [_forEachIndex, 0]) > EXT_EMPTY_ADV_THRESH_KG}) exitWith {
        _auxArmed = true;
    };
} forEach _auxTanks;

([_heli, _fuelMass, _mains, _deltaTime] call bmkhs_fnc_fuelDraw)
    params ["_eng1FuelAvail", "_eng2FuelAvail", "_apuFuelAvail"];

([_heli, _fuelMass, _fuelMax, _fuelLow, _fuelTanks, _mains, _transfers, _auxArmed, _deltaTime] call bmkhs_fnc_fuelTransfer)
    params ["_intercellActive", "_intercellDir", "_cellFlowing"];

[_heli, _fuelMass, _fuelTanks, _deltaTime] call bmkhs_fnc_fuelLeak;

([_heli, _fuelMass, _fuelMax, _auxMass, _auxTanks, _groupOn, _deltaTime] call bmkhs_fnc_fuelTransferAux)
    params ["_auxPresent", "_groupFlow"];

//Clamp every tank to its capacity.
{ _fuelMass set [_forEachIndex, 0 max _x min (_fuelMax param [_forEachIndex, 0])] } forEach _fuelMass;
{ _auxMass  set [_forEachIndex, 0 max _x min (_auxMax  param [_forEachIndex, 0])] } forEach _auxMass;

//Starvation gets a grace period so a brief interruption does not cut an engine.
{
    _x params ["_avail", "_var", "_sinceVar"];
    private _since = _heli getVariable [_sinceVar, -1];
    private _out   = _avail;
    if (!_avail) then {
        if (_since < 0) then {
            _since = CBA_missionTime;
            _heli setVariable [_sinceVar, _since];
        };
        _out = (CBA_missionTime - _since) < FUEL_STARVE_GRACE_SEC;
    } else {
        _heli setVariable [_sinceVar, -1];
    };
    [_heli, _var, _out] call bmkhs_fnc_utilUpdateNetworkGlobal;
} forEach [
    [_eng1FuelAvail, "bmkhs_eng1FuelAvail", "bmkhs_eng1StarvedSince"],
    [_eng2FuelAvail, "bmkhs_eng2FuelAvail", "bmkhs_eng2StarvedSince"]
];

//The APU gets no grace period - it cuts as soon as its tank is dry.
[_heli, "bmkhs_apuFuelAvail", _apuFuelAvail] call bmkhs_fnc_utilUpdateNetworkGlobal;

_heli setVariable ["bmkhs_intercellTransferActive", _intercellActive];
_heli setVariable ["bmkhs_intercellTransferDir",    _intercellDir];
_heli setVariable ["bmkhs_iafsFlowing",             _cellFlowing];

{
    _heli setVariable [format ["bmkhs_%1AuxFlowing", toLower _x], (_groupFlow getOrDefault [_x, 0]) > 0];
} forEach (keys _groupOn);

private _totFuelMass = 0;
{ _totFuelMass = _totFuelMass + _x } forEach (_fuelMass + _auxMass);
if (local _heli) then {
    _heli setFuel (_totFuelMass / _maxTotFuelMass);
};

{ _heli setVariable [((_fuelTanks select _forEachIndex) get "varName") + "Mass", _x] } forEach _fuelMass;
{ _heli setVariable [((_auxTanks  select _forEachIndex) get "varName") + "Mass", _x] } forEach _auxMass;
_heli setVariable ["bmkhs_totFuelMass", _totFuelMass];

//Whether the crew can see the fuel page is the aircraft's business - it sets this if it
//wants empty-tank advisories to re-arm only while the page is displayed.
private _fuelPageOpen = _heli getVariable ["bmkhs_fuelPageOpen", false];
{
    private _present = _auxPresent param [_forEachIndex, false];
    private _var     = ((_auxTanks select _forEachIndex) get "varName") + "EmptyArmed";
    if (!_present || _x >= EXT_EMPTY_ADV_THRESH_KG) then {
        _heli setVariable [_var, true];
    } else {
        if (_fuelPageOpen) then { _heli setVariable [_var, false] };
    };
} forEach _auxMass;
