/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_massUpdateStation

Description:
    Totals the mass carried on one wing station: the launcher, its remaining
    rounds, and the fuel in an external tank.

    What a store weighs comes from the aircraft's Stores table, not from Core.

Parameters:
    _heli      - The helicopter to get information from [Unit].
    _pylons    - 1-based pylon indices this station carries [Array].
    _stationNo - 1-based station number, used to find its external fuel [Number].

Returns:
    Station mass in kg [Number].

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_pylons", "_stationNo"];

private _pylonMagazines = getPylonMagazines _heli;
private _stores         = _heli getVariable ["bmkhs_stores", []];
private _stationMass    = 0.0;

//The store fitted to this station is whatever its first loaded pylon carries.
private _fittedMag = "";
{
    private _mag = _pylonMagazines param [_x - 1, ""];   //getPylonMagazines is 0-based
    if (_mag != "") exitWith { _fittedMag = toLower _mag };
} forEach _pylons;

if (_fittedMag == "") exitWith { 0.0 };

private _storeIdx = _stores findIf { [_x get "match", _fittedMag] call BIS_fnc_inString };
if (_storeIdx < 0) exitWith { 0.0 };

private _store = _stores select _storeIdx;
private _launcherMass = _store get "launcherMass";
private _massPerRound = _store get "massPerRound";
private _isTank       = _store get "isTank";

_stationMass = _launcherMass;

if (_isTank) then {
    //Find the aux tank sitting on this station and add whatever fuel it is holding. The
    //tank owns its variable name, so nothing here assumes one.
    private _fuelMass = 0.0;
    {
        if ((_x get "station") == _stationNo) exitWith {
            _fuelMass = _heli getVariable [(_x get "varName") + "Mass", 0.0];
        };
    } forEach (_heli getVariable ["bmkhs_auxTanks", []]);
    _stationMass = _stationMass + _fuelMass;
} else {
    private _rounds = 0;
    {
        if ((_pylonMagazines param [_x - 1, ""]) != "") then {
            _rounds = _rounds + (_heli ammoOnPylon format ["pylons%1", _x]);
        };
    } forEach _pylons;
    _stationMass = _stationMass + (_rounds * _massPerRound);
};

_stationMass
