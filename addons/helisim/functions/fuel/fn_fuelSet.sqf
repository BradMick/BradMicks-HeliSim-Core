/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fuelSet

Description:
    Distributes Arma's fuel fraction across the aircraft's tanks and publishes
    the resulting per-tank masses.

    Internal capacity fills first; whatever is left over goes to the auxiliary
    tanks actually fitted. Each tank takes a share of the load proportional to
    its own capacity, so the split works for any number of tanks of any size.

    An uninstalled removable tank holds nothing and contributes no capacity.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

private _percentFuel = fuel _heli;
private _fuelTanks   = _heli getVariable ["bmkhs_fuelTanks", []];
private _auxTanks    = _heli getVariable ["bmkhs_auxTanks",  []];
private _stations    = _heli getVariable ["bmkhs_stations",  []];

//Internal capacity, counting only the tanks that are actually fitted.
private _fuelFitted   = [];
private _fuelCapacity = [];
{
    private _capacity = _x get "capacity";
    private _fitted = !(_x get "removable")
                   || {_heli getVariable [(_x get "varName") + "Installed", false]};
    _fuelFitted   pushBack _fitted;
    _fuelCapacity pushBack ([0, _capacity] select _fitted);
} forEach _fuelTanks;

//Auxiliary capacity, counting only the stations carrying a tank.
private _pylonMagazines = getPylonMagazines _heli;
private _auxFitted      = [];
private _auxCapacity    = [];
{
    private _station  = _x get "station";
    private _capacity = _x get "capacity";
    private _stn     = _stations param [_station - 1, createHashMap];
    private _pylons  = _stn getOrDefault ["pylons", []];
    private _present = _pylons findIf {
        ["auxTank", _pylonMagazines param [_x - 1, ""]] call BIS_fnc_inString
    } > -1;
    _auxFitted   pushBack _present;
    _auxCapacity pushBack ([0, _capacity] select _present);
} forEach _auxTanks;

private _maxIntFuelMass = 0;
{ _maxIntFuelMass = _maxIntFuelMass + _x } forEach _fuelCapacity;
private _maxExtFuelMass = 0;
{ _maxExtFuelMass = _maxExtFuelMass + _x } forEach _auxCapacity;

private _maxTotFuelMass = _maxIntFuelMass + _maxExtFuelMass;
if (_maxTotFuelMass <= 0) exitWith {};

private _totFuelMass = _maxTotFuelMass * _percentFuel;

//Internal fills first; the aux tanks get whatever is above internal capacity.
private _intFuelMass = _totFuelMass min _maxIntFuelMass;
private _extFuelMass = 0 max (_totFuelMass - _intFuelMass) min _maxExtFuelMass;

//Each tank takes its share of the load in proportion to its capacity.
private _actualTotFuelMass = 0;
{
    private _mass = if (_maxIntFuelMass > 0) then { _intFuelMass * (_x / _maxIntFuelMass) } else { 0 };
    _heli setVariable [((_fuelTanks select _forEachIndex) get "varName") + "Mass", _mass];
    _actualTotFuelMass = _actualTotFuelMass + _mass;
} forEach _fuelCapacity;

{
    private _mass = if (_maxExtFuelMass > 0) then { _extFuelMass * (_x / _maxExtFuelMass) } else { 0 };
    _heli setVariable [((_auxTanks select _forEachIndex) get "varName") + "Mass", _mass];
    _actualTotFuelMass = _actualTotFuelMass + _mass;
} forEach _auxCapacity;

_heli setVariable ["bmkhs_totFuelMass",    _actualTotFuelMass];
_heli setVariable ["bmkhs_maxTotFuelMass", _maxTotFuelMass];
