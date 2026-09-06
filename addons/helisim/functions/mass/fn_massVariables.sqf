/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_massVariables

Description:
    Loads the mass and balance configuration - fuselage datum, CG limits, and
    the indexed seat, tank, station, magazine and store tables.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

//Mass and balance - datum and CG limits
_heli setVariable ["bmkhs_fsDatum",        getNumber (_config >> "fsDatum")];
_heli setVariable ["bmkhs_fwdCgLimit",     getNumber (_config >> "fwdCgLimit")];
_heli setVariable ["bmkhs_aftCgLimit",     getNumber (_config >> "aftCgLimit")];

//Empty mass and moment
_heli setVariable ["bmkhs_emptyMassFCR",       getNumber (_config >> "emptyMassFCR")];        //kg
_heli setVariable ["bmkhs_emptyMomFCR",        getNumber (_config >> "emptyMomFCR")];
_heli setVariable ["bmkhs_emptyMassNonFCR",    getNumber (_config >> "emptyMassNonFCR")];     //kg
_heli setVariable ["bmkhs_emptyMomNonFCR",     getNumber (_config >> "emptyMomNonFCR")];

//Indexed mass items. Each table is flattened at load so the per-frame massUpdate never
//touches config. Entries are HASHMAPS keyed by the config property name, not positional
//arrays: adding or removing a field then cannot silently shift what a reader sees.
//Class names are zero-padded to two digits (Seat01, Store01) so they sort correctly.
private _readClass = {
    params ["_parent", "_prefix", "_count"];
    private _out = [];
    for "_i" from 1 to _count do {
        _out pushBack (_parent >> format ["%1%2", _prefix, (["0", ""] select (_i > 9)) + str _i]);
    };
    _out
};

//SEATS
private _seats = [];
{
    _seats pushBack (createHashMapFromArray [
        ["arm",        getArray  (_x >> "arm")],
        ["mass",       getNumber (_x >> "mass")],
        ["role",       toLower getText (_x >> "role")],
        ["turret",     getArray  (_x >> "turret")],
        ["cargoIndex", getNumber (_x >> "cargoIndex")]
    ]);
} forEach ([_config >> "Seats", "Seat", getNumber (_config >> "numSeats")] call _readClass);
_heli setVariable ["bmkhs_seats", _seats];

//Tanks are not read here - fn_fuelVariables owns them and publishes bmkhs_fuelTanks,
//which massUpdate walks for the arms.

//STATIONS
private _stations = [];
{
    _stations pushBack (createHashMapFromArray [
        ["arm",    getArray (_x >> "arm")],
        ["pylons", getArray (_x >> "pylons")]
    ]);
} forEach ([_config >> "Stations", "Station", getNumber (_config >> "numStations")] call _readClass);
_heli setVariable ["bmkhs_stations", _stations];

//MAGAZINES
private _magazines = [];
{
    _magazines pushBack (createHashMapFromArray [
        ["match",        toLower getText (_x >> "match")],
        ["arm",          getArray  (_x >> "arm")],
        ["massPerRound", getNumber (_x >> "massPerRound")]
    ]);
} forEach ([_config >> "Magazines", "Mag", getNumber (_config >> "numMagazines")] call _readClass);
_heli setVariable ["bmkhs_magazines", _magazines];

//STORES
private _stores = [];
{
    _stores pushBack (createHashMapFromArray [
        ["match",        toLower getText (_x >> "match")],
        ["launcherMass", getNumber (_x >> "launcherMass")],
        ["massPerRound", getNumber (_x >> "massPerRound")],
        ["isTank",       getNumber (_x >> "isTank") > 0]
    ]);
} forEach ([_config >> "Stores", "Store", getNumber (_config >> "numStores")] call _readClass);
_heli setVariable ["bmkhs_stores", _stores];
