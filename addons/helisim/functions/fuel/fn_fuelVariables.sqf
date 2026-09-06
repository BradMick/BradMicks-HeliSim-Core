/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fuelVariables

Description:
    Loads the fuel configuration and initialises the fuel and fuel
    management state - tank quantities, capacities, valves and pumps.

Parameters:
    _heli   - The helicopter to get information from [Unit].
    _config - The aircraft's HeliSim config [Config].

Returns:
    ...

Examples:
    ...

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

//Tanks. Each one NAMES its own variables through variableName, so what a display reads is
//obvious from the config with no index arithmetic. Core owns the bmkhs_ prefix.
private _numFuelTanks = getNumber (_config >> "numFuelTanks");
private _fuelTanks    = [];
private _seenNames    = [];

for "_i" from 1 to _numFuelTanks do {
    private _t = (_config >> "FuelTanks") >> format ["FuelTank%1%2", ["0", ""] select (_i > 9), _i];

    private _varName = [_t, "FuelTank", _i, _seenNames] call bmkhs_fnc_fuelTankVarName;
    _seenNames pushBack _varName;

    private _capacity  = getNumber (_t >> "capacity");
    private _lowFuel   = getNumber (_t >> "lowFuelKg");
    private _removable = getNumber (_t >> "removable") > 0;

    //Hashmap, not a positional array: adding or removing a field cannot silently shift
    //what every reader sees. Keys are the config property names.
    _fuelTanks pushBack (createHashMapFromArray [
        ["arm",        getArray (_t >> "arm")],
        ["capacity",   _capacity],
        ["lowFuelKg",  _lowFuel],
        ["removable",  _removable],
        ["role",       toLower getText (_t >> "role")],
        ["varName",    _varName]
    ]);

    _heli setVariable [_varName + "Mass", 0.0];
    _heli setVariable [_varName + "Max",  _capacity];
    _heli setVariable [_varName + "Low",  _lowFuel];
    //Fixed tanks are always fitted. A removable one defaults to absent, but the aircraft may
    //have already declared it installed BEFORE coreConfig runs (fn_setup does exactly that
    //for the centre cell), so only seed the flag when it has not been set.
    private _installedVar = _varName + "Installed";
    if (!_removable) then {
        _heli setVariable [_installedVar, true];
    } else {
        _heli setVariable [_installedVar, _heli getVariable [_installedVar, false]];
    };
};
_heli setVariable ["bmkhs_numFuelTanks", _numFuelTanks];
_heli setVariable ["bmkhs_fuelTanks",    _fuelTanks];

//Resolve roles to indices once, so nothing downstream has to assume a tank NUMBER.
//An aircraft with four mains gets four entries in bmkhs_fuelMains.
private _mains     = [];
private _transfers = [];
{
    switch (_x get "role") do {
        case "main": { _mains     pushBack _forEachIndex };
        case "xfer": { _transfers pushBack _forEachIndex };
    };
} forEach _fuelTanks;
_heli setVariable ["bmkhs_fuelMains",     _mains];
//Cockpit XFER labels, in main order, so Core matches the selection without knowing what
//the labels mean.
_heli setVariable ["bmkhs_xferDestinations", getArray (_config >> "xferDestinations") apply {toUpper _x}];

_heli setVariable ["bmkhs_fuelTransfers", _transfers];

//Auxiliary tanks - fuel on a wing station. The arm comes from the station, not from here.
//feedsTank and requires name their targets, so the config reads as plumbing rather than
//as index arithmetic; both are resolved to indices below.
private _numAuxTanks = getNumber (_config >> "numAuxTanks");
private _auxTanks    = [];
private _auxNames    = [];
private _auxRefs     = [];

for "_i" from 1 to _numAuxTanks do {
    private _t = (_config >> "AuxTanks") >> format ["AuxTank%1%2", ["0", ""] select (_i > 9), _i];

    private _varName = [_t, "AuxTank", _i, _auxNames] call bmkhs_fnc_fuelTankVarName;
    _auxNames pushBack _varName;

    private _capacity = getNumber (_t >> "capacity");

    _auxTanks pushBack (createHashMapFromArray [
        ["station",   getNumber (_t >> "station")],
        ["capacity",  _capacity],
        ["feedsIdx",  -1],                      //resolved below
        ["requires",  -1],                      //resolved below
        ["group",     toUpper getText (_t >> "group")],
        ["varName",   _varName]
    ]);
    _auxRefs pushBack [getText (_t >> "feedsTank"), getText (_t >> "requires")];

    _heli setVariable [_varName + "Mass",       0.0];
    _heli setVariable [_varName + "Max",        _capacity];
    _heli setVariable [_varName + "EmptyArmed", false];
};

//Resolve the named references. A name that matches nothing is a config error, not a
//silently dead tank.
private _fuelNames = _fuelTanks apply {_x get "varName"};
{
    _x params ["_feedsName", "_requiresName"];
    private _tank = _auxTanks select _forEachIndex;

    private _feedsIdx = _fuelNames find ("bmkhs_" + _feedsName);
    if (_feedsIdx < 0) then {
        diag_log text format [
            "[BMKHS] FUEL CONFIG ERROR: AuxTank%1 feedsTank '%2' matches no fuel tank variableName. This tank will not transfer.",
            _forEachIndex + 1, _feedsName
        ];
    };
    _tank set ["feedsIdx", _feedsIdx];

    //An empty requires is legitimate - the tank has no prerequisite.
    private _requiresIdx = -1;
    if (_requiresName != "") then {
        _requiresIdx = _auxNames find ("bmkhs_" + _requiresName);
        if (_requiresIdx < 0) then {
            diag_log text format [
                "[BMKHS] FUEL CONFIG ERROR: AuxTank%1 requires '%2' matches no aux tank variableName. Treating it as having no prerequisite.",
                _forEachIndex + 1, _requiresName
            ];
        };
    };
    _tank set ["requires", _requiresIdx];
} forEach _auxRefs;

_heli setVariable ["bmkhs_numAuxTanks", _numAuxTanks];
_heli setVariable ["bmkhs_auxTanks",    _auxTanks];

//Crossfeed positions - which main each engine draws from in each valve position. Static
//aircraft data, resolved once here rather than rebuilt every frame. The valve starts in
//the first position declared.
private _crossfeed  = createHashMap;
private _defaultPos = "";
for "_i" from 1 to (getNumber (_config >> "numCrossfeedModes")) do {
    private _c   = (_config >> "CrossfeedModes") select (_i - 1);
    private _pos = toUpper getText (_c >> "position");
    if (_defaultPos == "") then { _defaultPos = _pos };
    _crossfeed set [_pos, getArray (_c >> "engSources")];
};
_heli setVariable ["bmkhs_crossfeedSources", _crossfeed];
_heli setVariable ["bmkhs_crossfeedMode",    _defaultPos];

//Which main tank the APU draws from. Independent of the crossfeed valve.
_heli setVariable ["bmkhs_apuFuelSource", getNumber (_config >> "apuFuelSource")];

// XFER pump selection: "OFF" | "AFT" | "FWD" | "AUTO"
_heli setVariable ["bmkhs_xferMode", "AUTO"];

// Boost pump state
_heli setVariable ["bmkhs_boostOn", false];

// Fuel system status flags
_heli setVariable ["bmkhs_intercellTransferActive", false];
_heli setVariable ["bmkhs_intercellTransferDir", 0];
_heli setVariable ["bmkhs_eng1FuelAvail", true];
_heli setVariable ["bmkhs_eng2FuelAvail", true];
_heli setVariable ["bmkhs_apuFuelAvail",  true];

// AUX tank on/off — L controls all left-side stations, R controls all right-side stations
// Default OFF — crew must arm before transfer begins
_heli setVariable ["bmkhs_lAuxOn", false];
_heli setVariable ["bmkhs_rAuxOn", false];

// CHECK sub-mode
_heli setVariable ["bmkhs_checkMinutes",   15];
_heli setVariable ["bmkhs_checkRunning",   false];
_heli setVariable ["bmkhs_checkDone",      false];
_heli setVariable ["bmkhs_checkStartTime", 0];
_heli setVariable ["bmkhs_checkStartFuel", 0];
_heli setVariable ["bmkhs_checkActivePlt", false];
_heli setVariable ["bmkhs_checkActiveCpg", false];
_heli setVariable ["bmkhs_checkPendingAdvisory", false];
_heli setVariable ["bmkhs_checkStartZulu",   ""];
_heli setVariable ["bmkhs_checkBurnoutZulu", ""];
_heli setVariable ["bmkhs_checkVFRZulu",     ""];
_heli setVariable ["bmkhs_checkIFRZulu",     ""];

// CHECK computed display values (lb/hr, seconds elapsed)
_heli setVariable ["bmkhs_checkElapsedSec", 0];
_heli setVariable ["bmkhs_checkBurnRate",   0];

//RUNTIME STATE - totals the update loop maintains.
_heli setVariable ["bmkhs_totFuelMass",    0.0];
_heli setVariable ["bmkhs_maxTotFuelMass", 0.0];
