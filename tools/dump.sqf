//Seats, weapons, pylons and animation sources of every variant of the HeliSim aircraft you are in.
//Debug console, LOCAL EXEC. Output to RPT and clipboard.
private _veh = vehicle player;
//The pack's airframe - the bmkhsBaseClass of whichever pack this aircraft belongs to.
private _base = "";
{
    private _b = getText (_x >> "bmkhsBaseClass");
    if (_veh isKindOf _b) exitWith { _base = _b };
} forEach ("isText (_x >> 'bmkhsBaseClass')" configClasses (configFile >> "CfgPatches"));
if (_base == "") exitWith { hint "Get in a HeliSim aircraft first" };

private _out = [];
private _log = { _out pushBack _this; diag_log text ("BMKHSDUMP| " + _this) };
private _magInfo = {
    private _c = configFile >> "CfgMagazines" >> _this;
    format ["%1 - %2 rds, mass %3, ammo %4", _this, getNumber (_c >> "count"), getNumber (_c >> "mass"), getText (_c >> "ammo")]
};

private _variants = format ["(configName _x) isKindOf '%1' && {getNumber (_x >> 'scope') == 2}", _base] configClasses (configFile >> "CfgVehicles");
format ["%1 - %2 variants", _base, count _variants] call _log;

{
    private _v = _x;
    "" call _log;
    format ["=== %1 - %2 (parent %3)", configName _v, getText (_v >> "displayName"), configName inheritsFrom _v] call _log;
    format ["  driver: weapons %1", getArray (_v >> "weapons")] call _log;
    { ("  driver mag: " + (_x call _magInfo)) call _log } forEach getArray (_v >> "magazines");
    format ["  transportSoldier (cargo seats): %1", getNumber (_v >> "transportSoldier")] call _log;

    private _turrets = [];
    private _walk = {
        params ["_cfg", "_path"];
        {
            private _p = _path + [_forEachIndex];
            _turrets pushBack [_p, _x];
            [_x >> "Turrets", _p] call _walk;
        } forEach ("true" configClasses _cfg);
    };
    [_v >> "Turrets", []] call _walk;
    {
        _x params ["_p", "_t"];
        format ["  turret %1 %2: gunnerName '%3', primaryGunner %4, primaryObserver %5, isCopilot %6, personTurret %7",
            _p, configName _t, getText (_t >> "gunnerName"), getNumber (_t >> "primaryGunner"),
            getNumber (_t >> "primaryObserver"), getNumber (_t >> "isCopilot"), getNumber (_t >> "isPersonTurret")] call _log;
        format ["    weapons %1", getArray (_t >> "weapons")] call _log;
        { ("    mag: " + (_x call _magInfo)) call _log } forEach getArray (_t >> "magazines");
    } forEach _turrets;

    private _pyl = _v >> "Components" >> "TransportPylonsComponent";
    {
        format ["  pylon %1 %2: attachment '%3', turret %4, maxweight %5, hardpoints %6",
            _forEachIndex + 1, configName _x, getText (_x >> "attachment"), getArray (_x >> "turret"),
            getNumber (_x >> "maxweight"), getArray (_x >> "hardpoints")] call _log;
        if (getText (_x >> "attachment") != "") then { ("    default: " + (getText (_x >> "attachment") call _magInfo)) call _log };
    } forEach ("true" configClasses (_pyl >> "Pylons"));
    {
        format ["  pylon preset %1: %2", configName _x, getArray (_x >> "attachment")] call _log;
    } forEach ("true" configClasses (_pyl >> "Presets"));

    {
        format ["  anim source %1: source '%2', initPhase %3", configName _x, getText (_x >> "source"), getNumber (_x >> "initPhase")] call _log;
    } forEach ("true" configClasses (_v >> "AnimationSources"));
} forEach _variants;

"" call _log;
format ["=== LIVE %1", typeOf _veh] call _log;
{
    _x params ["", "_role", "_cargoIndex", "_turretPath", "_personTurret"];
    format ["  seat: role %1, cargoIndex %2, turret %3, personTurret %4", _role, _cargoIndex, _turretPath, _personTurret] call _log;
} forEach fullCrew [_veh, "", true];
{ format ["  pylon: %1", _x] call _log } forEach getAllPylonsInfo _veh;
{ ("  loaded: " + (_x select 0) + " x" + str (_x select 1)) call _log } forEach magazinesAmmo _veh;
format ["  mass %1, centre of mass %2, boundingCenter %3", getMass _veh, getCenterOfMass _veh, boundingCenter _veh] call _log;

copyToClipboard (_out joinString endl);
hint format ["%1 dump: %2 variants, %3 lines - in the RPT and on the clipboard", _base, count _variants, count _out];
