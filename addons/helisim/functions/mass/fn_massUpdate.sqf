/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_massUpdate

Description:
    Totals the aircraft's mass and moments and applies the resulting gross
    weight and centre of mass.

    Every contributor is an indexed table loaded by fn_massVariables, so the
    airframe's seat, tank and station counts are config, not code.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

if (!local _heli) exitWith {};

private _fs0    = _heli getVariable "bmkhs_fsDatum";
private _fwdCg  = _heli getVariable "bmkhs_fwdCgLimit";
private _aftCg  = _heli getVariable "bmkhs_aftCgLimit";

private _curMass = 0;
private _latMom  = 0;
private _longMom = 0;

//Empty airframe. The config moment is measured about the datum, so it converts to
//model space here; the payload arms below are already in model space.
private _emptyMass = 0;
private _emptyMom  = 0;
if (_heli animationPhase "fcr_enable" == 1) then {
    _emptyMass = _heli getVariable "bmkhs_emptyMassFCR";
    _emptyMom  = (_emptyMass * _fs0) - (_heli getVariable "bmkhs_emptyMomFCR");
} else {
    _emptyMass = _heli getVariable "bmkhs_emptyMassNonFCR";
    _emptyMom  = (_emptyMass * _fs0) - (_heli getVariable "bmkhs_emptyMomNonFCR");
};
_curMass = _emptyMass;
_longMom = _emptyMom;

//Seats. A seat contributes only when someone is in it, matched against fullCrew by the
//identity the config declares, so the CG reflects who is actually aboard.
//Single-argument fullCrew returns ONLY occupied positions, so every entry here is a real
//occupant - do not switch to the [_heli, "", true] form without adding an isNull check.
//Entries are [unit, role, cargoIndex, turretPath, isPersonTurret]; role is "driver",
//"gunner", "commander", "Turret" or "cargo". Turret seats are matched on the PATH rather
//than the role string, since a second turret reports "Turret" where the first reports
//"gunner" and both are the same kind of seat.
private _crew = fullCrew _heli;
{
    private _arm        = _x get "arm";
    private _mass       = _x get "mass";
    private _role       = _x get "role";
    private _turret     = _x get "turret";
    private _cargoIndex = _x get "cargoIndex";

    private _occupied = _crew findIf {
        _x params ["", "_cRole", "_cCargo", "_cTurret"];
        private _cRoleL = toLower _cRole;
        switch (_role) do {
            case "cargo":  { _cRoleL == "cargo" && {_cCargo == _cargoIndex} };
            case "driver": { _cRoleL == "driver" };
            default        { _cRoleL != "cargo" && {_cTurret isEqualTo _turret} };
        };
    } > -1;

    if (_occupied) then {
        _curMass = _curMass + _mass;
        _latMom  = _latMom  + (_mass * (_arm select 0));
        _longMom = _longMom + (_mass * (_arm select 1));
    };
} forEach (_heli getVariable ["bmkhs_seats", []]);

//Internal fuel. Each tank carries its own arm, so a tank is a mass at a position.
//A removable tank that is not fitted contributes nothing. Auxiliary tanks are counted
//with their wing station instead.
{
    private _arm       = _x get "arm";
    private _removable = _x get "removable";
    private _varName   = _x get "varName";

    if (!_removable || {_heli getVariable [_varName + "Installed", false]}) then {
        private _mass = _heli getVariable [_varName + "Mass", 0.0];
        _curMass = _curMass + _mass;
        _latMom  = _latMom  + (_mass * (_arm select 0));
        _longMom = _longMom + (_mass * (_arm select 1));
    };
} forEach (_heli getVariable ["bmkhs_fuelTanks", []]);

//Internal magazines - rounds carried in the airframe rather than on a pylon.
private _magsAmmo = magazinesAmmo _heli;
{
    private _match        = _x get "match";
    private _arm          = _x get "arm";
    private _massPerRound = _x get "massPerRound";
    private _rounds = 0;
    {
        _x params ["_magName", "_magAmmo"];
        if ([_match, toLower str _magName] call BIS_fnc_inString) then {
            _rounds = _rounds + _magAmmo;
        };
    } forEach _magsAmmo;

    private _mass = _rounds * _massPerRound;
    _curMass = _curMass + _mass;
    _latMom  = _latMom  + (_mass * (_arm select 0));
    _longMom = _longMom + (_mass * (_arm select 1));
} forEach (_heli getVariable ["bmkhs_magazines", []]);

//Wing stations - launcher, remaining rounds, and external tank fuel.
{
    private _arm    = _x get "arm";
    private _pylons = _x get "pylons";
    private _stationNo = _forEachIndex + 1;
    private _mass = [_heli, _pylons, _stationNo] call bmkhs_fnc_massUpdateStation;

    _curMass = _curMass + _mass;
    _latMom  = _latMom  + (_mass * (_arm select 0));
    _longMom = _longMom + (_mass * (_arm select 1));
} forEach (_heli getVariable ["bmkhs_stations", []]);

private _curLongCG = _longMom / _curMass;
private _curLatCG  = _latMom  / _curMass;

//setCenterOfMass works in the engine's shifted frame, so the surveyed CG has boundingCenter
//removed before it goes in. See the debug readout below, which adds it back.
private _comDatum = boundingCenter _heli;
_heli setCenterOfMass [
    _curLatCG  - (_comDatum select 0),
    _curLongCG - (_comDatum select 1),
               - (_comDatum select 2)
];

_heli setMass _curMass;

_heli setVariable ["bmkhs_GWT", _curMass,   true];
_heli setVariable ["bmkhs_CG",  _curLongCG, true];

if (BMKHS_FM_DEBUG) then {
    private _vecX = [5.0, 0.0, 0.0];
    private _vecY = [0.0, 5.0, 0.0];
    private _vecZ = [0.0, 0.0, 5.0];

    private _heliCoM = getCenterOfMass _heli;

    [_heli, _heliCoM, _heliCoM vectorAdd _vecX, "red"]   call bmkhs_fnc_debugDrawLine;
    [_heli, _heliCoM, _heliCoM vectorAdd _vecY, "green"] call bmkhs_fnc_debugDrawLine;
    [_heli, _heliCoM, _heliCoM vectorAdd _vecZ, "blue"]  call bmkhs_fnc_debugDrawLine;

    [_heli, [0.0, _fs0   - (_comDatum select 1),-5], [0.0, _fs0   - (_comDatum select 1), 5], "green"] call bmkhs_fnc_debugDrawLine;
    [_heli, [0.0, _fwdCg - (_comDatum select 1),-5], [0.0, _fwdCg - (_comDatum select 1), 5], "red"]   call bmkhs_fnc_debugDrawLine;
    [_heli, [0.0, _aftCg - (_comDatum select 1),-5], [0.0, _aftCg - (_comDatum select 1), 5], "red"]   call bmkhs_fnc_debugDrawLine;
};
