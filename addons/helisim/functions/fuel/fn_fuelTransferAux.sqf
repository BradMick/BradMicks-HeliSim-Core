/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fuelTransferAux

Description:
    Transfers fuel from the auxiliary tanks into the internal tanks they feed,
    and zeroes any station whose tank is no longer fitted.

    Each aux tank declares the fuel tank it feeds, the aux tank that must be
    present for its pressurised air path, and the switch group that arms it.
    Tanks with no prerequisite run first, so an inner tank empties before the
    outer one behind it, and the outer feeds through the inner to reach the
    internal cells.

    Mutates _fuelMass and _auxMass in place.

Parameters:
    _heli      - The helicopter [Object]
    _fuelMass  - Per-tank internal masses, mutated [Array]
    _fuelMax   - Per-tank internal capacities [Array]
    _auxMass   - Per-tank auxiliary masses, mutated [Array]
    _auxTanks  - Auxiliary tank table [Array]
    _groupOn   - Group name to armed state [HashMap]
    _deltaTime - Frame time [Number]

Returns:
    [_auxPresent, _groupFlow] - presence per aux tank, and kg moved per group
    [Array]

Author:
    BradMick / FZA Development Team
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\fuel\fuel.hpp"
params ["_heli", "_fuelMass", "_fuelMax", "_auxMass", "_auxTanks", "_groupOn", "_deltaTime"];

private _xferStep       = XFER_RATE_KGS * _deltaTime;
private _pylonMagazines = getPylonMagazines _heli;
private _stations       = _heli getVariable ["bmkhs_stations", []];

//Presence per aux tank, from the pylons of the station it hangs on. A tank that is gone -
//jettisoned or never fitted - holds no fuel.
private _auxPresent = [];
{
    private _station = _x get "station";
    private _stn     = _stations param [_station - 1, createHashMap];
    private _pylons  = _stn getOrDefault ["pylons", []];
    private _present = _pylons findIf {
        ["auxTank", _pylonMagazines param [_x - 1, ""]] call BIS_fnc_inString
    } > -1;
    _auxPresent pushBack _present;
    if (!_present) then { _auxMass set [_forEachIndex, 0] };
} forEach _auxTanks;

private _groupFlow = createHashMap;

//Two passes: tanks with no prerequisite, then the ones gated behind another tank.
{
    private _wantDependent = _x;
    {
        private _dstIdx   = _x get "feedsIdx";
        private _requires = _x get "requires";
        private _group    = _x get "group";
        private _idx = _forEachIndex;
        if ((_requires >= 0) != _wantDependent) then { continue };
        if (_dstIdx < 0) then { continue };            //unresolved feedsTank, already logged
        private _src    = _auxMass  param [_idx,    0];
        private _dst    = _fuelMass param [_dstIdx, 0];
        private _room   = (_fuelMax param [_dstIdx, 0]) - _dst;
        private _armed  = _groupOn getOrDefault [_group, false];
        private _ready  = _requires < 0 || {_auxPresent param [_requires, false]};

        if ((_auxPresent param [_idx, false]) && _armed && _ready && _src > 0 && _room > 0) then {
            private _flow = _xferStep min _src min _room;
            _auxMass  set [_idx,    _src - _flow];
            _fuelMass set [_dstIdx, _dst + _flow];
            _groupFlow set [_group, (_groupFlow getOrDefault [_group, 0]) + _flow];
        };
    } forEach _auxTanks;
} forEach [false, true];

[_auxPresent, _groupFlow]
