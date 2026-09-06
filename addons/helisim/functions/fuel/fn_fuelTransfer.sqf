/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fuelTransfer

Description:
    Moves fuel between the internal tanks: the XFER pump between the two mains,
    and any transfer cell gravity-feeding into them.

    The XFER mode names a DESTINATION main by position in _mains - "0" or "1",
    or whatever labels the aircraft maps to those - so nothing here means fore,
    aft, left or right. AUTO balances the two mains against the Table 2-6
    thresholds and needs bleed air (APU or a running engine).

    A transfer cell feeds both mains but is inhibited while any armed auxiliary
    tank still has fuel, so the aux tanks empty first.

    Mutates _fuelMass in place.

Parameters:
    _heli      - The helicopter [Object]
    _fuelMass  - Per-tank masses, mutated [Array]
    _fuelMax   - Per-tank capacities [Array]
    _fuelLow   - Per-tank low-level thresholds [Array]
    _fuelTanks - Fuel tank table, for the per-tank variable names [Array]
    _mains     - Indices of the tanks with role "main" [Array]
    _transfers - Indices of the tanks with role "xfer" [Array]
    _auxArmed  - True if any armed auxiliary tank still holds fuel [Bool]
    _deltaTime - Frame time [Number]

Returns:
    [_intercellActive, _intercellDir, _cellFlowing] - dir 1 = first main to
    second, 2 = second to first [Array]

Author:
    BradMick / FZA Development Team
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\fuel\fuel.hpp"
params ["_heli", "_fuelMass", "_fuelMax", "_fuelLow", "_fuelTanks", "_mains", "_transfers", "_auxArmed", "_deltaTime"];

//Two mains to balance between. A is the first, B the second; neither has any fore/aft or
//left/right meaning - the aircraft decides what its XFER labels map onto.
private _idxA = _mains param [0, -1];
private _idxB = _mains param [1, -1];
if (_idxA < 0 || _idxB < 0 || _idxA == _idxB) exitWith { [false, 0, false] };

private _xferStep = XFER_RATE_KGS * _deltaTime;
private _massA    = _fuelMass param [_idxA, 0];
private _massB    = _fuelMass param [_idxB, 0];
private _maxA     = _fuelMax  param [_idxA, 0];
private _maxB     = _fuelMax  param [_idxB, 0];

private _engState = _heli getVariable "bmkhs_engState";
private _eng1On   = (_engState select 0) == "ON";
private _eng2On   = (_engState select 1) == "ON";
private _airAvail = (_heli getVariable ["bmkhs_pneuAvail", false]) || _eng1On || _eng2On;

//The aircraft's XFER selection maps to the DESTINATION main. Anything that is not a
//destination or AUTO leaves the pump off.
private _xferMode = toUpper (_heli getVariable ["bmkhs_xferMode", "OFF"]);
private _xferDest = _heli getVariable ["bmkhs_xferDestinations", ["A", "B"]];

private _doAToB = false;
private _doBToA = false;

if (_xferMode == "AUTO") then {
    if (_airAvail) then {
        private _lowA = _massA < (_fuelLow param [_idxA, 0]);
        private _lowB = _massB < (_fuelLow param [_idxB, 0]);
        private _leadB = _massB - _massA;
        private _leadA = _massA - _massB;

        //A bigger imbalance is tolerated when the source tank is still well filled.
        private _bLeadEnough = _leadB > ([AUTO_SPLIT_50_KG, AUTO_SPLIT_100_KG] select (_massB > AUTO_500_KG));
        private _aLeadEnough = _leadA > ([AUTO_SPLIT_50_KG, AUTO_SPLIT_100_KG] select (_massA > AUTO_500_KG));

        //Topping up A needs only one engine running; drawing A down needs both, so a
        //single-engine failure cannot strand fuel away from the surviving side.
        _doBToA = (_eng1On || _eng2On)
               && {_massA < AUTO_FILL_THRESH_KG}
               && {!_lowB}
               && {_massB > (_fuelLow param [_idxA, 0])}
               && {_bLeadEnough}
               && {_leadB >= AUTO_SPLIT_STOP_KG}
               && {_massA < (_maxA - 0.1)};

        _doAToB = _eng1On && _eng2On
               && {_massB < AUTO_FILL_THRESH_KG}
               && {!_lowA}
               && {_massA > AUTO_MIN_SRC_KG}
               && {_aLeadEnough}
               && {_leadA >= AUTO_SPLIT_STOP_KG};
    };
} else {
    private _dest = _xferDest find _xferMode;
    if (_dest == 1) then { _doAToB = _massA > _xferStep && _massB < _maxB };
    if (_dest == 0) then { _doBToA = _massB > _xferStep && _massA < _maxA };
};

private _intercellActive = false;
private _intercellDir    = 0;

if (_doAToB) then {
    private _amt = _massA min _xferStep min (_maxB - _massB);
    if (_amt > 0) then {
        _massA = _massA - _amt;
        _massB = _massB + _amt;
        _intercellActive = true;
        _intercellDir    = 1;
    };
};
if (_doBToA) then {
    private _amt = _massB min _xferStep min (_maxA - _massA);
    if (_amt > 0) then {
        _massB = _massB - _amt;
        _massA = _massA + _amt;
        _intercellActive = true;
        _intercellDir    = 2;
    };
};

_fuelMass set [_idxA, _massA];
_fuelMass set [_idxB, _massB];

//Transfer cells gravity-feed the mains, but only once the aux tanks are done.
private _cellFlowing = false;
if (!_auxArmed) then {
    {
        private _cellIdx = _x;
        private _varName   = (_fuelTanks select _cellIdx) get "varName";
        private _switchOn  = _heli getVariable [_varName + "XferOn",    false];
        private _installed = _heli getVariable [_varName + "Installed", false];

        if (_switchOn && _installed) then {
            {
                private _dstIdx = _x;
                private _cell   = _fuelMass param [_cellIdx, 0];
                private _dst    = _fuelMass param [_dstIdx, 0];
                private _room   = (_fuelMax param [_dstIdx, 0]) - _dst;

                if (_cell > 0 && _room > 0) then {
                    private _flow = _xferStep min _cell min _room;
                    _fuelMass set [_cellIdx, _cell - _flow];
                    _fuelMass set [_dstIdx,  _dst  + _flow];
                    if (_flow > 0) then { _cellFlowing = true };
                };
            } forEach _mains;

            //Auto-shutoff once the cell is dry.
            if ((_fuelMass param [_cellIdx, 0]) <= 0) then {
                _heli setVariable [_varName + "XferOn", false, true];
            };
        };
    } forEach _transfers;
};

[_intercellActive, _intercellDir, _cellFlowing]
