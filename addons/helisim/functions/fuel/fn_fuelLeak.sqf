/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fuelLeak

Description:
    Drains damaged tanks. The hitpoint that damages a tank claims role
    "fuelTanks" at that tank's index, so a tank with no such hitpoint cannot
    leak. The rate ramps linearly from the onset threshold to full damage.

    Mutates _fuelMass in place.

Parameters:
    _heli      - The helicopter [Object]
    _fuelMass  - Per-tank masses, mutated [Array]
    _fuelTanks - Fuel tank table [Array]
    _deltaTime - Frame time [Number]

Returns:
    Nothing

Author:
    BradMick / FZA Development Team
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\fuel\fuel.hpp"
params ["_heli", "_fuelMass", "_fuelTanks", "_deltaTime"];

{
    private _removable = _x get "removable";
    private _varName   = _x get "varName";
    private _idx = _forEachIndex;
    private _m   = _fuelMass param [_idx, 0];

    if (_m <= 0) then { continue };

    private _fitted = !_removable || {_heli getVariable [_varName + "Installed", false]};
    if (!_fitted) then { continue };

    //The hitpoint that damages this tank declares itself with role "fuelTanks" and the
    //tank's index; a tank with no such hitpoint simply cannot leak.
    private _dmg = [_heli, "fuelTanks", _idx] call bmkhs_fnc_damageGet;
    if (_dmg > TANK_LEAK_START_DMG) then {
        private _frac = ((_dmg - TANK_LEAK_START_DMG) / (1 - TANK_LEAK_START_DMG)) min 1;
        _fuelMass set [_idx, _m - ((TANK_LEAK_MAX_RATE_KGS * _frac * _deltaTime) min _m)];
    };
} forEach _fuelTanks;
