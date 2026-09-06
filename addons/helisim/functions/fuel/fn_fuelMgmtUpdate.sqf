/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fuelMgmtUpdate

Description:
    Updates fuel management state: CHECK sub-mode burn calculations.
    Called each tick from bmkhs_fnc_coreUpdate after fuelUpdate.

Parameters:
    _heli - The helicopter to update [Unit].

Returns:
    None

Author:
    FZA Development Team
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\fuel\fuel.hpp"
params ["_heli"];

private _checkRunning = _heli getVariable ["bmkhs_checkRunning", false];
if (!_checkRunning) exitWith {};

private _checkStartTime = _heli getVariable ["bmkhs_checkStartTime", 0];
if (_checkStartTime <= 0) exitWith {};

private _totalFuelMass  = _heli getVariable ["bmkhs_totFuelMass", 0];
private _elapsed        = CBA_missionTime - _checkStartTime;
private _startFuelMass  = _heli getVariable ["bmkhs_checkStartFuel", _totalFuelMass];
private _burnedKg       = _startFuelMass - _totalFuelMass;
private _checkMinutes   = _heli getVariable ["bmkhs_checkMinutes", 15];
private _targetSec      = (_checkMinutes max 0) * 60;
private _elapsedClamped = if (_targetSec > 0) then { _elapsed min _targetSec } else { _elapsed };

// Cumulative average burn rate over the full elapsed check time (lb/hr)
private _burnRate = if (_elapsedClamped > 0) then { (_burnedKg * KG_TO_LBS) / (_elapsedClamped / 3600) } else { 0 };

if (_targetSec > 0 && _elapsed >= _targetSec) then {
    private _totalLbs     = _totalFuelMass * KG_TO_LBS;
    private _burnoutHours = if (_burnRate > 0) then { _totalLbs / _burnRate } else { 0 };
    private _fnZulu = {
        params ["_dt"];
        private _h = floor (_dt % FUEL_HOURS_PER_DAY);
        format ["%1:%2L", _h, [floor ((_dt % FUEL_HOURS_PER_DAY - _h) * 60), 2] call CBA_fnc_formatNumber]
    };

    [_heli, "bmkhs_checkRunning", false]     call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "bmkhs_checkDone",     true]      call bmkhs_fnc_utilUpdateNetworkGlobal;

    //Core reports the fuel check finished; whether that warrants an advisory
    //depends on what the crew is looking at, which is the aircraft's business.
    if (!(_heli getVariable ["bmkhs_checkActivePlt", false]) &&
        !(_heli getVariable ["bmkhs_checkActiveCpg", false])) then {
        [_heli, "bmkhs_checkPendingAdvisory", true] call bmkhs_fnc_utilUpdateNetworkGlobal;
    };
    [_heli, "fuelCheckComplete"] call bmkhs_fnc_utilNotify;
    [_heli, "bmkhs_checkBurnRate", _burnRate] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "bmkhs_checkBurnoutZulu", [dayTime + _burnoutHours]        call _fnZulu] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "bmkhs_checkVFRZulu",    [dayTime + _burnoutHours - FUEL_CHECK_VFR_RESERVE_HR] call _fnZulu] call bmkhs_fnc_utilUpdateNetworkGlobal;
    [_heli, "bmkhs_checkIFRZulu",    [dayTime + _burnoutHours - FUEL_CHECK_IFR_RESERVE_HR] call _fnZulu] call bmkhs_fnc_utilUpdateNetworkGlobal;
};
