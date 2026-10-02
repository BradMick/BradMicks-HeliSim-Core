/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engineFuelAvail

Description:
    Each engine's fuel pump: draws its flow from the tank the crossfeed selects and
    reports whether it has fuel. An armed fire handle shuts it off.

Parameters:
    _heli      - The helicopter [Object]
    _deltaTime - Frame time [Number]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_deltaTime"];
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\functions\engine\engine.hpp"

private _flow = _heli getVariable "bmkhs_engFuelFlow";

//No tanks modelled - always fuel.
if ((_heli getVariable "bmkhs_numFuelTanks") == 0) exitWith {
    [_heli, "bmkhs_engFuelAvail", _flow apply {true}] call bmkhs_fnc_utilUpdateNetworkGlobal;
};

private _state = _heli getVariable "bmkhs_engState";
private _since = _heli getVariable "bmkhs_engStarvedSince";
private _tanks = (_heli getVariable "bmkhs_crossfeedSources") getOrDefault [_heli getVariable "bmkhs_crossfeedMode", []];
private _ctrls = _heli getVariable "bmkhs_ctrlIndex";
private _dcOn  = _heli getVariable "bmkhs_dcBusOn";

private _avail    = [];
private _newSince = [];
{
    private _e    = _forEachIndex;
    private _tank = _tanks param [_e, ""];
    //An armed fire handle closes this engine's fuel while DC is up.
    private _handle = format ["eng%1FireHandle", _e + 1];
    private _shut   = _dcOn && {_handle in _ctrls} && {_heli getVariable (format ["bmkhs_%1On", _handle])};
    private _mass = if (_tank == "" || _shut) then {0} else {_heli getVariable (_tank + "Mass")};
    private _has  = _mass > EPSILON;

    if (_has && {(_state select _e) == "ON"}) then {
        _heli setVariable [_tank + "Mass", (_mass - (_x * _deltaTime)) max 0];
    };

    //Starved once the tank has been dry for the grace period.
    private _t = _since select _e;
    if (_has) then { _t = -1 } else { if (_t < 0) then { _t = CBA_missionTime } };
    _newSince pushBack _t;
    _avail pushBack (_t < 0 || {(CBA_missionTime - _t) < FUEL_STARVE_GRACE_SEC});
} forEach _flow;

_heli setVariable ["bmkhs_engStarvedSince", _newSince];
[_heli, "bmkhs_engFuelAvail", _avail] call bmkhs_fnc_utilUpdateNetworkGlobal;
