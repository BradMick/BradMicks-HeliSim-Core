/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_wingUpdate

Description:
    Applies the aerodynamic forces of every lifting surface the aircraft
    declares - wings, fins and the stabilator.

Parameters:
    _heli - The helicopter to get information from [Unit].

Returns:
    Nothing
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

private _numWings           = _heli getVariable "bmkhs_numWings";
private _wingIsStabilator   = _heli getVariable "bmkhs_wingIsStabilator";
private _wingPos            = _heli getVariable "bmkhs_wingPos";
private _wingPitch          = _heli getVariable "bmkhs_wingPitch";
private _wingRoll           = _heli getVariable "bmkhs_wingRoll";
private _wingSpan           = _heli getVariable "bmkhs_wingSpan";
private _wingChord          = _heli getVariable "bmkhs_wingChord";
private _wingSweep          = _heli getVariable "bmkhs_wingSweep";
private _wingTwist          = _heli getVariable "bmkhs_wingTwist";
private _wingTipWidthScalar = _heli getVariable "bmkhs_wingTipWidthScalar";
private _wingAirfoil        = _heli getVariable ["bmkhs_wingAirfoil", []];

for "_i" from 0 to (_numWings - 1) do {
    [ _heli
     ,_wingPos            select _i
     ,_wingPitch          select _i
     ,_wingRoll           select _i
     ,_wingSpan           select _i
     ,_wingChord          select _i
     ,_wingSweep          select _i
     ,_wingTwist          select _i
     ,_wingTipWidthScalar select _i
     ,(_wingIsStabilator  select _i) > 0
     ,_i
     ,[_heli, _wingAirfoil select _i, format ["wing %1", _i + 1]] call bmkhs_fnc_airfoilGet
     ] call bmkhs_fnc_wing;
};
