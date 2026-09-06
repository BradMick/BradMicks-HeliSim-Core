#include "\bmkhs_helisim\functions\rotor\rotor.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

private _numRotor      = _heli getVariable "bmkhs_numRotors";
private _pivot         = _heli getVariable "bmkhs_rotorPivot";
private _rot           = _heli getVariable "bmkhs_rotorRotation";
private _type          = _heli getVariable "bmkhs_rotorType";
private _dir           = _heli getVariable "bmkhs_rotorDirection";
private _numBlades     = _heli getVariable "bmkhs_rotorNumBlades";
private _numElements   = _heli getVariable "bmkhs_rotorNumElements";
private _mastLength    = _heli getVariable "bmkhs_rotorMastLength";
private _gearRatio     = _heli getVariable "bmkhs_rotorGearRatioArr";
private _flapTimeConst = _heli getVariable "bmkhs_rotorFlapTimeConst";
//Dynamic-inflow smoothing rate and damage thresholds are how the MODEL behaves, not what
//the aircraft is, so they stay in Core.
private _inflowAlpha   = [0.05, 0.01];
private _delta3        = _heli getVariable "bmkhs_rotorDelta3";
//Each rotor names its section; resolve to tables once rather than per blade element.
private _rotorAirfoil  = _heli getVariable ["bmkhs_rotorAirfoil", []];
private _airfoilTable  = [];
{
    _airfoilTable pushBack ([_heli, _x, format ["rotor %1", _forEachIndex + 1]] call bmkhs_fnc_airfoilGet);
} forEach _rotorAirfoil;
private _bladeCutout   = _heli getVariable "bmkhs_rotorBladeCutout";
private _bladeLength   = _heli getVariable "bmkhs_rotorBladeLength";
private _bladeChord    = _heli getVariable "bmkhs_rotorBladeChordArr";
private _bladeTwist    = _heli getVariable "bmkhs_rotorBladeTwist";
private _bladeMass     = _heli getVariable "bmkhs_rotorBladeMassArr";

private _pitchMin      = _heli getVariable "bmkhs_rotorPitchMin";
private _pitchMid      = _heli getVariable "bmkhs_rotorPitchMid";
private _pitchMax      = _heli getVariable "bmkhs_rotorPitchMax";
private _rollMin       = _heli getVariable "bmkhs_rotorRollMin";
private _rollMid       = _heli getVariable "bmkhs_rotorRollMid";
private _rollMax       = _heli getVariable "bmkhs_rotorRollMax";
private _collMin       = _heli getVariable "bmkhs_rotorCollMin";
private _collMid       = _heli getVariable "bmkhs_rotorCollMid";
private _collMax       = _heli getVariable "bmkhs_rotorCollMax";
private _animSource    = _heli getVariable "bmkhs_rotorAnimSource";
private _hitPoint      = _heli getVariable "bmkhs_rotorHitPoint";
private _dmgThreshold  = [ 0.99
                         , 0.85];

// Debug: draw CG position as a sphere with crosshair lines
private _cgPos = getCenterOfMass _heli;
private _cgR   = 5.0;
[_heli, 16, _cgPos, _cgR, 0, "red"]   call bmkhs_fnc_debugDrawCircle;
[_heli, 16, _cgPos, _cgR, 1, "red"]   call bmkhs_fnc_debugDrawCircle;
[_heli, 16, _cgPos, _cgR, 2, "red"]   call bmkhs_fnc_debugDrawCircle;
[_heli, _cgPos vectorAdd [-_cgR, 0, 0], _cgPos vectorAdd [_cgR, 0, 0], "white"] call bmkhs_fnc_debugDrawLine;
[_heli, _cgPos vectorAdd [0, -_cgR, 0], _cgPos vectorAdd [0, _cgR, 0], "white"] call bmkhs_fnc_debugDrawLine;
[_heli, _cgPos vectorAdd [0, 0, -_cgR], _cgPos vectorAdd [0, 0, _cgR], "white"] call bmkhs_fnc_debugDrawLine;

for "_rotorIndex" from 0 to (_numRotor - 1) do {
    [ _heli
    , _rotorIndex
    , _pivot         select _rotorIndex
    , _rot           select _rotorIndex
    , _type          select _rotorIndex
    , _dir           select _rotorIndex
    , _numBlades     select _rotorIndex
    , _numElements   select _rotorIndex
    , _mastLength    select _rotorIndex
    , _gearRatio     select _rotorIndex
    , _flapTimeConst select _rotorIndex
    , _inflowAlpha   select _rotorIndex
    , _delta3        select _rotorIndex
    , _airfoilTable  select _rotorIndex
    , _bladeCutout   select _rotorIndex
    , _bladeLength   select _rotorIndex
    , _bladeChord    select _rotorIndex
    , _bladeTwist    select _rotorIndex
    , _bladeMass     select _rotorIndex
    , _pitchMin      select _rotorIndex
    , _pitchMid      select _rotorIndex
    , _pitchMax      select _rotorIndex
    , _rollMin       select _rotorIndex
    , _rollMid       select _rotorIndex
    , _rollMax       select _rotorIndex
    , _collMin       select _rotorIndex
    , _collMid       select _rotorIndex
    , _collMax       select _rotorIndex
    , _animSource    select _rotorIndex
    , _hitPoint      select _rotorIndex
    , _dmgThreshold  select _rotorIndex
    ] call bmkhs_fnc_rotor;
};
