/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_inputAutoTaxi

Description:
    Auto taxi - the keyboard's and the mouse's pitch on the wheels. On the wheels the key or the
    mouse is not the cyclic: it asks for a taxi speed and the cyclic is moved to hold it.

    Two regimes, on the highest engine torque, with a gap so the boundary cannot chatter:
        TAXI    - the forward key holds AUTOTAXI_SPD forward, the back key the same backward,
                  neither brings the aircraft to a stop. Parking brake set, the cyclic is neutral.
        TAKEOFF - neither key, the cyclic is neutral. The forward key holds the level-flight
                  attitude (autoAttLevelPitch), so a held key cannot nose the aircraft over.
                  The back key is aft cyclic.

    Aft cyclic is limited by the weight on the squat strut - it backs off as the gear unloads.
    The forward key held through lift-off in TAKEOFF is a rolling takeoff: the rate command takes
    the cyclic and blends the key from that hold into its rate by AUTOTAXI_BLEND_FT (CLIMB,
    bmkhs_fnc_inputWrite). Any other key held from the wheels stays here until it is released.

Parameters:
    _heli      - The helicopter [Object].
    _deltaTime - Frame time [Number].
    _key       - The pitch key or mouse travel, + forward [Number].

Returns:
    The cyclic, + forward [Number].

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli", "_deltaTime", "_key"];

private _pid    = _heli getVariable "bmkhs_pid_autoTaxi";
private _regime = _heli getVariable "bmkhs_autoTaxiRegime";
//Taking the cyclic, the hand starts from where the stick is
private _out    = [_heli getVariable "bmkhs_autoTaxiOut", _heli getVariable "bmkhs_cyclicFwdAft"] select !(_regime in ["taxi", "takeoff"]);

//Taxi or taking off
private _tq = selectMax (_heli getVariable "bmkhs_engPctTq");
if (_regime == "takeoff") then {
    if (_tq <= AUTOTAXI_TQ_TAXI) then { _regime = "taxi" };
} else {
    _regime = ["taxi", "takeoff"] select (_tq >= AUTOTAXI_TQ_TAKEOFF);
};
_heli setVariable ["bmkhs_autoTaxiRegime", _regime];

//Weight on the gear: the squat strut's compression as a fraction of its sag, 0 as the wheel lifts
private _compression = _heli getVariable "bmkhs_wheelCompression";
private _wow         = _heli getVariable "bmkhs_wheelWow";
private _weight      = 0.0;
{
    if (_wow < 0 || {_forEachIndex == _wow}) then {
        _weight = _weight max ((_compression select _forEachIndex) / (_x get "sag"));
    };
} forEach (_heli getVariable "bmkhs_wheels");
_weight = _weight min 1.0;

private _brakeCtl = (_heli getVariable "bmkhs_wheelSettings") get "brakeControl";
private _brake    = _brakeCtl isNotEqualTo "" && {_heli getVariable format ["bmkhs_%1On", _brakeCtl]};

//The key or mouse: forward, back or released
private _dir = 0.0;
if (abs _key >= RATECMD_DEADBAND) then { _dir = [1.0, -1.0] select (_key < 0.0) };

private _spd    = 0.0;
private _cyclic = 0.0;
if (_regime == "taxi" && !_brake) then {
    _spd = _dir * AUTOTAXI_SPD;
    //Released, the aircraft is stopped on the speed alone - nothing wound up to roll it back
    if (_dir == 0.0) then { [_pid] call bmkhs_fnc_pidReset };
    private _velFwd = (_heli getVariable "bmkhs_velModelSpaceNoWind") select 1;
    _cyclic = [[_pid, _deltaTime, _spd, _velFwd] call bmkhs_fnc_pidRun, -AUTOTAXI_CYCLIC_MAX, AUTOTAXI_CYCLIC_MAX] call BIS_fnc_clamp;
} else {
    [_pid] call bmkhs_fnc_pidReset;
};
if (_regime == "takeoff") then {
    if (_dir > 0.0) then {
        //The rate command's hold, on the level-flight attitude. + nose down is the cyclic's sense.
        private _pitch     = (_heli call BIS_fnc_getPitchBank) select 0;
        private _pitchRate = deg ((_heli getVariable "bmkhs_angVelModelSpace") select 0);
        private _rate      = RATECMD_HOLD_GAIN * (_pitch - (_heli getVariable "bmkhs_autoAttLevelPitch"));
        _cyclic = [((_heli getVariable "bmkhs_pid_rateCommandPitch") get "kp") * (_rate - _pitchRate), -1.0, 1.0] call BIS_fnc_clamp;
    };
    if (_dir < 0.0) then { _cyclic = -1.0 };
};

//A key is 0 or 1 - it is the hand that makes a movement of it. Aft backs off as the gear unloads.
_out = [_out, _cyclic, _deltaTime, AUTOTAXI_HAND_TIME] call bmkhs_fnc_utilSmooth;
_out = _out max (-_weight);

_heli setVariable ["bmkhs_autoTaxiSpd", _spd];
_heli setVariable ["bmkhs_autoTaxiOut", _out];

_out
