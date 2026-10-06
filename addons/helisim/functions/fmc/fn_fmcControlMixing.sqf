/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcControlMixing

Description:
    Sums the aircraft's declared control mixes (ControlMixing) per rotor axis.

    A mix adds control travel to one axis from one control's position, through its
    table. A gated mix (electronic - an FCC, say) applies only while every gate holds;
    an ungated one is mechanical and always applies. An optional airspeed table scales
    it. An aircraft that declares no mixes gets none.

    A "pedal" source is the tail rotor's pitch command: the pilot's pedal and trim plus
    what the collective-sourced yaw mixes add - the mixing unit sees tail rotor pitch, not
    the pedals. So collective mixes are summed first, then pedal mixes read that.

    Mixing is a REALISTIC feature - outside it there is none.

Parameters:
    _heli - The helicopter [Object]
    _coll - The collective the rotor gets, pilot's and holds', 0..1 [Number]

Returns:
    Added travel, [pitch (+ fwd), roll (+ left), yaw (+ right pedal)] [Array]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli", "_coll"];

private _out = [0.0, 0.0, 0.0];
if (bmkhs_helisimRealismSetting != REALISTIC) exitWith {_out};

private _pedal = [_heli getVariable "bmkhs_pedalLeftRight", _heli getVariable "bmkhs_forceTrimPosYaw"] call bmkhs_fnc_inputGetInterp;
private _kts   = (_heli getVariable "bmkhs_vel2D") * MPS_TO_KNOTS;   //the airspeed[] table is in knots

private _apply = {
    params ["_mix", "_src"];
    //Gates in component form: a variable name, or {circuit, threshold} read live
    private _open = ((_mix get "gate") findIf {
        !(if (_x isEqualType []) then {
            ([_heli, _x select 0] call bmkhs_fnc_systemCircuit) >= (_x select 1)
        } else {
            _heli getVariable [_x, false]
        })
    }) < 0;

    if (_open) then {
        private _add = ([_mix get "table", _src] call bmkhs_fnc_mathLinearInterp) select 1;
        private _spd = _mix get "airspeed";
        if (_spd isNotEqualTo []) then {
            _add = _add * (([_spd, _kts] call bmkhs_fnc_mathLinearInterp) select 1);
        };
        private _axis = _mix get "target";
        _out set [_axis, (_out select _axis) + _add];
    };
};

private _mixes = _heli getVariable "bmkhs_mixes";
{ [_x, _coll] call _apply; } forEach (_mixes select {(_x get "source") == "collective"});

//Tail rotor pitch command - pedal plus the collective-sourced yaw mixes
private _trCommand = [_pedal + (_out select 2), -1.0, 1.0] call BIS_fnc_clamp;
{ [_x, _trCommand] call _apply; } forEach (_mixes select {(_x get "source") == "pedal"});

_out
