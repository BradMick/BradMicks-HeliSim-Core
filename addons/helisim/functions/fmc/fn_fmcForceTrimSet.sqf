params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

if (currentPilot _heli != player || !local _heli) exitWith {};

//Auto attitude writes force-trim every frame as a PID output, so trimming from stick position
//would stomp it. Same gates as fn_inputUpdate.
private _casual    = bmkhs_helisimRealismSetting != REALISTIC;
private _autoPitch = bmkhs_autoPitch && _casual;
private _autoRoll  = bmkhs_autoRoll  && _casual;

//Cyclic pitch trim
if (!_autoPitch) then {
    private _curCyclicFwdAft  = (_heli getVariable "bmkhs_cyclicFwdAft");
    private _prevCyclicFwdAft = _heli getVariable "bmkhs_forceTrimPosPitch";
    private _pitchTrimVal     = [_curCyclicFwdAft, _prevCyclicFwdAft] call bmkhs_fnc_inputGetInterp;
    if (bmkhs_springlessCyclic || bmkhs_keyboardStickyPitch) then {
        _heli setVariable ["bmkhs_forceTrimPosPitch", 0.0];
    } else {
        _heli setVariable ["bmkhs_forceTrimPosPitch", _pitchTrimVal, true];
    };
};
//Cyclic roll trim
if (!_autoRoll) then {
    private _curCyclicLeftRight  = (_heli getVariable "bmkhs_cyclicLeftRight");
    private _prevCyclicLeftRight = _heli getVariable "bmkhs_forceTrimPosRoll";
    private _rollTrimVal         = [_curCyclicLeftRight, _prevCyclicLeftRight] call bmkhs_fnc_inputGetInterp;
    if (bmkhs_springlessCyclic || bmkhs_keyboardStickyRoll) then {
        _heli setVariable ["bmkhs_forceTrimPosRoll",  0.0];
    } else {
        _heli setVariable ["bmkhs_forceTrimPosRoll", _rollTrimVal, true];
    };
};
//Pedal trim
if (!bmkhs_autoPedal) then {
    private _curPedalLeftRight  = (_heli getVariable "bmkhs_pedalLeftRight");
    private _prevPedalLeftRight = _heli getVariable "bmkhs_forceTrimPosYaw";
    private _pedalTrimVal       = [_curPedalLeftRight, _prevPedalLeftRight] call bmkhs_fnc_inputGetInterp;
    if (bmkhs_springlessPedals || bmkhs_keyboardStickyYaw) then {
        _heli setVariable ["bmkhs_forceTrimPosYaw", 0.0];
    } else {
        _heli setVariable ["bmkhs_forceTrimPosYaw", _pedalTrimVal, true];
    };
};
