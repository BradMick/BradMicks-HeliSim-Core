params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

if (currentPilot _heli != player || !local _heli) exitWith {};

//Keyboard auto-attitude (CASUAL) OWNS the pitch and roll trim channels: it writes force-trim
//every frame as the output of an attitude PID. Setting force-trim from raw stick position here
//would stomp that write and leave the PID fighting a trim offset it did not command, so both
//owned axes are skipped. Same single gate as fn_getInput - realistic pilots are untouched, and
//yaw is never owned by auto-attitude so it always trims normally.
private _autoAttOwns = bmkhs_helisimRealismSetting != REALISTIC;

//Cyclic pitch trim
if (!_autoAttOwns) then {
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
if (!_autoAttOwns) then {
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
private _curPedalLeftRight  = (_heli getVariable "bmkhs_pedalLeftRight");
private _prevPedalLeftRight = _heli getVariable "bmkhs_forceTrimPosYaw";
private _pedalTrimVal       = [_curPedalLeftRight, _prevPedalLeftRight] call bmkhs_fnc_inputGetInterp;
if (bmkhs_springlessPedals || bmkhs_keyboardStickyYaw) then {
    _heli setVariable ["bmkhs_forceTrimPosYaw", 0.0];
} else {
    _heli setVariable ["bmkhs_forceTrimPosYaw", _pedalTrimVal, true];
};
