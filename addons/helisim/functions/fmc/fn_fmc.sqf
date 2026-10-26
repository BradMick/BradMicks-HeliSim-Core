params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"
#include "\bmkhs_helisim\fmc.hpp"

private _fmc = _heli getVariable "bmkhs_fmc";

//Each declared feature runs while its gate[] holds - the same form as a component gate. A
//closed gate is the feature switched off; an undeclared one does not exist.
private _open = {
    private _f = _fmc getOrDefault [_this, createHashMap];
    (count _f > 0) && {[_heli, _f get "gate"] call bmkhs_fnc_utilGateOpen}
};
private _sasOn = "Sas"          call _open;
private _attOn = "AttitudeHold" call _open;
private _altOn = "AltitudeHold" call _open;
private _hdgOn = "HeadingHold"  call _open;
private _fdOn  = "FlightDirector" call _open;
[_heli, "bmkhs_fmcSasAvail",     _sasOn] call bmkhs_fnc_utilUpdateNetworkGlobal;
[_heli, "bmkhs_fmcAttHoldAvail", _attOn] call bmkhs_fnc_utilUpdateNetworkGlobal;
[_heli, "bmkhs_fmcAltHoldAvail", _altOn] call bmkhs_fnc_utilUpdateNetworkGlobal;
[_heli, "bmkhs_fmcHdgHoldAvail", _hdgOn] call bmkhs_fnc_utilUpdateNetworkGlobal;
[_heli, "bmkhs_fmcFdAvail",      _fdOn]  call bmkhs_fnc_utilUpdateNetworkGlobal;

//Attitude Hold
([_heli, _fmc getOrDefault ["AttitudeHold", createHashMap], _attOn] call bmkhs_fnc_fmcAttitudeHold)
    params ["_attHoldCycPitchOut", "_attHoldCycRollOut"];
//Altitude Hold
private _altHoldCollOut     = [_heli, _fmc getOrDefault ["AltitudeHold", createHashMap], _altOn] call bmkhs_fnc_fmcAltitudeHold;
//Heading Hold
private _hdgHoldPedalYawOut = [_heli, _fmc getOrDefault ["HeadingHold", createHashMap], _hdgOn] call bmkhs_fnc_fmcHeadingHold;
//Stability Augmentation System (SAS)
([_heli, _fmc getOrDefault ["Sas", createHashMap], _sasOn] call bmkhs_fnc_fmcSas)
    params ["_SASPitchOutput", "_SASRollOutput", "_SASYawOutput"];

//Flight director - an axis it holds replaces that axis's hold, blended in and out
private _fdOut = [_heli, _fmc getOrDefault ["FlightDirector", createHashMap], _fdOn] call bmkhs_fnc_fmcFlightDirector;
if ("coll"  in _fdOut) then { _altHoldCollOut     = _fdOut get "coll"  };
private _blend = _heli getVariable ["bmkhs_fmcFdBlend", createHashMap];
private _step  = (_heli getVariable "bmkhs_deltaTime") / FMC_FD_HANDOFF_SEC;
private _axes  = [["pitch", _attHoldCycPitchOut], ["roll", _attHoldCycRollOut], ["yaw", _hdgHoldPedalYawOut]] apply {
    _x params ["_axis", "_hold"];
    (_blend getOrDefault [_axis, [0, 0]]) params ["_w", "_last"];
    if (_axis in _fdOut) then { _last = _fdOut get _axis; _w = (_w + _step) min 1 } else { _w = (_w - _step) max 0 };
    _blend set [_axis, [_w, _last]];
    (_w * _last) + ((1 - _w) * _hold)
};
_heli setVariable ["bmkhs_fmcFdBlend", _blend];
_attHoldCycPitchOut = _axes select 0;
_attHoldCycRollOut  = _axes select 1;
_hdgHoldPedalYawOut = _axes select 2;

if (bmkhs_springlessPedals || bmkhs_autoPedal) then {
    _hdgHoldPedalYawOut = 0.0;
};

if (!(_heli getVariable "bmkhs_fmcPitchOn")) then {
    _attHoldCycPitchOut = 0.0;
    _SASPitchOutput     = 0.0;
};

if (!(_heli getVariable "bmkhs_fmcRollOn")) then {
    _attHoldCycRollOut = 0.0;
    _SASRollOutput     = 0.0;
};

if (!(_heli getVariable "bmkhs_fmcYawOn")) then {
    _hdgHoldPedalYawOut = 0.0;
    _SASYawOutput       = 0.0;
};

if (!(_heli getVariable "bmkhs_fmcCollOn")) then {
    _altHoldCollOut = 0.0;
};

//A flight director vertical mode drives the collective lever itself, from where the lever was
//when the mode coupled - so the lever is always where the collective is, and whoever takes it
//back takes it from there (fn_inputUpdate). The altitude hold still adds on top of the lever.
private _fd = _fmc getOrDefault ["FlightDirector", createHashMap];
if ("coll" in _fdOut && {_heli getVariable "bmkhs_fmcCollOn"}) then {
    private _base = _fd getOrDefault ["collBase", _heli getVariable "bmkhs_collectiveOutput"];
    _fd set ["collBase", _base];
    _heli setVariable ["bmkhs_collectiveOutput", ((_base + _altHoldCollOut) max 0) min 1];
    _altHoldCollOut = 0.0;
} else {
    _fd deleteAt "collBase";
};

//Control mixing - mechanical mixes always apply; electronic ones carry their own gates. It
//sees the collective the rotor gets, the pilot's and the coupler's.
([_heli, (((_heli getVariable "bmkhs_collectiveOutput") + _altHoldCollOut) max 0) min 1] call bmkhs_fnc_fmcControlMixing)
    params ["_mixPitchOut", "_mixRollOut", "_mixYawOut", "_mixCollOut"];

//Control mixing outputs
_heli setVariable ["bmkhs_mixPitchOut",                  _mixPitchOut];
_heli setVariable ["bmkhs_mixRollOut",                   _mixRollOut];
_heli setVariable ["bmkhs_mixYawOut",                    _mixYawOut];
_heli setVariable ["bmkhs_mixCollOut",                   _mixCollOut];
//Flight Management Computer (FMC) outputs
_heli setVariable ["bmkhs_fmcAttHoldCycPitchOut",        _attHoldCycPitchOut];
_heli setVariable ["bmkhs_fmcAttHoldCycRollOut",         _attHoldCycRollOut];
_heli setVariable ["bmkhs_fmcHdgHoldPedalYawOut",        _hdgHoldPedalYawOut];
_heli setVariable ["bmkhs_fmcAltHoldCollOut",            _altHoldCollOut];
//Stability Augmentation System
_heli setVariable ["bmkhs_fmcSasPitchOut",               _SASPitchOutput];
_heli setVariable ["bmkhs_fmcSasRollOut",                _SASRollOutput];
_heli setVariable ["bmkhs_fmcSasYawOut",                 _SASYawOutput];
