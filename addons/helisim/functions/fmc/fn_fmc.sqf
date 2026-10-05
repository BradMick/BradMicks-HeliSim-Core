params ["_heli"];
#include "\bmkhs_helisim\functions\core\core.hpp"

private _fmc = _heli getVariable "bmkhs_fmc";

//Each declared feature runs while its gate[] holds - the same form as a component gate. A
//closed gate is the feature switched off; an undeclared one does not exist.
private _open = {
    private _f = _fmc getOrDefault [_this, createHashMap];
    (count _f > 0) && {((_f get "gate") findIf {
        !(if (_x isEqualType []) then {
            ([_heli, _x select 0] call bmkhs_fnc_systemCircuit) >= (_x select 1)
        } else {
            _heli getVariable [_x, false]
        })
    }) < 0}
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

//Control mixing - mechanical mixes always apply; electronic ones carry their own gates
([_heli] call bmkhs_fnc_fmcControlMixing)
    params ["_mixPitchOut", "_mixRollOut", "_mixYawOut"];
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

//Flight director - an axis it holds replaces that axis's hold
private _fdOut = [_heli, _fmc getOrDefault ["FlightDirector", createHashMap], _fdOn] call bmkhs_fnc_fmcFlightDirector;
if ("coll"  in _fdOut) then { _altHoldCollOut     = _fdOut get "coll"  };
if ("pitch" in _fdOut) then { _attHoldCycPitchOut = _fdOut get "pitch" };
if ("roll"  in _fdOut) then { _attHoldCycRollOut  = _fdOut get "roll"  };
if ("yaw"   in _fdOut) then { _hdgHoldPedalYawOut = _fdOut get "yaw"   };

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

//Control mixing outputs
_heli setVariable ["bmkhs_mixPitchOut",                  _mixPitchOut];
_heli setVariable ["bmkhs_mixRollOut",                   _mixRollOut];
_heli setVariable ["bmkhs_mixYawOut",                    _mixYawOut];
//Flight Management Computer (FMC) outputs
_heli setVariable ["bmkhs_fmcAttHoldCycPitchOut",        _attHoldCycPitchOut];
_heli setVariable ["bmkhs_fmcAttHoldCycRollOut",         _attHoldCycRollOut];
_heli setVariable ["bmkhs_fmcHdgHoldPedalYawOut",        _hdgHoldPedalYawOut];
_heli setVariable ["bmkhs_fmcAltHoldCollOut",            _altHoldCollOut];
//Stability Augmentation System
_heli setVariable ["bmkhs_fmcSasPitchOut",               _SASPitchOutput];
_heli setVariable ["bmkhs_fmcSasRollOut",                _SASRollOutput];
_heli setVariable ["bmkhs_fmcSasYawOut",                 _SASYawOutput];
