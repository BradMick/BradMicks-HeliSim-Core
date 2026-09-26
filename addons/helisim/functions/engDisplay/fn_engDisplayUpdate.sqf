/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engDisplayUpdate

Description:
    Draws the engine readout - Ng, Np, TGT, Nr and torque.

    Shown only when the aircraft runs with useSystems = 0. An aircraft with the
    systems model has a cockpit display fed with this data; one without has
    nowhere for it to go, so Core shows it.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

if !(_heli getVariable ["bmkhs_initialised", false]) exitWith {};

private _display = uiNamespace getVariable ["bmkhs_engdisplay", displayNull];
private _wanted  = !(_heli getVariable ["bmkhs_useSystems", false])
                && {driver _heli == player || gunner _heli == player};

if (!_wanted) exitWith {
    if !(isNull _display) then {
        ("bmkhs_engdisplay" call BIS_fnc_rscLayer) cutText ["", "PLAIN", 0, false];
        uiNamespace setVariable ["bmkhs_engdisplay", displayNull];
    };
};

if (isNull _display) exitWith {
    ("bmkhs_engdisplay" call BIS_fnc_rscLayer) cutRsc ["bmkhs_engdisplay", "PLAIN", 0, false];
};

private _ctrl = _display displayCtrl 5303;
if (isNull _ctrl) exitWith {};

(_heli getVariable ["bmkhs_engPctNG", [0,0]]) params ["_ng1", "_ng2"];
(_heli getVariable ["bmkhs_engPctNP", [0,0]]) params ["_np1", "_np2"];
(_heli getVariable ["bmkhs_engPctTQ", [0,0]]) params ["_tq1", "_tq2"];
(_heli getVariable ["bmkhs_engTGT",   [0,0]]) params ["_tgt1", "_tgt2"];
private _nr = _heli getVariable ["bmkhs_rtrRPM", 0.0];

private _pad = {
    params ["_s", "_w"];
    _s = str _s;
    while {count _s < _w} do { _s = " " + _s };
    _s
};
private _pct = { [(_this * 100.0) toFixed 0, 6] call _pad };
private _deg = { [_this toFixed 0, 6] call _pad };

private _txt = "<t size='0.9' font='EtelkaMonospacePro'>";
_txt = _txt + "<t color='#88ff88'>" + ([" ", 10] call _pad) + ([1, 6] call _pad) + ([2, 6] call _pad) + "</t><br/>";
_txt = _txt + ([ "TORQUE %", 10] call _pad) + (_tq1  call _pct) + (_tq2  call _pct) + "<br/>";
_txt = _txt + ([ "NP %",     10] call _pad) + (_np1  call _pct) + (_np2  call _pct) + "<br/>";
_txt = _txt + ([ "NG %",     10] call _pad) + (_ng1  call _pct) + (_ng2  call _pct) + "<br/>";
_txt = _txt + ([ "TGT C",    10] call _pad) + (_tgt1 call _deg) + (_tgt2 call _deg) + "<br/>";
_txt = _txt + "<br/>" + ([ "NR %", 10] call _pad) + (_nr call _pct) + "<br/>";

_ctrl ctrlSetStructuredText parseText (_txt + "</t>");
