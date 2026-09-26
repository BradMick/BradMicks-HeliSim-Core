/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engDisplayUpdate

Description:
    Draws the engine readout - torque, Np, Nr, TGT, Ng, oil pressure and the
    active rating, with warnings, cautions and advisories beneath.

    Always shown with useSystems = 0, where nothing else is fed this data, and
    optionally with the CBA debug setting so it can be compared against an
    aircraft's own cockpit display.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];
#include "\bmkhs_helisim\functions\engDisplay\engDisplay.hpp"
#include "\bmkhs_helisim\functions\systems\systems.hpp"

if !(_heli getVariable ["bmkhs_initialised", false]) exitWith {};

private _display = uiNamespace getVariable ["bmkhs_engdisplay", displayNull];
private _wanted  = (!(_heli getVariable ["bmkhs_useSystems", false]) || bmkhs_engDisplay)
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

private _bgPos = ctrlPosition (_display displayCtrl 5301);
if (count _bgPos < 4) exitWith {};
_bgPos params ["_x0", "_y0", "_W", "_H"];
if (_W < 0.001) exitWith {};

//The new model publishes gt* while it runs beside the old one; after the Phase 3b
//rename these are simply the real names and the fallback is what reads.
private _fetch = {
    params ["_gt", "_old", "_default"];
    private _v = _heli getVariable [_gt, nil];
    if (isNil "_v") then { _v = _heli getVariable [_old, _default] };
    _v
};

private _ng  = ["bmkhs_gtEngPctNg",  "bmkhs_engPctNG",  []] call _fetch;
private _np  = ["bmkhs_gtEngPctNp",  "bmkhs_engPctNP",  []] call _fetch;
private _tq  = ["bmkhs_gtEngPctTq",  "bmkhs_engPctTQ",  []] call _fetch;
private _tgt = ["bmkhs_gtEngTgt",    "bmkhs_engTGT",    []] call _fetch;
private _oil = ["bmkhs_gtEngOilPsi", "bmkhs_engOilPSI", []] call _fetch;
private _rtg = _heli getVariable ["bmkhs_engRatingName", []];

private _n = _heli getVariable ["bmkhs_numEngines", count _tq];
_n = (_n min ED_MAX_ENG) max 1;

private _nr = _heli getVariable ["bmkhs_rtrRPM", 0.0];

//Scale ends and band boundaries. Phase 2 moves these onto the declared PowerRatings;
//until then they come from the config the old model already reads.
private _tgtDe  = _heli getVariable ["bmkhs_engMaxTGT_DE", 867];
private _tgtSe  = _heli getVariable ["bmkhs_engMaxTGT_SE", 896];
private _npOvsp = _heli getVariable ["bmkhs_engOvrspdNP",  1.196];
private _cfg    = configOf _heli >> "BMKHS_HeliSim";
private _tqMax  = getNumber (_cfg >> "engMaxTQ");
if (_tqMax <= 0) then { _tqMax = 1.5 };

//Full scale sits above the top limit so the red band has somewhere to be drawn.
private _tqFs  = _tqMax;
private _tgtFs = _tgtSe * 1.08;
private _npFs  = _npOvsp * 1.05;

// ── Geometry ─────────────────────────────────────────────────────────────────
private _pad   = _W * 0.030;
private _dragH = _H * 0.020;          //grab strip, unlabelled
private _inner = _W - (_pad * 2);

//Three groups: torque (N tapes), tachs (N + 1 tapes), TGT (N tapes).
private _nTach  = _n + 1;
private _nTapes = (_n * 2) + _nTach;
private _gapG   = _inner * 0.045;
private _tapeW  = ((_inner - (_gapG * 2)) / _nTapes) * 0.68;
private _gapT   = (((_inner - (_gapG * 2)) - (_tapeW * _nTapes)) / ((_nTapes - 1) max 1)) max 0;

//Text rows take a fixed share of the panel, so everything left over is tape.
private _lblH  = _H * 0.040;
private _numH  = _H * 0.042;
private _rowH  = _H * 0.040;
private _annH  = _H * 0.110;

private _yLbl  = _y0 + _dragH + (_H * 0.010);
private _yEng  = _yLbl + _lblH;
private _yTape = _yEng + _lblH;
//Two label rows at the top (group + engine number), one more above the digital rows.
private _tapeH = _H - (_dragH + (_H * 0.010) + (_lblH * 3) + _numH + (_rowH * 3) + _annH + (_H * 0.030));
if (_tapeH < 0.01) then { _tapeH = _H * 0.30 };
private _yNum  = _yTape + _tapeH;
private _yRows = _yNum + _numH + (_H * 0.010) + _lblH;
private _yAnn  = _yRows + (_rowH * 3) + (_H * 0.008);

//Left edge of each group, walked across the panel.
private _xTq   = _x0 + _pad;
private _xTach = _xTq + (_n * _tapeW) + ((_n - 1) * _gapT) + _gapG;
private _xTgt  = _xTach + (_nTach * _tapeW) + ((_nTach - 1) * _gapT) + _gapG;

private _tapeX = { params ["_base", "_i"]; _base + (_i * (_tapeW + _gapT)) };

//A tape: frame, fill bottom-up, and the limit ticks over the top of it.
private _drawTape = {
    params ["_fIdc", "_lIdc", "_tx", "_val", "_fs", "_amber", "_red", "_aIdc", "_rIdc"];

    private _f = _display displayCtrl _fIdc;
    _f ctrlSetPosition [_tx, _yTape, _tapeW, _tapeH];
    _f ctrlCommit 0;
    _f ctrlShow true;

    //Limit ticks - a line across the tape at each threshold, not a filled band.
    private _tickH = (_tapeH * 0.012) max 0.0015;
    {
        _x params ["_idc", "_at"];
        private _c = _display displayCtrl _idc;
        if (!isNull _c) then {
            if (_idc > 0 && {_fs > 0} && {_at > 0} && {_at < _fs}) then {
                _c ctrlSetPosition [_tx, _yTape + (_tapeH * (1 - (_at / _fs))) - (_tickH / 2),
                                    _tapeW, _tickH];
                _c ctrlCommit 0;
                _c ctrlShow true;
            } else {
                _c ctrlShow false;
            };
        };
    } forEach [[_aIdc, _amber], [_rIdc, _red]];

    private _frac = ((_val / _fs) max 0) min 1;
    private _fill = _display displayCtrl _lIdc;
    _fill ctrlSetPosition [_tx, _yTape + (_tapeH * (1 - _frac)), _tapeW, _tapeH * _frac];
    _fill ctrlSetBackgroundColor (switch (true) do {
        case (_val >= _red):   { [0.90, 0.15, 0.15, 0.95] };
        case (_val >= _amber): { [0.95, 0.75, 0.10, 0.95] };
        default                { [0.20, 0.90, 0.20, 0.95] };
    });
    _fill ctrlCommit 0;
    _fill ctrlShow true;
};

private _setText = {
    params ["_idc", "_txt", "_x", "_y", "_w", "_h", ["_col", [0.80, 1.00, 0.80, 1.00]]];
    private _c = _display displayCtrl _idc;
    if (isNull _c) exitWith {};
    _c ctrlSetPosition [_x, _y, _w, _h];
    _c ctrlSetText _txt;
    _c ctrlSetTextColor _col;
    _c ctrlCommit 0;
    _c ctrlShow true;
};

private _hide = { { private _c = _display displayCtrl _x; if !(isNull _c) then { _c ctrlShow false } } forEach _this };

// ── Tapes ────────────────────────────────────────────────────────────────────
for "_i" from 0 to (ED_MAX_ENG - 1) do {
    if (_i < _n) then {
        private _tqV  = [_tq,  _i, 0.0] call BIS_fnc_param;
        private _npV  = [_np,  _i, 0.0] call BIS_fnc_param;
        private _tgV  = [_tgt, _i, 0.0] call BIS_fnc_param;

        private _xT = [_xTq, _i] call _tapeX;
        [5310 + _i, 5320 + _i, _xT, _tqV, _tqFs, 1.0, 1.29, 5440 + _i, 5450 + _i] call _drawTape;
        [5330 + _i, (_tqV * 100) toFixed 0, _xT, _yNum, _tapeW, _numH,
            switch (true) do {
                case (_tqV >= 1.29): { [1.00, 0.35, 0.35, 1.00] };
                case (_tqV >= 1.00): { [1.00, 0.85, 0.30, 1.00] };
                default               { [0.80, 1.00, 0.80, 1.00] };
            }] call _setText;
        [5400 + _i, str (_i + 1), _xT, _yEng, _tapeW, _lblH] call _setText;

        //Np tape - engine 0 left of Nr, the rest to its right, so Nr stays centred.
        private _slot = [_i, _i + 1] select (_i >= (_nTach / 2) - 0.5);
        private _xN = [_xTach, _slot] call _tapeX;
        [5340 + _i, 5350 + _i, _xN, _npV, _npFs, 1.05, _npOvsp, 5490 + _i, 5500 + _i] call _drawTape;
        [5360 + _i, (_npV * 100) toFixed 0, _xN, _yNum, _tapeW, _numH] call _setText;
        [5470 + _i, str (_i + 1), _xN, _yEng, _tapeW, _lblH] call _setText;

        private _xG = [_xTgt, _i] call _tapeX;
        [5370 + _i, 5380 + _i, _xG, _tgV, _tgtFs, _tgtDe, _tgtSe, 5510 + _i, 5520 + _i] call _drawTape;
        [5390 + _i, _tgV toFixed 0, _xG, _yNum, _tapeW, _numH,
            switch (true) do {
                case (_tgV >= _tgtSe): { [1.00, 0.35, 0.35, 1.00] };
                case (_tgV >= _tgtDe): { [1.00, 0.85, 0.30, 1.00] };
                default                 { [0.80, 1.00, 0.80, 1.00] };
            }] call _setText;
        [5540 + _i, str (_i + 1), _xG, _yEng, _tapeW, _lblH] call _setText;
    } else {
        [5310 + _i, 5320 + _i, 5330 + _i, 5340 + _i, 5350 + _i, 5360 + _i, 5370 + _i,
         5380 + _i, 5390 + _i, 5400 + _i, 5410 + _i, 5420 + _i, 5430 + _i, 5440 + _i,
         5450 + _i, 5470 + _i, 5490 + _i, 5500 + _i, 5510 + _i, 5520 + _i,
         5540 + _i, 5550 + _i] call _hide;
    };
};

//Nr sits in the middle slot of the tach group.
private _nrSlot = floor (_nTach / 2);
private _xNr    = [_xTach, _nrSlot] call _tapeX;
[5304, 5305, _xNr, _nr, _npFs, ED_NR_HIGH, _npOvsp, 5530, 5531] call _drawTape;
[5306, (_nr * 100) toFixed 0, _xNr, _yNum, _tapeW, _numH] call _setText;
[5309, "R", _xNr, _yEng, _tapeW, _lblH] call _setText;

// ── Group labels, centred on the tapes they name ─────────────────────────────
private _span = { params ["_base", "_c"]; (_c * _tapeW) + ((_c - 1) * _gapT) };
[5307, "TORQUE", _xTq,   _yLbl, [_xTq, _n] call _span,     _lblH] call _setText;
[5308, "TGT",    _xTgt,  _yLbl, [_xTgt, _n] call _span,    _lblH] call _setText;
[5560, "NP / NR", _xTach, _yLbl, [_xTach, _nTach] call _span, _lblH] call _setText;

// ── Digital rows - a row label, then one even column per engine ──────────────
private _labW = _inner * 0.22;
private _colW = (_inner - _labW) / _n;
private _colX = { params ["_i"]; _x0 + _pad + _labW + (_i * _colW) };

//Header, so the columns here carry engine numbers like the tapes do.
for "_i" from 0 to (_n - 1) do {
    [5550 + _i, str (_i + 1), [_i] call _colX, _yRows - _lblH, _colW, _lblH] call _setText;
};

{
    _x params ["_lblIdc", "_txt", "_row"];
    [_lblIdc, _txt, _x0 + _pad, _yRows + (_rowH * _row), _labW, _rowH] call _setText;
} forEach [[5460, "NG", 0], [5461, "OIL", 1], [5462, "RTG", 2]];

for "_i" from 0 to (_n - 1) do {
    private _xc = [_i] call _colX;
    [5410 + _i, ((([_ng, _i, 0.0] call BIS_fnc_param) * 100) toFixed 1),
        _xc, _yRows, _colW, _rowH] call _setText;
    [5420 + _i, ((([_oil, _i, 0.0] call BIS_fnc_param) * 100) toFixed 0),
        _xc, _yRows + _rowH, _colW, _rowH] call _setText;
    [5430 + _i, ([_rtg, _i, "--"] call BIS_fnc_param),
        _xc, _yRows + (_rowH * 2), _colW, _rowH] call _setText;
};

// ── Annunciators ─────────────────────────────────────────────────────────────
private _warn = [];
private _caut = [];
private _advs = [];

private _state = _heli getVariable ["bmkhs_engState", []];
private _ovsp  = _heli getVariable ["bmkhs_engineOverspeed", []];

for "_i" from 0 to (_n - 1) do {
    private _e  = str (_i + 1);
    private _st = [_state, _i, "OFF"] call BIS_fnc_param;

    if (([_ng, _i, 0.0] call BIS_fnc_param) < ED_ENG_OUT_NG && {_st == "ON"}) then {
        _warn pushBack ("ENG" + _e + " OUT");
    };
    if ([_ovsp, _i, false] call BIS_fnc_param) then { _warn pushBack ("ENG" + _e + " OVSP") };
    if (([_tgt, _i, 0.0] call BIS_fnc_param) > _tgtSe) then { _warn pushBack ("ENG" + _e + " TGT") };

    if (([_heli, "engines", _i] call bmkhs_fnc_damageGet) > SYS_ENG_DMG_THRESH) then {
        _caut pushBack ("ENG" + _e + " CHIPS");
    };
    if (_st == "STARTING") then { _advs pushBack ("ENG" + _e + " START") };
    if ((_heli getVariable [format ["bmkhs_eng%1StartSwVal", _i + 1], 0]) < 0) then {
        _advs pushBack ("ENG " + _e + " ORIDE");
    };
};

if (_nr > 0.01 && {_nr < ED_NR_LOW}) then { _warn pushBack "LOW RTR" };
if (_nr > ED_NR_HIGH) then                { _warn pushBack "HIGH RTR" };
if ((fuel _heli) < ED_FUEL_LOW) then      { _caut pushBack "FUEL LOW" };

private _line = {
    params ["_list", "_col"];
    if (_list isEqualTo []) exitWith { "" };
    "<t color='" + _col + "'>" + (_list joinString "   ") + "</t><br/>"
};

private _ann = _display displayCtrl 5480;
_ann ctrlSetPosition [_x0 + _pad, _yAnn, _inner, _annH];
_ann ctrlSetStructuredText parseText (
      "<t size='0.85' font='EtelkaMonospacePro' align='left'>"
    + ([_warn, "#ff4040"] call _line)
    + ([_caut, "#ffc020"] call _line)
    + ([_advs, "#40ff40"] call _line)
    + "</t>"
);
_ann ctrlCommit 0;
_ann ctrlShow true;
