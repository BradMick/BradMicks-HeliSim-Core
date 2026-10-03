/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_engDisplayUpdate

Description:
    Draws the engine readout - torque, Np, Nr, TGT, Ng and oil pressure, with
    warnings, cautions and advisories beneath.

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

private _ng  = _heli getVariable "bmkhs_engPctNg";
private _np  = _heli getVariable "bmkhs_engPctNp";
private _tq  = _heli getVariable "bmkhs_engPctTq";
private _tgt = _heli getVariable "bmkhs_engTgt";
private _oil = _heli getVariable "bmkhs_engOilPsi";

private _n = _heli getVariable "bmkhs_numEngines";
_n = (_n min ED_MAX_ENG) max 1;

private _nr = _heli getVariable "bmkhs_rtrRpm";
(_heli getVariable "bmkhs_nrLimits") params ["_nrLow", "_nrHigh", "_nrHighRtr", "_nrMax"];

private _engines = _heli getVariable "bmkhs_engines";

//Single engine switches torque and TGT to their Se limits.
private _single = _heli getVariable "bmkhs_isSingleEng";
private _tqKey  = ["tqLimits",  "tqLimitsSe"]  select _single;
private _tgtKey = ["tgtLimits", "tgtLimitsSe"] select _single;

//A book limit set's first limit and its last.
private _band = {
    params ["_engine", "_key"];
    private _lims = _engine get _key;
    [(_lims # 0) # 0, (_lims # ((count _lims) - 1)) # 0]
};
//{control, limit} for every line of a set. Lines between the first and last use the fixed
//controls, then ones made here when the set has more - a control left over reads 0 and is hidden.
private _lines = {
    params ["_engine", "_key", "_first", "_mids", "_last", "_tag"];
    private _lims  = _engine get _key;
    private _k     = count _lims;
    private _need  = (_k - 2) max 0;
    private _extra = _display getVariable [_tag, []];
    private _out   = [[_display displayCtrl _first, (_lims # 0) # 0], [_display displayCtrl _last, (_lims # (_k - 1)) # 0]];
    for "_m" from 0 to ((_need max ((count _mids) + (count _extra))) - 1) do {
        private _c = if (_m < count _mids) then { _display displayCtrl (_mids # _m) } else {
            private _e = _m - (count _mids);
            if (_e >= count _extra) then {
                private _new = _display ctrlCreate ["RscText", -1];
                _new ctrlSetBackgroundColor [0.95, 0.75, 0.10, 0.85];
                _extra pushBack _new;
            };
            _extra # _e
        };
        _out pushBack [_c, if (_m < _need) then { (_lims # (_m + 1)) # 0 } else { 0 }];
    };
    _display setVariable [_tag, _extra];
    _out
};
private _colour = {
    params ["_val", "_amber", "_red"];
    switch (true) do {
        case (_val >= _red):   { [1.00, 0.35, 0.35, 1.00] };
        case (_val >= _amber): { [1.00, 0.85, 0.30, 1.00] };
        default                { [0.80, 1.00, 0.80, 1.00] };
    }
};

//Full scales from engine 1, so the tapes line up.
private _tgtRed = ([_engines # 0, _tgtKey] call _band) # 1;
private _npOvsp = (_engines # 0) get "maxNp";

//Full scale sits above the top limit so the red band has somewhere to be drawn.
private _tqFs  = ED_TQ_FULL_SCALE;
private _tgtFs = _tgtRed * 1.08;
private _npFs  = _npOvsp * 1.05;

// ── Annunciators - gathered before the layout, which sizes their block ───────
private _warn = [];
private _caut = [];
private _advs = [];

private _state = _heli getVariable "bmkhs_engState";
private _ovsp  = _heli getVariable "bmkhs_engineOverspeed";

for "_i" from 0 to (_n - 1) do {
    private _e  = str (_i + 1);
    private _st = [_state, _i, "OFF"] call BIS_fnc_param;

    if ((([_ng, _i, 0.0] call BIS_fnc_param) < ((_engines # _i) get "ngMin") && {_st == "ON"})
        || {(_heli getVariable "bmkhs_engFailed") select _i}) then {
        _warn pushBack ("ENG" + _e + " OUT");
    };
    if ([_ovsp, _i, false] call BIS_fnc_param) then { _warn pushBack ("ENG" + _e + " OVSP") };
    if (([_tgt, _i, 0.0] call BIS_fnc_param) > (([_engines # _i, _tgtKey] call _band) # 1)) then {
        _warn pushBack ("ENG" + _e + " TGT");
    };

    if ((_heli getVariable "bmkhs_engChips") select _i) then {
        _caut pushBack ("ENG" + _e + " CHIPS");
    };
    if ((_heli getVariable "bmkhs_engOilPsiLow") select _i) then {
        _caut pushBack ("ENG" + _e + " OIL PSI");
    };
    if (_st == "STARTING") then { _advs pushBack ("ENG" + _e + " START") };
    if ((_heli getVariable [format ["bmkhs_eng%1StartSwVal", _i + 1], 0]) < 0) then {
        _advs pushBack ("ENG " + _e + " ORIDE");
    };
};

if (_nr > 0.01 && {_nr < _nrLow}) then { _warn pushBack "LOW RTR" };
if (_nr >= _nrHighRtr) then           { _warn pushBack "HIGH RTR" };
if ((fuel _heli) < ED_FUEL_LOW) then      { _caut pushBack "FUEL LOW" };

//Hold modes - the AH-64D WCA short forms, unpadded.
if (_heli getVariable ["bmkhs_attHoldActive", false]) then {
    _advs pushBack "ATT HOLD";

    //Drifting off the hold point. The sub-mode is written lower case; the WCA compares
    //it upper case, so its own copy never fires.
    if ((_heli getVariable ["bmkhs_attHoldSubMode", ""]) == "pos"
        && {!(_heli getVariable ["bmkhs_forceTrimInterupted", false])}) then {
        private _desired = _heli getVariable ["bmkhs_attHoldDesiredPos", getPos _heli];
        if ((_heli distance2D _desired) >= ED_HOVER_DRIFT_M) then { _advs pushBack "HOVER DRIFT" };
    };
};
if (_heli getVariable ["bmkhs_altHoldActive", false]) then {
    _advs pushBack (["BAR HOLD", "RAD HOLD"] select ((_heli getVariable ["bmkhs_altHoldSubMode", ""]) == "rad"));
};

// ── Geometry ─────────────────────────────────────────────────────────────────
private _pad   = _W * 0.030;
private _inner = _W - (_pad * 2);

//Three groups: torque (N tapes), tachs (N + 1 tapes), TGT (N tapes). Tapes sit tight
//within a group; the leftover width goes between the GROUPS, evenly.
private _nTach  = _n + 1;
private _nTapes = (_n * 2) + _nTach;
private _tapeW  = (_inner / _nTapes) * 0.76;
private _gapT   = _tapeW * 0.33;
private _grpTq  = (_n * _tapeW) + ((_n - 1) * _gapT);
private _grpTch = (_nTach * _tapeW) + ((_nTach - 1) * _gapT);
private _gapG   = ((_inner - (_grpTq * 2) - _grpTch) / 2) max 0;

//Text rows take a fixed share of the panel, so everything left over is tape.
private _lblH  = _H * 0.048;
private _numH  = _H * 0.056;
private _tmrH  = _H * 0.046;
private _rowH  = _H * 0.062;

//Always three rows - the block stays put whether or not anything is annunciating.
private _annLines = 3;
private _annH  = _rowH * _annLines;

private _yLbl  = _y0 + (_H * 0.010);
private _yEng  = _yLbl + _lblH;
private _yTape = _yEng + _lblH;
//Two label rows at the top (group + engine number), one more above the digital rows.
private _tapeH = _H - ((_H * 0.010) + (_lblH * 3) + _numH + _tmrH + (_rowH * 3) + _annH + (_H * 0.030));
if (_tapeH < 0.01) then { _tapeH = _H * 0.30 };
private _yNum  = _yTape + _tapeH;
private _yTmr  = _yNum + _numH;
private _yRows = _yTmr + _tmrH + (_H * 0.010) + _lblH;
private _yAnn  = _yRows + (_rowH * 3) + (_H * 0.008);

//Left edge of each group, walked across the panel.
private _xTq   = _x0 + _pad;
private _xTach = _xTq + _grpTq + _gapG;
private _xTgt  = _xTach + _grpTch + _gapG;

private _tapeX = { params ["_base", "_i"]; _base + (_i * (_tapeW + _gapT)) };

//A tape: frame, fill bottom-up, and the limit ticks over the top of it.
private _drawTape = {
    params ["_fIdc", "_lIdc", "_tx", "_val", "_fs", "_amber", "_red", "_ticks", ["_low", 0]];

    private _f = _display displayCtrl _fIdc;
    _f ctrlSetPosition [_tx, _yTape, _tapeW, _tapeH];
    _f ctrlCommit 0;
    _f ctrlShow true;

    //Limit ticks - a line across the tape at each threshold, not a filled band.
    private _tickH = (_tapeH * 0.012) max 0.0015;
    {
        _x params ["_c", "_at"];
        if (!isNull _c) then {
            if (_fs > 0 && {_at > 0} && {_at < _fs}) then {
                _c ctrlSetPosition [_tx, _yTape + (_tapeH * (1 - (_at / _fs))) - (_tickH / 2),
                                    _tapeW, _tickH];
                _c ctrlCommit 0;
                _c ctrlShow true;
            } else {
                _c ctrlShow false;
            };
        };
    } forEach _ticks;

    private _frac = ((_val / _fs) max 0) min 1;
    private _fill = _display displayCtrl _lIdc;
    _fill ctrlSetPosition [_tx, _yTape + (_tapeH * (1 - _frac)), _tapeW, _tapeH * _frac];
    _fill ctrlSetBackgroundColor (switch (true) do {
        case (_val >= _red):   { [0.90, 0.15, 0.15, 0.95] };
        case (_val >= _amber): { [0.95, 0.75, 0.10, 0.95] };
        case (_val < _low):    { [0.95, 0.75, 0.10, 0.95] };
        default                { [0.20, 0.90, 0.20, 0.95] };
    });
    _fill ctrlCommit 0;
    _fill ctrlShow true;
};

//Glyphs scale with the row they sit in, so the text grows with the panel.
//_over widens the box either side without moving its centre, so a centred numeric
//wider than its tape is not clipped.
private _setText = {
    params ["_idc", "_txt", "_x", "_y", "_w", "_h", ["_col", [0.80, 1.00, 0.80, 1.00]], ["_over", 0.0]];
    private _c = _display displayCtrl _idc;
    if (isNull _c) exitWith {};
    _c ctrlSetPosition [_x - _over, _y, _w + (_over * 2), _h];
    _c ctrlSetFontHeight (_h * 0.82);
    _c ctrlSetText _txt;
    _c ctrlSetTextColor _col;
    _c ctrlCommit 0;
    _c ctrlShow true;
};

private _hide = { { private _c = _display displayCtrl _x; if !(isNull _c) then { _c ctrlShow false } } forEach _this };

//Countdown, m:ss. Blank with no band; blinks at 0:00 once damage has started.
private _blink = (floor (time * 4)) mod 2 == 0;
private _timer = {
    params ["_idc", "_left", "_x", "_y", "_w", "_h", "_col"];
    if (_left < 0 || {_left <= 0 && {!_blink}}) exitWith { [_idc] call _hide };
    private _s = ceil _left;
    private _sec = _s mod 60;
    [_idc, format ["%1:%2", floor (_s / 60), [str _sec, "0" + str _sec] select (_sec < 10)],
        _x, _y, _w, _h, _col, _gapT * 0.5] call _setText;
};

// ── Tapes ────────────────────────────────────────────────────────────────────
private _limTmrs = _heli getVariable "bmkhs_engLimitTimers";
private _tqTmrs  = _heli getVariable "bmkhs_engTqTimer";

for "_i" from 0 to (ED_MAX_ENG - 1) do {
    if (_i < _n) then {
        private _tqV  = [_tq,  _i, 0.0] call BIS_fnc_param;
        private _npV  = [_np,  _i, 0.0] call BIS_fnc_param;
        private _tgV  = [_tgt, _i, 0.0] call BIS_fnc_param;

        private _eng = _engines # _i;
        ([_eng, _tqKey]     call _band) params ["_tqAmb",  "_tqRed"];
        ([_eng, "npLimits"] call _band) params ["_npAmb",  "_npRed"];
        ([_eng, _tgtKey]    call _band) params ["_tgtAmb", "_tgtRedE"];

        private _xT = [_xTq, _i] call _tapeX;
        [5310 + _i, 5320 + _i, _xT, _tqV, _tqFs, _tqAmb, _tqRed,
            [_eng, _tqKey, 5440 + _i, [5570 + _i], 5450 + _i, format ["bmkhs_edTqLines%1", _i]] call _lines] call _drawTape;
        [5330 + _i, (_tqV * 100) toFixed 0, _xT, _yNum, _tapeW, _numH,
            [_tqV, _tqAmb, _tqRed] call _colour, _gapT * 0.5] call _setText;
        [5400 + _i, str (_i + 1), _xT, _yEng, _tapeW, _lblH] call _setText;

        //Np tape - engine 0 left of Nr, the rest to its right, so Nr stays centred.
        private _slot = [_i, _i + 1] select (_i >= (_nTach / 2) - 0.5);
        private _xN = [_xTach, _slot] call _tapeX;
        [5340 + _i, 5350 + _i, _xN, _npV, _npFs, _npAmb, _npRed,
            [_eng, "npLimits", 5490 + _i, [], 5500 + _i, format ["bmkhs_edNpLines%1", _i]] call _lines] call _drawTape;
        [5360 + _i, (_npV * 100) toFixed 0, _xN, _yNum, _tapeW, _numH,
            [_npV, _npAmb, _npRed] call _colour, _gapT * 0.5] call _setText;
        [5470 + _i, str (_i + 1), _xN, _yEng, _tapeW, _lblH] call _setText;

        private _xG = [_xTgt, _i] call _tapeX;
        [5370 + _i, 5380 + _i, _xG, _tgV, _tgtFs, _tgtAmb, _tgtRedE,
            [_eng, _tgtKey, 5510 + _i, [5580 + _i, 5590 + _i, 5600 + _i], 5520 + _i, format ["bmkhs_edTgtLines%1", _i]] call _lines] call _drawTape;
        [5390 + _i, _tgV toFixed 0, _xG, _yNum, _tapeW, _numH,
            [_tgV, _tgtAmb, _tgtRedE] call _colour, _gapT * 0.5] call _setText;
        [5540 + _i, str (_i + 1), _xG, _yEng, _tapeW, _lblH] call _setText;

        //Countdowns under the numbers - {np, ng, tgt}, torque from the drivetrain it loads.
        private _lim = _limTmrs # _i;
        [5610 + _i, _tqTmrs # _i, _xT, _yTmr, _tapeW, _tmrH, [_tqV, _tqAmb, _tqRed] call _colour] call _timer;
        [5620 + _i, _lim # 0,     _xN, _yTmr, _tapeW, _tmrH, [_npV, _npAmb, _npRed] call _colour] call _timer;
        [5630 + _i, _lim # 2,     _xG, _yTmr, _tapeW, _tmrH, [_tgV, _tgtAmb, _tgtRedE] call _colour] call _timer;
    } else {
        [5310 + _i, 5320 + _i, 5330 + _i, 5340 + _i, 5350 + _i, 5360 + _i, 5370 + _i,
         5380 + _i, 5390 + _i, 5400 + _i, 5410 + _i, 5420 + _i, 5430 + _i, 5440 + _i,
         5450 + _i, 5470 + _i, 5490 + _i, 5500 + _i, 5510 + _i, 5520 + _i,
         5540 + _i, 5550 + _i, 5570 + _i, 5580 + _i, 5590 + _i, 5600 + _i,
         5610 + _i, 5620 + _i, 5630 + _i, 5640 + _i] call _hide;
    };
};

//Nr sits in the middle slot of the tach group.
private _nrSlot = floor (_nTach / 2);
private _xNr    = [_xTach, _nrSlot] call _tapeX;
[5304, 5305, _xNr, _nr, _npFs, _nrHigh, _nrMax,
    [[5530, _nrLow], [5650, _nrHigh], [5651, _nrHighRtr], [5531, _nrMax]] apply {[_display displayCtrl (_x # 0), _x # 1]},
    _nrLow] call _drawTape;
[5306, (_nr * 100) toFixed 0, _xNr, _yNum, _tapeW, _numH,
    [0.80, 1.00, 0.80, 1.00], _gapT * 0.5] call _setText;
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
} forEach [[5460, "NG", 0], [5461, "OIL", 2]];
[5462] call _hide;

//NG, its countdowns, then oil.
for "_i" from 0 to (_n - 1) do {
    private _xc  = [_i] call _colX;
    private _ngV = [_ng, _i, 0.0] call BIS_fnc_param;
    ([_engines # _i, "ngLimits"] call _band) params ["_ngAmb", "_ngRed"];
    private _ngCol = [[_ngV, _ngAmb, _ngRed] call _colour, [1.00, 0.35, 0.35, 1.00]] select (_ngV < ((_engines # _i) get "ngMin"));
    [5410 + _i, (_ngV * 100) toFixed 1, _xc, _yRows, _colW, _rowH, _ngCol] call _setText;
    [5640 + _i, (_limTmrs # _i) # 1, _xc, _yRows + _rowH, _colW, _rowH, _ngCol] call _timer;
    [5420 + _i, ((([_oil, _i, 0.0] call BIS_fnc_param) * 100) toFixed 0),
        _xc, _yRows + (_rowH * 2), _colW, _rowH] call _setText;
    [5430 + _i] call _hide;
};

// ── Annunciators ─────────────────────────────────────────────────────────────
private _line = {
    params ["_list", "_col"];
    if (_list isEqualTo []) exitWith { "" };
    "<t color='" + _col + "'>" + (_list joinString "  ") + "</t><br/>"
};

private _ann = _display displayCtrl 5480;
_ann ctrlSetPosition [_x0 + _pad, _yAnn, _inner, _annH];
_ann ctrlSetFontHeight (_rowH * 0.62);
_ann ctrlSetStructuredText parseText (
      "<t font='EtelkaMonospacePro' align='left'>"
    + ([_warn, "#ff4040"] call _line)
    + ([_caut, "#ffc020"] call _line)
    + ([_advs, "#40ff40"] call _line)
    + "</t>"
);
_ann ctrlCommit 0;
_ann ctrlShow true;
