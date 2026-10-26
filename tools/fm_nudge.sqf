if (!isNil "bmkhs_nudgeEH") then { (findDisplay 46) displayRemoveEventHandler ["KeyDown", bmkhs_nudgeEH]; };
bmkhs_nudgeTarget  = 0;
bmkhs_nudgeMode    = 0;
bmkhs_nudgeSteps   = [0.05, 0.5, 0.05];
bmkhs_nudgeAeroOff = [0, 0, 0];
bmkhs_nudgeAeroRot = [0, 0, 0];
bmkhs_nudgeStart   = [
    ((vehicle player) getVariable ["bmkhs_simpleRotors", []]) apply { [+(_x get "pivot"), +(_x get "rotation"), _x get "mastLength"] },
    +((vehicle player) getVariable ["bmkhs_comCorrection", [0, 0, 0]]),
    +((vehicle player) getVariable ["bmkhs_casualModeCom", [0, 0, 0]])
];

bmkhs_nudgePlacement = getArray (configOf (vehicle player) >> "BMKHS_HeliSim" >> "fmPlacement");
if (count bmkhs_nudgePlacement != 7) then { bmkhs_nudgePlacement = [0, 0, 0, 0, 0, 0, 0] };
bmkhs_nudgeOrigin = [bmkhs_nudgePlacement select 6, bmkhs_nudgePlacement select 4, 0 - (bmkhs_nudgePlacement select 5)];
bmkhs_nudgeAngles = [bmkhs_nudgePlacement select 1, bmkhs_nudgePlacement select 2, bmkhs_nudgePlacement select 3];

bmkhs_nudgeUnrotate = {
    private _a  = bmkhs_nudgeAngles;
    private _cx = [[1, 0, 0], _a select 0, _a select 1, _a select 2] call bmkhs_fnc_mathVectorRotate;
    private _cy = [[0, 1, 0], _a select 0, _a select 1, _a select 2] call bmkhs_fnc_mathVectorRotate;
    private _cz = [[0, 0, 1], _a select 0, _a select 1, _a select 2] call bmkhs_fnc_mathVectorRotate;
    _this apply { _x apply {
        private _v = _x vectorDiff bmkhs_nudgeOrigin;
        [_v vectorDotProduct _cx, _v vectorDotProduct _cy, _v vectorDotProduct _cz]
    } }
};
bmkhs_nudgeAeroRaw = [createHashMap, []];
{ (bmkhs_nudgeAeroRaw select 0) set [_x, (_y get "panels") call bmkhs_nudgeUnrotate] } forEach ((vehicle player) getVariable ["bmkhs_fuselagePanels", createHashMap]);
{ (bmkhs_nudgeAeroRaw select 1) pushBack ((_x get "panels") call bmkhs_nudgeUnrotate) } forEach ((vehicle player) getVariable ["bmkhs_wings", []]);

bmkhs_nudgeAeroApply = {
    params ["_heli"];
    private _a = bmkhs_nudgeAngles vectorAdd bmkhs_nudgeAeroRot;
    private _o = bmkhs_nudgeOrigin vectorAdd bmkhs_nudgeAeroOff;
    private _place = { _this apply { _x apply { ([_x, _a select 0, _a select 1, _a select 2] call bmkhs_fnc_mathVectorRotate) vectorAdd _o } } };
    { _y set ["panels", ((bmkhs_nudgeAeroRaw select 0) get _x) call _place] } forEach (_heli getVariable ["bmkhs_fuselagePanels", createHashMap]);
    { _x set ["panels", ((bmkhs_nudgeAeroRaw select 1) select _forEachIndex) call _place] } forEach (_heli getVariable ["bmkhs_wings", []]);
};

bmkhs_nudgeNum = { private _v = parseNumber (_this toFixed 3); (_v + 0) toFixed 3 };
bmkhs_nudgeFmt = { format ["{%1, %2, %3}", (_this select 0) call bmkhs_nudgeNum, (_this select 1) call bmkhs_nudgeNum, (_this select 2) call bmkhs_nudgeNum] };

bmkhs_nudgeTargets = {
    params ["_heli"];
    private _targets = [["aero"]];
    { _targets pushBack ["rotor", _forEachIndex] } forEach (_heli getVariable ["bmkhs_simpleRotors", []]);
    _targets pushBack ["cg"];
    _targets
};

bmkhs_nudgeModes = {
    switch (_this select 0) do {
        case "aero":  { ["move", "rotate"] };
        case "rotor": { ["move", "rotate", "mast"] };
        default       { ["move"] };
    }
};

bmkhs_nudgeLines = {
    params ["_heli", "_target"];
    switch (_target select 0) do {
        case "aero": {
            bmkhs_nudgeAeroOff params ["_r", "_f", "_u"];
            bmkhs_nudgeAeroRot params ["_p", "_rl", "_y"];
            ["Fuselage + wings", [[
                format ["python tools/fm_generateAeroSurfacePoints.py FM_P3D BMKHS_CONFIG --add --forward %1 --down %2 --right %3 --pitch %4 --roll %5 --yaw %6   # fuselage + wings - from @bmkhs/python",
                    _f call bmkhs_nudgeNum, (0 - _u) call bmkhs_nudgeNum, _r call bmkhs_nudgeNum,
                    _p call bmkhs_nudgeNum, _rl call bmkhs_nudgeNum, _y call bmkhs_nudgeNum],
                ((bmkhs_nudgeAeroOff call bmkhs_nudgeFmt) != ([0, 0, 0] call bmkhs_nudgeFmt)) || {(bmkhs_nudgeAeroRot call bmkhs_nudgeFmt) != ([0, 0, 0] call bmkhs_nudgeFmt)}
            ]]]
        };
        case "rotor": {
            private _i     = _target select 1;
            private _rotor = (_heli getVariable "bmkhs_simpleRotors") select _i;
            private _start = (bmkhs_nudgeStart select 0) select _i;
            private _name  = format ["SimpleRotor%1%2", ["0", ""] select (_i >= 9), _i + 1];
            private _items = [
                ["pivot[] = %1;   //%2 - helisim_simpleRotor.hpp",    _rotor get "pivot",      _start select 0, bmkhs_nudgeFmt],
                ["rotation[] = %1;   //%2 - helisim_simpleRotor.hpp", _rotor get "rotation",   _start select 1, bmkhs_nudgeFmt],
                ["mastLength = %1;   //%2 - helisim_simpleRotor.hpp", _rotor get "mastLength", _start select 2, bmkhs_nudgeNum]
            ];
            [format ["%1, %2 rotor", _name, ["main", "tail"] select ((_rotor get "type") == 1)], _items apply {
                _x params ["_line", "_now", "_was", "_fn"];
                [format [_line, _now call _fn, _name], (_now call _fn) != (_was call _fn)]
            }]
        };
        case "cg": {
            private _real = bmkhs_helisimRealismSetting == 2;
            private _var  = ["bmkhs_casualModeCom", "bmkhs_comCorrection"] select _real;
            private _now  = (_heli getVariable _var) call bmkhs_nudgeFmt;
            private _was  = (bmkhs_nudgeStart select ([2, 1] select _real)) call bmkhs_nudgeFmt;
            private _line = ["casualModeCom[] = %1;   //centre of mass, casual modes - helisim_mass.hpp", "comCorrection[] = %1;   //centre of mass, Realistic - helisim_mass.hpp"] select _real;
            [["Centre of mass (casual)", "Centre of mass (Realistic)"] select _real, [[format [_line, _now], _now != _was]]]
        };
    };
};

bmkhs_nudgeEH = (findDisplay 46) displayAddEventHandler ["KeyDown", {
    params ["", "_key"];
    private _heli    = vehicle player;
    private _targets = [_heli] call bmkhs_nudgeTargets;
    bmkhs_nudgeTarget = bmkhs_nudgeTarget mod (count _targets);
    private _target  = _targets select bmkhs_nudgeTarget;
    private _modes   = _target call bmkhs_nudgeModes;
    private _mode    = [0, bmkhs_nudgeMode] select (bmkhs_nudgeMode < count _modes);
    private _s       = bmkhs_nudgeSteps select _mode;
    private _d = switch (_key) do {
        case 72: { [0,  _s, 0] };
        case 80: { [0, -_s, 0] };
        case 77: { [ _s, 0, 0] };
        case 75: { [-_s, 0, 0] };
        case 73: { [0, 0,  _s] };
        case 81: { [0, 0, -_s] };
        default  { [] };
    };
    switch (_key) do {
        case 82: {
            bmkhs_nudgeTarget = (bmkhs_nudgeTarget + 1) mod (count _targets);
            _target = _targets select bmkhs_nudgeTarget;
            _modes  = _target call bmkhs_nudgeModes;
            _mode   = [0, bmkhs_nudgeMode] select (bmkhs_nudgeMode < count _modes);
        };
        case 71: {
            bmkhs_nudgeMode = (_mode + 1) mod (count _modes);
            _mode = bmkhs_nudgeMode;
        };
        case 78: { bmkhs_nudgeSteps set [_mode, _s * 2] };
        case 74: { bmkhs_nudgeSteps set [_mode, _s / 2] };
        case 76: {
            private _out = [];
            {
                { if (_x select 1) then { _out pushBack (_x select 0) } } forEach (([_heli, _x] call bmkhs_nudgeLines) select 1);
            } forEach _targets;
            if (_out isEqualTo []) then {
                systemChat "FM nudge - nothing has changed";
            } else {
                copyToClipboard (_out joinString endl);
                systemChat format ["FM nudge - copied %1 line(s):", count _out];
                { systemChat _x } forEach _out;
            };
        };
    };
    if (_d isNotEqualTo []) then {
        private _turn = [_d select 1, _d select 0, _d select 2];
        switch (_target select 0) do {
            case "aero": {
                if (_mode == 0) then {
                    bmkhs_nudgeAeroOff = bmkhs_nudgeAeroOff vectorAdd _d;
                } else {
                    bmkhs_nudgeAeroRot = bmkhs_nudgeAeroRot vectorAdd _turn;
                };
                [_heli] call bmkhs_nudgeAeroApply;
            };
            case "rotor": {
                private _rotor = (_heli getVariable "bmkhs_simpleRotors") select (_target select 1);
                switch (_mode) do {
                    case 0: { _rotor set ["pivot", (_rotor get "pivot") vectorAdd _d] };
                    case 1: { _rotor set ["rotation", (_rotor get "rotation") vectorAdd _turn] };
                    case 2: { _rotor set ["mastLength", (_rotor get "mastLength") + (_d select 2)] };
                };
            };
            case "cg": {
                private _var = ["bmkhs_casualModeCom", "bmkhs_comCorrection"] select (bmkhs_helisimRealismSetting == 2);
                _heli setVariable [_var, (_heli getVariable _var) vectorAdd _d];
            };
        };
    };
    ([_heli, _target] call bmkhs_nudgeLines) params ["_label", "_lines"];
    private _modeName = _modes select _mode;
    private _keys = switch (_modeName) do {
        case "rotate": { "Num 8/2 pitch  6/4 roll  9/3 yaw" };
        case "mast":   { "Num 9/3 mast longer/shorter" };
        default        { "Num 8/2 fwd/aft  6/4 right/left  9/3 up/down" };
    };
    hintSilent format ["FM nudge - %1%2\nstep %3 %4\n\n%5\n\n%6\nNum 0 next target%7\nNum +/- step  5 copy every change",
        _label,
        ["", format ["\nmode: %1", _modeName]] select (count _modes > 1),
        (bmkhs_nudgeSteps select _mode) toFixed 3,
        ["m", "deg"] select (_modeName == "rotate"),
        (_lines apply { (["  ", "* "] select (_x select 1)) + (_x select 0) }) joinString "\n",
        _keys,
        ["", "  7 mode"] select (count _modes > 1)
    ];
    _key in [72, 80, 77, 75, 73, 81, 82, 71, 78, 74, 76]
}];
hint "FM nudge on - Numpad 0 picks the target: fuselage + wings, each rotor, centre of mass. Numpad 7 cycles the mode: move, rotate (fuselage + wings, rotors), mast (rotors). Numpad 5 copies every change.";
