//FM nudge - moves the aerosurfaces, rotor hubs and centre of mass live. Debug console, LOCAL EXEC.
//Numpad 0 target, 8/2 fwd/aft, 6/4 right/left, 9/3 up/down, +/- step, 5 copy result.
if (!isNil "bmkhs_nudgeEH") then { (findDisplay 46) displayRemoveEventHandler ["KeyDown", bmkhs_nudgeEH]; };
bmkhs_nudgeTarget  = 0;
bmkhs_nudgeStep    = 0.05;
bmkhs_nudgeAeroOff = [0, 0, 0];    //right, forward, up

bmkhs_nudgeTargets = {
    params ["_heli"];
    private _targets = [["aero"]];
    { _targets pushBack ["rotor", _forEachIndex] } forEach (_heli getVariable ["bmkhs_simpleRotors", []]);
    _targets pushBack ["cg"];
    _targets
};

bmkhs_nudgeReport = {
    params ["_heli", "_target"];
    private _num = { private _v = parseNumber (_this toFixed 3); (_v + 0) toFixed 3 };
    private _fmt = { format ["{%1, %2, %3}", (_this select 0) call _num, (_this select 1) call _num, (_this select 2) call _num] };
    switch (_target select 0) do {
        case "aero": {
            bmkhs_nudgeAeroOff params ["_r", "_f", "_u"];
            ["Fuselage + wings", format ["python tools\fm_generateAeroSurfacePoints.py <fm.p3d> <bmkhs_config> --add --forward %1 --down %2 --right %3   # fuselage + wings - from @bmkhs/python",
                _f call _num, (0 - _u) call _num, _r call _num]]
        };
        case "rotor": {
            private _i     = _target select 1;
            private _rotor = (_heli getVariable "bmkhs_simpleRotors") select _i;
            private _name  = format ["SimpleRotor%1%2, %3 rotor", ["0", ""] select (_i >= 9), _i + 1, ["main", "tail"] select ((_rotor get "type") == 1)];
            [_name, format ["pivot[] = %1;   //%2 - helisim_simpleRotor.hpp", (_rotor get "pivot") call _fmt, _name]]
        };
        case "cg": {
            if (bmkhs_helisimRealismSetting == 2) then {
                ["Centre of mass (Realistic)", format ["comCorrection[] = %1;   //centre of mass, Realistic - helisim_mass.hpp", (_heli getVariable "bmkhs_comCorrection") call _fmt]]
            } else {
                ["Centre of mass (casual)", format ["casualModeCom[] = %1;   //centre of mass, casual modes - helisim_mass.hpp", (_heli getVariable "bmkhs_casualModeCom") call _fmt]]
            };
        };
    };
};

bmkhs_nudgeEH = (findDisplay 46) displayAddEventHandler ["KeyDown", {
    params ["", "_key"];
    private _heli    = vehicle player;
    private _targets = [_heli] call bmkhs_nudgeTargets;
    bmkhs_nudgeTarget = bmkhs_nudgeTarget mod (count _targets);
    private _target  = _targets select bmkhs_nudgeTarget;
    private _s = bmkhs_nudgeStep;
    //Arma model frame {right, forward, up}
    private _d = switch (_key) do {
        case 72: { [0,  _s, 0] };   //Numpad 8
        case 80: { [0, -_s, 0] };   //Numpad 2
        case 77: { [ _s, 0, 0] };   //Numpad 6
        case 75: { [-_s, 0, 0] };   //Numpad 4
        case 73: { [0, 0,  _s] };   //Numpad 9
        case 81: { [0, 0, -_s] };   //Numpad 3
        default  { [] };
    };
    switch (_key) do {
        case 82: {   //Numpad 0
            bmkhs_nudgeTarget = (bmkhs_nudgeTarget + 1) mod (count _targets);
            _target = _targets select bmkhs_nudgeTarget;
        };
        case 78: { bmkhs_nudgeStep = _s * 2 };   //Numpad +
        case 74: { bmkhs_nudgeStep = _s / 2 };   //Numpad -
        case 76: {   //Numpad 5
            private _line = ([_heli, _target] call bmkhs_nudgeReport) select 1;
            copyToClipboard _line;
            systemChat ("copied: " + _line);
        };
    };
    if (_d isNotEqualTo []) then {
        switch (_target select 0) do {
            case "aero": {
                private _move = { _this apply { _x apply { _x vectorAdd _d } } };
                { _y set ["panels", (_y get "panels") call _move] } forEach (_heli getVariable ["bmkhs_fuselagePanels", createHashMap]);
                { _x set ["panels", (_x get "panels") call _move] } forEach (_heli getVariable ["bmkhs_wings", []]);
                bmkhs_nudgeAeroOff = bmkhs_nudgeAeroOff vectorAdd _d;
            };
            case "rotor": {
                private _rotor = (_heli getVariable "bmkhs_simpleRotors") select (_target select 1);
                _rotor set ["pivot", (_rotor get "pivot") vectorAdd _d];
            };
            case "cg": {
                private _var = ["bmkhs_casualModeCom", "bmkhs_comCorrection"] select (bmkhs_helisimRealismSetting == 2);
                _heli setVariable [_var, (_heli getVariable _var) vectorAdd _d];
            };
        };
    };
    ([_heli, _target] call bmkhs_nudgeReport) params ["_label", "_line"];
    hintSilent format ["FM nudge - %1\nstep %2 m\n\n%3\n\nNum 0 next target\nNum 8/2 fwd/aft  6/4 right/left  9/3 up/down\nNum +/- step  5 copy", _label, bmkhs_nudgeStep toFixed 3, _line];
    _key in [72, 80, 77, 75, 73, 81, 82, 78, 74, 76]
}];
hint "FM nudge on - Numpad 0 picks the target: fuselage + wings, each rotor, centre of mass";
