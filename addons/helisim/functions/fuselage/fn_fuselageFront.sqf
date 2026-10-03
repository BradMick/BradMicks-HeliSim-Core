
#include "\bmkhs_helisim\functions\core\core.hpp"

params ["_heli"];

if (!local _heli) exitWith {};

private _cfg            = configOf _heli;
private _sfmPlusConfig  = _cfg >> "BMKHS_HeliSim";

private _deltaTime      = _heli getVariable "bmkhs_deltaTime";
private _heliCom        = getCenterOfMass _heli;
private _rho            = _heli getVariable "bmkhs_rho";
private _debugLineScale = 1.0 / 30.0;

private _panelSet       = (_heli getVariable "bmkhs_fuselagePanels") get "fuselageFront";
private _facing         = [[0.0, 1.0, 0.0], [0.0, -1.0, 0.0]] select ((_panelSet get "facing") == "backward");
private _dragCoefTable  = _panelSet get "dragCoefTable";
private _count          = _panelSet get "count";
private _coords         = _panelSet get "panels";

for "_i" from 0 to (_count - 1) do {
    private _verts = _coords select _i;
    private _a     = _verts select 0;
    private _b     = _verts select 1;
    private _c     = _verts select 2;
    private _d     = _verts select 3;

    private _f = _d vectorDiff ((_d vectorDiff _c) vectorMultiply 0.5);
    private _g = _a vectorDiff ((_a vectorDiff _b) vectorMultiply 0.5);

    private _e = _g vectorAdd ((_f vectorDiff _g) vectorMultiply 0.5);

    //Normal from the quad itself, pointed the way it faces.
    private _vecFwd = vectorNormalized ((_c vectorDiff _a) vectorCrossProduct (_d vectorDiff _b));
    if ((_vecFwd vectorDotProduct _facing) < 0.0) then { _vecFwd = _vecFwd vectorMultiply -1.0; };

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e, _e vectorAdd _vecFwd, "white"] call bmkhs_fnc_debugDrawLine;
    };

    private _velFwd     = (_heli getVariable "bmkhs_velModelSpace") vectorDotProduct _vecFwd;
    private _v          = [_velFwd, -VEL_VNE, VEL_VNE] call BIS_fnc_clamp;
    private _pa         = _heli getVariable "bmkhs_pa";
    private _CD         = [_dragCoefTable, _pa] call bmkhs_fnc_mathLinearInterp select 1;
    private _area       = [_a, _b, _c, _d] call bmkhs_fnc_mathGetArea;
    private _drag       = _CD * 0.5 * _rho * _area * (_v * _v);

    private _dragVector = _vecFwd vectorMultiply (if (_velFwd < 0.0) then {1.0} else {-1.0});
    _dragVector         = _dragVector vectorMultiply (_drag * _deltaTime);

    if (BMKHS_FM_DEBUG) then {
    [_heli, _e vectorAdd (_dragVector vectorMultiply _debugLineScale), _e, "red"]   call bmkhs_fnc_debugDrawLine;
    };

    //Applied AT the CoM, so the arm is zero and this makes no moment - published
    //with a zero arm so the readout says that rather than implying one.
    if (BMKHS_FORCES_DEBUG) then {
        private _acc = _heli getVariable "bmkhs_dbgForces";
        _acc pushBack ["fuse front", _dragVector, [0,0,0]];
        _heli setVariable ["bmkhs_dbgForces", _acc];
    };

    _heli addForce[_heli vectorModelToWorld _dragVector, _heliCom];

    if (BMKHS_FM_DEBUG) then {
    //Draw the wing
    [_heli, _a, _b, "red"]   call bmkhs_fnc_debugDrawLine;
    [_heli, _b, _c, "white"] call bmkhs_fnc_debugDrawLine;
    [_heli, _c, _d, "red"]   call bmkhs_fnc_debugDrawLine;
    [_heli, _d, _a, "white"] call bmkhs_fnc_debugDrawLine;
    };
};
