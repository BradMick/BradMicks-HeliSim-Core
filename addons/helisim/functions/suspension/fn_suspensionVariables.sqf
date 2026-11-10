/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_suspensionVariables

Description:
    Loads class Wheels - every wheel the aircraft declares, or none - and seeds
    the suspension's state. Fits the static load split and the wheel plane once.

Parameters:
    _heli   - The helicopter [Object]
    _config - The aircraft's HeliSim config [Config]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_config"];

private _cfg         = _config >> "Wheels";
private _numFields   = ["radius", "castors", "braked"];
private _arrFields   = ["position", "extended"];
private _textFields  = ["name", "lockControl", "animationSource", "compressionSource"];
//Positions are Object Builder positions; Arma's model frame is offset from those by boundingCenter.
private _modelOffset = boundingCenter _heli;
private _ctrlIndex   = _heli getVariable "bmkhs_ctrlIndex";

private _settings = createHashMap;
{ _settings set [_x, getNumber (_cfg >> _x)]; } forEach
    ["muRoll", "muBrake", "muSide", "muCastorFree", "muCastorLocked", "castorTrail", "castorPinAngle"];
_settings set ["brakeControl",   getText (_cfg >> "brakeControl")];
_settings set ["weightOnWheels", getText (_cfg >> "weightOnWheels")];

if ((_settings get "brakeControl") isNotEqualTo "" && {!((_settings get "brakeControl") in _ctrlIndex)}) then {
    [format [
        "[BMKHS] WHEELS CONFIG ERROR: brakeControl '%1' is not a declared control. The wheels have no brake.",
        _settings get "brakeControl"
    ]] call bmkhs_fnc_utilConfigError;
    _settings set ["brakeControl", ""];
};

private _wheels = [];
{
    private _w = _x;
    private _i = _forEachIndex + 1;

    private _wheel = createHashMap;
    { _wheel set [_x, getNumber (_w >> _x)]; } forEach _numFields;
    { _wheel set [_x, getArray  (_w >> _x)]; } forEach _arrFields;
    { _wheel set [_x, getText   (_w >> _x)]; } forEach _textFields;
    _wheel set ["castors",  (_wheel get "castors") > 0];
    _wheel set ["braked",   (_wheel get "braked") > 0];
    _wheel set ["position", (_wheel get "position") vectorDiff _modelOffset];
    _wheel set ["extended", (_wheel get "extended") vectorDiff _modelOffset];

    private _name = _wheel get "name";
    if (_name isEqualTo "") then { _name = format ["wheel %1", _i]; _wheel set ["name", _name]; };

    //The stroke, full extension to static; its vertical part is the strut's sag
    private _stroke = (_wheel get "position") vectorDiff (_wheel get "extended");
    _wheel set ["stroke", _stroke];
    _wheel set ["sag",    _stroke select 2];
    if ((_stroke select 2) <= 0.0) then {
        [format ["[BMKHS] WHEELS CONFIG ERROR: %1 extended[] is not below position[]. The suspension is off.", _name]]
            call bmkhs_fnc_utilConfigError;
        _wheels = [];
        break;
    };

    if ((_wheel get "braked") && {(_settings get "brakeControl") isEqualTo ""}) then {
        [format ["[BMKHS] WHEELS CONFIG ERROR: %1 is braked but Wheels declares no brakeControl. It rolls free.", _name]]
            call bmkhs_fnc_utilConfigError;
        _wheel set ["braked", false];
    };
    private _lock = _wheel get "lockControl";
    if (_lock isNotEqualTo "" && {!(_lock in _ctrlIndex)}) then {
        [format ["[BMKHS] WHEELS CONFIG ERROR: %1 lockControl '%2' is not a declared control. It never pins.", _name, _lock]]
            call bmkhs_fnc_utilConfigError;
        _wheel set ["lockControl", ""];
    };

    _wheels pushBack _wheel;
} forEach ("true" configClasses _cfg);

//The static load split and the wheel plane, about the wheels' centroid
private _loadFit = [];
private _normal  = [];
if (_wheels isNotEqualTo []) then {
    private _bottoms = _wheels apply { (_x get "position") vectorAdd [0.0, 0.0, -(_x get "radius")] };
    _normal = [_bottoms] call bmkhs_fnc_suspensionPlane;

    private _n   = count _wheels;
    private _mid = [0.0, 0.0, 0.0];
    { _mid = _mid vectorAdd (_x get "position") } forEach _wheels;
    _mid = _mid vectorMultiply (1.0 / _n);
    private _sxx = 0.0;
    private _sxy = 0.0;
    private _syy = 0.0;
    {
        ((_x get "position") vectorDiff _mid) params ["_px", "_py"];
        _sxx = _sxx + (_px * _px);
        _sxy = _sxy + (_px * _py);
        _syy = _syy + (_py * _py);
    } forEach _wheels;
    private _det = (_sxx * _syy) - (_sxy * _sxy);

    if (_n < 3 || {_normal isEqualTo []} || {abs _det < 0.000001}) then {
        ["[BMKHS] WHEELS CONFIG ERROR: fewer than three wheels, or all in a line. The suspension is off."]
            call bmkhs_fnc_utilConfigError;
        _wheels = [];
    } else {
        _loadFit = [_mid select 0, _mid select 1, _syy / _det, -_sxy / _det, _sxx / _det];
    };
};

//The squat switch: the wheel named by weightOnWheels, or -1 for any wheel
private _wow = -1;
if (_wheels isNotEqualTo [] && {(_settings get "weightOnWheels") isNotEqualTo ""}) then {
    _wow = _wheels findIf {(_x get "name") == (_settings get "weightOnWheels")};
    if (_wow < 0) then {
        [format ["[BMKHS] WHEELS CONFIG ERROR: weightOnWheels '%1' names no wheel. Any wheel on the ground counts.",
            _settings get "weightOnWheels"]] call bmkhs_fnc_utilConfigError;
    };
};

_heli setVariable ["bmkhs_numWheels",           count _wheels];
_heli setVariable ["bmkhs_wheels",              _wheels];
_heli setVariable ["bmkhs_wheelWow",            _wow];
_heli setVariable ["bmkhs_wheelSettings",       _settings];
_heli setVariable ["bmkhs_wheelLoadFit",        _loadFit];
_heli setVariable ["bmkhs_wheelPlaneNormal",    _normal];

//Per wheel. Compression is -1 off the ground, and rides in the net state.
_heli setVariable ["bmkhs_wheelCompression",    _wheels apply {-1.0}];
_heli setVariable ["bmkhs_wheelGripBend",       _wheels apply {[0.0, 0.0]}];
_heli setVariable ["bmkhs_wheelCastorAngle",    _wheels apply {0.0}];
_heli setVariable ["bmkhs_wheelCastorAnimated", _wheels apply {0.0}];
_heli setVariable ["bmkhs_wheelStrokeAnimated", _wheels apply {-1.0}];

//The aircraft. All local: only the owner runs the wheels, and a new owner starts from clean.
_heli setVariable ["bmkhs_suspHeldPose",        []];
_heli setVariable ["bmkhs_suspRestTime",        0.0];
_heli setVariable ["bmkhs_suspPlaced",          false];
_heli setVariable ["bmkhs_suspLoadSteady",      -1.0];
_heli setVariable ["bmkhs_suspLoadUntil",       0.0];
_heli setVariable ["bmkhs_suspSleepMass",       0.0];
_heli setVariable ["bmkhs_suspHeldLogged",      -10.0];
