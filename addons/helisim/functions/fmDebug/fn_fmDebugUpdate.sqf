/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmDebugUpdate

Description:
    Draws the flight model forces and moments readout.

    Every contributor publishes a row into bmkhs_dbgForces during its own update:

        [_name, _force, _arm]

    where _force and _arm are model-space vectors and _arm is already measured
    from the centre of mass. This reads that list, works out each row's moment,
    totals them, and prints the lot. Nothing here computes physics - if a number
    looks wrong the contributor is what to go and look at.

    The yaw column is what the pedal has to balance, so the total at the bottom
    is the number that should sit near zero in trimmed flight.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli"];

if !(_heli getVariable ["bmkhs_initialised", false]) exitWith {};

//The CBA setting owns the window - no separate toggle. It follows the setting on
//and off, and closes itself when the player is not the one flying.
private _display = uiNamespace getVariable ["bmkhs_fmdebug", displayNull];
private _wanted  = bmkhs_forcesDebug && {driver _heli == player || gunner _heli == player};

if (!_wanted) exitWith {
    if !(isNull _display) then {
        ("bmkhs_fmdebug" call BIS_fnc_rscLayer) cutText ["", "PLAIN", 0, false];
        uiNamespace setVariable ["bmkhs_fmdebug", displayNull];
    };
    _heli setVariable ["bmkhs_dbgForces", []];
};

if (isNull _display) exitWith {
    //Opened this frame; the display is not there to write to until the next one.
    ("bmkhs_fmdebug" call BIS_fnc_rscLayer) cutRsc ["bmkhs_fmdebug", "PLAIN", 0, false];
    _heli setVariable ["bmkhs_dbgForces", []];
};

private _ctrl = _display displayCtrl 5203;
if (isNull _ctrl) exitWith {};

private _rows = _heli getVariable ["bmkhs_dbgForces", []];

private _pad = {
    params ["_s", "_w"];
    _s = str _s;
    while {count _s < _w} do { _s = _s + " " };
    _s
};
private _num = {
    params ["_v", "_w"];
    [_v toFixed 1, _w] call _pad
};

//Merge rows that share a name, so a surface built from many panels or elements
//reads as the one generator it is. The main rotor's four patches are named
//separately on purpose - they act at genuinely different points and the cyclic
//differential between them is the thing worth seeing.
//
//Forces add. The arm is the force weighted centroid, which is where the summed
//force has to act to make the same moment the individual panels made between them.
//A panel contributing no force cannot move that centroid, so it drops out rather
//than dragging the average toward itself.
private _merged = [];
private _order  = [];
{
    _x params ["_name", "_force", "_arm", ["_couple", []]];

    private _at = _order find _name;
    if (_at < 0) then {
        _order pushBack _name;
        _merged pushBack [_name, [0,0,0], [0,0,0], [0,0,0], 0];
        _at = (count _merged) - 1;
    };

    (_merged select _at) params ["_accName", "_fAcc", "_aAcc", "_cAcc", "_wAcc"];

    if (_couple isNotEqualTo []) then {
        //Couples have no arm; they just add.
        _merged set [_at, [_name, _fAcc, _aAcc, _cAcc vectorAdd _couple, _wAcc]];
    } else {
        //A contributor mid-rewrite can hand this a nil or a NaN. The readout is a
        //readout - it reports what it got and carries on, rather than taking the
        //frame down with it.
        private _w = vectorMagnitude _force;
        if (!(_w isEqualType 0) || {!(finite _w)}) then { _w = 0 };

        _merged set [_at, [
            _name,
            _fAcc vectorAdd _force,
            _aAcc vectorAdd (_arm vectorMultiply _w),
            _cAcc,
            _wAcc + _w
        ]];
    };
} forEach _rows;

//TEMP - CSV trace to the RPT, one row per merged component plus the state that
//produced it. RTR_LOG = true to start, false to stop.
if (!isNil "RTR_LOG" && {RTR_LOG}) then {
    private _vel = _heli getVariable ["bmkhs_velModelSpace", [0,0,0]];
    (_heli call BIS_fnc_getPitchBank) params ["_pitchDeg", "_bankDeg"];

    private _state = format ["%1,%2,%3,%4,%5,%6,%7,%8,%9",
        CBA_missionTime toFixed 3,
        (_vel select 0) toFixed 2, (_vel select 1) toFixed 2, (_vel select 2) toFixed 2,
        _pitchDeg toFixed 2, _bankDeg toFixed 2,
        (_heli getVariable ["bmkhs_collectiveOutput", 0]) toFixed 3,
        (_heli getVariable ["bmkhs_cyclicFwdAft", 0])     toFixed 3,
        (_heli getVariable ["bmkhs_cyclicLeftRight", 0])  toFixed 3];

    {
        _x params ["_n", "_f", "_a", ["_c", []], ["_w", 0]];
        if (_w > 0) then { _a = _a vectorMultiply (1.0 / _w) };

        //A couple has no force or arm - its three numbers are the moment, so they
        //go in the same columns rather than logging a row of zeros.
        if (_c isNotEqualTo [] && {vectorMagnitude _c > 0}) then {
            _f = _c;
            _a = [0,0,0];
        };

        diag_log text format ["RTRLOG,%1,%2,%3,%4,%5,%6,%7,%8",
            _state, _n,
            (_f select 0) toFixed 1, (_f select 1) toFixed 1, (_f select 2) toFixed 1,
            (_a select 0) toFixed 2, (_a select 1) toFixed 2, (_a select 2) toFixed 2];
    } forEach _merged;
};

private _txt = "<t size='0.8' font='EtelkaMonospacePro'>";
_txt = _txt + "<t color='#88ff88'>" + (["component", 14] call _pad)
            + (["Fx", 10] call _pad) + (["Fy", 10] call _pad) + (["Fz", 10] call _pad)
            + (["armX", 8] call _pad) + (["armY", 8] call _pad) + (["armZ", 8] call _pad)
            + "</t><br/>";

private _totF = [0, 0, 0];

{
    //Force and arm exactly as the contributor handed them to addForce. The moment
    //is PhysX's to work out from those two - deriving it here would be a second
    //opinion, and a readout that disagrees with the aircraft is worse than none.
    //The rotor reaction is the exception: it is a real addTorque, so it prints as
    //a moment because that is what it is.
    _x params ["_name", "_force", "_arm", ["_couple", []], ["_w", 0]];

    //Undo the weighting to get the centroid back.
    if (_w > 0) then { _arm = _arm vectorMultiply (1.0 / _w) };

    if (_couple isNotEqualTo [] && {vectorMagnitude _couple > 0}) then {
        _txt = _txt + ([_name, 14] call _pad)
                   + "<t color='#ffcc66'>couple  "
                   + ([_couple select 0, 11] call _num)
                   + ([_couple select 1, 11] call _num)
                   + ([_couple select 2, 11] call _num)
                   + "</t><br/>";
    } else {
        _totF = _totF vectorAdd _force;

        _txt = _txt + ([_name, 14] call _pad)
                   + ([_force select 0, 10] call _num)
                   + ([_force select 1, 10] call _num)
                   + ([_force select 2, 10] call _num)
                   + ([_arm select 0, 8] call _num)
                   + ([_arm select 1, 8] call _num)
                   + ([_arm select 2, 8] call _num)
                   + "<br/>";
    };
} forEach _merged;

_txt = _txt + "<t color='#88ff88'>" + (["TOTAL force", 14] call _pad)
            + ([_totF select 0, 10] call _num)
            + ([_totF select 1, 10] call _num)
            + ([_totF select 2, 10] call _num)
            + "</t><br/><br/>";

//State, so a reading can be tied to the condition that produced it.
//velModelSpace is what every force generator reads, so it is what belongs here -
//anything derived would be a different number from the one driving the physics.
//bmkhs_aero_beta_g is the trim ball signal - lateral acceleration, not the
//kinematic slip angle. That is the one that should read zero in balanced flight.
private _vel = _heli getVariable ["bmkhs_velModelSpace", [0,0,0]];
_txt = _txt + format ["vel [%1, %2, %3] m/s   coll %4   pedal %5   ball %6<br/>",
    ((_vel select 0) toFixed 2), ((_vel select 1) toFixed 2), ((_vel select 2) toFixed 2),
    ((_heli getVariable ["bmkhs_collectiveOutput", 0]) toFixed 3),
    ((_heli getVariable ["bmkhs_pedalLeftRight", 0]) toFixed 3),
    ((_heli getVariable ["bmkhs_aero_beta_g", 0]) toFixed 3)];

_ctrl ctrlSetStructuredText parseText (_txt + "</t>");

//Cleared here, at the end of the frame, so every contributor starts the next one
//with an empty list.
_heli setVariable ["bmkhs_dbgForces", []];
