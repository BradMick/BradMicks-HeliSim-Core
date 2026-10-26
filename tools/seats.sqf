//Seat and pylon arms of the aircraft you are in. Debug console, LOCAL EXEC. Output to RPT and clipboard.
[] spawn {
    private _veh = vehicle player;
    if (_veh == player) exitWith { hint "Get in the aircraft first" };
    private _out = [];
    private _log = { _out pushBack _this; diag_log text ("BMKHSSEATS| " + _this) };
    private _bc  = boundingCenter _veh;
    private _fmt = { format ["{%1, %2, %3}", (_this select 0) toFixed 3, (_this select 1) toFixed 3, (_this select 2) toFixed 3] };

    private _grp  = createGroup [side group player, true];
    private _temp = [];
    {
        _x params ["_unit", "_role", "_cargoIndex", "_turret"];
        if (isNull _unit) then {
            private _u = _grp createUnit [typeOf player, [0, 0, 0], [], 0, "CAN_COLLIDE"];
            switch (toLower _role) do {
                case "driver": { _u moveInDriver _veh };
                case "cargo":  { _u moveInCargo [_veh, _cargoIndex] };
                default        { _u moveInTurret [_veh, _turret] };
            };
            _temp pushBack _u;
        };
    } forEach fullCrew [_veh, "", true];
    hint "Close the pause menu - the seats are measured 1 s after the game resumes";
    sleep 1;

    format ["=== %1 - boundingCenter %2", typeOf _veh, _bc call _fmt] call _log;
    {
        _x params ["_unit", "_role", "_cargoIndex", "_turret"];
        private _p = (_veh worldToModelVisual (_unit modelToWorldVisual (_unit selectionPosition "pelvis"))) vectorAdd _bc;
        format ["  seat role %1, turret %2, cargoIndex %3: arm %4", _role, _turret, _cargoIndex, _p call _fmt] call _log;
    } forEach fullCrew [_veh, "", true];
    {
        _x params ["_index", "_name", "", "_mag", "", "", "_posDir"];
        format ["  pylon %1 %2 (%3): arm %4", _index, _name, _mag, ((_posDir select 0) vectorAdd _bc) call _fmt] call _log;
    } forEach getAllPylonsInfo _veh;
    format ["  centre of mass %1", ((getCenterOfMass _veh) vectorAdd _bc) call _fmt] call _log;

    { deleteVehicle _x } forEach _temp;
    copyToClipboard (_out joinString endl);
    hint format ["%1: %2 seats measured - in the RPT and on the clipboard", typeOf _veh, count fullCrew [_veh, "", true]];
};
