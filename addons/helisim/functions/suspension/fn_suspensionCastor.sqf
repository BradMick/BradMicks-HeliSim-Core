/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_suspensionCastor

Description:
    A castoring wheel's swivel. Pinned, it sits along the aircraft's long axis.
    Free, it trails its swivel axis and follows the direction it is moving. The
    pin only drops in once the wheel swings back near centre.

Parameters:
    _heli      - The helicopter [Object]
    _index     - Wheel index into bmkhs_wheels [Number]
    _onGround  - The wheel is on the ground [Bool]
    _deltaTime - Frame time, s [Number]

Returns:
    [swivel angle, deg clockwise from above, pin engaged]

Author:
    BradMick
---------------------------------------------------------------------------- */
#include "\bmkhs_helisim\functions\suspension\suspension.hpp"

params ["_heli", "_index", "_onGround", "_deltaTime"];

private _wheel    = (_heli getVariable "bmkhs_wheels") select _index;
private _settings = _heli getVariable "bmkhs_wheelSettings";
private _pos      = _wheel get "position";
private _lock     = _wheel get "lockControl";
private _angle    = (_heli getVariable "bmkhs_wheelCastorAngle") select _index;
private _locked   = _lock isNotEqualTo "" && {_heli getVariable format ["bmkhs_%1On", _lock]};
private _pinned   = _locked && {abs _angle <= (_settings get "castorPinAngle")};

if (_pinned) then {
    _angle = 0.0;
} else {
    if (_onGround) then {
        //Velocity at the wheel, as in bmkhs_fnc_suspensionWheel
        private _velModel = _heli vectorWorldToModel (velocity _heli);
        private _yawRate  = (_heli vectorWorldToModel (angularVelocity _heli)) select 2;
        private _velX     = (_velModel select 0) + (_yawRate * (_pos select 1));
        private _velY     = (_velModel select 1) - (_yawRate * (_pos select 0));
        private _speed    = vectorMagnitude [_velX, _velY, 0.0];

        //Closes on its direction of travel over one trail length rolled
        if (_speed > 0.01) then {
            private _error = (((_velX atan2 _velY) - _angle + 540.0) % 360.0) - 180.0;
            _angle = _angle + (_error * (1.0 - exp (-_speed * _deltaTime / (_settings get "castorTrail"))));
        };

        //A wheel standing still stays put while the aircraft turns over it
        _angle = _angle - (deg (_yawRate * _deltaTime));
        _angle = ((_angle + 540.0) % 360.0) - 180.0;
    };
};
[_heli, "bmkhs_wheelCastorAngle", _index, _angle] call bmkhs_fnc_utilSetArrayVariable;

//Only when it has visibly moved: animateSource goes to every machine
private _source = _wheel get "animationSource";
if (_source isNotEqualTo "" && {abs (_angle - ((_heli getVariable "bmkhs_wheelCastorAnimated") select _index)) > SUSP_CASTOR_ANIM_STEP}) then {
    [_heli, "bmkhs_wheelCastorAnimated", _index, _angle] call bmkhs_fnc_utilSetArrayVariable;
    _heli animateSource [_source, _angle, true];
};

[_angle, _pinned]
