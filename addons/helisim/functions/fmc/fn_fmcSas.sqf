params ["_heli", "_sas", "_on"];
#include "\bmkhs_helisim\functions\core\core.hpp"

if (!_on) exitWith {[0.0, 0.0, 0.0]};

private _pidSASPitch = _sas get "pitch";
private _pidSASRoll  = _sas get "roll";
private _pidSASYaw   = _sas get "yaw";
(_sas get "authority") params ["_authPitch", "_authRoll", "_authYaw"];

((_heli getVariable "bmkhs_angVelModelSpace"))
    params [
             "_angVelX"   // pitch rate (about model +X, right axis), rad/s
           , "_angVelY"   // roll rate  (about model +Y, fwd axis),   rad/s
           , "_angVelZ"   // yaw rate   (about model +Z, up axis),    rad/s
           ];

private _deltaTime      = _heli getVariable "bmkhs_deltaTime";
private _sasPitchOutput = 0.0;
private _sasRollOutput  = 0.0;
private _sasYawOutput   = 0.0;

//SCAS = STABILITY augmentation = proportional RATE DAMPING (a "shock absorber" on body rate).
//The COMMAND term was removed: the SAS servo gives a "speed of light" crisp pilot path
//(fn_actuator returns input un-lagged when SCAS is available), so the pilot's crisp input IS the
//command. SCAS's job is purely STABILITY - it continuously opposes body rate, PROPORTIONALLY and
//ALWAYS (not a threshold/limiter): setpoint = 0, so pidRun error = 0 - rate = -rate, and the
//output = kp * -rate opposes any rotation. This adds the artificial damping the airframe lacks
//(helicopters are under-damped, esp. the low-inertia roll) so it feels solid - stop commanding
//and the rate bleeds off fast. Not a maneuver limit; you just hold a bit more input to sustain a
//rate. Runs ALWAYS (incl. force-trim interrupt); holds no attitude/heading reference (that's the
//holds). Authority per axis is the aircraft's (FMC >> Sas >> authority[]); its gate[] and the FMC
//axis channels are applied in fn_fmc.

//SAS RUNS ON EVERY AXIS, ALWAYS - including when keyboard auto-attitude owns pitch and roll.
//
//An earlier version stood SAS down on those axes, on the reasoning that auto-attitude "owns" them.
//That was wrong and it made the auto-attitude loop oscillate violently: the two do DIFFERENT jobs
//and are complementary, not competing. SAS damps RATE (setpoint 0 on body rate, +-10-20% servo);
//auto-attitude commands ATTITUDE. An attitude loop with no rate damping underneath it has nothing
//opposing the overshoot it creates, so it hunts - and raising its gains to fix the sluggishness
//just made the hunt violent. Rate damping under an attitude loop is the standard arrangement and
//is exactly what lets the outer loop carry useful gain without ringing.
//
//SAS is also the cheaper of the two to leave running: it is a small, bounded, always-stabilising
//term that cannot fight an attitude command (it only ever opposes RATE, and a deliberate attitude
//change simply carries a little more input to sustain its rate).

//ROLL: proportional rate damping - oppose actual roll rate.
private _roll  = [_pidSASRoll, _deltaTime, 0.0, _angVelY] call bmkhs_fnc_pidRun;
_roll          = [_roll,  -_authRoll, _authRoll] call BIS_fnc_clamp;
_sasRollOutput = _roll;

//YAW: proportional rate damping - oppose actual yaw rate. (Heading Hold is a separate
//reference-hold submode on top, in fn_fmcHeadingHold; not built here.)
private _yaw   = [_pidSASYaw, _deltaTime, 0.0, _angVelZ] call bmkhs_fnc_pidRun;
_yaw           = [_yaw, -_authYaw, _authYaw] call BIS_fnc_clamp;
_sasYawOutput  = _yaw;

//PITCH: proportional rate damping - oppose actual pitch rate. Runs always (see the note above).
private _pitch = [_pidSASPitch, _deltaTime, 0.0, _angVelX] call bmkhs_fnc_pidRun;
_pitch         = [_pitch, -_authPitch, _authPitch] call BIS_fnc_clamp;
_sasPitchOutput = _pitch;

//systemChat format ["Pitch SAS = %1 -- Roll SAS = %2", _SASPitchOutput, _SASRollOutput];
//systemChat format ["_cyclicFwdAft = %1 -- _cyclicLeftRight = %2 -- _pedalLeftRight = %3", _heli getVariable "bmkhs_cyclicFwdAft" toFixed 2, _heli getVariable "bmkhs_cyclicLeftRight" toFixed 2, _heli getVariable "bmkhs_pedalLeftRight" toFixed 2];
//systemChat format ["_angVelX = %1 - _angVelY = %2 - _angVelZ = %3", _angVelX, _angVelY, _angVelZ];

[_sasPitchOutput, _sasRollOutput, _sasYawOutput];
