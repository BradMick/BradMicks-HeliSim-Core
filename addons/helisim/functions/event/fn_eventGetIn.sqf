params ["_heli", "", "_unit"];

bmkhs_keyboardCollective         = true;
bmkhs_keyboardCollectivePrevious = true;

//The player getting in restarts the frame clock and holds input for one frame.
if (_unit != player) exitWith {};
bmkhs_lastFrameGetIn = true;
_heli setVariable ["bmkhs_previousTime", diag_tickTime];
