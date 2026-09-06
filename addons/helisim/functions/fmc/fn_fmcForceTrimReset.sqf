/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_fmcForceTrimReset

Description:
    Force trim panic button. Zeroes the trim references and the accumulated
    keyboard input so the controls return to centre.

Parameters:
    _heli - The helicopter [Object]

Returns:
    Nothing
---------------------------------------------------------------------------- */
params ["_heli"];

//Force-trim reference positions
_heli setVariable ["bmkhs_forceTrimPosPitch", 0.0, true];
_heli setVariable ["bmkhs_forceTrimPosRoll",  0.0, true];
_heli setVariable ["bmkhs_forceTrimPosYaw",   0.0, true];
//Keyboard sticky input accumulated values
_heli setVariable ["bmkhs_cyclicPitchValue",     0.0];
_heli setVariable ["bmkhs_cyclicRollValue",      0.0];
_heli setVariable ["bmkhs_pedalYawValue",        0.0];
//prev* shadow values used by the sticky-interrupt branch of fn_getInput
_heli setVariable ["bmkhs_prevCyclicPitchValue", 0.0];
_heli setVariable ["bmkhs_prevCyclicRollValue",  0.0];
_heli setVariable ["bmkhs_prevPedalYawValue",    0.0];
