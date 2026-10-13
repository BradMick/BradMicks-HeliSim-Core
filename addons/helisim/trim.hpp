#ifndef BMKHS_HELISIM_TRIM_HPP
#define BMKHS_HELISIM_TRIM_HPP

//BMKHS trim - where the controls sit with no force on them.
//
//Declared in class Trim, beside class FMC in helisim_flightControls.hpp. It needs no FMC. No
//class Trim is force trim with no beep.
//
//    class Trim {
//        forceTrim = 1;
//        class Beep {...};
//    };
//
//  forceTrim    1 = force trim / centering device: the force trim button sets the trim
//               position to where the controls are. 0 = none - the trim position stays 0. 1
//
//The force trim switch is bmkhs_fmcTrimOn (bmkhs_fnc_fmcSetChannel, "trim"). Off, the trim
//positions are held at 0 and the force trim button and beep do nothing; back on, the trim is
//set to where the controls are. A casual assist that owns an axis keeps writing it.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// Beep - trim moved by a switch, at a rate while held
/////////////////////////////////////////////////////////////////////////////////////////////
//
//Undeclared, the aircraft has no beep and the keys do nothing.
//
//  gate[]       when it works - the trim actuators' power. Component gate form, as the FMC
//  rate[]       {pitch, roll} trim travel per second held, control fraction. {0.10, 0.10}
//  lag          s from press to movement. 0.1
//  moves[]      {pitch, roll}: "stick" moves the trim position. "reference" (the hold
//               reference, stick still) is not built yet. {"stick", "stick"}
//
//Beep does nothing on an axis a casual assist owns, or with a springless cyclic or sticky
//keyboard on that axis - the same as the force trim button.
//
//Inputs, through bmkhs_fnc_inputControlHandle:
//    bmkhs_beepTrimForward / Backward / Left / Right    held
//    bmkhs_forceTrim                                    held, release sets the trim
//    bmkhs_forceTrimPanic                               zero the trim and keyboard input
//
//Published: bmkhs_forceTrimPosPitch / Roll / Yaw (-1..1, + = forward / left / right, in the
//packed state), bmkhs_trimOn (force trim working, local).

#endif
