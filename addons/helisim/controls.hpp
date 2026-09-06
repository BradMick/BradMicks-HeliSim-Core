#ifndef BMKHS_HELISIM_CONTROLS_HPP
#define BMKHS_HELISIM_CONTROLS_HPP

//BMKHS control model - the switches, knobs and levers an aircraft declares.
//
//A SWITCH IS NOTHING MORE THAN A GATE. The APU does not care that it is an APU switch,
//only that the signal arrived - so Core moves an index and publishes a value, and
//whatever gates on that value reacts. Core learns no switch semantics and knows no
//airframe, exactly as with components.
//
//Scope is the aircraft's own controls - what drives the systems model and the flight
//model. Not avionics: MPD bezels, sight select and weapon actions are the aircraft's,
//not HeliSim's. The test is whether it feeds a gate or the flight model.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// A CONTROL IS N POSITIONS
/////////////////////////////////////////////////////////////////////////////////////////////
//
//Not a boolean. A control has N POSITIONS declared in order, each with its own value, and
//THE INDEX IS CANONICAL - Core tracks an index and nothing else. Position class names are
//display labels for the bindings menu; Core never interprets them.
//
//    class Positions {
//        class Oride { displayName = "Engine 1 Start - Ignition Override"; value = -1; springsBack = 1; };
//        class Off   { displayName = "Engine 1 Start - Off";               value =  0; };
//        class Start { displayName = "Engine 1 Start - Start";             value =  1; springsBack = 1; };
//    };
//
//INDEX 0 IS THE FIRST CLASS DECLARED. Order is declaration order, so reordering the
//classes renumbers every position - and anything mapping a name to an index moves with it.
//
//That one model covers every behaviour a cockpit needs, with nothing left over:
//
//    momentary        2 positions, one springsBack
//    latching         2 positions, neither springs
//    one-way          3 positions, one springs
//    multi-position   N positions, none spring
//    detented lever   N positions, none spring
//    continuous       steps = 0, value interpolated rather than snapped
//    rotary           N positions, wraps = 1
//
//GUARD COVERS ARE DELIBERATELY NOT MODELLED. A physical guard - the jettison button's lid -
//would need a keybind of its own just to lift it before the switch underneath could be
//thrown. That is a bind spent on ceremony, and every other position here is one the player
//wants to reach directly. A guarded switch is declared as the switch it is. This is a
//decision, not an oversight; do not add it.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// WHAT A CONTROL PUBLISHES
/////////////////////////////////////////////////////////////////////////////////////////////
//
//Three variables, all derived, none interpreted by Core:
//
//    bmkhs_<name>Idx   the position index - canonical
//    bmkhs_<name>Val   the current position's declared value
//    bmkhs_<name>On    Val != 0
//
//THE NAMING IS LOAD-BEARING. `On` is what keeps existing gates working untouched: a
//control with variableName = "battSwitch" publishes bmkhs_battSwitchOn, which is the name
//the electrical components already gate on. Get it wrong and the bus silently never comes
//up - no error, just a component whose gate is never satisfied.
//
//LEVELS, NOT EDGES. "Begin the start sequence" looks like it needs an edge, but nothing in
//the engine path depends on one - the engine transitions STARTING -> ON by testing whether
//Ng has crossed a threshold, and the start switch only compares engine state against the
//requested action. So a consumer reads a LEVEL - Val == 1 while the switch is held - and
//spring-back is what makes that level transient. This is also what leaves room for motoring
//later: making the position maintained instead of spring-loaded is a declaration change.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// FIELDS
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  variableName  what it publishes as. Core owns the bmkhs_ prefix and the Idx/Val/On
//                suffixes
//  rest          index the control sits at, and returns to when a position springs back.
//                This is also its value at spawn, so it must agree with what the systems
//                model expects a cold aircraft to look like
//  wraps         1 if stepping past the last position comes round to the first. A rotary
//                selector; also what a two-position switch needs to be toggled by a
//                caller that only knows how to step
//  steps         reserved; controls are quantised to their declared positions
//  enabledBy[]   interlocks that must ALL hold for the control to move
//  inhibitedBy[] interlocks, ANY of which blocks it
//  hysteresis    dead band around the current detent, so a lever resting on a boundary does
//                not chatter between two positions. Quantised controls only
//  networked     publish through the change-gated network helper. Same rule as components:
//                anything a crew station displays or acts on, or it is frozen for everyone
//                but the pilot and looks perfect in singleplayer
//
//  Positions:
//  displayName   what the player reads in the bindings menu. The designer's words; Core
//                passes them through untouched
//  value         what this position publishes as Val
//  springsBack   1 if releasing returns the control to rest
//
/////////////////////////////////////////////////////////////////////////////////////////////
// INTERLOCKS - the gate form, reused
/////////////////////////////////////////////////////////////////////////////////////////////
//
//A control that cannot be thrown right now says so exactly as every component does - a
//variable name, or {circuit, threshold} read live:
//
//    enabledBy[]   = {{"BATT", 0.25}};        //all must hold
//    inhibitedBy[] = {"bmkhs_rotorBrakeOn"};  //any blocks it
//
//AN INHIBITED CONTROL DOES NOT MOVE, AND DOES NOT SPRING. A rotor brake set while the
//lever is at FLY leaves the lever at FLY - the interlock stops the throw, not the state.
//
//These are MECHANICAL interlocks, not electrical ones. See below.
//
//INTERLOCKS ARE PER-CONTROL, NOT PER-POSITION, and some switches want the finer grain. The
//start switch is the case: a locked-rotor start is a real procedure, so the rotor brake
//blocks START while ABORTING must always be available - a pilot who hits trouble mid-start
//has to be able to override whatever else is set. One inhibitedBy on the control would
//block the abort with everything else.
//
//Where a control needs that, the interlock belongs with whatever knows the difference
//rather than on the control. Per-position interlocks are not modelled; add them only if a
//second airframe needs them, rather than for this one case.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// CONTROLS AND POWER - there is no poweredBy, and that is deliberate
/////////////////////////////////////////////////////////////////////////////////////////////
//
//A SWITCH IS A PIECE OF METAL. It moves whether or not the bus is up. What stops an
//unpowered switch doing anything is the gate that is already on the component:
//
//    class APU { gate[] = {"bmkhs_apuBtnOn", {"BATT", 0.25}}; };
//
//Press that button on a dead aircraft: the switch throws, apuBtnOn publishes true, the walk
//wakes the APU - and the APU's OWN gate refuses, because BATT is down. Power-awareness is a
//property of the COMPONENT, not the control, and it needs nothing new. The battery switch
//creating the power everything else needs falls out for free: it gates on nothing.
//
//So do not reach for enabledBy to mean "needs power". A real start switch moves whether or
//not it is energised; enabledBy is the stronger claim that the switch physically cannot be
//thrown, which is a rotor brake locking a power lever, not a dead bus.
//
//The consequence to know: a switch thrown with no power still publishes its variable, so
//anything gating ONLY on that variable and never on a circuit will act on it. That is a
//missing gate in the component declaration, not a gap in the control layer.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// CORE IS NOT THE ONLY WRITER
/////////////////////////////////////////////////////////////////////////////////////////////
//
//Something outside HeliSim may force a switch - a fire handle pulling the APU button off.
//So a control publishes only when its own index MOVES, and reconciles when it finds its
//variable disagreeing: it adopts the external value and moves its index to match, which
//keeps the external actor authoritative without Core knowing what it is.
//
//Reconciling compares `On`, which is a boolean and therefore LOSSY. A control that anything
//may force off SHOULD DECLARE A ZERO-VALUED POSITION - with none, there is nothing to adopt,
//so Core leaves the index alone, leaves the external value standing, and reports the
//disagreement in the systems debug rather than inventing a position you never declared.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// KEYBINDS - Core ships the macro, the aircraft ships the rows
/////////////////////////////////////////////////////////////////////////////////////////////
//
//EVERY POSITION IS INDIVIDUALLY BINDABLE, so a player binds the position they want instead
//of cycling a switch to reach it. Keybinds are config-time and static while position counts
//are declared, and Arma's preprocessor has no loop - so the aircraft writes one row per
//position, which is the side that knows how many there are.
//
//    BMKHS_CONTROL(eng1StartSw,p0,0,"Engine 1 Start - Ignition Override") BMKHS_CONTROL_SEP()
//    BMKHS_CONTROL(eng1StartSw,p1,1,"Engine 1 Start - Off")               BMKHS_CONTROL_SEP()
//    BMKHS_CONTROL(eng1StartSw,p2,2,"Engine 1 Start - Start")             BMKHS_CONTROL_SEP()
//
//
//Two tokens for the position, not one: `##` cannot paste a bare number into a class name,
//so ptok is an identifier for the class name and pnum a number for the dispatch. They must
//agree - nothing checks them.
//
//That row table is INCLUDED TWICE - once inside CfgUserActions to emit the classes, then
//with the macros redefined inside a group[] array to emit the group list. One data table,
//two views, so the binds and the group cannot drift apart.
//
//A DANGLING GROUP ENTRY FAILS SILENTLY - a name in the group with no matching class gives a
//keybind that appears in the menu and does nothing, with no build error. Keep both macro
//definitions adjacent so a rename touches them together.
//
//Give cockpit controls their own group, separate from flight controls, and leave them OUT
//of the conflict groups: every position of every switch would otherwise be flagged as
//conflicting with every other, which is not what a conflict means.

#endif
