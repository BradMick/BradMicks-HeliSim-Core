# Building an aircraft on HeliSim

How to take a helicopter from nothing to a full flight model with modelled
systems. Follow it in order - each step depends on the ones before it.

**HeliSim Core (`bmkhs_helisim`) knows no airframe.** It provides functions and
reads declarations. Everything specific to your aircraft lives in your own
addon, which this guide calls the PACK. The AH-64's pack is
`fza_ah64_helisim`, and it is the worked example throughout.

Two field references sit beside this one and are the authority on what each
field means:

- `addons/bmkhs_helisim/components.hpp` - systems: producers, converters,
  storage, circuits, consumers
- `addons/bmkhs_helisim/controls.hpp` - switches, buttons and levers

This guide is the ORDER. Those are the DETAIL.

---

## Step 0 - What you need before starting

A flyable Arma helicopter: a p3d, a `CfgVehicles` class, and the memory points
the model needs. HeliSim replaces the flight model and adds systems; it does not
give you an aircraft.

Know these numbers about your airframe before you begin, because the flight
model needs them and guessing produces an aircraft that flies like nothing:

- Rotor radius, blade count, chord, twist, design RPM
- Empty mass and its centre of gravity, in model space
- Engine continuous and contingency power, design Ng/Np
- Transmission gear ratio, drivetrain torque ratings

---

## Step 1 - Create the pack

A new addon. The AH-64's is `addons/fza_ah64_helisim`; copy its shape.

    yourAircraft_helisim/
        $PBOPREFIX$
        addon.toml
        config.cpp
        version.hpp
        CfgFunctions.hpp
        XEH_preInit.sqf
        config/
            CfgEventHandlers.hpp
            cfgVehicles.hpp
            yourAircraft_config.hpp
            bmkhs_config/          <- the declarations, one file per domain
        functions/
            fn_perFrame.sqf
            fn_setup.sqf
        headers/
            bmkhs_controls.hpp     <- keybind rows, if you declare controls

### config.cpp

```cpp
class CfgPatches {
    class yourAircraft_helisim {
        units[] = {};
        weapons[] = {};
        requiredVersion = 2.10;
        requiredAddons[] = {"bmkhs_helisim"};
        //THE AIRFRAME THIS PACK DRIVES. XEH_preInit reads this to know what to
        //schedule, so a pack for another aircraft changes one line and no SQF.
        bmkhsBaseClass = "yourAircraftBase";
        #include "version.hpp"
    };
};

#include "CfgFunctions.hpp"
#include "config\CfgEventHandlers.hpp"
#include "config\CfgUserActions.hpp"    //only if you declare controls
#include "config\cfgVehicles.hpp"
```

`bmkhsBaseClass` is load-bearing: it is how the per-frame scheduler finds your
aircraft. Get it wrong and nothing runs, with no error.

---

## Step 2 - Wire up initialisation

**Core does not start itself, and neither should anything outside your pack.**
The pack starts itself from its own event handler, so nothing else has to know
HeliSim exists.

`config/CfgEventHandlers.hpp`:

```cpp
class Extended_PreInit_EventHandlers {
    class yourAircraft_helisim_preInit {
        init = "call compile preprocessFileLineNumbers 'yourAircraft_helisim\XEH_preInit.sqf';";
    };
};

//The pack starts itself. Nothing outside it calls in.
class Extended_Init_EventHandlers {
    class yourAircraftBase {
        class yourAircraft_helisim_init_eh {
            init = "_this call yourAircraft_helisim_fnc_setup";
        };
    };
};
```

`fn_setup.sqf`:

```sqf
params ["_heli"];

//Once per aircraft.
if (_heli getVariable ["bmkhs_initialised", false]) exitWith {};

//Aircraft equipment that is not flight model state. Set BEFORE coreConfig -
//fuelSet runs inside it and reads these with no default.
if (local _heli) then {
    _heli setVariable ["bmkhs_ctrTankInstalled", true, true];
};

[_heli] call bmkhs_fnc_coreInit;
[_heli, configOf _heli >> "BMKHS_HeliSim"] call bmkhs_fnc_coreConfig;
```

`coreInit` sets `bmkhs_initialised`, which everything downstream gates on - and
which the guard above uses so a second call cannot double-init.

**Anything `fn_setup` depends on, it must set itself.** It runs from an XEH init
with no guarantee about what else has run, so a variable `coreConfig` reads with
no default has to be written in the lines above the call, not by some other
addon's init.

### The per-frame scheduler

`XEH_preInit.sqf`, copied from the AH-64's:

```sqf
yourAircraft_helisim_frameHandler = addMissionEventHandler ["EachFrame", {
    {
        if (alive _x && {_x getVariable ["bmkhs_initialised", false]}) then {
            [_x] call yourAircraft_helisim_fnc_perFrame;
        };
    } forEach (vehicles select {
        private _veh = _x;
        local _veh && {bmkhs_packBaseClasses findIf {_veh isKindOf _x} > -1}
    });
}];
```

This runs for every LOCAL aircraft of your declared base class - AI included, so
an unoccupied aircraft still burns fuel and overtorques its gearboxes.

`fn_perFrame.sqf` calls Core in this order:

```sqf
[_heli] call bmkhs_fnc_coreUpdate;
[_heli] call bmkhs_fnc_systemsUpdate;
[_heli] call bmkhs_fnc_coreUpdateFlightModel;
[_heli] call bmkhs_fnc_ctrlVisUpdate;
[_heli] call bmkhs_fnc_repair;
```

---

## Step 3 - Declare the flight model

Create `config/yourAircraft_config.hpp` with a `class BMKHS_HeliSim`, and split
the declarations one file per domain under `bmkhs_config/`:

```cpp
class BMKHS_HeliSim {
    //Systems are ALL OR NOTHING. Start with 0 and fly it before turning it on.
    useSystems = 0;

    //Drivetrain ratings for useSystems = 0 only - worst first,
    //{fraction of rated torque, seconds it holds there, divisor}.
    xmsnTqLimits[]  = {{2.30, 0, 20}, {2.00, 6, 10}};
    ngbTqLimitsSE[] = {{1.25, 0, 40}, {1.22, 6, 20}, {1.10, 150, 10}};

    #include "bmkhs_config\helisim_airfoils.hpp"
    #include "bmkhs_config\helisim_engine.hpp"
    #include "bmkhs_config\helisim_flightControls.hpp"
    #include "bmkhs_config\helisim_fuel.hpp"
    #include "bmkhs_config\helisim_fuselage.hpp"
    #include "bmkhs_config\helisim_mass.hpp"
    #include "bmkhs_config\helisim_misc.hpp"
    #include "bmkhs_config\helisim_rotor.hpp"
    #include "bmkhs_config\helisim_simpleRotor.hpp"
    #include "bmkhs_config\helisim_wings.hpp"
};
```

Include it from your `cfgVehicles.hpp` inside the vehicle class.

**Get it flying before you go further.** Systems off, no components, no controls.
An aircraft that does not fly well will not fly better with hydraulics.

---

## Step 4 - Declare hitpoints

Hitpoints have to live in `class HitPoints` inside the vehicle class, where Arma
requires them - so they are separate from the component declarations.

**`damageRole` is the join between the two**, and it is what makes member count
automatic:

```cpp
BMKHS_HITPOINT(hit_elec_generator1, "hit_elec_generator1", ..., "generators", 0)
BMKHS_HITPOINT(hit_elec_generator2, "hit_elec_generator2", ..., "generators", 1)
```

Two hitpoints claiming `"generators"` means two generators. Add a third and you
have three, with no config change and no code change anywhere else.

Rules worth knowing before you write them:

- **A role nothing claims means the airframe does not have that component.**
  It is absent, not failed.
- **Declaring NO role is different** - present, but not separately damageable.
  The accumulator does this: no p3d selection, so it cannot be shot out.
- **Damage is read AT THE MEMBER'S INDEX.** Reading a role without one returns
  the WORST member, which would fail all three generators because one is
  destroyed.

---

## Step 5 - Declare components

`bmkhs_config/helisim_components.hpp`. This is where systems come from, and
`components.hpp` in Core is the field reference.

The six kinds:

| kind | is |
|---|---|
| Producer | puts a value onto a circuit, given whatever drives it |
| Converter | consumes from one circuit, produces onto another - creates nothing |
| Storage | a producer holding a charge, which drains and refills |
| Circuit | publishes whether a named node is up |
| Consumer | supplied if ANY of its circuits is up, or ALL with `needsAll` |

**Circuits are named nodes carrying a value** in whatever unit the domain uses -
psi, volts, Nr as a fraction. You choose the names; Core matches them as
strings. Several feeders on one node take the HIGHEST value rather than summing,
which is what makes a seamless handover work: the APU and a running engine both
put 1.0 on `PNEU`, so either holds it up.

Build it in dependency order and test as you go:

1. **Drive** - the transmission, driven by `Nr`, feeding an accessory circuit
2. **Electrical** - battery (Storage), generators (Producer), rectifiers
   (Converter), and the bus Circuits that report them
3. **Hydraulics** - reservoirs (Storage), pumps (Producer), accumulator
4. **APU** - one component with two outputs, drive and bleed air
5. **Consumers** - what stops working without supply

**A component is a physical thing.** What it puts out is not: an APU driving the
accessory section AND supplying bleed air is one component with two outputs.

Then flip `useSystems = 1` and cold-start it.

---

## Step 6 - Declare controls

`bmkhs_config/helisim_controls.hpp`, with `controls.hpp` in Core as the field
reference.

**A control is N POSITIONS, each with a value.** The index is canonical.

```cpp
class Controls {
    class BattSwitch {
        variableName = "battSwitch";
        rest         = 0;
        wraps        = 1;
        networked    = 1;
        class Positions {
            class Off { displayName = "Battery - Off"; value = 0; };
            class On  { displayName = "Battery - On";  value = 1; };
        };
    };
};
```

Each control publishes three variables: `bmkhs_<name>Idx`, `Val`, and `On`
(`Val != 0`).

**THE NAMING IS LOAD-BEARING.** `variableName = "battSwitch"` publishes
`bmkhs_battSwitchOn`, which is the name your Battery component gates on. Get it
wrong and the bus silently never comes up - no error, just a gate never
satisfied.

**Index 0 is the first class declared.** Order is declaration order, so
reordering positions renumbers everything.

### Consumers read, controls do not push

A control publishes a value and stops. Whatever cares reads it:

- A component **gates** on `bmkhs_apuBtnOn`
- The engine controller reads `bmkhs_eng1PwrLvrVal` each frame and moves
  `bmkhs_engState` itself

This is why Core needs no switch semantics. Do not look for a way to make a
switch "do" something - make the consumer read it.

### Interlocks

Declared per control, or per POSITION where only one is affected:

```cpp
class Fly { value = 1.0; inhibitedBy[] = {"bmkhs_rotorBrakeOn"}; };
```

The rotor brake blocks the power lever reaching FLY without blocking IDLE, so a
locked-rotor start still works. `enabledBy[]` is the opposite - all must hold.

**An inhibited control does not move.** It is a mechanical stop, not a veto on
the consequence.

**These are MECHANICAL interlocks, not electrical.** A switch is a piece of
metal and moves whether or not the bus is up - what stops an unpowered switch
doing anything is the gate on the component. There is deliberately no
`poweredBy`.

### Keybinds

Core ships the macros, your pack ships the rows - there is no config-time loop
in Arma's preprocessor, and a row naming your switch is airframe knowledge.

`headers/bmkhs_controls.hpp`, one row per position:

```cpp
#pragma hemtt suppress pw3_padded_arg file
BMKHS_CONTROL(battSwitch,p0,0,"Battery - Off") BMKHS_CONTROL_SEP()
BMKHS_CONTROL(battSwitch,p1,1,"Battery - On") BMKHS_CONTROL_SEP()
```

Two tokens for the position: `##` cannot paste a bare number into a class name,
so `ptok` is an identifier and `pnum` a number. They must agree - nothing checks.

Include that file TWICE from `config/CfgUserActions.hpp` - once to emit the
classes, once with the macros redefined to emit the group list. One data table,
two views, so the binds and the group cannot drift apart. Copy the AH-64's file.

**A dangling group entry fails SILENTLY** - a name in the group with no matching
class gives a keybind that appears in the menu and does nothing, with no build
error.

---

## Step 7 - Animation and audio

Core raises events through a single-slot handler. Your pack assigns it in
`XEH_preInit.sqf`:

```sqf
bmkhs_notifyHandler = {
    params ["_heli", "_event", ["_data", []]];
    switch (_event) do {
        case "controlMoved": {
            _data params ["_name", "_idx", "_prevIdx", "_value", "_posName"];
            //The control's VALUE is the animation phase. Nothing here restates
            //what a position is worth - move a detent in config and this follows.
        };
    };
};
```

**It is one slot, not a bus.** A second assignment replaces the first.

---

## Step 8 - Verify

**`hemtt check` after every config edit.** It is fast and catches macro and
config errors that otherwise ship silently.

Then in game, in this order - stop at the first failure rather than pressing on:

1. **Cold and dark.** Battery on, bus up.
2. **APU.** Button, spool, accessory drive, pumps, generators.
3. **Engines.** Start switch, both engines to idle, then fly.
4. **Interlocks.** Confirm each one actually blocks.
5. **Damage.** Shoot a component and confirm what it takes with it.
6. **Walk cost.** `fn_systemsDebug` shows `walk N peak N` - near zero on a
   settled aircraft, spiking only when something changes.

### Multiplayer

**The solve runs where the aircraft is local; everyone else reads published
results.** Anything a crew station displays or acts on needs `networked = 1`.

This fails SILENTLY in singleplayer, which looks perfect either way. If a value
is read outside the flight model, network it.

---

## Things that will catch you

Learned the hard way; all of them are in `SYSTEMS_REDESIGN.md` with more detail.

**`bmkhs_fnc_utilUpdateNetworkGlobal` throws on a variable that has never been
set.** It reads with no default. Seed with a plain `setVariable` before the first
networked publish.

**Absent is not failed.** Undeclared circuits publish NOTHING rather than zero,
so the read-side defaults that keep a no-hydraulics airframe flying still fire.

**A store must compare against OTHER sources, not the whole node.** Otherwise it
reads its own supply back, decides it is covered, and cuts its feed.

**Rates are defined over a RANGE.** A recharge that spans 0..1 finishes in half
its configured time when the store only moves through the band above its floor.

**`_x` is rebound by every inner `forEach`.** Capture the outer one first. This
defect has recurred repeatedly in this codebase.

**A gate that reads a variable published later in the same solve is a frame
stale**, and that can deadlock a start. Gates can name a circuit instead, which
reads the live value.
