# Building an aircraft on HeliSim

How to take a helicopter from nothing to a full flight model with modelled
systems. Follow it in order - each step depends on the ones before it.

**HeliSim Core (`bmkhs_helisim`) knows no airframe.** It provides functions and
reads declarations. Everything specific to your aircraft lives in your own
addon, which this guide calls the PACK. The AH-64's pack is
`fza_ah64_helisim`, and it is the worked example throughout.

Three field references sit beside this one and are the authority on what each
field means:

- `addons/helisim/components.hpp` - systems: producers, converters,
  storage, circuits, consumers
- `addons/helisim/controls.hpp` - switches, buttons and levers
- `addons/helisim/engine.hpp` - the gas turbine engine, station by station

This guide is the ORDER. Those are the DETAIL.

Those three cover what you DECLARE. What Core PUBLISHES back - every variable, event and
read function a pack can use - is the **Reference** at the end of this guide.

---

## Step 0 - What you need before starting

A flyable Arma helicopter: a p3d, a `CfgVehicles` class, and the memory points
the model needs. HeliSim replaces the flight model and adds systems; it does not
give you an aircraft.

Know these numbers about your airframe before you begin, because the flight
model needs them and guessing produces an aircraft that flies like nothing:

- Rotor radius, blade count, chord, twist, design RPM
- Empty mass and its centre of gravity, in model space
- Engine: power at 100% torque, compressor pressure ratio and airflow, power
  turbine RPM, Ng/Np/TGT limits, and the Maximum Torque Available charts
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

### Building your mod against Core's headers

**Your mod ships a committed copy of the Core headers it uses.** Anything in your
mod that writes `#include "\bmkhs_helisim\..."` is preprocessed at BUILD time, and
HEMTT resolves `\bmkhs_helisim\` from your project's `include\bmkhs_helisim\`
folder. So that folder must hold real files, committed to your repo - the same
arrangement mods already use for CBA and ACE headers (`include\x\cba`,
`include\z\ace`).

Done this way, a fresh clone builds with nothing but `hemtt build`: no submodule
to forget, no junction, no setup script, no network step. That is the point -
this is the arrangement with the fewest ways to fail for someone building your
mod for the first time.

**Do not junction or symlink it to a Core checkout.** It works on the one machine
that has the link and nowhere else: git cannot track files through a junction,
so a clone gets an empty folder and every Core include fails. That is precisely
how the AH-64's first outside build broke.

**Copy only the headers your mod includes, keeping Core's paths.** Find them
with a search for `\bmkhs_helisim\` across your mod. The AH-64 needs six:

| Core header | what it carries |
|---|---|
| `functions\systems\systems.hpp` | `SYS_*` damage and pressure thresholds |
| `functions\core\core.hpp` | unit conversions, speeds, mode constants |
| `functions\fuel\fuel.hpp` | fuel transfer and advisory thresholds |
| `fmOverride.hpp` | the flight model override block for your vehicle class |
| `hitPoints.hpp` | Core's hitpoint base |
| `controlMacros.hpp` | the keybind row macros, if you declare controls |

Core's field references (`components.hpp`, `controls.hpp`) are documentation, not
headers you include - they do not need copying.

**Put a README in the folder** naming the Core version the copy came from and
saying not to edit it. The AH-64's is `include\bmkhs_helisim\README.md`.

#### When Core's headers change

**The copy must match the Core mod loaded in game.** Headers are constants baked
into your PBO at build time, while Core's functions run from the Core mod at
runtime. A stale copy builds without a single warning and then misbehaves - a
threshold one side thinks is 0.85 and the other 0.9, or a macro that no longer
exists evaluating to nil in game.

Refresh the copy whenever you move your mod to a new Core version:

1. Copy the files from Core's `addons\helisim\` into your `include\bmkhs_helisim\`,
   keeping their paths. From your mod's root, in PowerShell:

   ```powershell
   $core = "<path to Core checkout>\addons\helisim"
   $dest = "include\bmkhs_helisim"
   "functions\systems\systems.hpp", "functions\core\core.hpp", "functions\fuel\fuel.hpp",
   "fmOverride.hpp", "hitPoints.hpp", "controlMacros.hpp" | ForEach-Object {
       New-Item -ItemType Directory -Force (Split-Path "$dest\$_") | Out-Null
       Copy-Item "$core\$_" "$dest\$_"
   }
   ```

2. Update the Core version in the folder's README.
3. `hemtt check`. A macro your mod uses that Core removed or renamed shows up
   here as an unresolved name - fix it now, not in game.
4. Commit the copy on its own, with the Core version in the message, so the
   history shows exactly when your mod moved to which Core.

**Never edit the copy.** A change belongs in Core, then comes back over with the
next refresh. An edit made only in the copy is silently lost the next time
anyone refreshes it.

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

class Extended_GetIn_EventHandlers {
    class yourAircraftBase {
        class yourAircraft_helisim_getin_eh {
            getIn = "_this call bmkhs_fnc_eventGetIn";
        };
    };
};
```

The GetIn handler restarts the aircraft's frame clock when the player climbs
in, so the first frame does not see the whole time the aircraft sat empty.

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

### The per-frame tick and your event handler

`XEH_preInit.sqf`, copied from the AH-64's:

```sqf
//This pack's own airframe, from its own CfgPatches entry - never another pack's.
yourAircraft_helisim_baseClass = getText (configFile >> "CfgPatches" >> "yourAircraft_helisim" >> "bmkhsBaseClass");

//Core's events for THIS pack's aircraft. Optional - leave it out and events are ignored.
[yourAircraft_helisim_baseClass, {
    params ["_heli", "_event", ["_data", []]];
    switch (_event) do {
        case "apuStateChanged": { /* your cockpit light */ };
    };
}] call bmkhs_fnc_utilNotifyRegister;

yourAircraft_helisim_frameHandler = addMissionEventHandler ["EachFrame", {
    {
        if (alive _x && {_x getVariable ["bmkhs_initialised", false]}) then {
            [_x] call yourAircraft_helisim_fnc_perFrame;
        };
    } forEach (vehicles select {local _x && {_x isKindOf yourAircraft_helisim_baseClass}});
}];
```

**Each aircraft runs on its own.** Every frame, the handler ticks each aircraft
of your base class that is local to this machine, one at a time - the one you
are flying, and any AI or empty ones this machine owns. Each keeps its own state;
nothing is shared between them. An aircraft owned by another machine is ticked
on that machine, not this one.

An empty aircraft is still ticked: it still burns fuel, and its engines and
gearboxes still take damage.

**Use your own base class and nothing else.** Every installed pack has its own
handler for its own aircraft type. If yours also ticked another pack's aircraft,
they would be ticked twice a frame and every force on them doubled.

**Register your event handler; never assign a global.** Core hands each event to
the handler registered for that aircraft's base class, so two packs never hear
each other's aircraft. A shared global would give every aircraft's events to
whichever pack loaded last.

`fn_perFrame.sqf` makes one call:

```sqf
params ["_heli"];

[_heli] call bmkhs_fnc_coreUpdate;
```

`coreUpdate` runs the whole frame for that aircraft. Do not call any other Core
function from here - it is already run, and a second call runs it twice. Work
of your own aircraft's goes after the call.

---

## Step 3 - Declare the flight model

Create `config/yourAircraft_config.hpp` with a `class BMKHS_HeliSim`, and split
the declarations one file per domain under `bmkhs_config/`:

```cpp
class BMKHS_HeliSim {
    //Systems are ALL OR NOTHING. Start with 0 and fly it before turning it on.
    useSystems = 0;

    //How many engines, when no hitpoints declare them (Step 4). With engine hitpoints,
    //the count comes from those and this is ignored.
    numEngines = 2;

    //The drivetrain is rated by each engine's tqLimits / tqLimitsSe in helisim_engine.hpp.

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

### The engines

`bmkhs_config/helisim_engine.hpp`, one `class EngineNN` per engine inside
`class Engines`. `engine.hpp` in Core is the field reference and carries a full
template; copy it rather than starting from nothing.

**The engine is physics worked from Ng, not a schedule.** You declare what a
data sheet gives you - pressure ratio, airflow, power, limits - and Core works
the compressor, combustor and both turbines every frame. Starts, idle, spool-up,
hot and cold days and altitude all come out of that. Core carries the T700-701C's
compressor map, normalised, and scales it by your engine's numbers. An engine
that is not a T700 can declare its own `Compressor >> compressorMap[]`; see
`engine.hpp`.

Set it up in this order:

1. **Spec numbers.** `pressureRatio`, `massFlow`, `powerKw`, `designRpm`,
   `npFly`, `maxNg`, `maxNp`, and the book limits.
2. **Idle.** Adjust `fuelIdle` until the engine, lever at IDLE, settles at
   `idleNg`.
3. **Start.** `startFuelBase` sets the peak TGT, `Starter >> torque` how quickly
   it lights, `runawayNg` where it motors with no fuel.
4. **Max torque available.** Fly or run max power at each FAT on your Maximum
   Torque Available chart and set that row of `airflowTable` - below 1.0 where
   the engine makes too much, above where too little. Leave the 15 C row at 1.0:
   a standard day is the untrimmed engine. The Ng and TGT limiters then decide
   which limit holds, as on the real engine.

**An engine that is not a T700 - build its compressor map.** If you have the
engine's limitations table (N1, T45 and torque for each rating, at sea level and
a stated temperature), Core's release ships a generator in `@bmkhs/python/tools/`.
It needs Python 3 and nothing else.

1. Fill in the spec numbers in your `helisim_engine.hpp` first (step 1 above).
2. Write a ratings file, one rating per line, fractions for N1 and torque:

       #name  FAT_C  N1     T45_C  torque
       MCP    5      0.950  894    0.940
       TOP    5      0.968  928    1.006
       MAX    5      0.987  962    1.085
       SUP    5      1.015  1036   1.339

3. Run it against your pack's config folder:

       python tools\compressor_map.py path\to\yourPack\addons\...\config\bmkhs_config ratings.txt

4. Paste the printed `compressorMap[] = {...};` into your engine's
   `class Compressor`, and set `fuelIdle` to the value it reports.

It also flies every rating in its own copy of the engine and prints what it
reached against what you asked for. Every line should land on its target.
Rows below your lowest rating (start, idle, flat pitch) come from Core's T700
map scaled to your pressure ratio, and rows above your highest rating carry on
along your last two. The checks fly your `helisim_simpleRotor.hpp`, and assume
two engines declared as `Engine01` and `Engine02`.

**`maxNg` is also the compressor map's scale.** The map's Ng axis runs 0 to 1 as
a fraction of it, so set it to the true mechanical maximum.

**`maxFuelFlow` is physical.** The combustor burns fuel units times
`maxFuelFlow` kg/s, so it sets how much heat a unit of fuel carries, not just
the gauge.

**The governor never pulls Ng below idle in flight.** A power-on autorotation
holds Ng at `idleNg` while the clutch releases and the rotor runs free - an
engine below `ngMin` is an engine out.

### The fuselage and wings - model them, do not type them

You do not write `helisim_fuselage.hpp` or `helisim_wings.hpp` by hand. You
model every aerodynamic surface as flat four-sided faces in a small model,
`fm.p3d`, and a generator in Core's release writes both files from it. The old
wing fields - `span`, `chord`, `sweep`, `twist`, `tipWidthScalar`, `pos`,
`pitch`, `roll`, `isStabilator` - are gone; the geometry is the model.

**What the surfaces do.** Each quad is a surface's outline; the airfoil it is
given supplies the section. Core works the airflow over it every frame and
applies the forces:

- **Fuselage** - three sets. Each top and side panel makes lift and drag from
  `fuselageAirfoil`, applied at the panel's centre. The front makes drag only,
  against forward airspeed, applied at the centre of mass.
- **Wings** - wings, fins and stabilisers. Each face is cut into strips along its
  span and every strip makes lift and drag from its airfoil at its own angle of
  attack, including the airflow from the aircraft rotating, applied at
  `chordLinePos` on the strip.

#### 1. Make fm.p3d

Create a new model in Object Builder and save it as `fm.p3d` in your pack's
addon folder, beside your `config.cpp` (the Tiger's is
`addons/helisim/fm.p3d`). Keep it there: it is the source for every regeneration.
It is never loaded or referenced in game - it is only read by the tool.

- **Save it unbinarized** - the normal Object Builder save. The tool cannot read
  a binarized p3d. Your build packs it into the PBO like any other file in the
  folder; that is harmless, nothing uses the packed copy.
- **One LOD.** Everything goes in the first resolution LOD, `0.000`. The tool
  reads only that LOD. No Memory LOD, no named points.
- **Same space as your aircraft.** Build the faces over your aircraft's model -
  in your modeller of choice, from the model or from blueprints - and import them
  into fm.p3d, so they sit on the airframe in Object Builder where they act. The
  tool writes the coordinates exactly as they are in fm.p3d, and Core moves them
  into Arma's frame by your model's `boundingCenter` when the aircraft starts.
  Copying your aircraft's model into the LOD as a reference is the easiest way to
  check; delete it before running the tool, because every named selection left in
  the LOD must be a surface.

#### 2. Model each surface as quads

Every face must be a **quad** - exactly four vertices. Triangles and faces with
more corners are rejected.

**A quad is the surface's outline, not its shape.** Do not model an airfoil
section, camber or thickness - the section comes from the `airfoil` the surface
is given (section 6), and Core works its lift and drag from that. Model the
quad's planform, place it where the surface is, and set its angle:

- **Incidence and dihedral** - tilt the quad.
- **Taper and sweep** - shape the outline.
- **Twist (washout)** - twist the quad: move the tip's corners so it is no
  longer flat. Each strip of the quad (`numElements`) takes its incidence from
  its own part of the leading and trailing edges, so the twist carries along
  the span.

A surface can have more than one quad where its outline changes along the span
- the Tiger's wings are two each, inboard and outboard. Each quad is cut into
`numElements` strips of its own.

Give **every quad its own named selection**: select the face (its four vertices
come with it) and name the selection. The selection must hold that one face and
its four vertices and nothing else.

The selection's name is the surface it belongs to, from this list, exactly as
written (it is case-sensitive):

| Name | Kind | What Core does with it |
|---|---|---|
| `fuselageTop` | Fuselage | Lift and drag across the top of the body |
| `fuselageSide` | Fuselage | Lift and drag across the side - the weathervane |
| `fuselageFront` | Fuselage | Drag against airspeed |
| `leftWing`, `rightWing` | Wing | Lift and drag |
| `horizontalStabilizer` | Wing | Lift and drag; **fixed** |
| `stabilator` | Wing | Lift and drag; **moves** - Core schedules its incidence from `heliSimStabTable`, it is damaged through the `"stabilator"` damage role, and it animates `Hstab` |
| `verticalFin` | Wing | Lift and drag - the side force |
| `leftVerticalFin`, `rightVerticalFin` | Wing | Lift and drag - end-plate fins |

**Number the quads of a surface** - `fuselageSide01`, `fuselageSide02`, ...
`fuselageSide14`. The number only keeps the selections apart; all of a
surface's quads become one surface. A surface with one quad needs no number
(`verticalFin`).

**All three fuselage surfaces are required.** Wings are optional - declare only
the ones your aircraft has, or none.

#### 3. Point each face the way its force acts

Every face points one way - the order its corners run in decides which. That
direction is its **facing**. You do not type it; the tool reads it from the
face, and reversing the face in Object Builder reverses it.

**Each face's lift is reckoned from the face itself**: Core takes the normal of
every quad, every fuselage panel included, from its corners, and `facing` only
says which side of it is the front. A quad you tilt 5° acts 5° tilted.

The tool prints each surface's facing when it runs (section 5) as one of `up`,
`down`, `left`, `right`, `forward`, `backward` - the axis the face points along
most. **Check that list**: a surface showing the wrong direction is a face to
reverse.

- **Wings, stabilisers:** point them where their lift goes. A wing lifts `up`.
  A horizontal stabiliser that holds the tail down (a cambered section mounted
  upside down) faces `down`.
- **Fins:** point them to the side their cambered side faces.
- **Fuselage:** `fuselageTop` must face `up` or `down`, `fuselageSide` `left` or
  `right`, `fuselageFront` `forward` or `backward`. The fuselage's section is
  symmetric, so which of the pair makes no difference to the force.

**Every quad of a surface should face the same way.** If most of them agree and
a few face exactly the opposite way, the tool turns those few to match and
tells you which - `fuselageSide04 was flipped to conform to its neighbours`. Fix
them in the model when you next touch it. If there is no majority (two up, two
down) the tool stops.

#### 4. Corner order does not matter

Place the four corners in any order. The tool sorts them itself: it winds them
around the face and starts each quad at its **leading edge** - of the two edges
running across the airflow, the one further forward. The leading edge is what
the chord line is measured from, where the force acts (`chordLinePos`), and the
hinge a stabilator turns about.

#### 5. Run the generator

Core's release ships it in `@bmkhs/python/tools/`. It needs Python 3 and nothing
else. From `@bmkhs/python/`:

    python tools\fm_generateAeroSurfacePoints.py path\to\yourPack\addons\yourAddon\fm.p3d path\to\yourPack\addons\yourAddon\config\bmkhs_config

The first path is your `fm.p3d`; the second is the folder that holds your
`helisim_fuselage.hpp` and `helisim_wings.hpp`. It checks the whole model
first, then lists what it found:

      fuselageTop            up       5 quads
      fuselageSide           right    14 quads
      fuselageFront          forward  18 quads
      leftWing               up       2 quads
      ...

    All data will be reset to default. Continue? (y/n)

Answer `y` to write both files. Anything else writes nothing. **Both files are
rewritten in full every time** - geometry, facing, and every other value back
at its default. Any value you tuned by hand is lost: note your changes before
you regenerate, and put them back after.

If the model has a problem, the tool writes nothing and says what:

| Message | Fix |
|---|---|
| `you didn't name X correctly - it must be one of: ...` | Rename the selection to a name from the table, with or without a number |
| `X must be exactly one 4-sided face` | The selection holds a triangle, more than one face, or stray vertices - reselect just the quad |
| `quad facing mismatch, please ensure all quads are facing the same direction on surface X` | The surface's quads split evenly between two directions, or one faces along a different axis - reverse the wrong ones |
| `X faces Y - it must face ...` | A fuselage surface faces the wrong axis - reverse or remodel it |
| `the fuselage needs all three surfaces - X missing` | Model the missing fuselage surface |
| `... is not an unbinarized (MLOD) .p3d` | Point it at the Object Builder save, not a binarized copy |
| `... has no 0.000 LOD` | Put the quads in the first resolution LOD |

#### 6. What it writes

Both files are included from `class BMKHS_HeliSim` (see the top of this step).

**`helisim_fuselage.hpp`:**

| Field | Written as | What it is |
|---|---|---|
| `fuselageAirfoil` | `"NACA 0012"` | Section every fuselage panel uses for lift and drag, by name from `helisim_airfoils.hpp` |
| `class FuselagePanels` | three classes | One per fuselage surface |
| ... `name` | the surface | `fuselageTop`, `fuselageSide`, `fuselageFront` - Core finds each set by it |
| ... `facing` | from the model | See section 3 |
| ... `dragCoefTable[]` | `{altitude ft, CD}` rows | Drag coefficient against pressure altitude. Top and side: 0.200 at sea level rising to 0.750 at 8,000 ft; front: 0.800 rising to 3.000 |
| ... `panels[]` | from the model | Each quad's four corners `{right, forward, up}` in m, leading edge first |

**`helisim_wings.hpp`:**

| Field | Written as | What it is |
|---|---|---|
| `class Wings` | one class per surface | Core reads every class inside |
| ... `name` | the surface | `stabilator` is the one that moves |
| ... `facing` | from the model | See section 3 |
| ... `numElements` | `4` | Strips each quad is cut into along its span. More strips follow the airflow across a rolling or yawing surface more closely |
| ... `airfoil` | `"NACA 4418"`; `"NACA 0012"` for `stabilator` and `horizontalStabilizer` | Section, by name from `helisim_airfoils.hpp` |
| ... `chordLinePos` | `0.25` | Where along the chord the force acts, as a fraction back from the leading edge |
| ... `panels[]` | from the model | Each quad's four corners `{right, forward, up}` in m, leading edge first |
| `heliSimStabTable[]` | only with a `stabilator` | The stabilator's incidence, deg: rows are collective 0 to 1, columns 30 to 180 kts |

Tune `airfoil`, `numElements`, `chordLinePos`, the drag tables and the
stabilator schedule in the written files once the geometry is right. Remember
the next regeneration resets them.

#### 7. Check it in game

Turn on **Enable FM Debugging** (CBA settings, *BradMick's HeliSim* > *Testing*)
and look at the aircraft. Every quad is drawn where Core has it:

- The outline of each wing quad, its **leading edge in red** and the other
  three edges white. A red edge anywhere but the front is a quad the tool could
  not read the way you meant - check that surface in the model.
- Each wing strip's chord line in blue and its facing in white; the airflow in
  red; lift in green and drag in red, scaled.
- The fuselage quads outlined red and white.

The quads should lie on the airframe. If they sit off it, the faces in fm.p3d
do not overlay your aircraft's model in Object Builder - paste the model in and
compare.

### Rotor control map

A simple rotor reads its `liftCoefTable` and `dragCoefTable` by its control - collective for a
main rotor, pedal for a tail rotor. `controlMap[]` (optional) reshapes that control into the
table's key, so the table carries the MAGNITUDES and the map carries the SHAPE between them:

```cpp
controlMap[] = {             //{control, table key}
    {-1.0, -1.0},
    { 1.0,  1.0}
};
liftCoefTable[] = {
     {"A/S", 0.00,   10.29,  ...}
    ,{-1.00, 0.6293, 0.6917, ...}   //full left pedal
    ,{ 0.00, 0.0000, 0.0000, ...}   //centred
    ,{ 1.00,-0.0629,-0.0692, ...}   //full right pedal
};
```

A tail rotor set this way is three rows - left, mid, right - and a map, rather than a dense
table that has to encode the curve in its rows. Sparse rows WITHOUT a map interpolate in straight
segments, and a corner between two rows is felt in the pedals.

Set the magnitudes so hover trim sits where the pilot should hold it. Hover torque against
full-pedal authority decides it: full left at many times hover torque puts trim near centre,
however the map is shaped. `python/dev/forces.py` (the rig) solves hover trim at every collective
for a pack's config.

No map: the control is the key, as before. Cone angle, autorotation and control mixing read
the control itself, not the mapped key.

Outside REALISTIC Core sets every tail rotor upright - pitch 0, roll 90 - so a canted tail rotor
yaws without pitching or rolling the aircraft. Nothing to declare.

### Control mixing

For an airframe whose controls are mixed - the UH-60's mixing unit, compensating its canted
tail rotor - declare `class ControlMixing` beside the flight control gains. An airframe with
none declares nothing.

Each mix adds control travel to one rotor axis from one control's position:

```cpp
class ControlMixing {
    class YawToPitch {
        source  = "pedal";       //"collective" (0..1) or "pedal" (-1..1, + right)
        target  = "pitch";       //"pitch" (+ fwd), "roll" (+ left) or "yaw" (+ right pedal)
        table[] = {              //{source position, added travel}
            {-1.0, -0.060},
            { 1.0,  0.060}
        };
    };
    class CollectiveAirspeedToYaw {
        source     = "collective";
        target     = "yaw";
        table[]    = {
            {0.0, 0.000},
            {1.0, -0.050}
        };
        airspeed[] = {           //optional scale by airspeed, {knots, scale}
            {  0, 1.0},
            { 40, 1.0},
            {100, 0.0}
        };
        gate[]     = {"bmkhs_fmcYawOn", "bmkhs_dcBusOn"};
    };
};
```

**No gate means mechanical** - linkage, always applied. **A gate makes it electronic**: every
entry must hold, in the same form as a component gate. Gate an FCC-driven mix on its FMC
channel and the bus that powers the computer, so it drops out with either.

Mixes apply in REALISTIC only - casual has none. The per-axis totals are published as `bmkhs_mix<Axis>Out` and
added to the rotor's control sum (see the Reference). Use the signs above - they are Core's
control conventions, not the aircraft manual's.

### Mass and balance - converting the CG

`bmkhs_config/helisim_mass.hpp`. Three coordinate frames are involved, and
every number has to go into the right one.

| Frame | What it is | Which config values |
|---|---|---|
| **Model** | Object Builder (p3d) coordinates, metres. x right, **y toward the nose**, z up. | `fsDatum`, `fwdCgLimit`, `aftCgLimit`, every `arm[]` (seats, tanks, stations, magazines, equipment) |
| **Fuselage station (FS)** | The flight manual's. Distance **aft** of the datum, in inches or metres. | `emptyMom` and each `EmptyMassVariants` `moment` |
| **Arma** | Where Arma places everything in game - the centre of mass, forces, debug lines: the model frame minus the model's `boundingCenter` (`boundingCenter vehicle player` in the debug console). | None. Core converts every config position to it. Never enter a number from it. |

Only the longitudinal (y) position is computed. Lateral comes from the item arms,
with the empty airframe on the centreline. Vertical is not computed: the z of
`comCorrection` sets it.

**1. Place the datum.** Find FS 0 on the model and read its y in Object Builder.
That y is `fsDatum`. If the manual gives the datum relative to something on the
model, add the distances. The Tiger's datum is 7.04 m forward of the main rotor
hub, which sits at y 1.555, so `fsDatum = 1.555 + 7.04 = 8.595`.

**2. Convert a station to a model y.** Stations run aft and model y runs forward,
so:

    model y = fsDatum - FS (metres)        FS (m) = FS (in) x 0.0254

**3. CG limits.** Convert each limit station with step 2. The forward limit is
the larger y.

    fwdCgLimit = fsDatum - FS of the forward limit
    aftCgLimit = fsDatum - FS of the aft limit

If you already have the limits as model positions, enter them as they are.

**4. Empty moment.** `emptyMom` is the empty mass times the empty CG's station,
in kg x m:

    emptyMom = emptyMass x FS of the empty CG (metres)

If you have the empty CG as a model position instead:

    emptyMom = emptyMass x (fsDatum - empty CG model y)

Check it by going back the other way: `fsDatum - emptyMom / emptyMass` must give
the empty CG's model y. The same applies to every `EmptyMassVariants` `mass` and
`moment` pair.

**5. Item arms.** Seats, tanks, wing stations, magazines and equipment are model
positions `{x, y, z}`. Read them in Object Builder, or convert a manual station
with step 2 for y.

**Fitted equipment.** Parts the aircraft fits and removes by showing and hiding
them - a probe, a hoist, pylon wings, doors, seats - are declared in
`class Equipment`, one numbered class each, `numEquipment` of them. Make
`emptyMass` the airframe with none of them fitted; each adds its weight while it
is aboard.

    numEquipment = 2;
    class Equipment {
        class Equipment01 {  //ESSS
            animation      = "ESSS_show";            //the source, as the aircraft defines it
            installedPhase = 1;                      //the phase at which the part is fitted
            mass           = 198;                    //kg
            arm[]          = {0.000, 1.000, -0.500};
        };
        class Equipment02 {  //Rescue hoist
            animation      = "Hoist_hide";
            installedPhase = 0;                      //the source hides it, so 0 is installed
            mass           = 50;
            arm[]          = {1.100, 1.800, 0.900};
        };
    };

`installedPhase` is the one thing to get right: an airframe's sources read either
way round, so each item names its source exactly as the aircraft defines it and
the phase at which the part is fitted - 1 for a `_show` source, 0 for a `_hide`
one. A part counts while its source is on the same side of halfway as
`installedPhase`. Equipment reads the aircraft's own sources live, so whatever
fits or removes a part - an Eden attribute, an ACE action, a script - changes the
mass with no other wiring.
`EmptyMassVariants` is for the one case equipment cannot express: a fitted part
that replaces the whole empty mass and moment rather than adding to them.

**6. What Core does with them, every frame** (`fn_massUpdate`):

    longMom = emptyMass x fsDatum - emptyMom          (the empty airframe, model frame)
            + sum of (mass x arm y) over every item aboard
    CG y    = longMom / total mass
    CG x    = sum of (mass x arm x) / total mass
    setCenterOfMass [CG x - boundingCenter x + comCorrection x,
                     CG y - boundingCenter y + comCorrection y,
                        - boundingCenter z + comCorrection z]

An empty seat, an uninstalled removable tank, an empty pylon and a removed
piece of equipment add nothing.
Fuel moves the CG as it burns, because each tank's mass sits at its own arm.
Casual mode skips all of this and sets `casualModeCom` directly, in the Arma frame.

**7. Reading it back.** Core publishes the CG in the model frame as `bmkhs_cg`.
Compare that against your limits. The FM debug overlay's `_centerOfMass` is Arma's
`getCenterOfMass`, in the Arma frame. To compare it with the limits, convert it
back:

    model y = overlay y + boundingCenter y - comCorrection y

(`boundingCenter vehicle player` in the debug console gives the offset.) The
overlay's gross weight is in pounds. Every mass in the config is in kg.

**Worked example, AH-64D.** `fsDatum = 6.4`. Empty CG at FS 205.00 in = 5.207 m,
so `emptyMom = 6314 x 5.207 = 32877`, which puts the empty CG at model y
6.4 - 5.207 = 1.193. CG limits FS 201 in (5.105 m) and 207 in (5.258 m) give
`fwdCgLimit = 1.295` and `aftCgLimit = 1.142`.

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

The five kinds:

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

Core raises events to the handler your pack registers for its base class, in
`XEH_preInit.sqf` (see Step 2):

```sqf
[yourAircraft_helisim_baseClass, {
    params ["_heli", "_event", ["_data", []]];
    switch (_event) do {
        case "controlMoved": {
            _data params ["_name", "_idx", "_prevIdx", "_value", "_posName"];
            //The control's VALUE is the animation phase. Nothing here restates
            //what a position is worth - move a detent in config and this follows.
        };
    };
}] call bmkhs_fnc_utilNotifyRegister;
```

**One handler per base class.** Registering again for the same class replaces
it; another pack's aircraft never reach it.

### When the cockpit framework animates

Some interaction frameworks (Hatchet's lever interactions) animate a control
themselves when it is clicked. Then the framework, not `controlMoved`, is the
animator, and these hold:

- **One animator.** The framework moves every cockpit control - clicks,
  keybinds and linked controls alike. The pack never animates one as well, or the
  two run on different clocks.
- **Tell Core when the move starts, not when it ends.** Call `controlSet` from
  the framework's start hook, so the engine and the lever move together.
- **Refuse before moving.** Check the control's `enabledBy[]` / `inhibitedBy[]`
  in the framework's pre-move condition, so a refused control never moves rather
  than snapping back.
- **`controlSet` runs where the aircraft is local.** A click from another seat is
  forwarded to the owner.
- **Linked controls go through the framework too.** Two power levers advancing
  together are both moved by the framework, at one rate.
- **Match the framework's rate to Core's.** A lever's travel time is the
  engine's `leverTravelTime`.

The H-60's `docs/HATCHET.md` is the worked example.

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

Learned the hard way.

**Core publishes everything you declare, from init - so read it plainly.** Core
seeds every declared component's variable when the aircraft initialises, straight
from your declarations, so your pack reads `_heli getVariable "bmkhs_x"` with no
default. A declared variable that is nil is a Core bug; a default in your reader
would only hide it, and that is exactly how the reservoir levels went unpublished
for weeks while Core's own defaulted readers looked fine. If you publish a NEW
networked variable from your own code, seed it plainly first:
`bmkhs_fnc_utilUpdateNetworkGlobal` only writes on a change, and never writes a
variable that has never been set.

**Fuel stays out of the systems model - for transfer and for supply.** Fuel moves
mass from one tank to another and must conserve it; a circuit is a
highest-feeder-wins level that conserves nothing. Flow indicators are declared per
path in your fuel config (`flowingVar` on a transfer tank's `Outputs` or an aux
tank, `xferFlowingVars[]` for the pump), named by you, never as circuits.

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

---

## Reference - what Core publishes

Everything below is an output a pack can read: its type and units, whether it reaches other
machines, and what it means. Inputs are the config field references
(`components.hpp`, `controls.hpp`, `engine.hpp`); this is the other half.

### How to read it

- **Where it lives.** Every variable is on the aircraft - `_heli getVariable "bmkhs_..."` - except
  the CBA settings, which are globals.
- **Where it exists.** Core does not schedule itself; the pack calls `bmkhs_fnc_coreUpdate`, on
  the machine where the aircraft is local (see Step 2). **local** means the value is written only
  there; every other machine sees its init seed or nil. Anything another crew station displays
  must be **net**.
- **net** means Core publishes it. *On change* - sent when the value changes (through
  `bmkhs_fnc_utilUpdateNetworkGlobal` / `bmkhs_fnc_utilSetArrayVariable`). *10 Hz* - engine
  values `engine/fn_engineUpdate.sqf` re-broadcasts every 0.1 s in multiplayer. *Every frame* -
  written with a public `setVariable` each update.
- **Per engine / per rotor** means an array with one slot each, in `Engine01`, `Engine02`... order.
- **Fractions**: 1.0 = 100%.
- **`useSystems`**: many values are only solved with `useSystems = 1`, and some only exist if the
  config declares that component, tank or control. Each row says so.
- **Latched** means it stays set until repair or reset.
- Read plainly, as "Things that will catch you" says - but a value marked *only if declared* is
  nil on an aircraft that does not declare it.
- Paths in parentheses are under `addons/helisim/functions/`.


### Engines

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_numEngines` | number | local | Engine count. Taken from the `engines` hitpoint role count, or from config `numEngines` if there are none. Set at config load (set in `engine/fn_engineVariables.sqf`). |
| `bmkhs_engines` | array of hashmaps, per engine | local | Each engine's config. Keys are the config property names (e.g. `name`, `idleNg`, `maxTgt`, `maxTgtSe`, `startTgt`, `startMinTgt`, `npFly`, `designRpm`, `maxNg`, `maxNp`, `oilPsiLimits`, `ngLimits`, `npLimits`, `tqLimits`, `tgtLimits`, `tqLimitsSe`, `tgtLimitsSe`, `ngMin`), plus the derived `refTq` (Nm, 100% torque). Read limits from here (set in `engine/fn_engineVariables.sqf`). |
| `bmkhs_engDesignRpm` | number, rpm | local | Power turbine rpm at 100% Np, from Engine01 `designRpm`. This is the shaft reference for `bmkhs_xmsnOutputRpm`. Only set if there is at least one engine (set in `engine/fn_engineVariables.sqf`). |
| `bmkhs_engState` | array of strings, per engine: `"OFF"`, `"STARTING"`, `"ON"` | net on change + 10 Hz | Engine run state. Goes `STARTING` on a start (start switch at +1 with useSystems = 1, or Arma engine-on with useSystems = 0). Goes `ON` when Ng reaches `selfSustNg`. Goes `OFF` on ignition override (-1) during a start, lever to OFF while ON, an engine failure, fuel starvation, or no bleed air (`bmkhs_pneuAvail` false) during a start (set in `engine/fn_engineUpdate.sqf`, `engine/gasTurbine/fn_gasTurbineStarter.sqf`). |
| `bmkhs_engPowerLeverState` | array of strings, per engine: `"OFF"`, `"IDLE"`, `"FLY"` | net on change | Power lever detent. With useSystems = 1 it follows `bmkhs_eng<N>PwrLvrVal` (>= 1 FLY, > 0 IDLE, else OFF). With useSystems = 0 Core moves it to IDLE on Arma engine-on, then to FLY once Ng has held idle for 1 s (set in `engine/fn_engineUpdate.sqf`). |
| `bmkhs_engPctNg` | array of numbers, per engine, fraction (1.0 = 100% Ng) | 10 Hz | Gas generator speed. Clamped 0 to 1.1 (set in `engine/turboShaftEngine/fn_turboShaftEngine.sqf`). |
| `bmkhs_engPctNp` | array of numbers, per engine, fraction of `designRpm` (1.0 = 100%) | 10 Hz | Power turbine speed as the gauge reads it. Governed Np shows `npFly` (e.g. 1.01 = 101%) (set in `engine/turboShaftEngine/fn_turboShaftEngine.sqf`). |
| `bmkhs_engPctTq` | array of numbers, per engine, fraction (1.0 = 100%) | 10 Hz | Engine torque against `refTq` (torque at `powerKw` at `designRpm * npFly`). Includes clutch slip (set in `engine/turboShaftEngine/fn_turboShaftEngine.sqf`). |
| `bmkhs_engOutputTq` | array of numbers, per engine, Nm | local | Power turbine output torque, after clutch slip. Same value as `bmkhs_engPctTq` but in Nm. Not broadcast (set in `engine/turboShaftEngine/fn_turboShaftEngine.sqf`). |
| `bmkhs_engTgt` | array of numbers, per engine, °C | 10 Hz | Turbine gas temperature as the gauge reads it (lagged). Starts at free air temperature (set in `engine/turboShaftEngine/fn_turboShaftEngine.sqf`). |
| `bmkhs_engOilPsi` | array of numbers, per engine, gauge fraction (not psi) | 10 Hz | Oil pressure = Ng × 0.90 × oil health. Same units as config `oilPsiLimits[] = {min, max}` (e.g. {0.23, 1.20}) (set in `engine/turboShaftEngine/fn_turboShaftEngine.sqf`). |
| `bmkhs_engFuelFlow` | array of numbers, per engine, kg/s | 10 Hz | Fuel flow = commanded fuel units × config `maxFuelFlow`. Also what the fuel pump draws from the tank (set in `engine/turboShaftEngine/fn_turboShaftEngine.sqf`). |
| `bmkhs_engClutch` | array of booleans, per engine | local | True while the freewheel is engaged and the turbine is driving the rotor (set in `engine/turboShaftEngine/fn_turboShaftEngine.sqf`). |
| `bmkhs_isSingleEng` | boolean | local | True when any engine's torque is below 51% of the highest. Switches the TGT limiter to `maxTgtSe` and the TGT book limits to `tgtLimitsSe` (set in `engine/fn_engineGovernor.sqf`). |
| `bmkhs_engFuelAvail` | array of booleans, per engine | net on change | Engine has fuel. False once its selected tank (via crossfeed) has been empty for 2 s, or the engine's fire handle (`eng<N>FireHandle`) is armed with DC on. Always true if the aircraft has no fuel tanks. False forces the engine OFF (set in `engine/fn_engineFuelAvail.sqf`). |
| `bmkhs_engineOverspeed` | array of booleans, per engine | net on change | Latched. Set when Ng >= `maxNg` (fly-weight trip) or Np >= `maxNp` (electrical trip). Cuts fuel and locks out the starter. Cleared only by repair (set in `engine/turboShaftEngine/fn_turboShaftEngine.sqf`). |
| `bmkhs_engChips` | array of booleans, per engine | net on change | Latched chip detector. useSystems = 1: engine hitpoint damage >= 0.50. useSystems = 0: a random engine at shared damage >= 0.25 (50% chance, else oil failure), and any healthy engine at >= 0.75. Cleared by repair (set in `engine/fn_engineDamage.sqf`). |
| `bmkhs_engFailed` | array of booleans, per engine | net on change | Latched engine failure. useSystems = 1: engine hitpoint damage reaches 1.0. useSystems = 0: one engine at shared damage >= 0.50, all at 1.0. Forces the engine OFF and blocks restarts. Cleared by repair (set in `engine/fn_engineDamage.sqf`). |
| `bmkhs_lowOilPsiFailure` | array of booleans, per engine | net on change | Latched oil system failure. useSystems = 1: oil health reaches 0 while the engine is STARTING/ON. useSystems = 0: the oil branch of the random fault. Cleared by repair (set in `engine/fn_engineDamage.sqf`). |
| `bmkhs_engOilPsiLow` | array of booleans, per engine | net on change | Latched low-oil-pressure indication. Set when the engine is ON (or failed), its lever is not OFF, and `bmkhs_engOilPsi` < `oilPsiLimits[0]`. Cleared by repair (set in `engine/fn_engineUpdate.sqf`). |
| `bmkhs_engOilHealth` | array of numbers, per engine, fraction (1.0 = full) | local | Oil remaining. useSystems = 1: drains with engine damage above 0.65, faster above 0.75, 0 at 0.85. useSystems = 0: set to 0 by the oil fault. Already folded into `bmkhs_engOilPsi`. Reset to 1.0 by repair (set in `engine/fn_engineDamage.sqf`). |
| `bmkhs_engLimitTimers` | array per engine of `[np, ng, tgt]`, seconds | 10 Hz | Exceedance countdowns against `npLimits`, `ngLimits`, `tgtLimits` (`tgtLimitsSe` when single engine). -1 = inside limits; > 0 = seconds left in the current band; 0 = time used up and damage accruing (or a zero-second band). Only counts while the engine is STARTING/ON (set in `engine/fn_engineDamage.sqf`). |
| `bmkhs_engTqTimer` | array of numbers, per engine, seconds | 10 Hz | Same convention as `bmkhs_engLimitTimers`, for the drivetrain torque limits. Seeded to -1 in `engine/fn_engineVariables.sqf`; updated in `systems/fn_systemTorque.sqf`. |
| `bmkhs_engClutchSlip` | array of numbers, per engine, fraction (1.0 = no slip) | local | Share of torque the clutch passes. Already applied to `bmkhs_engOutputTq` and `bmkhs_engPctTq`. Seeded in `engine/fn_engineVariables.sqf`; updated in `systems/fn_systemTorque.sqf`. |
| `bmkhs_engBleedAvail` | boolean | net on change | True when any power lever is at FLY. useSystems = 1 only (set in `engine/fn_engineUpdate.sqf`). |
| `bmkhs_acBusOn`, `bmkhs_dcBusOn`, `bmkhs_battBusOn` | boolean | net on change | useSystems = 0 only: all follow Arma `isEngineOn`. With useSystems = 1 these belong to the systems model (set in `engine/fn_engineUpdate.sqf`). |
| `bmkhs_priHydPsi`, `bmkhs_utilHydPsi` | number, psi | net on change | useSystems = 0 only: 3000 when Arma engine is on, 0 when off. With useSystems = 1 these belong to the systems model (set in `engine/fn_engineUpdate.sqf`). |

### Drivetrain

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_xmsnOutputRpm` | number, rpm at the engine shaft | 10 Hz | Drivetrain speed referred to the engine shaft. Rotor rpm = this / rotor `gearRatio`. Nr as a fraction = this / `bmkhs_engDesignRpm`. Rotor brake: BRAKE drags it down, LOCK holds it at 0 (below a max Nr). Computed only on the machine where the aircraft is local and the player is pilot (set in `transmission/fn_transmissionUpdate.sqf`). |
| `bmkhs_xmsnDeltaRpm` | number, rpm per frame | 10 Hz | Change in `bmkhs_xmsnOutputRpm` over the last frame. Depends on frame time (set in `transmission/fn_transmissionUpdate.sqf`). |
| `bmkhs_rtrBrkStartLatch` | number, 0 or 1 | net on change | 1 when a start was begun with the rotor brake on (useSystems = 1). Cleared to 0 only when `bmkhs_rotorBrakeVal` returns to 0. Meant to suppress the brake caution during a locked-rotor start (set in `engine/fn_engineUpdate.sqf`, cleared in `transmission/fn_transmissionUpdate.sqf`). |

### Rotor

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_numSimpleRotors` | number | local | Rotor count, from config `numSimpleRotors` (set in `simpleRotor/fn_simpleRotorVariables.sqf`). |
| `bmkhs_simpleRotors` | array of hashmaps, per rotor | local | Each simple rotor's config, keyed by config property name (`type`, `dir`, `gearRatio`, `numBlades`, `bladeRadius`, ...). `gearRatio` converts `bmkhs_xmsnOutputRpm` to rotor rpm (set in `simpleRotor/fn_simpleRotorVariables.sqf`). |
| `bmkhs_nrLimits` | array of 4 numbers, fraction | local | From config `nrLimits[]`: {normal low, normal high, high rotor, maximum} (e.g. {0.96, 1.05, 1.06, 1.10}). Core only stores it (set in `simpleRotor/fn_simpleRotorVariables.sqf`). |
| `bmkhs_reqEngTorque` | array of numbers, per rotor, Nm at the engine shaft | net on change (effectively every frame) | Rotor torque demand referred to the engine shaft, filtered. Sum it for total load. Seeded as 2 slots (set in `simpleRotor/fn_simpleRotorTorque.sqf`, or `rotor/fn_rotor.sqf` with the BET model). |
| `bmkhs_rtrThrust` | array of numbers, per rotor, N | net on change | Rotor thrust. Only written by the BET rotor model (`bmkhs_rotorModel` = 1). With the Simple model (default) it stays 0 (set in `rotor/fn_rotor.sqf`). |

### Controls - named by your config

Each class under the aircraft's `Controls` gives one control. `variableName` names it, and Core adds the `bmkhs_` prefix. A control with no `Positions` is skipped. Positions are indexed 0..n-1 in the order they are declared.

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_<variableName>Idx` | Number, 0-based position index | net if the control's `networked = 1`, else local to the machine that moved it | The current position, and the one value Core treats as canonical. Seeded at `rest` on every machine that runs coreConfig, with a plain local setVariable. After that it is published only when it changes. (set in `controls/fn_controlsVariables.sqf`, `controls/fn_controlPublish.sqf`) |
| `bmkhs_<variableName>Val` | Number, the position's config `value` | same as Idx | The declared `value` of the current position. (set in `controls/fn_controlPublish.sqf`) |
| `bmkhs_<variableName>On` | Bool, `Val != 0` | same as Idx | This is what component `gate[]` entries normally read. Example: `batt1Switch` publishes `bmkhs_batt1SwitchOn`. Code outside Core may also write it. On the owner, with `useSystems = 1`, Core then moves Idx to a position that matches. It prefers `rest`, and raises `controlMoved`. If no position matches, it leaves Idx alone. (reconciled in `controls/fn_controlsUpdate.sqf`) |

Notes:
- A move goes through `bmkhs_fnc_controlSet`. This runs on whichever machine calls it, and the default target is `vehicle player`. It respects `enabledBy[]` / `inhibitedBy[]` on the control and on the position. A control blocked by an interlock does not move.
- A position with `springsBack = 1` stays thrown for one systems solve, then returns to `rest` (`controls/fn_controlsRelease.sqf`). The release runs only inside the systems solve, so it happens only on the owner and only with `useSystems = 1`. With `useSystems = 0`, nothing in Core returns a sprung position to rest.
- Two control names are hard-coded in Core. `apuBtn` is forced to index 0 when the APU stops or runs out of fuel (see APU). `apuFireHandle`, if declared, is read through `bmkhs_apuFireHandleOn`.

### Components - named by your config

Names are built in `systems/fn_systemsComponents.sqf`. A component with a `damageRole` gets one member for each hitpoint claiming that role (see `bmkhs_fnc_damageCount`). Members are numbered 1..n **only when there is more than one**. Example: `gen` with 2 members gives `bmkhs_gen1` and `bmkhs_gen2`, but `priHydPsi` alone gives `bmkhs_priHydPsi`. A role that no hitpoint claims gives zero members, so no variable exists. An empty `damageRole` gives one member that cannot be damaged.

Seeding: on the local machine at init, every declared variable that is still nil is seeded. The seed is broadcast if the component is `networked`. Values are solved and updated only with `useSystems = 1`, on the owner. With `useSystems = 0` the seed is final: producers/converters read `nominal`, and circuit/consumer/state flags read `true`.

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_<variableName>[n]` (Producer) | Number, in the config's own unit (psi, fraction, or 1 for on/off). No `nominal`: carries the `drivenBy` circuit's value (e.g. Nr fraction) | net if `networked = 1`, else local (owner only) | Current output. It is `nominal` (or the drive value), times the `requires` level scaling. It is 0 if the component is damaged > 0.85, any gate is off, or the drive is at or below its `drivenBy` threshold. It ramps over `rampSeconds` and is rounded to `increment`. Seeded 0 with systems. (set in `systems/fn_systemProducer.sqf`) |
| `bmkhs_<variableName>[n]` (Converter) | Number, config unit | net if `networked = 1`, else local (owner only) | Output: `nominal`, or input × `ratio`. It is 0 without input above `input[]`'s threshold, when damaged > 0.85, or when a gate is off. Never ramps. Rounded to `increment`. (set in `systems/fn_systemConverter.sqf`) |
| `bmkhs_<variableName>[n]` (Storage) | Number, `charge × nominal` (charge is 0..1). E.g. psi for an accumulator, fraction for a battery | net if `networked = 1`, else local (owner only) | Stored amount. It drains at `emerDischarge` while live and not covered, and refills over `startRecharge` from `rechargedBy`. It leaks from `leakStartDmg`, and is 0 when destroyed. A `startedBy` start drops it to `stopBelow`. Seeded full. (set in `systems/fn_systemStorage.sqf`) |
| `bmkhs_<stateName>` | Bool | net (always, whatever `networked` says) | Running flag for a producer, converter or store that declares `stateName`. True when its output is at or above `stateAbove` (for storage, `charge × nominal`). Example: the H-60 APU publishes `bmkhs_apuOn`. Seeded `!useSystems` (producers/converters). (set in `systems/fn_systemProducer.sqf`, `fn_systemConverter.sqf`, `fn_systemStorage.sqf`) |
| `bmkhs_<variableName>[n]StartOk` | Bool | net (sent every time it is written, not change-gated) | Exists only for storage with `startedBy`. **Latched**: when the `startedBy` variable goes true, Core checks once whether `charge × nominal >= startAbove`. The result holds until `startedBy` goes false, which resets it to true. The H-60 gates its APU on `bmkhs_accHydPsiStartOk`. (set in `systems/fn_systemStorage.sqf`) |
| `bmkhs_<variableName>` (Circuit) | Bool | net if `networked = 1`, else local (owner only) | True while the named `circuit` is at or above `minValue`. Republished every solve. Examples: `bmkhs_acBusOn`, `bmkhs_pneuAvail`. (set in `systems/fn_systemCircuitState.sqf`) |
| `bmkhs_<variableName>` (Consumer) | Bool | net if `networked = 1`, else local (owner only) | True when supplied. By default any one `suppliedBy[]` circuit at or above its threshold is enough; with `needsAll = 1` all of them must be. Example: `bmkhs_fltCtrlsSupplied`. (set in `systems/fn_systemConsumer.sqf`) |

### Core-named system variables

Core seeds these by name, whether or not the config declares them (`systems/fn_systemsVariables.sqf`). The first block is seeded once, on the local machine. A declared component with the same name then overwrites it.

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_battSwitchOn` | Bool | net | Seeded false. Core writes nothing else. A control named `battSwitch` would drive it. The H-60 uses `batt1Switch` / `batt2Switch` instead. |
| `bmkhs_battBusOn` | Bool | net | Seeded false. With `useSystems = 0`, set to `isEngineOn` (`engine/fn_engineUpdate.sqf`). With systems, it changes only if a Circuit declares this name (the H-60 does). |
| `bmkhs_acBusOn` | Bool | net | As battBusOn. |
| `bmkhs_dcBusOn` | Bool | net | As battBusOn. Core also reads it: an armed APU fire handle shuts APU fuel only while DC is up. |
| `bmkhs_apuBtnOn` | Bool | net | Seeded false. It is the `On` of a control named `apuBtn`. |
| `bmkhs_apuRpm_pct` | Number, 0..1 fraction (H-60 `nominal = 1.0`) | net | Seeded 0. Changes only if a producer declares this name (H-60: `apuRPM_pct`). Also re-broadcast every 0.1 s in multiplayer by `engine/fn_engineUpdate.sqf`. |
| `bmkhs_apuOn` | Bool | net | Seeded false. Changes only if a component declares `stateName = "apuOn"`. |
| `bmkhs_pneuAvail` | Bool | net | Seeded `!useSystems`, so true without systems. With systems, changes only if a Circuit declares it. |
| `bmkhs_priHydPsi` | Number, psi | net | Seeded 0. With `useSystems = 0`: 3000 when `isEngineOn`, else 0. With systems, only if a producer declares it. |
| `bmkhs_utilHydPsi` | Number, psi | net | As priHydPsi. |
| `bmkhs_accHydPsi` | Number, psi | net | Seeded 3000. Changes only if a store declares it. |
| `bmkhs_emerHydOn` | Bool | net | Seeded false. It is re-seeded and broadcast on every machine that runs coreConfig. Input: Core's flight-control input reads it, and the H-60 accumulator gates on it. Core never sets it true. |

### APU

Runs only with `useSystems = 1` (`systems/apu/fn_apu.sqf`). Running state and RPM come from the components above.

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_apuFuelAvail` | Bool | net (on change), seeded locally true | True when the APU's tank (`fuelSource`) has fuel above ~0, and the fire handle is not closing it. The handle closes it when DC is up, an `apuFireHandle` control is declared, and `bmkhs_apuFireHandleOn` is true. It is updated only if a producer has `damageRole = "apu"` and the aircraft has fuel tanks. Otherwise it stays true. When it goes false, or the APU stops, Core forces control `apuBtn` to index 0. The APU also burns `fuelFlow` (lb/h in config, kg/s internally) from `bmkhs_<tank>Mass` while `bmkhs_apuOn`. |

### Fuel

Tanks come from `FuelTanks` / `AuxTanks` (`FuelTank01`...). `variableName` names each tank, and Core adds `bmkhs_` (`fuel/fn_fuelTankVarName.sqf`). A missing or duplicate name falls back to `bmkhs_fueltank<n>` / `bmkhs_auxtank<n>` and logs an error. Fuel update runs wherever the pack calls coreUpdate (the H-60 does this on the owner only). It needs `maxTotFuelMass > 0`. It does not depend on `useSystems`.

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_<tank>Mass` | Number, kg | local (owner only) | Fuel in the tank. On init and whenever Arma `fuel` drifts > 1 %, it is set from `fuel _heli`: internal tanks are filled first, split by capacity. Each frame it is updated by transfer, leak (tank damage > 0.5), engine and APU burn. Clamped 0..Max. Not networked: remote machines keep their init value. (`fuel/fn_fuelSet.sqf`, `fuel/fn_fuelUpdate.sqf`) |
| `bmkhs_<tank>Max` | Number, kg | local (every machine that runs coreConfig) | Config `capacity`. Static. (`fuel/fn_fuelVariables.sqf`) |
| `bmkhs_<tank>Low` | Number, kg | local (every machine that runs coreConfig) | Config `lowFuelKg`. Static. Internal tanks only. Used by AUTO transfer. |
| `bmkhs_<tank>Installed` | Bool | local seed. The aircraft may write it, networked | Internal tanks only. Fixed tanks are seeded true. Removable tanks keep any value the aircraft set before coreConfig, else false. Input for the aircraft (H-60: `bmkhs_erfsTankInstalled`). An uninstalled removable tank does not leak or transfer, and its capacity is excluded. |
| `bmkhs_<tank>XferOn` | Bool | net when Core clears it | Input for `role = "xfer"` cells. The aircraft sets it true to gravity-feed the cell's `Outputs` (default: all mains). Core sets it false (broadcast) when the cell runs dry. Transfer is held off while any armed aux tank still has more than 10 kg. |
| `bmkhs_<auxTank>Mass` | Number, kg | local (owner only) | Fuel in an aux tank. Forced to 0 when no `auxTank` magazine is on that station's pylons. |
| `bmkhs_<auxTank>Max` | Number, kg | local | Config `capacity`. |
| `bmkhs_<auxTank>EmptyArmed` | Bool | local (owner only) | Re-arm flag for an "empty" advisory. Set true while the tank is absent or holds 10 kg or more. Set false while it holds less than 10 kg **and** `bmkhs_fuelPageOpen` is true. Seeded false. |
| `bmkhs_<flowingVar>` | Bool | net (on change), seeded locally false on every machine | Named by `flowingVar` on a tank `Outputs` entry or an aux tank, or by `xferFlowingVars[]` (one per main, in main order). True on any frame where fuel moved along a path with that name. Several paths may share one name. |
| `bmkhs_totFuelMass` | Number, kg | local (owner only) | Total fuel in all internal and aux tanks. Core also calls `setFuel` with total / max. |
| `bmkhs_maxTotFuelMass` | Number, kg | local | Capacity of the tanks currently fitted (internal and aux). Updated on resync. |
| `bmkhs_numFuelTanks` | Number | local | Config `numFuelTanks`. 0 means no tank model, so engines and APU are always fuelled. |
| `bmkhs_checkRunning` | Bool | net when Core clears it | Input: the aircraft sets it true to start a FUEL CHECK. Core sets it false when the check ends. |
| `bmkhs_checkDone` | Bool | net | Set true when a check completes. Core never clears it, so the aircraft must. |
| `bmkhs_checkPendingAdvisory` | Bool | net | Set true on completion if neither `bmkhs_checkActivePlt` nor `bmkhs_checkActiveCpg` is true. Core never clears it. |
| `bmkhs_checkBurnRate` | Number, lb/h | net | Average burn over the check: `(checkStartFuel - totFuelMass)` per elapsed time. Written only on completion. |
| `bmkhs_checkBurnoutZulu` | String, `"H:MML"` (e.g. `"9:05L"`, hour not padded) | net | In-game `dayTime` at which fuel runs out at that burn rate. Written on completion. |
| `bmkhs_checkVfrZulu` | String, `"H:MML"` | net | Burnout minus 20 min. |
| `bmkhs_checkIfrZulu` | String, `"H:MML"` | net | Burnout minus 30 min. |

Fuel inputs Core seeds or reads (the aircraft writes these):

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_xferMode` | String | local seed `"AUTO"` | XFER pump selection. `"AUTO"`, or one of the `xferDestinations[]` labels (upper case) to pump into that main. Anything else is off. Needs exactly two mains. AUTO also needs bleed air (`bmkhs_pneuAvail`) or a running engine. |
| `bmkhs_crossfeedMode` | String | local seed = first `CrossfeedModes` `position` | Picks which tank each engine draws from (`engSources[]`), for engines without their own selector. An entry of `"off"` is a deliberate no-fuel source and the engine starves and shuts down. Read in `engine/fn_engineFuelAvail.sqf`. |
| `bmkhs_<control>Idx` of an engine's `fuelSelector` | Number | as the control | An engine that declares `fuelSelector` and `fuelSources[]` (engine config) draws from `fuelSources[]` at that control's position instead - one lever per engine, no crossfeed table. `"off"` there is no fuel. |
| `bmkhs_<group>AuxOn` | Bool | local seed (`bmkhs_lAuxOn`, `bmkhs_rAuxOn`) | Arms the aux tanks whose `group` matches. The group name is lower-cased in the variable name. |
| `bmkhs_checkStartTime` | Number, seconds of `CBA_missionTime` | local seed 0 | When the check started. The check does nothing while this is 0 or less. |
| `bmkhs_checkStartFuel` | Number, kg | local seed 0 | `totFuelMass` at the start of the check. |
| `bmkhs_checkMinutes` | Number, minutes | local seed 15 | How long the check runs. |
| `bmkhs_checkActivePlt` / `bmkhs_checkActiveCpg` | Bool | local seed false | A crew station is viewing the check. Suppresses `checkPendingAdvisory`. |
| `bmkhs_fuelPageOpen` | Bool | not seeded | Read only. See `EmptyArmed`. |

### Damage and repair

Damage is read through `bmkhs_fnc_damageGet` (see Read functions), by role - not by hitpoint name.

`systems/repair/fn_repair.sqf` runs on the owner, after a HandleDamage event shows a hitpoint going down. It writes no new outputs. It does these things:

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| storage `bmkhs_<variableName>[n]` | as above | as above | An undamaged or role-less store is refilled (internal charge set to 1.0). The published value follows on the next solve. |
| `bmkhs_engineOverspeed`, `bmkhs_engChips`, `bmkhs_engFailed`, `bmkhs_lowOilPsiFailure`, `bmkhs_engOilPsiLow` | Array of Bool per engine (engine outputs) | net | Each repaired engine (damage 0) has its entry cleared to false. These are engine-owned outputs, documented with the engine. |

### State and air data

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_vel2D` | Number, knots, integer, clamped 0 to 180 | local (owner only) | Indicated-style airspeed. Forward (y) component of the air-relative model-space velocity, rounded. Never negative. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_vel3D` | Number, knots, integer | local (owner only) | Magnitude of the air-relative model-space velocity, rounded. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_gndSpeed` | Number, knots, integer | local (owner only) | Ground speed. Magnitude of model-space x and y ground velocity (no wind). Body axes, so it reads low when pitched or rolled. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_velClimb` | Number, ft/min, not rounded | local (owner only) | Vertical speed. World z of smoothed velocity. Positive = climbing. Wind has no vertical part, so this is ground-referenced. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_velModelSpace` | Array [x, y, z], m/s, model space | local (owner only) | Smoothed air-relative velocity (ground velocity minus wind). Wind is rotated by heading only, not pitch or roll. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_velModelSpaceNoWind` | Array [x, y, z], m/s, model space | local (owner only) | Smoothed ground-relative velocity in body axes. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_velWorldSpace` | Array [x, y, z], m/s, world space | local (owner only) | Smoothed air-relative velocity in world axes (velocity minus wind). (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_velWorldSpaceNoWind` | Array [x, y, z], m/s, world space | local (owner only) | Smoothed ground-relative velocity in world axes. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_velWindModelSpace` | Array [x, y, 0], m/s, model space | local (owner only) | Wind velocity rotated into body x/y by heading only. z is always 0. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_angVelModelSpace` | Array [x, y, z], rad/s (Arma `angularVelocityModelSpace`), model space | local (owner only) | Smoothed body angular rates. Sign follows Arma's `angularVelocityModelSpace`. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_worldAccel` | Array [x, y, z], m/s², world space | local (owner only) | Raw kinematic acceleration: frame difference of `bmkhs_velWorldSpaceNoWind`. Gravity not included. Not smoothed. (set in `state/fn_stateAccelerations.sqf`) |
| `bmkhs_worldAccelFiltered` | Array [x, y, z], m/s², world space | local (owner only) | `bmkhs_worldAccel` smoothed per axis. Source for the ball terms and body accel. (set in `state/fn_stateAccelerations.sqf`) |
| `bmkhs_bodyAccel` | Array [right, forward, up], m/s², body axes | local (owner only) | Specific force (what an accelerometer reads): filtered world accel plus 1 g up, projected onto the body right, forward and up vectors. Level and still reads about [0, 0, +9.806]. Element 0 equals `bmkhs_ballTerms # 2`. (set in `state/fn_stateAccelerations.sqf`) |
| `bmkhs_ballTerms` | Array [kLat, gLat, sum], m/s² | local (owner only) | Lateral ball breakdown along the body right axis. `# 0` kLat = filtered kinematic acceleration toward the right. `# 1` gLat = 9.806 × (z component of the body right vector); negative when the right side is low. `# 2` = kLat + gLat = lateral specific force, positive to the right. A physical ball deflects opposite to this: sum positive = ball LEFT, sum negative = ball RIGHT (e.g. right side low in a hover gives a negative sum, ball right). Not clamped or filtered beyond the accel smoothing. (set in `state/fn_stateAccelerations.sqf`) |
| `bmkhs_aero_beta_g` | Number, g, clamped -1 to +1 | net | Trim-ball value: `bmkhs_bodyAccel # 0` / 9.806, then first-order low-pass (tau 0.60 s). Positive = lateral specific force to the right, so a physical ball sits LEFT; negative = ball RIGHT. Core autopilot code relies on this raw sign; flip it only in your display. The AH-64D pack also blends the display sign with speed (`fn_avionicsSlipIndicator.sqf`). (set in `state/fn_stateAeroValues.sqf`) |
| `bmkhs_aero_beta_deg` | Number, degrees | net | Aerodynamic sideslip: asin(x / |v|) of `bmkhs_velModelSpace`. Positive = aircraft moving right through the air (relative wind from the right). 0 when the velocity is zero. (set in `state/fn_stateAeroValues.sqf`) |
| `bmkhs_accelX` | Number, m/s², body x (right) | local (owner only) | Smoothed time derivative of `bmkhs_velModelSpaceNoWind # 0`. Gravity not included. Derivative of a body-axis velocity, so rotation terms are included as they fall. (set in `state/fn_stateAccelerations.sqf`) |
| `bmkhs_accelY` | Number, m/s², body y (forward) | local (owner only) | Same as above for the forward axis. (set in `state/fn_stateAccelerations.sqf`) |
| `bmkhs_accelZ` | Number, m/s², body z (up) | local (owner only) | Same as above for the up axis. (set in `state/fn_stateAccelerations.sqf`) |
| `bmkhs_barAlt` | Number, feet | local (owner only) | Copy of `bmkhs_pa` (pressure altitude, rounded to 10 ft) clamped 0 to 20000. (set in `state/fn_stateAltitude.sqf`) |
| `bmkhs_radAlt` | Number, metres | local (owner only) | Radar altimeter display value. Height from `getPos`; above 15.24 m (50 ft) rounded to 3.048 m (10 ft) steps; clamped 0 to 432.816 m (1420 ft). Convert to feet yourself. (set in `state/fn_stateAltitude.sqf`) |
| `bmkhs_radAltRaw` | Number, metres | local (owner only) | Unrounded, unclamped `getPos _heli # 2`. (set in `state/fn_stateAltitude.sqf`) |
| `bmkhs_rtrRpm` | Number, ratio (1.0 = 100 % Nr) | local (owner only) | Rotor speed: `bmkhs_xmsnOutputRpm` / `bmkhs_engDesignRpm`. Forced to 0 when main rotor damage is 1.0. (set in `state/fn_stateRtrRpm.sqf`) |

Ground contact is not a variable. Call `[_heli] call bmkhs_fnc_stateOnGround`. It returns true when `isTouchingGround` is true or `bmkhs_radAltRaw` < 0.15 m (`state/fn_stateOnGround.sqf`). It works only where `bmkhs_radAltRaw` is updated (the owner).

### Environment

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_pa` | Number, feet, rounded to 10 ft | local (owner only) | Pressure altitude. MSL height in feet plus a base altitude set by the CBA setting `bmkhs_helisimEnvironment` (ISA 0, Europe 800, Middle East 1800, Central Asia 5000, Asia 3100 ft). Altimeter setting is fixed at 29.92 inHg; mission weather does not change it. (set in `environment/fn_environment.sqf`) |
| `bmkhs_fat` | Number, °C, integer steps | local (owner only) | Free air temperature. Base temperature of the selected environment (ISA 15, Europe summer 20 / winter 0, Middle East 30, Central Asia summer 30 / winter -5, Asia 25) minus round(2 °C per 1000 ft of MSL height). (set in `environment/fn_environment.sqf`) |
| `bmkhs_rho` | Number, kg/m³ | local (owner only) | Dry air density from barometric pressure at `bmkhs_pa` and `bmkhs_fat` (p / (287.05 × T)). Init value is 1.225. (set in `environment/fn_environment.sqf`) |
| `bmkhs_windSpeedKts` | Number, knots, integer | local (owner only) | Mission wind speed (`vectorMagnitude wind`). 0 when `bmkhs_windDisabled` is set. (set in `environment/fn_environment.sqf`) |
| `bmkhs_windDirFrom` | Number, degrees 0-359, integer | local (owner only) | Wind direction for display, computed as `(windDir + 180) mod 360`. 0 when `bmkhs_windDisabled` is set. (set in `environment/fn_environment.sqf`) |
| `bmkhs_velWindWorldSpace` | Array [east, north, 0], m/s | local (owner only) | Wind velocity vector (direction the air moves toward). [0,0,0] unless `bmkhs_rotorModel == 0`; also zero when wind is disabled. (set in `environment/fn_environment.sqf`) |

Pressure (hPa) and density altitude are computed in `fn_environment.sqf` but not stored.

### Mass and balance

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_gwt` | Number, kg | net | Gross mass: empty mass (or matching `EmptyMassVariants` entry), occupied seats, fitted equipment, internal fuel, internal magazine rounds, and wing-station stores and external fuel. Written only by the owner. When the CBA test-GWT option is on, replaced by that weight clamped between empty and `maxGrossMass`. All contributors come from config. (set in `mass/fn_massUpdate.sqf`) |
| `bmkhs_cg` | Number, metres, longitudinal only | net | Longitudinal CG: total forward moment / mass, in the same frame as the config arms (H-60 config: arm = {right, forward, up} m; larger = further forward). Compare directly with `bmkhs_fwdCgLimit` / `bmkhs_aftCgLimit`. Not a fuselage station; convert with `bmkhs_fsDatum` if needed. In test-GWT mode it is real moments divided by the test mass. Lateral CG is not published. (set in `mass/fn_massUpdate.sqf`) |
| `bmkhs_fwdCgLimit` | Number, metres, same frame as `bmkhs_cg` | local (owner only) | Forward CG limit, read from config `fwdCgLimit`. Static. (set in `mass/fn_massVariables.sqf`) |
| `bmkhs_aftCgLimit` | Number, metres, same frame as `bmkhs_cg` | local (owner only) | Aft CG limit, read from config `aftCgLimit`. Static. (set in `mass/fn_massVariables.sqf`) |
| `bmkhs_fsDatum` | Number, metres | local (owner only) | Fuselage-station 0 reference, config `fsDatum`. Empty-airframe arm = fsDatum − emptyMom/emptyMass. Static. (set in `mass/fn_massVariables.sqf`) |
| `bmkhs_emptyMass` | Number, kg | local (owner only) | Config `emptyMass`. Does not reflect `EmptyMassVariants`; the variant is only applied inside the gross-weight sum. Static. (set in `mass/fn_massVariables.sqf`) |
| `bmkhs_maxGrossMass` | Number, kg | local (owner only) | Config `maxGrossMass`. Used to bound the test weight. Static. (set in `mass/fn_massVariables.sqf`) |

### Performance

All values are recomputed only when rounded GWT (kg), `bmkhs_pa`, `bmkhs_fat` or the environment setting changes. They are interpolated from the aircraft's config tables by PA (ft) and FAT (°C, rows -40/-20/0/20/40). Units are whatever the pack's tables hold; the H-60 units are given as the example. All are local (owner only), set in `performance/fn_perfData.sqf`.

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_maxTq_cont` | Number, config units (H-60: torque fraction, 1.0 = 100 %) | local (owner only) | Max continuous torque, `perfTable*` column 1. |
| `bmkhs_maxTq_de` | Number, config units (H-60: torque fraction) | local (owner only) | Max torque available, dual engine, column 2. |
| `bmkhs_maxTq_se` | Number, config units (H-60: torque fraction) | local (owner only) | Max torque available, single engine, column 3. |
| `bmkhs_maxGwt_de_ige` | Number, config units (H-60 table values look like lb; units unclear) | local (owner only) | Max gross weight, dual engine, in ground effect, column 4. H-60 config notes this column is AH-64D data. |
| `bmkhs_maxGwt_de_oge` | Number, config units (units unclear) | local (owner only) | Max gross weight, dual engine, out of ground effect, column 5. Same caveat. |
| `bmkhs_maxGwt_se_ige` | Number, config units (units unclear) | local (owner only) | Max gross weight, single engine, IGE, column 6. Same caveat. |
| `bmkhs_maxGwt_se_oge` | Number, config units (units unclear) | local (owner only) | Max gross weight, single engine, OGE, column 7. Same caveat. |
| `bmkhs_goNoGoTq_ige` | Number, config units (H-60: torque fraction) | local (owner only) | Go/no-go torque IGE, column 8. AH-64D data in H-60 config. |
| `bmkhs_goNoGoTq_oge` | Number, config units (H-60: torque fraction) | local (owner only) | Go/no-go torque OGE, column 9. AH-64D data in H-60 config. |
| `bmkhs_hvrTq_ige` | Number, config units (H-60: torque fraction) | local (owner only) | Hover torque IGE at current GWT. `hoverTable*` interpolated by PA, FAT, then GWT over fixed breakpoints 6804/7711/8618/9525 kg (15/17/19/21k lb; hard-coded in Core). |
| `bmkhs_hvrTq_oge` | Number, config units (H-60: torque fraction) | local (owner only) | Hover torque OGE, same method. |
| `bmkhs_tas_vne` | Number, knots TAS | local (owner only) | Never-exceed speed, `TASTable*` column 1. Not GWT-dependent (H-60 tables are for 18000 lb). |
| `bmkhs_tas_vsse` | Number, knots TAS | local (owner only) | Minimum single-engine speed, column 2. 0 in the table means not achievable. |
| `bmkhs_tas_rngTas` | Number, knots TAS | local (owner only) | Max-range airspeed, column 3. |
| `bmkhs_tas_rngTq` | Number, config units (H-60: torque fraction) | local (owner only) | Torque at max-range speed, column 4. |
| `bmkhs_tas_rngFf` | Number, lb/hr total (if `engFFTable` is kg/s per engine, as in the H-60) | local (owner only) | Fuel flow at max-range torque: `engFFTable`(rngTq) × `bmkhs_numEngines` × 7936.64. |
| `bmkhs_tas_endTas` | Number, knots TAS | local (owner only) | Max-endurance airspeed, column 5. |
| `bmkhs_tas_endTq` | Number, config units (H-60: torque fraction) | local (owner only) | Torque at max-endurance speed, column 6. |
| `bmkhs_tas_endFf` | Number, lb/hr total (same condition as rngFf) | local (owner only) | Fuel flow at max-endurance torque. |

Cruise tables (`cruiseTable**`) are interpolated but the result is not stored.

### Stabilator

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_stabilatorPosition` | Number, degrees (per config `heliSimStabTable`) | local (owner only) for live value | Stabilator incidence. Moves toward the `heliSimStabTable` value (by collective and `bmkhs_vel2D`) at a lerp rate of (1/1.5) × dt. Frozen when stabilator damage ≥ `SYS_STAB_DMG_THRESH` or `bmkhs_dcBusOn` is false. Only the init 0 is broadcast; per-frame updates are not. Sign is that of the config table; the same value drives the `Hstab` animation source. Only updated for a wing named "stabilator". (set in `wing/fn_wing.sqf`, init in `wing/fn_wingVariables.sqf`) |

Fuselage and airfoil folders publish no designer-facing values.

### Flight management computer and hold modes

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_fmcPitchOn` | Bool, default true | net | FMC pitch channel on. When false, the pitch SAS and attitude hold pitch outputs are zeroed, and the actuator also uses this flag for its lag model. Set with `bmkhs_fnc_fmcSetChannel [heli,"pitch",bool]`. (set in `fmc/fn_fmcVariables.sqf`, `fmc/fn_fmcSetChannel.sqf`) |
| `bmkhs_fmcRollOn` | Bool, default true | net | FMC roll channel on. Same as pitch, for the roll SAS and attitude hold roll outputs. (set in `fmc/fn_fmcVariables.sqf`, `fmc/fn_fmcSetChannel.sqf`) |
| `bmkhs_fmcYawOn` | Bool, default true | net | FMC yaw channel on. When false, the yaw SAS and heading hold outputs are zeroed. (set in `fmc/fn_fmcVariables.sqf`, `fmc/fn_fmcSetChannel.sqf`) |
| `bmkhs_fmcCollOn` | Bool, default true | net | FMC collective channel on. When false, the altitude hold output is zeroed. (set in `fmc/fn_fmcVariables.sqf`, `fmc/fn_fmcSetChannel.sqf`) |
| `bmkhs_fmcTrimOn` | Bool, default true | net | Trim channel flag. Core stores it but never reads it, so it changes nothing in Core. A pack can use it as a switch state. (set in `fmc/fn_fmcVariables.sqf`, `fmc/fn_fmcSetChannel.sqf`) |
| `bmkhs_forceTrimInterupted` | Bool | net | True while the force-trim (trim release) button is held. While true, attitude hold does nothing and heading hold drops out. On release it goes false and new hold references are captured. Note the spelling ("Interupted"). (set in `fmc/fn_fmcForceTrimHold.sqf`, `fmc/fn_fmcForceTrimRelease.sqf`) |
| `bmkhs_attHoldActive` | Bool | net | Attitude/position/velocity hold engaged. Toggled by the `bmkhs_holdModeAttitude` key. Cleared by `bmkhs_holdModesOff`. This is the mode state only. It stays true even when the pitch/roll channel is off or primary hydraulics have failed, and both of those zero the output. (set in `fmc/fn_fmcAttitudeHoldEnable.sqf`, `fmc/fn_fmcHoldModesDisable.sqf`) |
| `bmkhs_attHoldSubMode` | String `"pos"` / `"vel"` / `"att"` | net | Which attitude hold law applies. Chosen every frame from ground speed, even when the hold is off: `pos` at 5 kt GS or less, `vel` above 5 and up to 40 kt, `att` above 40 kt. (set in `fmc/fn_fmcAttitudeHold.sqf`, `fmc/fn_fmcAttitudeHoldEnable.sqf`) |
| `bmkhs_attHoldDesiredPos` | Array `getPos` [x,y,z], m | net | Position hold reference. Captured on engage in `pos`, and again on force-trim release. (set in `fmc/fn_fmcAttitudeHoldEnable.sqf`, `fmc/fn_fmcForceTrimRelease.sqf`) |
| `bmkhs_attHoldDesiredVel` | Array [x, y], m/s, body axes | net | Velocity hold reference. x is the NEGATED model-space lateral velocity, so + = left. y is forward velocity, + = forward. Reset to [0,0] by hold-modes-off. (set in `fmc/fn_fmcAttitudeHoldEnable.sqf`, `fmc/fn_fmcForceTrimRelease.sqf`, `fmc/fn_fmcHoldModesDisable.sqf`) |
| `bmkhs_attHoldDesiredAtt` | Array [pitch, bank], deg | net | Attitude hold reference, as returned by `BIS_fnc_getPitchBank`. When turn coordination ends, heading hold sets bank to 0. (set in `fmc/fn_fmcAttitudeHoldEnable.sqf`, `fmc/fn_fmcForceTrimRelease.sqf`, `fmc/fn_fmcHeadingHold.sqf`, `fmc/fn_fmcHoldModesDisable.sqf`) |
| `bmkhs_altHoldActive` | Bool | net | Altitude hold engaged. Toggled by the `bmkhs_holdModeAltitude` key, which only engages if vertical speed is within ±200 fpm. It drops out on its own when collective moves more than ±5 % from `altHoldCollRef`, or when the highest engine torque (`bmkhs_engPctTq`) reaches 0.98 or more. (set in `fmc/fn_fmcAltitudeHold.sqf`, `fmc/fn_fmcAltitudeHoldEnable.sqf`, `fmc/fn_fmcHoldModesDisable.sqf`) |
| `bmkhs_altHoldSubMode` | String `"rad"` / `"bar"` | net | Radar or barometric altitude hold. While engaged it is `rad` below 1428 ft AGL and under 40 kt GS, and `bar` otherwise. It is only re-evaluated while the hold is engaged. (set in `fmc/fn_fmcAltitudeHold.sqf`, `fmc/fn_fmcAltitudeHoldEnable.sqf`) |
| `bmkhs_altHoldDesiredAlt` | Number, m (AGL in `rad`, ASL in `bar`) | net | Altitude hold reference, rounded to the metre on engage. It is 0 when the hold is off. Engage chooses AGL or ASL once. The sub-mode can switch later without the reference being re-captured. (set in `fmc/fn_fmcAltitudeHoldEnable.sqf`, `fmc/fn_fmcHoldModesDisable.sqf`) |
| `bmkhs_altHoldCollRef` | Number 0..1 (collective) | net | Collective position captured when altitude hold engaged. Used for the ±5 % disengage band. (set in `fmc/fn_fmcAltitudeHoldEnable.sqf`) |
| `bmkhs_hdgHoldActive` | Bool | net | Heading hold engaged. There is no button for it. It is true whenever the aircraft is off the ground, force trim is not held, and pedal input is inside the breakout (0.05 in `pos`, 0.10 in `vel`, 0.20 in `att`). The heading is re-captured each time it re-engages. (set in `fmc/fn_fmcHeadingHold.sqf`) |
| `bmkhs_hdgHoldSubMode` | String `"hdg"` / `"trn"` / `"yaw"` / `"aut"` | net | Heading hold law, only updated while engaged. `hdg` = hold heading, below 5 kt GS. `trn` = turn coordination: attitude hold on and bank over 7° (drops at under 3°). `yaw` = ball-centring yaw damping. `aut` = the auto-pedal assist owns the yaw axis. (set in `fmc/fn_fmcHeadingHold.sqf`) |
| `bmkhs_hdgHoldDesiredHdg` | Number, deg 0..360 (`getDir`) | net | Heading hold reference. Captured when the hold engages, when it re-enters `hdg`, and on force-trim release. (set in `fmc/fn_fmcHeadingHold.sqf`, `fmc/fn_fmcForceTrimRelease.sqf`) |
| `bmkhs_hdgHoldDesiredSideslip` | Number, lateral g | net | Sideslip (ball) reference. Core always sets it to 0. (set in `fmc/fn_fmcForceTrimRelease.sqf`) |
| `bmkhs_mixPitchOut` | Number, cyclic fraction, + = forward | local (owner only) | Sum of the aircraft's `ControlMixing` mixes targeting pitch, added to the cyclic pitch at the rotor. 0 with no mixes, while a mix's gate is shut, or outside REALISTIC. (set in `fmc/fn_fmc.sqf`, from `fmc/fn_fmcControlMixing.sqf`) |
| `bmkhs_mixRollOut` | Number, cyclic fraction, + = left | local (owner only) | As above, for roll. |
| `bmkhs_mixYawOut` | Number, pedal fraction, + = right | local (owner only) | As above, for the pedals. |
| `bmkhs_fmcSasPitchOut` | Number, ±0.2 cyclic fraction | local (owner only) | Pitch SAS rate-damping command added to the cyclic pitch. Same sign as `cyclicFwdAft`. Zero if the channel is off or primary hydraulics have failed. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcSasRollOut` | Number, ±0.1 cyclic fraction | local (owner only) | Roll SAS command added to the cyclic roll. Same sign as `cyclicLeftRight`. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcSasYawOut` | Number, ±0.1 pedal fraction | local (owner only) | Yaw SAS command added to the pedals. Same sign as `pedalLeftRight`. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcAttHoldCycPitchOut` | Number, ±1 cyclic fraction | local (owner only) | Attitude/position/velocity hold command added to the cyclic pitch. 0 when the hold is off, force trim is held, the channel is off, or primary hydraulics have failed. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcAttHoldCycRollOut` | Number, ±1 cyclic fraction | local (owner only) | Same as the pitch output, for cyclic roll. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcHdgHoldPedalYawOut` | Number, ±0.1 pedal fraction | local (owner only) | Heading hold / turn coordination command added to the pedals. Forced to 0 when the springless-pedal or auto-pedal setting is on. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcAltHoldCollOut` | Number, collective fraction. Range is set by the PID config. | local (owner only) | Altitude hold command added to the collective. 0 when the hold is off. (set in `fmc/fn_fmc.sqf`) |

### Pilot inputs

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_cyclicFwdAft` | Number -1..1, + = forward | local (pilot's machine) | Pilot cyclic pitch after the keyboard handling, the assists and the actuator lag. It is the stick displacement only: no trim, SAS or hold. Forced to 0 when flight-control hydraulics are lost (`bmkhs_fltCtrlsSupplied` false and `bmkhs_emerHydOn` false). With mouse-as-joystick it is multiplied by `bmkhs_mouseSense`. (set in `input/fn_inputUpdate.sqf`) |
| `bmkhs_cyclicLeftRight` | Number -1..1, + = LEFT | local (pilot's machine) | Pilot cyclic roll, worked out the same way as pitch. Calculated as left minus right. (set in `input/fn_inputUpdate.sqf`) |
| `bmkhs_pedalLeftRight` | Number -1..1, + = right pedal | local (pilot's machine) | Pilot pedal after actuator lag. It holds its last value when the tail rotor is unpowered or undriven (`bmkhs_tailRtrSupplied` / `bmkhs_tailRtrDriven` false). It is 0 when flight-control hydraulics are lost. (set in `input/fn_inputUpdate.sqf`) |
| `bmkhs_collectiveOutput` | Number 0..1, 0 = full down | local (pilot's machine) | Pilot collective position after actuator lag. It holds its last value while flight-control hydraulics are lost, unless emergency hydraulics are on, and while the game is not focused or a dialog is open. (set in `input/fn_inputUpdate.sqf`) |
| `bmkhs_forceTrimPosPitch` | Number -1..1, + = forward | net (owner), sometimes local only | Cyclic pitch trim position: where the stick rests. Set on force-trim release. Zeroed by force-trim reset. The auto attitude assist writes it every frame. In springless/sticky-keyboard mode it is set to 0 without being sent over the network. (set in `fmc/fn_fmcForceTrimSet.sqf`, `fmc/fn_fmcForceTrimReset.sqf`, `input/fn_inputAutoAttitude.sqf`) |
| `bmkhs_forceTrimPosRoll` | Number -1..1, + = left | net (owner), sometimes local only | Cyclic roll trim position. Same rules as pitch. (set in `fmc/fn_fmcForceTrimSet.sqf`, `fmc/fn_fmcForceTrimReset.sqf`) |
| `bmkhs_forceTrimPosYaw` | Number -1..1, + = right | net (owner), sometimes local only | Pedal trim position. When auto pedal is on, auto pedal writes it every frame. (set in `fmc/fn_fmcForceTrimSet.sqf`, `fmc/fn_fmcForceTrimReset.sqf`, `input/fn_inputAutoPedal.sqf`) |
| `bmkhs_autoAttCycRollOut` | Number, ±0.8 cyclic fraction | net | Roll command from the casual-mode auto attitude assist, added to cyclic roll at the rotor. It is 0 unless the auto-roll setting is on and realism is not REALISTIC. (set in `input/fn_inputAutoAttitude.sqf`) |
| `bmkhs_flightControlLockOut` | Bool | local (pilot's machine) | Only used with the center-trim mode settings. It is true after a force-trim release while the controls are off centre, and pilot cyclic/pedal input is ignored until they come back within ±0.05. A pack could show a "centre controls" cue from it. (set in `input/fn_inputCenterTrimMode.sqf`, `input/fn_inputUpdate.sqf`) |

The control position the rotor actually usesis not stored anywhere. `rotor/fn_rotorControl.sqf` (and `simpleRotor/fn_simpleRotorControl.sqf`) work it out each frame:
  - pitch = `inputGetInterp(cyclicFwdAft, forceTrimPosPitch) + fmcSasPitchOut + fmcAttHoldCycPitchOut + mixPitchOut`, clamped to -1..1
  - roll = `inputGetInterp(cyclicLeftRight, forceTrimPosRoll) + fmcSasRollOut + fmcAttHoldCycRollOut + autoAttCycRollOut + mixRollOut`, clamped to -1..1
  - yaw = `inputGetInterp(pedalLeftRight, forceTrimPosYaw) + fmcSasYawOut + fmcHdgHoldPedalYawOut + mixYawOut`, clamped to -1..1
  - collective = `collectiveOutput + fmcAltHoldCollOut`
  
  `inputGetInterp(stick, trim)` = `trim + (±1 - trim) * |stick|`. With the stick centred, the result is the trim position. To drive a control position indicator, a pack has to repeat this sum, and it can only do so on the owner/pilot machine. `ctrlVis/fn_ctrlVisUpdate.sqf` is a working example of reading these values.

### Core

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_initialised` | Bool | net | Set true once, by the local machine, in `core/fn_coreInit.sqf`. `controlSet` refuses to act until it is true. |
| `bmkhs_useSystems` | Bool | local (every machine that runs coreConfig) | Config `useSystems > 0`. With false: no component solve, no APU, no spring-back. Seeds read as "running". (`core/fn_coreConfig.sqf`) |

### CBA settings (globals)

Registered in `event/fn_eventPreInit.sqf`. These are missionNamespace globals, not vehicle variables.

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_helisimRealismSetting` | Number, 0 = CASUAL, 2 = REALISTIC | CBA setting | Realism level. |
| `bmkhs_helisimEnvironment` | Number 0..6 (ISA_STD, EUROPE_SUMMER, EUROPE_WINTER, MIDDLE_EAST, CENTRAL_ASIA_SUMMER, CENTRAL_ASIA_WINTER, ASIA) | CBA setting | Environment preset. |
| `bmkhs_rotorModel` | Number, 0 = Simple, 1 = BET | CBA setting | Rotor model. |
| `bmkhs_vrsWarning` | Bool | CBA setting | VRS warning on. |
| `bmkhs_sysDebug` / `bmkhs_fmDebug` / `bmkhs_engDisplay` / `bmkhs_forcesDebug` | Bool | CBA setting | Debug displays on. |
| `bmkhs_cyclicCenterTrimMode`, `bmkhs_pedalCenterTrimMode`, `bmkhs_springlessCyclic`, `bmkhs_springlessPedals`, `bmkhs_keyboardStickyPitch/Roll/Yaw`, `bmkhs_autoPedal`, `bmkhs_autoPitch`, `bmkhs_autoRoll`, `bmkhs_mouseAsJoystick` | Bool | CBA setting | Input options. |
| `bmkhs_mouseSense` | Number 0.1..1.0 | CBA setting | Mouse sensitivity. |
| `bmkhs_testGwtEnabled` / `bmkhs_testGwtLbs` | Bool / String (lb) | CBA setting | Fixed test gross weight. |
| `bmkhs_windDisabled` | Bool | CBA setting | Flight model ignores wind. |
| `bmkhs_ctrlVisColor` | Number 0..5 | CBA setting | Colour scheme for the control visualiser. |

### Events

Raised through `bmkhs_fnc_utilNotify`. It calls the handler registered for the aircraft's base class, as `[_heli, _event, _data]`. Events are raised only on the machine running the code. For per-frame events in the H-60, that is the owner.

| Event | Payload (`_data`) | Raised when |
|---|---|---|
| `controlMoved` | `[variableName (String, no prefix), newIdx, prevIdx, value (Number), positionClassName (String)]` | A control's index actually changes. Causes: a `controlSet`; a spring-back returning to rest (owner, `useSystems = 1`); or an external write to `bmkhs_<name>On` (owner, `useSystems = 1`). Not raised at init seeding. (`controls/fn_controlPublish.sqf`) |
| `apuStateChanged` | `[]` | `bmkhs_apuOn` differs from its value on the previous frame. Also raised on the first frame after init: the last-state memory starts at true, so if the APU is off then, the event fires. Only with `useSystems = 1`. Read `bmkhs_apuOn` for the state. (`systems/apu/fn_apu.sqf`) |
| `fuelCheckComplete` | `[]` | A running fuel check reaches `bmkhs_checkMinutes`. Raised after `checkDone`/`checkPendingAdvisory` are set, but before the burn rate and time strings are written in the same frame. (`fuel/fn_fuelMgmtUpdate.sqf`) |
| `holdModeDisengaged` | `[]` | Altitude hold drops because the collective moved more than 5 % from its reference (`fmc/fn_fmcAltitudeHold.sqf`). Or altitude hold is toggled off (`fmc/fn_fmcAltitudeHoldEnable.sqf`). Or attitude hold is toggled off (`fmc/fn_fmcAttitudeHoldEnable.sqf`). Or "hold modes off" is used while either hold is active (`fmc/fn_fmcHoldModesDisable.sqf`). Not raised when altitude hold drops because torque reaches 98 % or more. |

### Read functions

| Function | Params | Returns |
|---|---|---|
| `bmkhs_fnc_damageGet` | `[_heli, _role, _index = -1]` | Number 0..1. Damage on member `_index` of a role, or the worst member when -1. Returns 0 for a role nothing claims, or for a missing hitpoint (Arma's -1 is clamped). Works on any machine that has run coreConfig. |
| `bmkhs_fnc_damageCount` | `[_heli, _role]` | Number. How many hitpoints claim the role (`bmkhsRole` / `bmkhsRoleIndex`). 0 if none. If no hitpoint claims `engines`, there is one shared `hitengine` entry per `numEngines`. |
| `bmkhs_fnc_systemCircuit` | `[_heli, _circuit]` | Number. Current value of a circuit, in the config's unit; the highest feeder wins. Returns 0 for `""`, an unknown circuit, or nothing feeding it. Only meaningful on the owner with `useSystems = 1`: values are local and solved only there. `"Nr"` is fed from `bmkhs_rtrRpm`. |
| `bmkhs_fnc_controlSet` (write) | `[_name, _pos, _heli = vehicle player]`. `_pos` is a Number (absolute index) or a String step (`"+1"`, `"-1"`) | Bool, true if the control moved. False if HeliSim is not initialised, the name is unknown, an interlock blocks it, or it is already there. Indices are clamped, or wrap if `wraps = 1`. |
| `bmkhs_fnc_damageSet` (write) | `[_heli, _role, _damage 0..1, _index = -1]` | Nothing. Sets one member, or all of them when -1. Does nothing for an undeclared role. |
| `bmkhs_fnc_utilNotifyRegister` | `[_baseClass, _handler]` | Nothing. Registers `_handler` (called with `[_heli, _event, _data]`) for aircraft of that kind. Call from the pack's preInit. |
| `bmkhs_fnc_fuelTankVarName` (init helper) | `[_tankConfig, _kind, _index, _seenNames]` | String: the tank's variable prefix, e.g. `"bmkhs_no1Tank"`, or the fallback name. |

### Known gaps

Behaviour a designer will otherwise trip over. Each is a Core item, not a pack one.

- `bmkhs_engOilPsi` is a gauge fraction (Ng × 0.90 × oil health), not psi - the same units as
  `oilPsiLimits[]`.
- `bmkhs_rtrThrust` is written only by the BET rotor model; with the default Simple model it stays 0.
- Fuel tank masses (`bmkhs_<tank>Mass`, `bmkhs_totFuelMass`) are not networked.
- With `useSystems = 0` a spring-back position is never returned to rest.
- `apuStateChanged` fires once on the first frame after init even if nothing changed.
- `fuelCheckComplete` is raised before the burn rate and time strings are written that frame.
- `holdModeDisengaged` is not raised when altitude hold drops at 98% torque.
- `bmkhs_boostOn`, `bmkhs_checkStartZulu` and `bmkhs_checkElapsedSec` are seeded and never updated.
- Two control names are hard-coded: `apuBtn` (forced off when the APU stops or starves) and
  `apuFireHandle`.
- `bmkhs_prestonActive` is always false - the Preston AI is disabled.
- `bmkhs_fmcTrimOn` is stored but nothing in Core reads it.
- The control position the rotor uses is not published; a pack must rebuild it (Pilot inputs).


### Internal - do not read

Working state, solver bookkeeping, filters and debug. These change without notice.

- `bmkhs_engineInitialised` - one-shot init guard.
- `bmkhs_engNp` - raw Np state; use engPctNp.
- `bmkhs_engResidualHeat` - hot-section restart model state.
- `bmkhs_engPrevLever` - edge detect for lever.
- `bmkhs_engLeverSched` - governor fuel schedule working value.
- `bmkhs_engNpRef` - governor Np reference.
- `bmkhs_engLimFuel` - TGT/Ng limiter fuel allowance.
- `bmkhs_engMinFuel` - governor minimum-flow floor.
- `bmkhs_pid_engine` - governor PID objects.
- `bmkhs_engIdleSince` - useSystems=0 idle-to-fly timer.
- `bmkhs_engFailureResult` - useSystems=0 picked engine index.
- `bmkhs_engStarvedSince` - fuel starvation grace timer.
- `bmkhs_engSlipT` - clutch slip clock (systems).
- `bmkhs_engSlipWait` - clutch slip wait timer (systems).
- `bmkhs_engSlipDepth` - clutch slip depth (systems).
- `bmkhs_engTimer_<np|ng|tgt><engIdx>_<band>` - per-band exceedance accumulators.
- `bmkhs_shiftLocked` - stops shift spinning rotor.
- `bmkhs_lastTimePropagated` - 10 Hz broadcast timer.
- `bmkhs_gtDiagLast<idx>`, `bmkhs_gtDiagSw<idx>` - debug logging only.
- `bmkhs_govDiagLast<idx>` - debug logging only.
- `bmkhs_hotDiagLast_<name>` (missionNamespace) - debug logging only.
- `bmkhs_xmsnDiagLast` - debug logging only.
- `bmkhs_dbgForces` - forces debug readout.
- `bmkhs_rtrMoi` - rotor inertia for transmission.
- `bmkhs_numRotors` - BET rotor config mirror (WIP).
- `bmkhs_rotorType`, `bmkhs_rotorDirection`, `bmkhs_rotorNumBlades`, `bmkhs_rotorNumElements`, `bmkhs_rotorMastLength`, `bmkhs_rotorGearRatioArr`, `bmkhs_rotorPivot`, `bmkhs_rotorRotation`, `bmkhs_rotorFlapTimeConst`, `bmkhs_rotorAirfoil`, `bmkhs_rotorBladeCutout`, `bmkhs_rotorBladeLength`, `bmkhs_rotorBladeChordArr`, `bmkhs_rotorBladeTwist`, `bmkhs_rotorBladeMassArr`, `bmkhs_rotorDelta3`, `bmkhs_rotorPitchMin`, `bmkhs_rotorPitchMid`, `bmkhs_rotorPitchMax`, `bmkhs_rotorRollMin`, `bmkhs_rotorRollMid`, `bmkhs_rotorRollMax`, `bmkhs_rotorCollMin`, `bmkhs_rotorCollMid`, `bmkhs_rotorCollMax`, `bmkhs_rotorAnimSource`, `bmkhs_rotorHitPoint` - BET rotor config mirrors (WIP).
- `bmkhs_rotorFlapMoment` - BET per-blade working accumulator.
- `bmkhs_rotorBladeAzimuth` - BET per-blade working state.
- `bmkhs_rotorInducedFlow` - BET inflow filter state.
- `bmkhs_rotorInducedFlowAccum` - BET inflow accumulator (local only).
- `bmkhs_rotorReactionTorque` - BET accumulator; holds power (W).
- `bmkhs_rotorThrustAccum` - BET thrust accumulator.
- `bmkhs_rotorRateDampScalar` - BET tuning constant.
- `bmkhs_betMainLiftTable`, `bmkhs_betTailLiftTable` - BET tuning tables.
- `bmkhs_rotorBeta0`, `bmkhs_rotorA1`, `bmkhs_rotorB1` - BET flap state, degrees.
- `bmkhs_rotorBeta0Target`, `bmkhs_rotorA1Target`, `bmkhs_rotorB1Target` - BET flap filter targets.
- `bmkhs_prevLagInputPitch`, `bmkhs_prevLagOutputPitch`, `bmkhs_prevLagInputRoll`, `bmkhs_prevLagOutputRoll`, `bmkhs_prevLagInputYaw`, `bmkhs_prevLagOutputYaw`, `bmkhs_prevLagInputColl`, `bmkhs_prevLagOutputColl` - actuator lag filter state.
- `bmkhs_sysProducers` / `bmkhs_sysConverters` / `bmkhs_sysStorage` / `bmkhs_sysConsumers` / `bmkhs_sysNamed` / `bmkhs_sysTorqued` - parsed component tables, solver input.
- `bmkhs_sysCircuits` - circuit map; use `bmkhs_fnc_systemCircuit`.
- `bmkhs_sysReaders` / `bmkhs_sysWatchers` / `bmkhs_sysFeeds_of` - dependency graph for the walk.
- `bmkhs_sysFeeds` / `bmkhs_sysFeedIsProducer` - per-feeder circuit contributions.
- `bmkhs_sysProducerFeed_<circuit>` - producer-only total, used by storage.
- `bmkhs_sysWalkCost` / `bmkhs_sysWalkPeak` - solver cost, debug only.
- `bmkhs_sysWatchedLast` - gate values from the previous sweep.
- `bmkhs_systemsInitialised` - one-time seed guard.
- `bmkhs_repairPending` - repair trigger flag.
- `bmkhs_<comp>GateWhy` / `bmkhs_<comp>Why` / `bmkhs_<comp>Tgt` - debug explanation of the solve.
- `bmkhs_<comp>Awake` / `bmkhs_<comp>DmgLast` - solver wake bookkeeping.
- `bmkhs_<comp>Feed_<circuit>` - per-output contribution, debug.
- `bmkhs_<store>Charge` - raw 0..1 charge; read the published value instead.
- `bmkhs_<store>Drawn` - start-draw latch, solver internal.
- `bmkhs_tqTimer_<role><i>_<tier>` - per-tier over-torque clocks.
- `bmkhs_engClutchSlip` / `bmkhs_engSlipT` / `bmkhs_engSlipWait` / `bmkhs_engSlipDepth` - clutch-slip torque jitter state.
- `bmkhs_apuOnLast` - last APU state, for change detection.
- `bmkhs_<control>Held` / `bmkhs_<control>Awake` - spring-back hold bookkeeping.
- `bmkhs_<control>GateWhy` - interlock debug text.
- `bmkhs_ctrlList` / `bmkhs_ctrlIndex` - parsed control tables.
- `bmkhs_fuelTanks` / `bmkhs_auxTanks` - parsed tank tables.
- `bmkhs_fuelMains` / `bmkhs_fuelTransfers` - role-to-index lookups.
- `bmkhs_xferDestinations` / `bmkhs_xferFlowVars` / `bmkhs_fuelFlowVars` / `bmkhs_crossfeedSources` - parsed fuel config.
- `bmkhs_boostOn` - seeded, never read or written.
- `bmkhs_checkStartZulu` / `bmkhs_checkElapsedSec` - seeded, never updated by Core.
- `bmkhs_damagePoints` - role map; use damageGet/damageCount.
- `bmkhs_previousTime` / `bmkhs_deltaTime_avg` - frame timing state.
- `bmkhs_movingAverageSize` (global) - smoothing window constant.
- `bmkhs_keyboardCollective` / `bmkhs_keyboardCollectivePrevious` / `bmkhs_lastFrameGetIn` (global) - input path flags.
- `bmkhs_accelX_avg` - moving-average buffer for accelX.
- `bmkhs_accelY_avg` - moving-average buffer for accelY.
- `bmkhs_accelZ_avg` - moving-average buffer for accelZ.
- `bmkhs_aero_beta_g_prev` - low-pass filter state (broadcast anyway).
- `bmkhs_angVelModelSpaceX_avg` - angular-rate smoothing buffer.
- `bmkhs_angVelModelSpaceY_avg` - angular-rate smoothing buffer.
- `bmkhs_angVelModelSpaceZ_avg` - angular-rate smoothing buffer.
- `bmkhs_deltaTime` - Core frame step, capped 0.1 s.
- `bmkhs_deltaTime_avg` - frame-time smoothing buffer.
- `bmkhs_velModelSpaceX_avg` - velocity smoothing buffer.
- `bmkhs_velModelSpaceY_avg` - velocity smoothing buffer.
- `bmkhs_velModelSpaceZ_avg` - velocity smoothing buffer.
- `bmkhs_velWorldSpaceX_avg` - velocity smoothing buffer.
- `bmkhs_velWorldSpaceY_avg` - velocity smoothing buffer.
- `bmkhs_velWorldSpaceZ_avg` - velocity smoothing buffer.
- `bmkhs_velWorldSpaceNoWind_prev` - previous-frame value for differencing.
- `bmkhs_velX_prev` - previous-frame value for differencing.
- `bmkhs_velY_prev` - previous-frame value for differencing.
- `bmkhs_velZ_prev` - previous-frame value for differencing.
- `bmkhs_worldAccelX_avg` - acceleration smoothing buffer.
- `bmkhs_worldAccelY_avg` - acceleration smoothing buffer.
- `bmkhs_worldAccelZ_avg` - acceleration smoothing buffer.
- `bmkhs_perfDataChange` - change-detection key for perf recompute.
- `bmkhs_emptyMom` - raw config moment, frame-specific.
- `bmkhs_emptyMassVariants` - cached config table.
- `bmkhs_comCorrection` - config centre-of-mass offset.
- `bmkhs_casualModeCom` - config casual-mode centre of mass.
- `bmkhs_seats` - cached config table.
- `bmkhs_stations` - cached config table.
- `bmkhs_stores` - cached config table.
- `bmkhs_magazines` - cached config table.
- `bmkhs_equipment` - cached config table.
- `bmkhs_airfoils` - cached airfoil lift/drag tables.
- `bmkhs_wings` - cached wing geometry hashmaps.
- `bmkhs_numWings` - count of cached wings.
- `bmkhs_fuselageAirfoil` - cached config airfoil name.
- `bmkhs_fuselagePanels` - cached fuselage panel geometry.
- `bmkhs_pid_roll`, `bmkhs_pid_pitch`: position/velocity hold PID state.
- `bmkhs_pid_roll_att`, `bmkhs_pid_pitch_att`: attitude hold PID state.
- `bmkhs_pid_radHold`, `bmkhs_pid_barHold`: altitude hold PID state.
- `bmkhs_pid_hdgHold`, `bmkhs_pid_trnCoord`: heading/turn-coord PID state.
- `bmkhs_pid_sas_pitch`, `bmkhs_pid_sas_roll`, `bmkhs_pid_sas_yaw`: SAS PID state.
- `bmkhs_pid_autoAttPitch`, `bmkhs_pid_autoAttRoll`: assist PID state, casual only.
- `bmkhs_pid_autoPedalHdg`, `bmkhs_pid_autoPedalNtt`, `bmkhs_pid_autoPedalAero`: auto-pedal PID state.
- `bmkhs_posIntX`, `bmkhs_posIntY`: position hold integrator, clamped tiny.
- `bmkhs_mixes`: parsed `ControlMixing` table.
- `bmkhs_autoAttLevelPitch`, `bmkhs_autoAttRollLimit`: config copy for assist.
- `bmkhs_autoAttRollTarget`: assist roll setpoint, deg.
- `bmkhs_autoPedalHdg`: auto-pedal heading setpoint (networked).
- `bmkhs_autoPedalRegime`, `bmkhs_autoPedalRegimeWgt`: auto-pedal regime, weight always 1.
- `bmkhs_autoPedalHdgErr`, `bmkhs_autoPedalNttErr`, `bmkhs_autoPedalAeroErr`: auto-pedal loop errors.
- `bmkhs_autoPedalOut`: auto-pedal output; see forceTrimPosYaw.
- `bmkhs_kbPedalLeftRight`: auto-pedal keyboard lerp state.
- `bmkhs_cyclicPitchValue`, `bmkhs_cyclicRollValue`, `bmkhs_pedalYawValue`: sticky-keyboard accumulators.
- `bmkhs_prevCyclicPitchValue`, `bmkhs_prevCyclicRollValue`, `bmkhs_prevPedalYawValue`: sticky-keyboard shadow values.
- `bmkhs_kbStickyInterupt`: sticky interrupt key held.
- `bmkhs_kbHeliCollectiveRaiseOut`, `bmkhs_kbHeliCollectiveLowerOut`: raw keyboard collective key state.
- `bmkhs_heliCyclicForwardOut`, `bmkhs_heliCyclicBackwardOut`, `bmkhs_heliCyclicLeftOut`, `bmkhs_heliCyclicRightOut`: raw axis halves, pre-processing.
- `bmkhs_heliRudderLeftOut`, `bmkhs_heliRudderRightOut`: raw axis halves, pre-processing.
- `bmkhs_heliCollectiveRaiseOut`, `bmkhs_heliCollectiveLowerOut`: raw axis halves, pre-processing.
- `bmkhs_pid_prestonPitch`, `bmkhs_pid_prestonRoll`, `bmkhs_pid_prestonHoverX`, `bmkhs_pid_prestonHoverY`, `bmkhs_pid_prestonVelX`, `bmkhs_pid_prestonVelY`: Preston PIDs, AI disabled.
- `bmkhs_prestonPitchActive`, `bmkhs_prestonRollActive`: Preston internals, AI disabled.
- `bmkhs_prestonPitchTarget`, `bmkhs_prestonRollTarget`, `bmkhs_prestonVelCmdFwd`: Preston setpoints, AI disabled.
- `bmkhs_prestonPosWgt`, `bmkhs_prestonVelWgt`, `bmkhs_prestonAttWgt`: Preston regime weights, AI disabled.
- `bmkhs_prestonPrevPitch`, `bmkhs_prestonPrevRoll`, `bmkhs_prestonBreakout`: Preston filter state, AI disabled.
- `bmkhs_prestonHoverDatum`, `bmkhs_prestonHoverIntX`, `bmkhs_prestonHoverIntY`: Preston hover integrator, AI disabled.
- `bmkhs_prestonLearnedIntX`, `bmkhs_prestonLearnedIntY`: Preston learned trim, AI disabled.
- `bmkhs_dbgHovIntP`, `bmkhs_dbgHovIntR`, `bmkhs_dbgHovOutP`, `bmkhs_dbgHovOutR`: Preston debug readouts, never updated.
- `bmkhs_dbgHovSetX`, `bmkhs_dbgHovSetY`, `bmkhs_dbgHovVelX`, `bmkhs_dbgHovVelY`: Preston debug readouts, never updated.
- `bmkhs_dbgForces`: per-frame debug overlay scratch list.
- `bmkhs_ctrlvis`, `bmkhs_ctrlVisCircleW`, `bmkhs_ctrlVisColors`: uiNamespace overlay handle/cache.
- `bmkhs_engdisplay`: uiNamespace overlay display handle.
- `bmkhs_fmdebug`: uiNamespace overlay display handle.
