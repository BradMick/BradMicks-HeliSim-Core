# Building an aircraft on HeliSim

How to take a helicopter from nothing to a full flight model with modelled
systems. Follow it in order - each step depends on the ones before it.

**HeliSim Core (`bmkhs_helisim`) knows no airframe.** It provides functions and
reads declarations. Everything specific to your aircraft lives in your own
addon, which this guide calls the PACK. The AH-64's pack is
`fza_ah64_helisim`, and it is the worked example throughout.

Four field references sit beside this one and are the authority on what each
field means:

- `addons/helisim/components.hpp` - systems: producers, converters,
  storage, circuits, consumers
- `addons/helisim/controls.hpp` - switches, buttons and levers
- `addons/helisim/engine.hpp` - the gas turbine engine, station by station
- `addons/helisim/fmc.hpp` - the flight management computer: SAS and the holds

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

**Tick only what this machine owns - Core handles the rest.** The aircraft a
player is crewing but does not own is Core's job, not yours: Core passes it to
`coreUpdate` itself, which keeps it current from what its owner publishes
rather than solving it. Never widen your handler to include it - your
`fn_perFrame` would then run owner-only work, publishing included, on a machine
that does not own the aircraft.

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
    //With useSystems = 0 Core builds it: a transmission, plus a gearbox per engine on a
    //multi-engine aircraft (components.hpp).

    #include "bmkhs_config\helisim_airfoils.hpp"
    #include "bmkhs_config\helisim_engine.hpp"
    #include "bmkhs_config\helisim_flightControls.hpp"
    #include "bmkhs_config\helisim_fuel.hpp"
    #include "bmkhs_config\helisim_fuselage.hpp"
    #include "bmkhs_config\helisim_mass.hpp"
    #include "bmkhs_config\helisim_misc.hpp"      //the pack's own data, which Core does not read
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

**Inflow is read along the thrust.** The descent and VRS law takes the rotor's axial speed in
the direction it is actually pushing. A main rotor always pushes up its mast; a tail rotor
pushes either way with the pedal, and its sideslip is read against whichever way that is.

Outside REALISTIC Core sets every tail rotor upright - pitch 0, roll 90 - so a canted tail rotor
yaws without pitching or rolling the aircraft. Nothing to declare.

### Fitting the main rotor tables

The main rotor's `liftCoefTable` and `dragCoefTable` are solved in `python/dev/airframe.py` - the
rig, the virtual wind tunnel. The rig is a 1:1 port of Core's force path (`fn_simpleRotor`, the
fuselage, every wing and the stabilator schedule), so it IS the rotor model: fit the tables
through it as it is written. Do not generate them from outside rotor theory, and do not change
the rig to make a fit work - if the rig and the SQF disagree, the SQF is right.

The UH-60 is fitted this way, at sea level. (The EC665 Tiger, pack commit `5106547`, was fitted
by an earlier version of this method - rows built from fixed offsets, which put a step in the
power above its 0.64 row; it has not been refitted.)

The tables have two axes and each gets its own shape. Down the rows, collective: the blade's
lift curve, which rises, peaks and stalls (step 4). Across the columns, airspeed: the standard
power curve, which every column is fitted to (step 5).

**1. Targets.** From the aircraft's performance data, at its mid gross weight, sea level, 15 C,
out of ground effect:

| Target | What it sets | UH-60 (18,000 lb) |
|---|---|---|
| OGE hover: torque, and the collective it should sit near | the collective shape's hover row | 81%, ~0.64 (lands at 0.621) |
| Heavier OGE hovers: weight and torque | the collective shape above the hover | 22,000 lb 105%; 23,500 lb 115% (the heaviest) |
| Max endurance: speed and torque | the power curve's sag | 68 kt, 44% |
| Max range: speed, from the performance data | where the drag starts to climb (step 6.3) | 140 kt, 81% |
| Cruise attitude: the main rotor's mast tilt, as nose-low pitch | the front fuselage drag | 3 deg nose low at 140 kt |

Every aerodynamic value that sets these trims is solved for them together - the rotor tables,
the front fuselage drag and the stabilator schedule - not one at a time. Values Core takes as a
convention stay as Core has them: `flapBackPitchMax` and the other flap values keep their sign
and are not fitting variables.

**2. Mass and centre of mass.** Load the aircraft to the target weight from its own
`helisim_mass.hpp`, the way `fn_massUpdate` does:

- The empty airframe: `emptyMass`, its arm `fsDatum - emptyMom / emptyMass`.
- The variant's default equipment. A part is fitted when its animation source's `initPhase`
  (from the vehicle's `AnimationSources`, the variant's own `ANIM_INIT` overrides first) is on
  the same side of 0.5 as its `installedPhase`.
- Crew in their seats, then troops or cargo; magazine rounds at `massPerRound`; fuel in each
  tank at its arm. Trim the fuel to land on the target weight exactly.

Then, with every arm `{right, forward, up}`:

    longCG = (emptyMass * fsDatum - emptyMom + sum(mass * arm.forward)) / gwt
    latCG  = sum(mass * arm.right) / gwt
    CoM    = [latCG, longCG, 0] - boundingCenter + comCorrection

Give the rig `--bc` (`boundingCenter vehicle player`, from the game) and `--cg` (the CoM above)
and it solves cyclic for zero pitching moment. Without them it only balances forces, at a
cyclic you hand it, and the pitch attitudes it prints are not the aircraft's.

Check the CoM against the game before fitting: load the aircraft as above, then read
`getCenterOfMass vehicle player` in REALISTIC (CASUAL sets `casualModeCom` instead). Fit with the
game's value. The UH-60 at 18,000 lb reads [0.028, 1.787, 0.300]; the sum above gave 1.578, and
the logged attitudes matched only the game's.

**3. The stabilator - zero angle of attack at cruise.** An aircraft with a surface named
`stabilator` flies its `heliSimStabTable` in the rig, settled where `fn_wing` puts it.

At the cruise anchor the stabilator sits parallel to the airflow: zero angle of attack, carrying
no lift. In level flight that is parallel to the ground, so its incidence there equals the cruise
pitch attitude - and the cruise attitude is the main rotor's mast tilt. A mast tilted 3 deg
forward (`rotation[] = {-3, 0, 0}`) flies 3 deg nose low, and the stabilator reads -3 there
(negative is trailing edge down). The stabilator does not hold the attitude: a stabilator
trimming the aircraft with lift at cruise is fighting a rotor or airframe that is wrong.

The attitude itself comes from the force balance. The disc must lean forward far enough for its
thrust to pull against the airframe's drag, and the airframe follows the disc; with the rotor's
values as Core has them, the front fuselage drag sets how far. Solve it with the rotor tables
(step 5) so the cruise anchor trims at the mast tilt with the stabilator at that incidence. The
UH-60's front drag coefficient came out 0.497 at sea level, written flat across altitude.
`fn_fuselageFront` reads that table by `bmkhs_barAlt`; altitude is not fitted yet.

`--cruise-pitch` is the attitude and `--cruise-kt` the speed it is flown at, max range by
default; give the speed when the data gives the attitude at another - the fastest speed it gives
one for anchors the most drag. The AH-64D's is 10 deg nose low at 130 kt (max range 120 kt). Only the front drag moves
for it - a stabilator schedule already vetted against the real aircraft is kept as it is - and
the attitude at every other speed is whatever the force balance then gives, so check it against
what the aircraft flies rather than fitting it: the AH-64D should come out near 5 deg nose low
at 90 kt.

The schedule is rebuilt from the AH-64D's, the most complete stabilator schedule there is:
scaled about its -25 deg low-speed end so the cruise anchor's collective and speed read the
cruise attitude, which keeps its shape - trailing edge down at low speed against the rotor
wash, easing toward the incidence at speed. Core reads 14 columns: 30, 40, 50, 57.5, 80, 82.5,
100, 115, 120, 140, 150, 160, 165 and 180 kt.

**4. The collective shape - a blade's lift curve.** Down the rows, lift behaves like an airfoil's
lift against angle of attack: one straight line from collective 0 up to a peak, then a fall-off
as the blade stalls. Do not build it from offsets or an airfoil table - offsets put a step in
lift above a row, so a little collective buys a lot of thrust and torque stops answering density
and speed.

This step builds the HOVER COLUMN only - the 0 m/s column of both tables. Every rig call is a
level hover at 0 m/s, sea level (`density(0, 15)`), out of ground effect, at the fitting CoM.
Inputs: the hover points - weight and torque - from the lightest (the mid gross weight) to the
heaviest; the pack's current drag at collective 0; the peak collective `cp` (UH-60: 0.85).

Every hover point between the lightest and the heaviest gets its own drag row, so the more points
the data gives, the closer the hover torque follows it - give them all. The heaviest is the
peak: past it the blade stalls, so pick the most torque the aircraft can pull in a hover, not
its continuous limit, or the pilot runs out of collective at 100%. AH-64D: 18,000 lb 94%,
19,200 lb 100%, 20,260 lb 112%, 21,000 lb 125% (the peak).

If the aircraft's hover collective is known, solve `cp` to land it instead of choosing it:
`--hover-coll` takes the mid gross weight's hover collective and walks `cp` (a secant, starting
from `--peak`) until step 4.4 puts that weight there. The peak sets the slope of the lift line,
so it alone decides where each lighter weight lands. AH-64D: 0.64 at 18,000 lb.

CL (`liftCoefTable`), hover column:

1. Rows: `0`, `cp`, and three fall-off rows evenly spaced from `cp` to 1.0 (UH-60: 0, 0.85,
   0.90, 0.95, 1.00).
2. Write the column in terms of the peak value `CLp`:
   `CL(0) = 0.106 * CLp`, `CL(cp) = CLp`, then `0.957 * CLp`, `0.883 * CLp`, `0.766 * CLp`.
   Collective 0 to `cp` is then one straight line; past `cp` the blade stalls.
3. Solve `CLp`: the rotor's thrust at collective `cp` must equal the HEAVIEST hover weight, plus
   0.1% so that weight still trims just below the peak (exactly at it, rounding can leave it a
   hair short and the trim fails). Repeat `CLp = CLp * (Wmax * g * 1.001) / thrust(cp)` until
   it settles, where `thrust(cp)` is the main rotor's vertical force from
   `airframe.Airframe(...).forces` at 0 m/s, collective `cp`.
4. Trim each lighter hover weight in the rig (`trim` with `solveCyc`, 0 m/s). The collective it
   lands on is that weight's hover collective `ci` - it falls out of the line, it is not chosen.
   UH-60: 18,000 lb at 0.621, 22,000 lb at 0.786.

CD (`dragCoefTable`), hover column:

5. Rows: `0`, each hover collective `ci` from step 4 (ascending), `cp`, and the same three
   fall-off rows. The two grids are separate, so their rows need not match.
6. `CD(0)` stays the pack's own value - the torque at flat pitch, idle, does not move.
7. Solve `CD(ci)` for each lighter hover, and `CD(cp)` for the heaviest: trim each hover weight
   and repeat `CD = CD * torqueTarget / torqueTrimmed` for its row until every hover takes its
   torque (they couple, so iterate all together). Between rows drag runs in straight segments,
   which bend gently upward from one hover to the next.
8. Past the peak, the stall: `1.3 * CD(cp)`, `1.8 * CD(cp)`, `2.6 * CD(cp)` at the three fall-off
   rows, so full collective overtorques.

The UH-60's hover column:

| Collective | 0 | 0.621 (18,000 lb, 81%) | 0.786 (22,000 lb, 105%) | 0.85 peak (23,500 lb, 115%) | 0.90 | 0.95 | 1.00 |
|---|---|---|---|---|---|---|---|
| Lift | 0.0390 | (on the line) | (on the line) | 0.3666 | 0.3510 | 0.3237 | 0.2808 |
| Drag | 0.0078 | 0.0349 | 0.0455 | 0.0500 | 0.0650 | 0.0900 | 0.1300 |

Lift and drag are separate grids, so their rows need not match. Every airspeed column has this
same shape, scaled (step 6).

**5. The standard power curve.** Across the columns, torque against airspeed follows one
dimensionless shape - torque over the OGE hover torque, against airspeed over the max-range
speed. It is the mid gross weight line: a steep fall out of the hover as translational lift
builds, a flat bucket, then a hard climb as parasite power takes over:

| V / V_maxrange | 0 | .071 | .143 | .214 | .286 | .357 | .429 | .486 | .500 | .571 | .643 | .714 | .786 | .857 | .929 | 1.000 | 1.071 | 1.143 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| TQ / TQ_OGE | 1.000 | .914 | .778 | .667 | .593 | .568 | .543 | .543 | .543 | .543 | .568 | .617 | .679 | .765 | .864 | 1.000 | 1.210 | 1.407 |
| UH-60 kt | 0 | 10 | 20 | 30 | 40 | 50 | 60 | 68 | 70 | 80 | 90 | 100 | 110 | 120 | 130 | 140 | 150 | 160 |
| UH-60 TQ% | 81 | 74 | 63 | 54 | 48 | 46 | 44 | 44 | 44 | 44 | 46 | 50 | 55 | 62 | 70 | 81 | 98 | 114 |

Three set points place it for an aircraft: OGE hover torque `TQh`, max endurance (`Vme`, `TQme`)
and max range `Vmr`. The standard's own max range is where its torque is back to `TQh` - true of
the UH-60, not of every aircraft, so with your own curve take max range from the data. With
`S(x)` the standard row above and `x = V / Vmr`:

- Sag: `r = TQme / TQh`. Between the hover and max range (`x <= 1`) the curve deepens or
  flattens toward its floor in the same shape: `S'(x) = 1 - (1 - S(x)) * (1 - r) / (1 - 0.543)`.
  Past max range it keeps the standard: `S'(x) = S(x)`.
- Bucket position: the standard's bucket sits at `x = 0.486`. If `Vme / Vmr` differs, move it
  there by stretching the speed axis in two straight pieces, `0 -> 0.486` onto `0 -> Vme/Vmr` and
  `0.486 -> 1` onto `Vme/Vmr -> 1`.
- Torque target at any speed: `TQ(V) = TQh * S'(V / Vmr)`.

For the UH-60 at 18,000 lb (`TQh` 81%, `Vme` 68 kt at 44%, `Vmr` 140 kt) this gives back the
standard exactly.

**The aircraft's own curve.** When the performance data gives the whole curve at the mid gross
weight, fit to that instead - it is the aircraft, the standard is only a shape for when you have
three points. `--curve` takes it as `kt:torque%` pairs in place of `--me`, and every column is
fitted to it as written. It must start at 0 kt on the first `--hover` point's torque.

- Give max range with it, `--mr`, from the data. Without it, it is read off the curve where torque
  climbs back to the hover's - the standard's definition, and wrong for the AH-64D: 135.6 kt
  read, 120 kt real.
- Every point becomes a fitted column. Give every 10 kt the chart has; a speed it skips is
  filled in its shape - the AH-64D's 10 kt is 89.4%, the standard's share of the 0 to 20 kt fall.
- Run the curve to 160 kt, like the other packs. A chart that stops short is extended along its
  last slope: the AH-64D's stops at 140 kt (130 kt 85%, 140 kt 101%), so 150 kt is 117% and
  160 kt 133%. Those can sit above the hover peak's torque; step 6.3 raises the drag there so
  they are reachable.

**6. Solve the airspeed columns.** The hover column (step 4) is scaled into every other column:

1. Columns, in m/s: one at each power-curve speed - `kt * 0.514444` - plus one past the end at
   92.60 (180 kt). Core reads any column set. UH-60: 0, 5.14, 10.29, 15.43, 20.58, 25.72, 30.87,
   34.98, 36.01, 41.16, 46.30, 51.44, 56.59, 61.73, 66.88, 72.02, 77.17, 82.31, 92.60.
2. Each column `k` is the hover column times two numbers: `CL(row, k) = sL[k] * CL(row, 0)` and
   `CD(row, k) = sD[k] * CD(row, 0)`, every row. The hover column has `sL = sD = 1`.
3. `sD` is set, not solved - a collective is a torque, so a pilot holding collective holds
   torque through an acceleration. With `xe = 40/140` of max range (translational lift in):
   - through ETL, `V <= xe * Vmr`: `sD = 1 - drop * (TQh - TQ(V)) / (TQh - TQ(xe * Vmr))` -
     torque at a held collective falls by `drop` in the power curve's own shape (UH-60: 0.05);
   - from ETL to max range: `sD = 1 - drop`, flat;
   - past max range: up in a straight line to `top` at the last curve point. The stall caps
     every column's torque at what its drag gives at the peak collective, so `top` is whatever
     puts the top of the curve just below that: `1.03 * TQ(last) / TQ(peak hover)`, never less
     than 1. In forward flight the stall comes a little before the peak collective, so if the
     last column still falls short the tool raises `top` by the shortfall and solves again, and
     says so. UH-60: 114% against a 115% peak, `top` 1.02. AH-64D: 133% against 125%, `top` 1.10,
     raised to 1.15.
4. `sL` is solved: at the mid gross weight, sea level, 15 C, level flight at the column's speed,
   cyclic solved, must take `TQ(V)` from step 5. Drag fixes which collective gives that torque;
   lift decides whether that collective holds the weight - more lift, less collective, less
   torque. Solve each column by bracketing `sL` (a trim that fails is past the stall: too little
   lift), lowest speed first with the columns above riding along, then repeat passes over all
   columns until none moves. The power curve's sag is carried by collective: UH-60 0.620 in the
   hover, 0.267 in the bucket, 0.845 at 160 kt.
5. The extra columns (`--extra-cols`, default 180 kt alone): `sL` continues the line through
   the last two fitted columns; `sD` repeats the last.
6. Round to four places and write both tables: CL rows from step 4.1, CD rows from step 4.5, the
   columns above.

The UH-60 lands every power-curve point within 0.2%, and holding 0.58 collective gives 73.3%
from 40 to 140 kt. `rotortables.py --etl-drop` sets the drop. The table's columns are the rotor's
disc-plane speed, not the airspeed - the rig reads the trimmed velocity in the disc's own plane.

**Running steps 3 to 6 - `rotortables.py`.** One command does the cruise-attitude drag (step 3),
the hover column (step 4), the power curve (step 5) and every airspeed column (step 6), through
the rig, with the pack's own config:

```
set BMKHS_CONFIG=<pack>\config\bmkhs_config
python python/dev/rotortables.py --bc <boundingCenter> --cg <CoM> --hover <points> <curve> [options]
```

| Option | What it takes | Default |
|---|---|---|
| `--cg X Y Z` | The fitting CoM (step 2), model space. Required. | - |
| `--bc X Y Z` | `boundingCenter vehicle player`. | 0 0 0 |
| `--hover W:TQ ...` | OGE hover points, sea level, 15 C: weight (kg, or lb with an `lb` suffix) and torque %. The lightest is the mid gross weight, the heaviest the peak. | - |
| `--curve KT:TQ ...` | The aircraft's own power curve at the mid gross weight (step 5). Replaces `--me`. | - |
| `--mr KT` | Max range from the data. With `--curve`, give it; without, it is read off the curve (step 5). | - |
| `--me KT:TQ` | Max endurance, with `--mr` to place the standard curve instead of `--curve`. | - |
| `--peak C` | The peak collective `cp`. With `--hover-coll`, only the first guess. | 0.85 |
| `--hover-coll C` | The mid gross weight's hover collective: solves `cp` to land it (step 4). | - |
| `--etl-drop F` | How far torque at a held collective falls through ETL (step 6.3). | 0.05 |
| `--cruise-pitch DEG` | Cruise attitude, nose up positive: solves the front fuselage drag (step 3). | not solved |
| `--cruise-kt KT` | The speed `--cruise-pitch` is flown at, when the data gives the attitude near max range rather than at it. | max range |
| `--extra-cols KT ...` | Columns past the curve's last point, extrapolated rather than fitted (step 6.5). | 180 |

It prints, in order: max range; each peak tried, with `--hover-coll`; the hover collectives;
each front drag tried, with `--cruise-pitch`; both tables as `helisim_simpleRotor.hpp` blocks; the
front drag; then the written (rounded) tables flown against every curve point, every extra
column and every hover. It does not edit the pack - paste the blocks in, and write the front
drag flat across altitude in `helisim_fuselage.hpp`.

**When it fails, it says so.** Every column prints a line with its torque and time as it is
solved, so a slow one shows. Anything that misses or does not converge - a column that cannot
reach its torque, a secant out of steps, a written point or hover off by more than 0.5% (1% for
a hover) - prints `FAILED:` at once with what it got. The run ends with either `Fit OK` or a list
of every problem and exit code 1; do not paste tables from a run that ends in problems. Every
loop is bounded, so it cannot sit there indefinitely.

A plain fit takes under a minute (the AH-64D's, 58 s, including one raise of `top`).
`--hover-coll` reruns the hover column for each peak it tries, and `--cruise-pitch` reruns the
whole column solve for each drag it tries - starting from the pack's own drag and stepping 10%
from it - so with both expect a few minutes. Once they have answered, rerun with `--peak` and the
drag written into the pack. A failing trim costs about 50 times a good one, so a column that
takes many seconds is fighting the stall.

The AH-64D (the game's CoM at 18,000 lb; front drag 1.046 already in the pack, which trims
130 kt at 9.9 deg nose low and 90 kt at 5.1):

```
python python/dev/rotortables.py --bc -0.00205 -0.7246 1.39499 --cg 0.002 1.976 -0.895 ^
    --hover 18000lb:94 19200lb:100 20260lb:112 21000lb:125 --peak 0.766 ^
    --curve 0:94 10:89.4 20:82 30:72 40:62 50:54 60:50 70:48 80:49 90:52 100:56 110:63 ^
            120:72 130:85 140:101 150:117 160:133 --mr 120
```

Its first fit found the peak and the drag with `--hover-coll 0.64 --cruise-pitch -10 --cruise-kt 130`.

**7. Control mixing.** The collective mixes in `ControlMixing` (`CollectiveToYaw`,
`CollectiveToRoll`, `CollectiveToPitch`) are the hover trims of these tables, so they change
with them. Regenerate them with `forces.py` at the same CoM and the pack's compensation
fraction (the UH-60 uses 0.8, so the pilot still holds left pedal with power):

    python python/dev/forces.py --cg X Y Z --gwt KG --fraction 0.8

The yaw mixes (`YawToPitch`, `YawToRoll`) come from the tail rotor; they move only if its tables
did.

An aircraft without mechanical mixing - the AH-64D has none - declares no `ControlMixing`, and
this step does not apply.

**8. Check.** The written tables are rounded to four places: sweep again with the written
values, not the solver's. The fit is sea level; sweep at altitude (`--pa`, `--fat`) to see what
it does there. A trim that reports `NO` at collective 1.0 past the cruise anchor may
be the solver's starting guess, not the aircraft: the command line starts `trim` at pitch -1,
collective 0.6, cyclic 0, and a sweep starts each speed from the last one's answer. Start it from
the cruise anchor's trim before believing it:

```python
import airframe as A
af = A.Airframe(bc, cg)
t = A.trim(af, kt / A.MPS_TO_KNOTS, gwt, A.density(0, 15), 0, solveCyc=True,
           guess=(cruisePitch, cruiseColl, cruiseCyc))
```

Then fly it with the flight log on and `replay` the log through the rig.

```
set BMKHS_CONFIG=<pack>\config\bmkhs_config
python python/dev/airframe.py trim  --kt 140 --gwt 8165 --bc 0 0 0 --cg 0.028 1.787 0.300
python python/dev/airframe.py sweep --gwt 8165 --bc 0 0 0 --cg 0.028 1.787 0.300
python python/dev/airframe.py replay <Arma3.rpt> --gwt 8165
```

### The FMC

SAS, the attitude, altitude and heading holds and the flight director are declared in `class FMC`, beside the
flight control gains, one class per feature - `fmc.hpp` is the field reference. A feature
not declared does not exist. Each takes a `gate[]` in component form for what it needs to
work - the hydraulics its servos run on, the bus its computer runs on - and drops out while
the gate is shut. Its gains, switch speeds, breakouts and authority are the aircraft's.

```cpp
class FMC {
    class Sas {
        gate[]      = {{"UTIL_HYD", 1260}, "bmkhs_dcBusOn"};
        authority[] = {0.2, 0.1, 0.1};
        pitch[] = {...}; roll[] = {...}; yaw[] = {...};
    };
};
```

The flight director's modes and targets are Core's own actions (`bmkhs_fd<Mode>`,
`bmkhs_fd<Target>Up/Dn/Sync`, `bmkhs_fd<Target>Target`). A cockpit button or knob calls them
through `bmkhs_fnc_inputControlHandle` / `bmkhs_fnc_inputAnalogHandler` by the same names a
keybind uses, and passes the aircraft. A dragged knob passes its position as a fraction of the
target's range - the pack converts its animation to that, Core does the rest. The aircraft
writes `bmkhs_fdWaypoint` for NAV.

The director works in m, m/s and deg and publishes its targets that way; its config is in
pilot units (ft, kt, fpm), converted as read. A readout converts the target to whatever the
cockpit shows. Every command is eased onto at a declared rate, so a target step never steps a
control. ALT and ALTP fly pressure altitude, `bmkhs_barAlt` - the environment's base altitude
included, as the barometric altimeter reads it; RALT flies radar height. HVR slows the aircraft to a stop at `hvrDecelKts` and engages the attitude hold's
position hold over the spot; it has the cyclic, so HDG / NAV turn by pedal under it.

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

A `"collective"` source is the collective the rotor gets - the pilot's plus what the altitude
hold or flight director adds - so the mixes work against coupled power changes too.

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
- **Arma's default hitpoints stand in where no role is claimed:** `hitengine`
  for every engine, `hithrotor` for `mainRotor`, `hitvrotor` for `tailRotor`.
  An aircraft config-patched onto another mod declares none of these.
- **`transmission` is required with `useSystems = 1`** - logged at load as a
  DAMAGE CONFIG ERROR if missing. With `useSystems = 0` Core keeps the
  drivetrain's damage itself.
- **A gearbox per engine with no gearbox hitpoints:** declare it with no
  `damageRole` and `perEngine = 1`, and Core keeps its damage
  (`components.hpp`).

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

Each control publishes `bmkhs_<name>Idx`, `Val`, and `On` (`Val != 0`), plus one flag
per position, `bmkhs_<name>_<Position>`, true while it sits there. A gate names a switch
position with that flag: `gate[] = {"bmkhs_airSource_Apu"}` holds only with AIR SOURCE at APU.

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

**Ask Core, don't re-check.** `[_heli, "eng1PwrLvr", 2] call bmkhs_fnc_controlAllowed`
answers whether a control may move to a position - the same check Core makes when it moves
it. A cockpit that refuses a click before animating asks this; it never re-implements the
interlocks.

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

**The flight controls and the FMC are Core's own actions.** Cyclic, pedals, collective, force
trim, the hold modes and the flight director are in Core's `CfgUserActions`, in its "HeliSim
Flight Controls" group. Your pack ships no rows for them; a cockpit button or knob calls Core's
action by name (see the cockpit framework, Step 7).

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

**Core drives two sound controllers** for the translational lift, high speed and VRS
effects: `CustomSoundController64` (intensity) and `CustomSoundController63` (blend, 0
outside every band). Use them in your sound config for those effects; leave them free
for anything else.

### When the cockpit framework animates

Some interaction frameworks (Hatchet's lever interactions) animate a control
themselves when it is clicked. Then the framework, not `controlMoved`, is the
animator, and these hold:

- **One animator.** The framework moves every cockpit control - clicks,
  keybinds and linked controls alike. The pack never animates one as well, or the
  two run on different clocks.
- **Tell Core when the move starts, not when it ends.** Call `controlSet` from
  the framework's start hook, so the engine and the lever move together.
- **Refuse before moving.** Ask `bmkhs_fnc_controlAllowed` in the framework's
  pre-move condition, so a refused control never moves rather than snapping back.
  Core answers; the cockpit never re-checks the interlocks itself.
- **`controlSet` runs where the aircraft is local.** A click from another seat is
  forwarded to the owner.
- **Linked controls go through the framework too.** Two power levers advancing
  together are both moved by the framework, at one rate.
- **Match the framework's rate to Core's.** A lever's travel time is the
  engine's `leverTravelTime`.
- **FMC buttons and knobs send Core's action.** A button calls
  `bmkhs_fnc_inputControlHandle` with the action's name (`bmkhs_fdAlt`); its light follows
  `fdModeChanged`. Forward it to the owner, as `controlSet` is.
- **A dragged knob passes a fraction.** On the framework's drag hooks, turn the knob's
  animation into a fraction 0..1 of the target's range and call
  `bmkhs_fnc_inputAnalogHandler` with the target's action (`bmkhs_fdAltTarget`) - the same
  input an axis gives. Core clamps, wraps, snaps and publishes. Turn the knob to Core's
  published target when it changes from elsewhere (a sync, a step, a capture), but not while
  it is being dragged.

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

**Running state travels packed.** What changes continuously - engine speeds,
temperatures, the governor's terms, drivetrain rpm - is sent by the owner as the
one variable `bmkhs_netState`, 10 times a second (`core/fn_coreNetSend.sqf`), and
unpacked on the crew's machines (`core/fn_coreNetReceive.sqf`, which `coreUpdate`
runs for an aircraft this machine does not own - Core schedules that one itself,
from `event/fn_eventPreInit.sqf`). The list is
`core/netState.hpp`: everything a crew station displays, and everything a new
owner needs to carry on rather than start cold. A value the model carries from one
frame to the next and that a handover must not reset belongs in it.

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
- **Where it exists.** The pack calls `bmkhs_fnc_coreUpdate`, on the machine where the aircraft
  is local (see Step 2); the only aircraft Core schedules itself is the one a player crews but does
  not own, which it keeps current from the packed state rather than solving. **local** means the value is written only
  there; every other machine sees its init seed or nil. Anything another crew station displays
  must be **net**.
- **net** means Core publishes it. *On change* - sent when the value changes (through
  `bmkhs_fnc_utilUpdateNetworkGlobal` / `bmkhs_fnc_utilSetArrayVariable`). *10 Hz* - in the
  packed running state (`core/netState.hpp`) the owner sends every 0.1 s in multiplayer, unpacked
  only on the machines of the crew of that aircraft. *Every frame* -
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
| `bmkhs_engState` | array of strings, per engine: `"OFF"`, `"STARTING"`, `"ON"` | net on change + 10 Hz | Engine run state. Goes `STARTING` on a start (start switch at +1 with useSystems = 1, or Arma engine-on with useSystems = 0). Goes `ON` when Ng reaches `selfSustNg`. Goes `OFF` on ignition override (-1) during a start, lever to OFF while ON, an engine failure, an overspeed trip, fuel starvation, or no bleed air (`bmkhs_pneuAvail` false) during a start (set in `engine/fn_engineUpdate.sqf`, `engine/gasTurbine/fn_gasTurbineStarter.sqf`). |
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
| `bmkhs_reqEngTorque` | array of numbers, per rotor, Nm at the engine shaft | 10 Hz | Rotor torque demand referred to the engine shaft, filtered. Sum it for total load. Seeded as 2 slots (set in `simpleRotor/fn_simpleRotorTorque.sqf`, or `rotor/fn_rotor.sqf` with the BET model). |
| `bmkhs_rtrThrust` | array of numbers, per rotor, N | net on change | Rotor thrust. Only written by the BET rotor model (`bmkhs_rotorModel` = 1). With the Simple model (default) it stays 0 (set in `rotor/fn_rotor.sqf`). |

### Controls - named by your config

Each class under the aircraft's `Controls` gives one control. `variableName` names it, and Core adds the `bmkhs_` prefix. A control with no `Positions` is skipped. Positions are indexed 0..n-1 in the order they are declared.

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_<variableName>Idx` | Number, 0-based position index | net if the control's `networked = 1`, else local to the machine that moved it | The current position, and the one value Core treats as canonical. Seeded at `rest` on every machine that runs coreConfig, with a plain local setVariable. After that it is published only when it changes. (set in `controls/fn_controlsVariables.sqf`, `controls/fn_controlPublish.sqf`) |
| `bmkhs_<variableName>Val` | Number, the position's config `value` | same as Idx | The declared `value` of the current position. (set in `controls/fn_controlPublish.sqf`) |
| `bmkhs_<variableName>_<Position>` | Bool, one per position class | same as Idx | True while the control sits in that position. How a gate, interlock or mix gate names a switch position - `bmkhs_airSource_Apu`. Seeded at `rest`, published with Idx. (set in `controls/fn_controlsVariables.sqf`, `controls/fn_controlPublish.sqf`) |
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
| `bmkhs_apuRpm_pct` | Number, 0..1 fraction (H-60 `nominal = 1.0`) | net | Seeded 0. Changes only if a producer declares this name (H-60: `apuRPM_pct`). Also in the packed 10 Hz running state (`core/netState.hpp`). |
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
| `bmkhs_<variableName>[n]Dmg` | Number, 0..1 | local | Damage of a torque-rated part with no damage role, which Core keeps instead of a hitpoint - every useSystems = 0 drive part (`bmkhs_noseGearbox<n>Dmg`, `bmkhs_transmissionDmg`), and a declared `perEngine` gearbox. Accrued in `systems/fn_systemTorque.sqf`; set to 0 by a repair. |

### State and air data

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_vel2D` | Number, m/s, clamped 0 to 180 kt | local (owner only) | Indicated-style airspeed. Forward (y) component of the air-relative model-space velocity. Never negative. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_vel3D` | Number, m/s | local (owner only) | Magnitude of the air-relative model-space velocity. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_gndSpeed` | Number, m/s | local (owner only) | Ground speed. Magnitude of model-space x and y ground velocity (no wind). Body axes, so it reads low when pitched or rolled. (set in `state/fn_stateVelocities.sqf`) |
| `bmkhs_velClimb` | Number, m/s | local (owner only) | Vertical speed. World z of smoothed velocity. Positive = climbing. Wind has no vertical part, so this is ground-referenced. (set in `state/fn_stateVelocities.sqf`) |
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
| `bmkhs_aero_beta_g` | Number, g, clamped -1 to +1 | 10 Hz | Trim-ball value: `bmkhs_bodyAccel # 0` / 9.806, then first-order low-pass (tau 0.60 s). Positive = lateral specific force to the right, so a physical ball sits LEFT; negative = ball RIGHT. Core autopilot code relies on this raw sign; flip it only in your display. The AH-64D pack also blends the display sign with speed (`fn_avionicsSlipIndicator.sqf`). (set in `state/fn_stateAeroValues.sqf`) |
| `bmkhs_aero_beta_deg` | Number, degrees | 10 Hz | Aerodynamic sideslip: asin(x / |v|) of `bmkhs_velModelSpace`. Positive = aircraft moving right through the air (relative wind from the right). 0 when the velocity is zero. (set in `state/fn_stateAeroValues.sqf`) |
| `bmkhs_accelX` | Number, m/s², body x (right) | local (owner only) | Smoothed time derivative of `bmkhs_velModelSpaceNoWind # 0`. Gravity not included. Derivative of a body-axis velocity, so rotation terms are included as they fall. (set in `state/fn_stateAccelerations.sqf`) |
| `bmkhs_accelY` | Number, m/s², body y (forward) | local (owner only) | Same as above for the forward axis. (set in `state/fn_stateAccelerations.sqf`) |
| `bmkhs_accelZ` | Number, m/s², body z (up) | local (owner only) | Same as above for the up axis. (set in `state/fn_stateAccelerations.sqf`) |
| `bmkhs_radAlt` | Number, metres, exact | 10 Hz | Height above the ground, `getPos _heli # 2`, unrounded and unclamped. A radar altimeter's steps and range are the reader's to apply - Core publishes no display values. (set in `state/fn_stateAltitude.sqf`) |
| `bmkhs_rtrRpm` | Number, ratio (1.0 = 100 % Nr) | local (owner only) | Rotor speed: `bmkhs_xmsnOutputRpm` / `bmkhs_engDesignRpm`. Forced to 0 when main rotor damage is 1.0. (set in `state/fn_stateRtrRpm.sqf`) |

Ground contact is not a variable. Call `[_heli] call bmkhs_fnc_stateOnGround`. It returns true when `isTouchingGround` is true or `bmkhs_radAlt` < 0.15 m (`state/fn_stateOnGround.sqf`). It works only where `bmkhs_radAlt` is updated (the owner).

### Environment

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_barAlt` | Number, feet, exact | 10 Hz | Pressure altitude - what the barometric altimeter reads, before any display rounding, which is the reader's. MSL height in feet plus a base altitude set by the CBA setting `bmkhs_helisimEnvironment` (ISA 0, Europe 800, Middle East 1800, Central Asia 5000, Asia 3100 ft). Altimeter setting is fixed at 29.92 inHg; mission weather does not change it. (set in `environment/fn_environment.sqf`) |
| `bmkhs_fat` | Number, °C, exact | 10 Hz | Free air temperature. Base temperature of the selected environment (ISA 15, Europe summer 20 / winter 0, Middle East 30, Central Asia summer 30 / winter -5, Asia 25) minus 2 °C per 1000 ft of MSL height. (set in `environment/fn_environment.sqf`) |
| `bmkhs_rho` | Number, kg/m³ | local (owner only) | Dry air density from barometric pressure at `bmkhs_barAlt` and `bmkhs_fat`, both exact (p / (287.05 × T)). Init value is 1.225. (set in `environment/fn_environment.sqf`) |
| `bmkhs_windSpeed` | Number, m/s | 10 Hz | Mission wind speed (`vectorMagnitude wind`). 0 when `bmkhs_windDisabled` is set. (set in `environment/fn_environment.sqf`) |
| `bmkhs_windDirFrom` | Number, degrees true 0-359, integer | 10 Hz | The direction the wind blows FROM - the meteorological convention a pilot reads (a wind from the west is 270). It is already converted from Arma's `windDir`, `(windDir + 180) mod 360` - a readout of where the wind is from uses it as published. A wind arrow drawn pointing the way the wind BLOWS needs the opposite, `(bmkhs_windDirFrom + 180) mod 360`, converted in the pack (the UH-60's PFD / ND arrows do this). 0 when `bmkhs_windDisabled` is set. (set in `environment/fn_environment.sqf`) |
| `bmkhs_velWindWorldSpace` | Array [east, north, 0], m/s | local (owner only) | Wind velocity vector (direction the air moves toward). [0,0,0] unless `bmkhs_rotorModel == 0`; also zero when wind is disabled. (set in `environment/fn_environment.sqf`) |

Pressure (hPa) and density altitude are computed in `fn_environment.sqf` but not stored.

### Mass and balance

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_gwt` | Number, kg | 10 Hz | Gross mass: empty mass (or matching `EmptyMassVariants` entry), occupied seats, fitted equipment, internal fuel, internal magazine rounds, and wing-station stores and external fuel. Written only by the owner. When the CBA test-GWT option is on, replaced by that weight clamped between empty and `maxGrossMass`. All contributors come from config. (set in `mass/fn_massUpdate.sqf`) |
| `bmkhs_cg` | Number, metres, longitudinal only | 10 Hz | Longitudinal CG: total forward moment / mass, in the same frame as the config arms (H-60 config: arm = {right, forward, up} m; larger = further forward). Compare directly with `bmkhs_fwdCgLimit` / `bmkhs_aftCgLimit`. Not a fuselage station; convert with `bmkhs_fsDatum` if needed. In test-GWT mode it is real moments divided by the test mass. Lateral CG is not published. (set in `mass/fn_massUpdate.sqf`) |
| `bmkhs_fwdCgLimit` | Number, metres, same frame as `bmkhs_cg` | local (owner only) | Forward CG limit, read from config `fwdCgLimit`. Static. (set in `mass/fn_massVariables.sqf`) |
| `bmkhs_aftCgLimit` | Number, metres, same frame as `bmkhs_cg` | local (owner only) | Aft CG limit, read from config `aftCgLimit`. Static. (set in `mass/fn_massVariables.sqf`) |
| `bmkhs_fsDatum` | Number, metres | local (owner only) | Fuselage-station 0 reference, config `fsDatum`. Empty-airframe arm = fsDatum − emptyMom/emptyMass. Static. (set in `mass/fn_massVariables.sqf`) |
| `bmkhs_emptyMass` | Number, kg | local (owner only) | Config `emptyMass`. Does not reflect `EmptyMassVariants`; the variant is only applied inside the gross-weight sum. Static. (set in `mass/fn_massVariables.sqf`) |
| `bmkhs_maxGrossMass` | Number, kg | local (owner only) | Config `maxGrossMass`. Used to bound the test weight. Static. (set in `mass/fn_massVariables.sqf`) |

### Stabilator

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_stabilatorPosition` | Number, degrees (per config `heliSimStabTable`) | local (owner only) for live value | Stabilator incidence. Moves toward the `heliSimStabTable` value (by collective and `bmkhs_vel2D`) at a lerp rate of (1/1.5) × dt. Frozen when stabilator damage ≥ `SYS_STAB_DMG_THRESH` or `bmkhs_dcBusOn` is false. Only the init 0 is broadcast; per-frame updates are not. Sign is that of the config table; the same value drives the `Hstab` animation source. Only updated for a wing named "stabilator". (set in `wing/fn_wing.sqf`, init in `wing/fn_wingVariables.sqf`) |

Fuselage and airfoil folders publish no designer-facing values.

### Flight management computer and hold modes

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_fmcPitchOn` | Bool, default true | net | FMC pitch channel on. When false, the pitch SAS and attitude hold pitch outputs are zeroed (the flight director's too, when it holds pitch), and the actuator also uses this flag for its lag model. Set with `bmkhs_fnc_fmcSetChannel [heli,"pitch",bool]`. (set in `fmc/fn_fmcVariables.sqf`, `fmc/fn_fmcSetChannel.sqf`) |
| `bmkhs_fmcRollOn` | Bool, default true | net | FMC roll channel on. Same as pitch, for the roll SAS and attitude hold roll outputs. (set in `fmc/fn_fmcVariables.sqf`, `fmc/fn_fmcSetChannel.sqf`) |
| `bmkhs_fmcYawOn` | Bool, default true | net | FMC yaw channel on. When false, the yaw SAS and heading hold outputs are zeroed (the flight director's too). (set in `fmc/fn_fmcVariables.sqf`, `fmc/fn_fmcSetChannel.sqf`) |
| `bmkhs_fmcCollOn` | Bool, default true | net | FMC collective channel on. When false, the altitude hold output is zeroed (the flight director's too). (set in `fmc/fn_fmcVariables.sqf`, `fmc/fn_fmcSetChannel.sqf`) |
| `bmkhs_fmcSasAvail`, `bmkhs_fmcAttHoldAvail`, `bmkhs_fmcAltHoldAvail`, `bmkhs_fmcHdgHoldAvail`, `bmkhs_fmcFdAvail` | Bool | net on change | The feature is declared in `class FMC` and its `gate[]` holds. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fd_<mode>` | Bool, one per declared mode | net on change | Flight director mode engaged (`ralt`, `alt`, `altp`, `ias`, `hdg`, `nav`, `hvr`). (set in `fmc/fn_fmcFdMode.sqf`) |
| `bmkhs_fdTgt_<target>` | Number, m / m/s / deg | net on change | Flight director target, clamped or wrapped and snapped to its declared step. (set in `fmc/fn_fmcFdTarget.sqf`) |
| `bmkhs_fdWptBearing`, `bmkhs_fdWptDistance` | Number, deg / m, -1 with no waypoint | net on change | To `bmkhs_fdWaypoint`. NAV flies the bearing. (set in `fmc/fn_fmcFlightDirector.sqf`) |
| `bmkhs_fdWaypoint` | Array posASL, or [] | input | The aircraft's active waypoint. The aircraft writes it; seeded [] locally if unset. |
| `bmkhs_fmcTrimOn` | Bool, default true | net | Trim channel flag. Core stores it but never reads it, so it changes nothing in Core. A pack can use it as a switch state. (set in `fmc/fn_fmcVariables.sqf`, `fmc/fn_fmcSetChannel.sqf`) |
| `bmkhs_forceTrimInterupted` | Bool | net | True while the force-trim (trim release) button is held. While true, attitude hold does nothing and heading hold drops out. On release it goes false and new hold references are captured. Note the spelling ("Interupted"). (set in `fmc/fn_fmcForceTrimHold.sqf`, `fmc/fn_fmcForceTrimRelease.sqf`) |
| `bmkhs_attHoldActive` | Bool | net | Attitude/position/velocity hold engaged. Toggled by the `bmkhs_holdModeAttitude` key. Cleared by `bmkhs_holdModesOff`. This is the mode state only. It stays true even when the pitch/roll channel is off or the hold's `gate[]` is shut, and both of those zero the output. (set in `fmc/fn_fmcAttitudeHoldEnable.sqf`, `fmc/fn_fmcHoldModesDisable.sqf`) |
| `bmkhs_attHoldSubMode` | String `"pos"` / `"vel"` / `"att"` | net | Which attitude hold law applies. Chosen every frame from ground speed, even when the hold is off: `pos` at `posBelowKts` GS or less; `vel` up to `velBelowKts` accelerating, `att` above it until back below `attBelowKts` decelerating (`class FMC >> AttitudeHold`). (set in `fmc/fn_fmcAttitudeHold.sqf`, `fmc/fn_fmcAttitudeHoldEnable.sqf`) |
| `bmkhs_attHoldDesiredPos` | Array `getPos` [x,y,z], m | net | Position hold reference. Captured on engage in `pos`, and again on force-trim release. (set in `fmc/fn_fmcAttitudeHoldEnable.sqf`, `fmc/fn_fmcForceTrimRelease.sqf`) |
| `bmkhs_attHoldDesiredVel` | Array [x, y], m/s, body axes | net | Velocity hold reference. x is the NEGATED model-space lateral velocity, so + = left. y is forward velocity, + = forward. Reset to [0,0] by hold-modes-off. (set in `fmc/fn_fmcAttitudeHoldEnable.sqf`, `fmc/fn_fmcForceTrimRelease.sqf`, `fmc/fn_fmcHoldModesDisable.sqf`) |
| `bmkhs_attHoldDesiredAtt` | Array [pitch, bank], deg | net | Attitude hold reference, as returned by `BIS_fnc_getPitchBank`. When turn coordination ends, heading hold sets bank to 0. (set in `fmc/fn_fmcAttitudeHoldEnable.sqf`, `fmc/fn_fmcForceTrimRelease.sqf`, `fmc/fn_fmcHeadingHold.sqf`, `fmc/fn_fmcHoldModesDisable.sqf`) |
| `bmkhs_altHoldActive` | Bool | net | Altitude hold engaged. Toggled by the `bmkhs_holdModeAltitude` key, which only engages within `engageFpm` of level. It drops out on its own when collective moves more than `collBand` from `altHoldCollRef`, or when the highest engine torque (`bmkhs_engPctTq`) reaches `dropAboveTq` (`class FMC >> AltitudeHold`). (set in `fmc/fn_fmcAltitudeHold.sqf`, `fmc/fn_fmcAltitudeHoldEnable.sqf`, `fmc/fn_fmcHoldModesDisable.sqf`) |
| `bmkhs_altHoldSubMode` | String `"rad"` / `"bar"` | net | Radar or barometric altitude hold. While engaged it is `rad` below `radBelowFt` AGL and under `radBelowKts` GS, and `bar` otherwise. It is only re-evaluated while the hold is engaged. (set in `fmc/fn_fmcAltitudeHold.sqf`, `fmc/fn_fmcAltitudeHoldEnable.sqf`) |
| `bmkhs_altHoldDesiredAlt` | Number, m (AGL in `rad`, ASL in `bar`) | net | Altitude hold reference, rounded to the metre on engage. It is 0 when the hold is off. Engage chooses AGL or ASL once. The sub-mode can switch later without the reference being re-captured. (set in `fmc/fn_fmcAltitudeHoldEnable.sqf`, `fmc/fn_fmcHoldModesDisable.sqf`) |
| `bmkhs_altHoldCollRef` | Number 0..1 (collective) | net | Collective position captured when altitude hold engaged. Used for the `collBand` disengage band. (set in `fmc/fn_fmcAltitudeHoldEnable.sqf`) |
| `bmkhs_hdgHoldActive` | Bool | net | Heading hold engaged. There is no button for it. It is true whenever the aircraft is off the ground, force trim is not held, and pedal input is inside the breakout (`class FMC >> HeadingHold >> breakout[]`, by the attitude hold sub-mode, `pos` / `vel` / `att`). The heading is re-captured each time it re-engages. (set in `fmc/fn_fmcHeadingHold.sqf`) |
| `bmkhs_hdgHoldSubMode` | String `"hdg"` / `"trn"` / `"yaw"` / `"aut"` | net | Heading hold law, only updated while engaged. `hdg` = hold heading, below `hdgBelowKts` GS. `trn` = turn coordination: attitude hold on and bank over 7° (drops at under 3°). `yaw` = ball-centring yaw damping. `aut` = the auto-pedal assist owns the yaw axis. (set in `fmc/fn_fmcHeadingHold.sqf`) |
| `bmkhs_hdgHoldDesiredHdg` | Number, deg 0..360 (`getDir`) | net | Heading hold reference. Captured when the hold engages, when it re-enters `hdg`, and on force-trim release. (set in `fmc/fn_fmcHeadingHold.sqf`, `fmc/fn_fmcForceTrimRelease.sqf`) |
| `bmkhs_hdgHoldDesiredSideslip` | Number, lateral g | net | Sideslip (ball) reference. Core always sets it to 0. (set in `fmc/fn_fmcForceTrimRelease.sqf`) |
| `bmkhs_mixPitchOut` | Number, cyclic fraction, + = forward | local (owner only) | Sum of the aircraft's `ControlMixing` mixes targeting pitch, added to the cyclic pitch at the rotor. 0 with no mixes, while a mix's gate is shut, or outside REALISTIC. (set in `fmc/fn_fmc.sqf`, from `fmc/fn_fmcControlMixing.sqf`) |
| `bmkhs_mixRollOut` | Number, cyclic fraction, + = left | local (owner only) | As above, for roll. |
| `bmkhs_mixYawOut` | Number, pedal fraction, + = right | local (owner only) | As above, for the pedals. |
| `bmkhs_fmcSasPitchOut` | Number, ±0.2 cyclic fraction | local (owner only) | Pitch SAS rate-damping command added to the cyclic pitch. Same sign as `cyclicFwdAft`. Zero if the channel is off or the SAS `gate[]` is shut. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcSasRollOut` | Number, ±0.1 cyclic fraction | local (owner only) | Roll SAS command added to the cyclic roll. Same sign as `cyclicLeftRight`. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcSasYawOut` | Number, ±0.1 pedal fraction | local (owner only) | Yaw SAS command added to the pedals. Same sign as `pedalLeftRight`. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcAttHoldCycPitchOut` | Number, ±1 cyclic fraction | local (owner only) | Attitude/position/velocity hold command added to the cyclic pitch - or the flight director's, while IAS holds pitch or HVR slows to the hover. 0 when neither is flying it, force trim is held, the channel is off, or the `gate[]` is shut. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcAttHoldCycRollOut` | Number, ±1 cyclic fraction | local (owner only) | Same as the pitch output, for cyclic roll - the flight director's while HDG or NAV holds roll, or HVR slows to the hover. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcHdgHoldPedalYawOut` | Number, ±0.1 pedal fraction | local (owner only) | Heading hold / turn coordination command added to the pedals - or the flight director's, while HDG or NAV turns by pedal below `bankAboveKts` ground speed or under HVR. Forced to 0 when the springless-pedal or auto-pedal setting is on. (set in `fmc/fn_fmc.sqf`) |
| `bmkhs_fmcAltHoldCollOut` | Number, collective fraction. Range is set by the PID config. | local (owner only) | Altitude hold command added to the collective - or the flight director's, while RALT, ALT or ALTP holds it. 0 when neither is flying it. (set in `fmc/fn_fmc.sqf`) |

### Pilot inputs

| Variable | Type / units | Net | Meaning |
|---|---|---|---|
| `bmkhs_cyclicFwdAft` | Number -1..1, + = forward | local (pilot's machine) | Pilot cyclic pitch after the keyboard handling, the assists and the actuator lag. It is the stick displacement only: no trim, SAS or hold. Forced to 0 when flight-control hydraulics are lost (`bmkhs_fltCtrlsSupplied` false and `bmkhs_emerHydOn` false). With mouse-as-joystick it is multiplied by `bmkhs_mouseSense`. (set in `input/fn_inputUpdate.sqf`) |
| `bmkhs_cyclicLeftRight` | Number -1..1, + = LEFT | local (pilot's machine) | Pilot cyclic roll, worked out the same way as pitch. Calculated as left minus right. (set in `input/fn_inputUpdate.sqf`) |
| `bmkhs_pedalLeftRight` | Number -1..1, + = right pedal | local (pilot's machine) | Pilot pedal after actuator lag. It holds its last value when the tail rotor is unpowered or undriven (`bmkhs_tailRtrSupplied` / `bmkhs_tailRtrDriven` false). It is 0 when flight-control hydraulics are lost. (set in `input/fn_inputUpdate.sqf`) |
| `bmkhs_collectiveOutput` | Number 0..1, 0 = full down | local (pilot's machine) | Pilot collective position after actuator lag. It holds its last value while flight-control hydraulics are lost, unless emergency hydraulics are on, and while the game is not focused or a dialog is open. (set in `input/fn_inputUpdate.sqf`) |
| `bmkhs_forceTrimPosPitch` | Number -1..1, + = forward | 10 Hz | Cyclic pitch trim position: where the stick rests. Set on force-trim release. Zeroed by force-trim reset. The auto attitude assist writes it every frame. In springless/sticky-keyboard mode it is held at 0. (set in `fmc/fn_fmcForceTrimSet.sqf`, `fmc/fn_fmcForceTrimReset.sqf`, `input/fn_inputAutoAttitude.sqf`) |
| `bmkhs_forceTrimPosRoll` | Number -1..1, + = left | 10 Hz | Cyclic roll trim position. Same rules as pitch. (set in `fmc/fn_fmcForceTrimSet.sqf`, `fmc/fn_fmcForceTrimReset.sqf`) |
| `bmkhs_forceTrimPosYaw` | Number -1..1, + = right | 10 Hz | Pedal trim position. When auto pedal is on, auto pedal writes it every frame. (set in `fmc/fn_fmcForceTrimSet.sqf`, `fmc/fn_fmcForceTrimReset.sqf`, `input/fn_inputAutoPedal.sqf`) |
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
| `bmkhs_flightLog` | Bool | CBA setting | Writes the flight to the RPT at 10 Hz - `BMKHSLOG` lines under a `BMKHSLOG_HDR` header naming the columns (`debug/fn_debugFlightLog.sqf`). `python/dev/flightlog.py` reads them back. |
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
| `fdModeChanged` | `[mode (String), engaged (Bool)]` | A flight director mode engages or drops, on the machine that changed it. (`fmc/fn_fmcFdMode.sqf`) |
| `holdModeDisengaged` | `[]` | Altitude hold drops because the collective moved more than `collBand` from its reference (`fmc/fn_fmcAltitudeHold.sqf`). Or altitude hold is toggled off (`fmc/fn_fmcAltitudeHoldEnable.sqf`). Or attitude hold is toggled off (`fmc/fn_fmcAttitudeHoldEnable.sqf`). Or "hold modes off" is used while either hold is active (`fmc/fn_fmcHoldModesDisable.sqf`). Not raised when altitude hold drops because torque reaches `dropAboveTq`. |

### Read functions

| Function | Params | Returns |
|---|---|---|
| `bmkhs_fnc_damageGet` | `[_heli, _role, _index = -1]` | Number 0..1. Damage on member `_index` of a role, or the worst member when -1. Returns 0 for a role nothing claims, or for a missing hitpoint (Arma's -1 is clamped). Works on any machine that has run coreConfig. |
| `bmkhs_fnc_damageCount` | `[_heli, _role]` | Number. How many hitpoints claim the role (`bmkhsRole` / `bmkhsRoleIndex`). 0 if none. If no hitpoint claims `engines`, there is one shared `hitengine` entry per `numEngines`. |
| `bmkhs_fnc_systemCircuit` | `[_heli, _circuit]` | Number. Current value of a circuit, in the config's unit; the highest feeder wins. Returns 0 for `""`, an unknown circuit, or nothing feeding it. Only meaningful on the owner with `useSystems = 1`: values are local and solved only there. `"Nr"` is fed from `bmkhs_rtrRpm`. |
| `bmkhs_fnc_controlAllowed` | `[_heli, _control, _pos]` - variableName or index, position index | Bool: may it move there - the control's and the position's `enabledBy[]` / `inhibitedBy[]`, the check Core itself makes before moving. Records the blocker in `bmkhs_<control>GateWhy`. Off the owner, returns the owner's answer. |
| `bmkhs_fnc_controlSet` (write) | `[_name, _pos, _heli = vehicle player]`. `_pos` is a Number (absolute index) or a String step (`"+1"`, `"-1"`) | Bool, true if the control moved. False if HeliSim is not initialised, the name is unknown, an interlock blocks it, or it is already there. Indices are clamped, or wrap if `wraps = 1`. |
| `bmkhs_fnc_inputControlHandle` (write) | `[_name, _pressed, _heli = vehicle player]` | Nothing. Core's discrete flight-control and FMC actions by name - force trim, the hold modes, the flight director's modes and target steps and syncs (`bmkhs_fd<Mode>`, `bmkhs_fd<Target>Up/Dn/Sync`). What a keybind calls; a cockpit button calls it by the same name. Runs where called. |
| `bmkhs_fnc_inputAnalogHandler` (write) | `[_name, _value, _heli = vehicle player]` | Nothing. Core's analog actions by name - the flight controls, and the flight director's targets (`bmkhs_fd<Target>Target`, `_value` a fraction 0..1 of the target's range). What an axis calls; a dragged cockpit knob calls it by the same name. Runs where called. |
| `bmkhs_fnc_fmcSetChannel` (write) | `[_heli, _channel, _on]` - `"pitch"`, `"roll"`, `"yaw"`, `"coll"` or `"trim"` | Nothing. Switches an FMC axis channel (`bmkhs_fmc<Axis>On`). |
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
- `holdModeDisengaged` is not raised when altitude hold drops at `dropAboveTq` torque.
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
- `bmkhs_lastTimePropagated` - when the packed running state was last sent (`core/fn_coreNetSend.sqf`).
- `bmkhs_netState` - the packed running state itself; `bmkhs_netStateApplied` - the last packet unpacked.
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
- `bmkhs_<control>Allowed` - per-position interlock result, published by the owner; ask `bmkhs_fnc_controlAllowed` instead.
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
- `bmkhs_fmc`: the declared FMC features, their settings and PID state.
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
