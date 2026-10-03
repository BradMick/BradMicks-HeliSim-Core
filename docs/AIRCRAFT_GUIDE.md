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

### Mass and balance - converting the CG

`bmkhs_config/helisim_mass.hpp`. Three coordinate frames are involved, and
every number has to go into the right one.

| Frame | What it is | Which config values |
|---|---|---|
| **Model** | Object Builder (p3d) coordinates, metres. x right, **y toward the nose**, z up. | `fsDatum`, `fwdCgLimit`, `aftCgLimit`, every `arm[]` (seats, tanks, stations, magazines) |
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

**5. Item arms.** Seats, tanks, wing stations and magazines are model positions
`{x, y, z}`. Read them in Object Builder, or convert a manual station with step 2
for y.

**6. What Core does with them, every frame** (`fn_massUpdate`):

    longMom = emptyMass x fsDatum - emptyMom          (the empty airframe, model frame)
            + sum of (mass x arm y) over every item aboard
    CG y    = longMom / total mass
    CG x    = sum of (mass x arm x) / total mass
    setCenterOfMass [CG x - boundingCenter x + comCorrection x,
                     CG y - boundingCenter y + comCorrection y,
                        - boundingCenter z + comCorrection z]

An empty seat, an uninstalled removable tank and an empty pylon add nothing.
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
