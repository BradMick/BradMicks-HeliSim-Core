# Systems redesign — converted

**Part of the BMKHS refactor.** See `SFMPLUS_BOUNDARY_REPORT.md` for the wider
plan. This document tracks the systems model itself moving from hardcoded
AH-64 structure to declared components.

**HeliSim Core now lives in its own repository** at `E:\bmkhs_helisim`, built
separately with its own HEMTT project. This repo reaches it through the
`include\bmkhs_helisim` junction, so edits there are live here. The canonical
copy of this document and the aircraft guide ship with Core, in its `docs/`.

**The field reference is `components.hpp`**, in Core beside the code, not this
document. That header is what a builder reads to declare an airframe; this one
holds the design the code answers to, what is converted, and what bit us.

## The model — what the kinds ARE

This is the agreed design, and it is the thing to check an implementation
against. If the code and this section disagree, that is a bug in one of them
and worth resolving explicitly rather than quietly following the code.

    Source     produces onto a circuit, given whatever drives it
    Converter  consumes from one circuit, produces onto another
    Storage    a source that DEPLETES while supplying - the time-limited kind
    Circuit    a named node carrying a VALUE; consumers threshold it themselves
    Consumer   fed by a SET of circuits; supplied if ANY of them is up
    Reservoir  a consumable that leaks when damaged and starves its consumers

Domain-agnostic by design: `drivenBy` and `input` reference circuits in ANY
domain, which is what makes an electrically-driven hydraulic pump or an
engine-driven generator expressible without Core knowing either exists.

`Consumer` is what makes redundancy declarative rather than hardcoded - flight
controls on two circuits keep working when one dies, while SAS on one circuit
does not. Without it, every "which failures survive which" rule goes back to
being an if-chain naming this airframe's specific circuits.

**A component is a physical thing** - the APU, a generator, a pump, the
accumulator. What it PUTS OUT is not a component: an APU that drives the
accessory section and supplies bleed air is one component with two outputs.

**As built**, against the above:

| design | built as | note |
|---|---|---|
| Source | `fn_systemProducer` | renamed; same job |
| Converter | `fn_systemConverter` | |
| Storage | `fn_systemStorage` | |
| Circuit | `fn_systemCircuit` + `fn_systemCircuitState` | reading a node and reporting it are separate |
| Consumer | `fn_systemConsumer` | |
| Reservoir | folded into Storage | a reservoir is a store that leaks; agreed, not an accident |
| — | `fn_systemTorque` | not in the original design: overtorque damage, which is a component property but not a supply one |

## Scheduling — dirty-flag propagation

This is the agreed design and the reason for the redesign. **Built and flown** -
a cold start propagates end to end: APU spools, drives the accessory section,
pumps pressurise both sides, generators feed AC, rectifiers feed DC, the
accumulator recharges.

It was specified once, then not built - signature polling went in instead, which
cost frame rate rather than saving it and left the APU unable to start. This
section was restored from `c8e685dde`, where it was the only surviving copy,
so the document can contradict the code again.

**Systems sleep until something changes.** Most components are pure state
functions recomputing an unchanged answer 60 times a second. A rectifier is
`generatorOn && damage <= threshold` — it can only change when one of those
changes.

**Continuous is a runtime answer, not a static property.** The timer-driven
components are not always integrating either:

| component | integrates only while |
|---|---|
| battery | on battery bus AND AC bus down |
| APU | spooling up or down, not at steady RPM |
| reservoir | actually leaking |
| accumulator | bleeding down |
| transmission | over a torque limit |

On a healthy running aircraft **none of these are integrating**, so steady-state
cost should approach zero.

The scheduler that expresses this: an update returns whether it wants the next
frame.

```sqf
//true = keep me scheduled, false = sleep until a dependency changes
[_heli, _index, _deltaTime] call bmkhs_fnc_systemProducer
```

Dirty-flag propagation wakes a sleeping component when a dependency changes; a
component that is mid-transition keeps itself awake by returning true. Damage
changes are just another dependency.

**As built.** `fn_systemsComponents` derives the graph at load from the fields a
component already declares — `gates`, `drivenBy`, `input`, `requires`,
`rechargedBy`, `disengageOn` — rather than a separate `dependsOn` that could
drift from what the code actually reads. Two indices come out of it:
`bmkhs_sysReaders` maps a circuit to the components reading it, and
`bmkhs_sysWatchers` maps a variable to the components gated on it.

`fn_systemsSolve` then walks from what changed: variables whose value moved,
damage that moved, Nr, and anything that asked for another frame. Each component
that moves its own node dirties whatever reads that node, which appends to the
queue, so the walk reaches exactly as far as the change does and stops.

**Ordering falls out of the walk**, which is what removed the fixed
`SYS_SOLVE_PASSES` re-resolve and the `deltaTime = 0` passes that went with it.
A topological sort was never possible anyway: the accumulator starts the APU,
which drives the accessory section, which turns the pumps, which recharge the
accumulator. What cuts the cycle is that **charge is state, not supply** — a
store delivers what was put there earlier, so it is a root of the walk and its
recharge edge settles afterwards from the walk's own result.

Circuit states and consumers feed nothing, so they are not in the walk; the
solve publishes them from the settled graph, which also stops a node that fell
quiet from keeping its last published state.

## Where it stands

| domain | state |
|---|---|
| hydraulics | **converted, flown** - pumps, reservoirs, accumulator, accessory drive |
| electrical | **converted, flown** - battery, generators, rectifiers, buses |
| APU | **converted, flown** - one component, driving accessories and bleed air |
| drivetrain | **converted, flown** - transmission, gearboxes, torque limits, damage model |
| scheduling | **built, flown** - dirty propagation; see below |
| fuel | stays separate - it set the pattern the kinds follow |
| controls | **converted, flown** - switches, buttons and power levers |

Nineteen hardcoded functions replaced by declarations, and four per-domain
configs absorbed into `helisim_components.hpp`, which is now the single place an
airframe says what it has.

Core runs one function per KIND rather than per system, so a pump and a
generator are the same code with different declarations:

```
fn_systemProducer      damage, gate, drive and consumable -> a value on a circuit
fn_systemConverter     takes from one circuit, feeds another; creates nothing
fn_systemStorage       a producer holding a charge, which drains and refills
fn_systemConsumer      supplied if ANY of its circuits is up, or all with needsAll
fn_systemCircuit       a named node; highest feeder wins
fn_systemCircuitState  publishes whether a node is up
fn_systemTorque        damages anything run past its limits
fn_systemTorqueJitter  what a damaged drive is doing to an engine's torque needle
fn_systemCircuitFeed   records one feeder's contribution to a node
fn_systemsSolve        dirty walk from what changed; storage roots it, charge settles last
fn_systemsComponents   config -> hashmaps, once, at init
fn_control             one control, one index move: interlocks, wrap, publish, hold
fn_controlPublish      the only writer of a control's Idx/Val/On
fn_controlSet          the keybind target; resolve by name, translate a step
fn_controlsUpdate      pre-walk: reconcile a control against an external writer
fn_controlsRelease     post-walk: release a spring-back the walk has now read
fn_controlsVariables   config -> bmkhs_ctrlList, once, at init
```

Member count comes from the damage role, and damage is read AT THE MEMBER'S
INDEX - the role alone returns the worst member, which would fail all three
generators because one is destroyed. That was the bug that started this.

## Drivetrain damage — the model, written down

This was deleted once by the conversion and rebuilt from the old code, so it is
recorded here rather than living only in `fn_systemTorque`.

A component declares its tiers worst-first as `{torque, grace seconds, divisor}`,
torque being a fraction of rated. A tier's clock runs **only while the torque is
in that tier** and resets the moment it leaves, so time spent higher up does not
spend a lower tier's grace and a brief excursion is not cumulative. 0 seconds
means no grace at all.

Once any tier's clock expires, the rate is the **sum over every exceeded tier**
of `(torque - limit) / divisor`. So it scales with the abuse - pulled harder, it
comes apart faster - rather than being a flat rate whatever the overtorque.
Nothing accrues with the engines off.

Damage feeds itself past 25%: the persistent rate is `damage/600`, `/500` or
`/400` by band, and the bands **replace** each other rather than stacking.

`breaksOnFailure[]` is what a destroyed component takes with it. An entry naming
a damage role destroys that role outright - the transmission is what holds the
rotors, the generators and the pumps up. An entry naming a `bmkhs_` variable
sets it at the member's index instead, which is how a nose gearbox that has come
apart overspeeds its engine.

`jittersTorque` makes a damaged drive wander the torque needle. The component
publishes its OWN wander under its own variable and `fn_systemTorqueJitter` sums
what reaches a given engine - its own component plus any that carries every
engine. The old shared four-slot array was the "exactly two of everything,
indexed by hand" assumption this refactor exists to remove.

**The AH-64's ratings.** Transmission, both engines summed: 200% continuous, 200
to 230 for six seconds, above 230 at once. Nose gearboxes, per engine and rated
**single-engine only** - with both running neither carries enough to hurt it -
110% continuous, 110 to 122 for two and a half minutes, 122 to 125 for six
seconds, above 125 at once.

### With no systems modelled

`useSystems = 0` does exactly one thing: overtorque damages `hithrotor` and
`hitvrotor`. Nothing else. No overspeed, no cascade, no jitter - there are no
systems to fail. Limits come from `xmsnTqLimits` and `ngbTqLimitsSE` at the top
level, which are read **only** on this path; with systems on the components
carry their own. Damage is read back off the rotor hitpoints, so battle damage
and overtorque are one number and a shot-up rotor is fragile under torque.

## What is left

**Controls are converted** - the four `fn_interact*` functions are gone and every
switch is a declaration. Consumers read the published value rather than being
pushed at: `fn_engineController` reads the start switch and power lever levels
each frame and moves `bmkhs_engState` itself.

Interlocks are declared **per position as well as per control**, which is what
the power lever needed - the rotor brake blocks FLY without blocking IDLE, so a
locked-rotor start still works. The `case "control"` wake that re-evaluates a
control when its interlock moves is still UNPROVEN; nothing has exercised it.

**Axis-bound levers were built and removed.** See "Axis binding" below. Worth
retrying: the axis had its own publish path writing `Val`/`Idx` directly instead
of going through `fn_controlPublish`, so two writers fought over the same
variables. That is a better explanation for the oscillation than anything
diagnosed at the time, and it no longer exists.

**Frame rate is unconfirmed.** Observed more stable and not dropping after the
dirty walk went in, but that is an impression, not a measurement, and it needs
much more testing. The honest number is not FPS - it is how many components the
walk runs per frame, which should be near zero on a settled aircraft and spike
only when something changes. **That counter is now in the debug panel** as
`walk N peak N` at the top; the peak decays so it shows the last burst rather
than the highest ever.

**Multiplayer with a CPG.** Never exercised, in any domain. The gunner is a
genuine remote reader of everything a crew station displays, so anything missing
`networked = 1` is frozen for them while singleplayer looks perfect.

**Repair, now event-driven and generic.** A `HandleDamage` handler in
`fn_coreInit` flags a repair when a hitpoint goes down, and `fn_repair` exits
unless it sees that flag. It walks the declared components rather than naming
any, so it restores whatever the airframe has. Worth confirming a repaired
component gets its state back, and that ordinary damage still applies - the
handler sits in the damage path for everything.

The accumulator refills on any repair, having no hitpoint of its own. That is
correct while nothing else charges it: the recharge path off the accessory drive
works, but a ground cart or hand pump is not implemented.

**Overtorque with no drivetrain declared** is flown and works - the top-level
ratings damage `hithrotor` and `hitvrotor` directly. What is NOT exercised is an
airframe that declares no drivetrain hitpoints at all, which is the case the
fallback was written for.

**Everything now runs from the pack.** `fn_perFrame` calls systemsUpdate,
coreUpdate, coreUpdateFlightModel, ctrlVisUpdate and repair. Four of those moved
out of `fza_ah64_controls`, and `ctrlVisUpdate` moved from a Draw3D context to
per-frame.

**The pack is standalone.** It starts itself from an `Extended_Init_EventHandlers`
on its own `bmkhsBaseClass`, so nothing outside it calls in - `fza_ah64_controls`
used to invoke `fn_setup` and no longer does. `fn_setup` guards on
`bmkhs_initialised` so it cannot double-init.

Its `XEH_preInit` then runs `fn_perFrame` for every LOCAL aircraft matching any
pack's declared `bmkhsBaseClass`, so AI and unoccupied aircraft run their systems
too - an AI Apache burns fuel and overtorques its gearboxes like a crewed one.

The pack still calls OUT to `fza_fnc_animSetValue` and `fza_audio_fnc_flightTone`
for cockpit animation and audio. That direction is correct: an aircraft pack is
allowed to know its own aircraft, and Core knows neither.

**`breaksOnFailure` has two users, and they use different halves of it.** The
transmission names damage ROLES - it takes the rotors, generators and pumps with
it - while a nose gearbox names a bmkhs_ VARIABLE, setting the overspeed flag at
its own member index. Both shapes are declared, neither is proven in flight.

**The tail rotor needs two consumers**, because its failure modes do not
combine: hydraulic authority is an either-or across two circuits, the drive is a
chain that must be intact. One consumer cannot express both, so `fn_inputUpdate`
reads both. It works, but it is the model bending rather than fitting.

**Hitpoints stay separate.** `class HitPoints` has to live inside the vehicle
class where Arma requires it, and `damageRole` is the join. That indirection
earns its place - it is what lets member count come from hitpoint count.

## Things that caught us

Worth knowing before converting another domain.

**A store must compare against OTHER sources, not the whole node.** Storage asks
whether anything else already supplies its output circuit before feeding it. Once
contributions persist between frames - which is what lets a component sleep - a
store reading the node total reads its OWN supply back, decides it is covered,
and cuts its feed. The battery did this to `BATT`, dropping it at the instant the
APU evaluated its gate, so the APU would not start while the panel showed every
gate passing. Read `bmkhs_sysProducerFeed_<circuit>`, never
`bmkhs_fnc_systemCircuit`.

**A debug panel that re-derives conditions can disagree with the component.** The
panel evaluated gates live while the component evaluated them when it ran; both
looked correct alone. Have the component record the terms it actually used.

**`_x` is rebound by every inner `forEach`.** Any component-field read after a
loop over gates or outputs must use a captured `_comp`, and `_forEachIndex` must
be captured too. This was fixed in three kind files and then reintroduced twice
in the graph builder written one commit later.

**Damage by role returns the WORST member.** Reading it without an index fails
every member because one is broken.

**Absent is not failed.** A role nothing claims means the airframe does not
have that component. Declaring NO role is different - present, but not
separately damageable, like an accumulator with no p3d selection. Undeclared
circuits must publish NOTHING rather than zero, so the read-side defaults that
keep a no-hydraulics airframe flying still fire.

**`utilUpdateNetworkGlobal` throws on a variable that has never been set.** It
reads with the single-argument `getVariable` - no default - so the compare on the
next line reads an undefined value. Latent forever, because every existing caller
passes something `fn_systemsVariables` already seeded; the controls layer hit it
immediately by publishing brand-new names. Seed with a plain `setVariable` before
the first networked publish. The symptom misleads: the throw happens BEFORE the
write, so the variable stays unset and the panel shows its fallback, which reads
like a lookup failure rather than an exception.

**Multiplayer fails silently.** The solve runs where the aircraft is local;
everything else reads published results. The per-frame scheduler runs for the
pilot OR gunner of the aircraft they occupy, so a gunner is a genuine remote
reader. Anything a crew station displays needs `networked = 1` or it is frozen
for them - and singleplayer looks perfect either way.

**Rates defined over the wrong range.** Twice: a start draw that left the store
above its own advisory threshold, and a recharge that finished in half its
configured time because the rate spanned 0..1 while the store only moves
through the usable band above its floor. If a number does not produce the
behaviour it names, check what range it is defined over.

**`_x` shadowing.** An inner `forEach` rebinds `_x`, so a component read after
one silently reads the wrong thing. Bit both storage and consumers; bind the
component to `_comp` first.

**Ordering within the solve.** Storage supplies before anything is solved,
because charge is state - that is what cuts the accumulator -> APU -> pumps ->
accumulator startup loop. But charge can only MOVE once the rest has solved, so
draining and refilling are a settle pass at the end. Getting that wrong left
the accumulator reading zero forever.

**Chains run deeper than one hop.** Nr feeds the transmission feeds the
accessory drive feeds the pumps; a generator feeds AC feeds a rectifier feeds
DC. Producers and converters re-resolve `SYS_SOLVE_PASSES` times so a chain
settles in one frame rather than lagging.

**A gate that reads a variable published later in the same solve is a frame
stale**, and that can deadlock a start. Gates can name a circuit instead, which
reads the live value.

**Change-detect needs a default the value can differ from.** Making a state
notify fire only on a change broke it outright: the previous value defaulted to
the current one, so the first comparison was always equal and the event never
fired at all.

**A threshold the model passes through legitimately is not a failure.** The
engine flips to ON at `engRunNG`, which is below the engine-out warning
threshold, so every start tripped the warning on the way up. Wait for the
condition to have been true once before believing it can be false.

**Conditions dropped in conversion are invisible.** The nose gearbox torque
check was wrapped in `isSingleEng`; losing that would have damaged gearboxes in
normal two-engine flight. Read what the old function GUARDED, not only what it
did.

## useSystems = 0

Nothing is simulated. The solve exits, so no hydraulics spool, no buses come
up, no APU exists - the engines and transmission still run because they are the
flight model, and everything else stays at its seeded value. There is no damage
model either, since a system exists because hitpoints declare it.

The aircraft spawns cold and dark and wakes on the player's first collective or
throttle input, spooling over ten seconds. Torque limits still apply: they run
outside the solve, and the ratings sit at the top level beside `useSystems` so
an airframe that declares no components at all still respects them.

## Controls are components too — converted

**Built and flown.** The battery switch, APU button, both start switches and both
power levers are declarations; the four `fn_interact*` functions are shims onto
them. The field reference is `addons/bmkhs_helisim/controls.hpp`.

Every gate names a control: `bmkhs_emerHydOn`, `bmkhs_battSwitchOn`,
`bmkhs_apuBtnOn`. A control is declared once and produces three things: the
variable a gate reads, the keybind, and the cockpit interaction.

`CfgUserActions.hpp` already had `BMKHS_ANALOG` / `BMKHS_NONANALOG` /
`BMKHS_ACTION`, each generating a keybind and its dispatch together. What was
missing was control BEHAVIOUR - everything was momentary, so anything else was
hand-written SQF.

**Scope: HeliSim's own controls only.** The switches, knobs and levers an
AIRCRAFT is concerned with - what drives the systems model and the flight model.
Not the mod's avionics: MPD bezels, sight select, weapon actions and the rest
stay where they are. The test is whether a control feeds a gate or the flight
model, not whether it happens to be in the cockpit.

**HeliSim is a library.** Core provides the kinds and the macros; the aircraft
pack declares which controls it has, and Core generates the keybinds from the
pack's declaration. Same shape as every other domain: Core knows no airframe.

### A control is N positions

**A switch is nothing more than a gate.** The APU does not care that it is an APU
switch, only that the signal arrived - so Core moves an index and publishes a
value, and whatever gates on that value reacts. Core learns no switch semantics.

**A control has N POSITIONS, and each position has its own output.** Not a
boolean. The engine start switch is three positions: aft holds, centre rests,
forward springs back. The index is canonical - Core tracks an index and nothing
else, and the designer knows what each index means because they declared them in
order. Position names are display labels for the bindings menu; Core never
interprets them.

That single model covers every behaviour a cockpit needs, with nothing left over:

| behaviour | expressed as |
|---|---|
| momentary | 2 positions, one `springsBack` |
| latching | 2 positions, neither springs |
| one-way | 3 positions, one springs |
| multi-position | N positions, none spring |
| detented lever | N positions, none spring |
| continuous | `steps = 0`, value interpolated |
| rotary | N positions, `wraps = 1` |

Multi-position and rotary are the generic answer to something like the UH-1's
rotating generator selector: N positions the systems model reads, declared not
coded.

**Guard covers are not modelled.** A physical guard - the jettison button's lid -
would need a keybind of its own just to lift it before the switch underneath
could be thrown. That is a bind spent on ceremony, and every other position here
is one the player wants to reach directly. A guarded switch is declared as the
switch it is.

**Power levers and throttles are not switches, and not each other.**

| | power lever | throttle |
|---|---|---|
| what | engine condition - a fuel gate | continuous power modulation |
| range | detented positions | smooth 0-1 |
| use | set once per phase of flight | flown continuously |

An aircraft may have one, both or neither - the AH-64 has no throttle at all
because the governor holds Nr.

### What a position publishes

Three variables per control, all derived, none interpreted by Core:

| variable | is |
|---|---|
| `bmkhs_<name>Idx` | the position index - canonical |
| `bmkhs_<name>Val` | the current position's declared value |
| `bmkhs_<name>On` | `Val != 0` |

`On` is what keeps the existing gates working untouched: a control named
`battSwitch` publishes `bmkhs_battSwitchOn`, which is the name the electrical
components already gate on. That makes the naming load-bearing - get it wrong
and the bus silently never comes up.

**Levels, not edges.** "Begin the start sequence" looks like it needs an edge,
but nothing in the engine path depends on one: `fn_engine.sqf` transitions
`STARTING -> ON` by testing whether Ng has crossed a threshold, and the start
switch only ever compares engine state against the requested action. So the
engine reads a level - `Val == 1` while the switch is held - and spring-back is
what makes that level transient. Core never learns what STARTING means.

This is also what leaves room for motoring later. Ignition override stops the
start sequence today and this pass mirrors that exactly; making the position
maintained instead of spring-loaded is a declaration change, not a Core one.

### Interlocks reuse the gate form

A control that cannot be thrown right now says so the way every component
already does - a variable name, or `{circuit, threshold}` read live. No new
syntax, because the whole point is reusing the established one.

    enabledBy[]   = {{"BATT", 0.25}};        //all must be true to move it
    inhibitedBy[] = {"bmkhs_rotorBrakeOn"};  //any true blocks it

`enabledBy` is the APU button needing its bus; `inhibitedBy` is the rotor brake
locking the power levers. Both entries go into `bmkhs_sysWatchers` exactly like
a producer's gates, so a control is woken by the same walk.

### Axis binding — attempted, removed

**Not modelled. Controls are click-only.** An axis-bound power lever was built and
abandoned: the lever oscillated between two discrete positions, flipping the engine
between IDLE and FLY many times a second and spiking torque.

What the instrumentation established, so it is not re-derived: the stored axis half
dropped to exactly 0.000 for **exactly one frame** at irregular intervals - measured
gaps of 12, 89, 154, 29 and 531 frames, every dip one frame long - while the player's
hand was off the hardware. Each full drop moved the lever a quarter of its travel and
republished the engine state. The cause was never found.

Several explanations were proposed and disproved along the way: the actuator lag
filter (it returns its input unchanged on a healthy aircraft, so it filters nothing),
the two-half bind design (correct - the collective binds identically), and the
hardware (a live readout showed a clean -1 to +1 sweep). One real difference WAS
found and fixed - the axis was read inside `fn_systemsSolve`, behind the locality
and `useSystems` gates, rather than on the input path beside the collective - but
correcting that did not stop the oscillation.

Anyone reviving this should start from `fn_inputUpdate`'s collective block, put the
lever read beside it, and confirm with a per-frame log before trusting it.

### Where a control sits in the graph

**A control is not a component**, and this is the part worth getting right. A
component feeds a circuit; a control feeds a VARIABLE, and the variable is what
gates already read. So a control needs no new edge type - `fn_systemsSolve`
already wakes on any watched variable changing, and `bmkhs_apuBtnOn` is already
a watcher key today.

Controls are solved BEFORE the walk, not inside it: a control reads its input,
publishes its variable, and the existing dirty propagation carries it the rest of
the way.

**As built, a control DOES get a wake ref** - one word of nuance the design missed.
It has no place in the walk as a producer of circuit values: it feeds no circuit,
dirties no node, appears in no `_feedsOf`. But its `enabledBy` / `inhibitedBy`
entries land in `bmkhs_sysWatchers` like any gate, so the walk pops a
`["control", N]` ref and `fn_systemsSolve` dispatches it with a `-1` sentinel
meaning "re-evaluate where you are". Without that case the interlock wake would be
silent dead weight. **Untested** - no airframe declares an interlock yet.

Those edges are registered from inside `fn_systemsComponents`, not from
`fn_controlsVariables` where the design put them: the graph builder creates
`_readers`/`_watchers` FRESH and overwrites them at the end, so anything registered
before it runs is destroyed.

**`useSystems = 0` still has controls declared**, which the design got wrong.
`fn_controlsVariables` runs from `fn_coreConfig` unconditionally, so the list and
its variables exist whether or not systems are modelled - publishing needs no
solve. Only the reconcile and spring-back release sit inside the gate, so a
spring-back never releases with systems off. Harmless for the AH-64, whose only
springing position is on a switch that path never reaches.

This is not the `fn_systemTorque` case. Torque limits run outside the gate
because a drivetrain is rated whether or not its systems are modelled - a
physical fact about the airframe. A switch is not: it exists only because
something declared a system for it to act on.

### Keybinds

**Keybinds are config-time and static** while component counts are
aircraft-declared. Three engines needs three power-lever binds, and a count from
hitpoints cannot reach a preprocessor.

The answer follows from HeliSim being a library: **Core provides the macro, the
pack writes one row per POSITION.** A pack with three engines writes three
levers' worth of rows, so the count is fixed at config time on the side that
knows it. `fza_ah64_controls` already does exactly this - `controls.hpp` has
`e1off`, `e1idle` and `e1fly` as separate binds - so one bind per position is a
proven pattern here rather than a new idea.

**Every position is individually bindable, with a name the designer sets** -
"Engine 1 Start - Ignition Override", "Battery - On". A player binds the position
they want directly instead of cycling a switch to reach it, and what they read in
the menu is what the designer wrote.

The rows are dual-included: once inside `CfgUserActions` to emit the classes,
then the macro is redefined and the same file included inside a `group[]` array
to emit the group list. One data table, two views, so the binds and the group
cannot drift apart. **The separator carries the terminator** - `;` in the class
view, `,` in the group view - because the row itself can carry neither.

Cockpit controls get **their own group**, separate from flight controls, so the
menu stays readable. They are deliberately left out of the conflict groups: every
position of every switch would otherwise be flagged as conflicting with every
other, which is not what a conflict means.

**As built, two things differ from the design.**

The macros live in their own `controlMacros.hpp`, not in Core's
`CfgUserActions.hpp`. `requiredAddons` orders LOADING at runtime, not the
preprocessor - each config is preprocessed independently - so a pack must include
the macro header itself, and including Core's `CfgUserActions.hpp` would drag in
its `class CfgUserActions` and collide with the pack reopening it.

**Core ships no group class**, where the design had it declaring
`bmkhs_cockpitControls`. Filling a `group[]` means `#include`ing a pack's header by
path, which would name an airframe. The pack declares its own group instead - Arma
merges `UserActionGroups` across addons - so Core stays airframe-agnostic.
