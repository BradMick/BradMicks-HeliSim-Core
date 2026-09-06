#ifndef BMKHS_HELISIM_COMPONENTS_HPP
#define BMKHS_HELISIM_COMPONENTS_HPP

//BMKHS component model - what an airframe declares, and what Core does with it.
//
//The AIRCRAFT declares what it has; Core declares nothing and knows no airframe. A
//component names the damage role it answers to, and the hitpoints claiming that role ARE
//its members - so a third generator hitpoint gives a third generator with no code change.
//Declaring no role means the component exists but cannot be damaged separately.
//
//Circuits are named nodes carrying a value in whatever unit the domain uses - psi, volts,
//Nr as a fraction. The names are the aircraft's to choose; Core matches them as strings.
//Several feeders on one node take the HIGHEST value rather than summing.
//
//Every reference to a circuit carries its own threshold:
//
//    drivenBy[]    = {"ACCESSORY_DRIVE", 0.45};   //below this it has no drive
//    drivenBy[]    = {"AC"};                      //any value at all will do
//    suppliedBy[]  = {{"PRI_HYD", 1260}, {"UTIL_HYD", 1260}};
//
//Durations are in seconds, never rates - Core converts once at init.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// ONE CLASS, MANY MEMBERS - how count works
/////////////////////////////////////////////////////////////////////////////////////////////
//
//A class is a KIND of component, not one of them. How many exist comes from the hitpoints
//claiming its damage role, so one declaration covers any count:
//
//    class Generator { damageRole = "generators"; variableName = "gen"; ... };
//
//    hit_elec_generator1  "generators" 0   ->  bmkhs_gen1
//    hit_elec_generator2  "generators" 1   ->  bmkhs_gen2
//    hit_elec_generator3  "generators" 2   ->  bmkhs_gen3
//
//Add the third hitpoint and there is a third generator - no config change here, no code
//change in Core. Declare none and the component does not exist on this airframe.
//
//Each member is independent: its own damage, its own state, its own contribution to the
//circuit. Two healthy generators and one destroyed still hold the bus up, because the
//highest feeder wins the node.
//
//ONE CLASS PER JOB, NOT PER UNIT. Two generators are two of the same thing on the same
//bus, so they are one class. The primary and utility pumps are different jobs feeding
//different circuits, so they are two classes even though both are pumps. The test is
//whether they share a damage role AND a circuit.
//
//Consumers are not per-member. acBusOn is one consumer of the AC circuit however many
//generators feed it, which is why adding one needs no consumer change.
//
//NAMING TRAP: the member number only appears when there IS more than one, so a single
//battery publishes bmkhs_battPower_pct and a second one would silently rename it to
//bmkhs_battPower_pct1 - breaking every external reader. Choose variableName for the count
//the role might reach, not the count it has today. "gen" is safe because it is already
//written as one of several; "priHydPsi" is safe because an airframe has exactly one
//primary pump by definition.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// NETWORKING - read this before declaring anything a crew station displays
/////////////////////////////////////////////////////////////////////////////////////////////
//
//The graph is SOLVED on the machine the aircraft is local to. Every other machine reads
//the published results, so any state a crew station displays or acts on must be declared
//networked or it will be stale for everyone but the pilot.
//
//This fails SILENTLY in singleplayer: the gunner's page is correct locally and frozen in
//multiplayer. If a value is read outside the flight model, network it.
//
//    networked = 1;   an MPD page, a caution, a warning light, a weapon interlock
//    (default)        flight-model state consumed on the machine that computes it
//
//An aircraft that sets useSystems = 0 solves NONE of this, and one that declares no
//components has nothing to solve - both fly on the read-side defaults of whatever would
//have consumed the state. A designer who only wants the flight model declares nothing.
//
//Networked state publishes through a change-gated helper, so it only sends when the value
//actually differs. Pair it with `increment` on anything continuous - rounding pressure to
//tens instead of single psi cuts traffic during a ramp by roughly a factor of ten.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// PRODUCERS - pumps, generators, the APU: anything that feeds a circuit
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  damageRole    hitpoint role whose members are this component's; "" for undamageable
//  variableName  what it publishes as, per member. Core owns the bmkhs_ prefix, and
//                numbers members only when there is more than one: gen1On, gen2On, but
//                priHydPsi on its own
//  output        circuit it feeds, for something feeding only one. A component that feeds
//                several declares a nested Outputs block instead
//  drivenBy[]    circuit that must be turning or live, with its threshold
//  nominal       what it produces at full output. Omit it and the component carries
//                whatever drives it instead, which is what a shaft does
//  requires      level variable it draws from; SCALES output rather than gating it, so a
//                leaking reservoir shows as falling pressure rather than a cliff
//  requiresAbove level below which it has nothing left to move and produces nothing
//  gate[]        switches that must ALL be on; omit for always armed. A gated component
//                that is off is not failed - it just contributes nothing. An entry is a
//                variable name, or {circuit, threshold} read live; use the second for
//                anything the solve itself publishes
//  rampSeconds   zero to full; 0 is instant. A pump builds pressure, a contactor does not
//  increment     round the published value to this step, as a real gauge reads
//  stateName     publishes whether this component is RUNNING, which is a property of the
//                component and not of any circuit - an APU is on above a threshold, the
//                way an engine publishes its own state. Always networked
//  stateAbove    output at or above which it counts as running
//  torqueFrom    variable carrying the torque this component sees. Indexed per member
//                where the source is, so engine 2's torque reaches gearbox 2
//  tqLimits[]    what it is rated for, worst first: {fraction of rated torque, seconds it
//                will hold there, divisor}. 0 seconds damages immediately. A tier's clock
//                runs only while the torque is in THAT tier and resets when it leaves, so
//                a brief excursion is not cumulative. Once any tier's clock expires the
//                damage rate is the sum over every exceeded tier of (torque - limit) /
//                divisor - so the harder it is pulled the faster it comes apart. Nothing
//                accrues with the engines off
//  tqLimitsSE[]  the same set, used while single-engine. Declare ONLY this one for a
//                component that can only be hurt with one engine doing the work of two -
//                a nose gearbox - and it is unrated the rest of the time
//  damagesHitpoints[]  hitpoints to damage directly, for a component whose damageRole
//                nothing claims. Core uses this for the useSystems = 0 drivetrain, so an
//                airframe declaring no drivetrain still has something to break
//  jittersTorque  1 if damage to this component makes the torque needle wander. It
//                publishes its own wander; anything reading torque asks Core for the
//                total via systemTorqueJitter rather than knowing who contributes
//  breaksOnFailure[]  what a destroyed component takes with it. An entry naming a damage
//                role destroys that role outright - a transmission is what holds the
//                rotors, the generators and the pumps up. An entry naming a bmkhs_
//                variable sets it true at this member's index instead, which is how a
//                nose gearbox that has come apart overspeeds its engine
//  networked     see above
//
//A COMPONENT IS A PHYSICAL THING - the APU, a generator, a pump, the accumulator. What it
//puts out is not a component: an APU that drives the accessory section AND supplies bleed
//air is one component with two outputs, not two components.
//
//  class Outputs {
//      class Drive    { circuit = "ACCESSORY_DRIVE"; disengageAbove[] = {"Nr", 0.95}; };
//      class BleedAir { circuit = "PNEU"; };
//  };
//
//  circuit         node this output feeds
//  ratio           of the component value; 1 passes it straight through
//  nominal         a fixed value instead, for an output that does not scale with the source
//  disengageAbove  circuit and threshold above which THIS output drops out, for a clutch.
//                  Compared live rather than latched, so it picks the load back up on the
//                  way down - an APU left running through an engine failure drives the
//                  accessories again as Nr decays. The APU keeps running either way, and
//                  its other outputs are unaffected
//
/////////////////////////////////////////////////////////////////////////////////////////////
// CONVERTERS - a rectifier, an inverter, a gearbox: takes from one circuit, feeds another
/////////////////////////////////////////////////////////////////////////////////////////////
//
//A converter CREATES nothing - without its input it has nothing to pass on, which is the
//whole difference between it and a producer. Direction is data, so swapping input and
//output turns a rectifier into an inverter and a DC-generator aircraft needs no new code.
//
//Takes every producer field except drivenBy, plus:
//
//  input[]       circuit it consumes from, with its threshold
//  ratio         what it multiplies its input by, for a converter that scales rather than
//                converting to a level - a gearbox. Ignored when nominal is set
//
/////////////////////////////////////////////////////////////////////////////////////////////
// STORAGE - accumulators, batteries, reservoirs: a producer holding a charge
/////////////////////////////////////////////////////////////////////////////////////////////
//
//Charge is state rather than supply, so storage is solved FIRST and can feed a circuit
//before anything upstream has been solved. That is what makes a cold aircraft startable:
//an accumulator cranks the APU, and the APU turning the pumps refills it.
//
//A store DRAINS while nothing is covering for it - its charging circuit where it has one,
//otherwise whatever else feeds its output - and REFILLS when something is.
//
//Takes every producer field, plus:
//
//  rechargedBy[]  circuit that refills it, with its threshold. Never name the circuit it
//                 feeds, or it will top itself up forever
//  startedBy      gate of something it cranks; spends its usable charge when that rises
//  startAbove     value it must reach for that start to happen at all
//  stopBelow      value it stops discharging at. For a gas-charged store this is the
//                 precharge, which is not usable pressure
//  startRecharge  seconds to refill, once its recharge circuit is up
//  emerDischarge  seconds full to empty while supplying as an emergency source
//  leakStartDmg   damage at which it starts leaking; 0 for never
//  leakSeconds    full to empty at FULL damage, ramping from the threshold, so a light hit
//                 weeps and a bad one dumps. Destroyed empties at once
//  drainedBy[]    other damage roles that vent this store - a gun or pylons sharing a
//                 reservoir add to its damage rather than being a second mechanism
//
/////////////////////////////////////////////////////////////////////////////////////////////
// CIRCUITS - state Core publishes about a node
/////////////////////////////////////////////////////////////////////////////////////////////
//
//A bus being up is a fact about the circuit, not something drawing from it, so it is
//declared here rather than as a consumer. This is how acBusOn, apuOn and the rest reach
//everything outside Core.
//
//  variableName  what it publishes as
//  circuit       the node it reports on
//  minValue      value at or above which it reads as up
//  networked     see above
//
/////////////////////////////////////////////////////////////////////////////////////////////
// CONSUMERS - anything that needs supply to work
/////////////////////////////////////////////////////////////////////////////////////////////
//
//Not for reporting a circuit - use a Circuit for that. A consumer is a thing that stops
//working without supply: flight controls, a tail rotor.
//
//  variableName  what it publishes as
//  suppliedBy[]  circuits that can feed it, each with its threshold
//  needsAll      1 to require all of them; default is ANY, so a consumer naming two
//                circuits survives losing one and a consumer naming one dies with it.
//                Entries may span units - a pressure and a level in the same set
//  networked     see above
//
//Core publishes whether a consumer has supply. What that MEANS is the aircraft's business:
//Core reports the primary circuit is at 0 psi, the aircraft decides whether that warrants
//a caution and whether it is expected on the ground.

#endif
