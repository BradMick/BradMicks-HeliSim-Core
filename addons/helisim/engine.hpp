#ifndef BMKHS_HELISIM_ENGINE_REFERENCE_HPP
#define BMKHS_HELISIM_ENGINE_REFERENCE_HPP

//BMKHS gas turbine engine - field reference.
//
//The engine is a gas turbine worked station by station, every frame, from Ng:
//
//    2    inlet           ambient air, raised by ram in forward flight
//    3    compressor      pressure ratio, airflow and temperature from corrected Ng
//    4    combustor       fuel heat raises the gas to T4
//    4.5  compressor turbine  takes back exactly what the compressor needs; TGT is read here
//    5    power turbine   expands the rest to ambient - that is the torque
//
//Ng is NOT scheduled. The compressor turbine's power against the compressor's accelerates
//the spool, so start, light-off, self-sustain, idle and spool-up all fall out of one power
//balance. Np is its own state behind a sprag clutch. Hot days, cold days and altitude come in
//through corrected Ng (Ng / sqrt(theta)) and inlet pressure - nothing is bolted on per day.
//
//Declared per engine, in class Engines >> EngineNN, one class per engine. Core loops them,
//so an aircraft declares as many engines as it has.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// UNITS
/////////////////////////////////////////////////////////////////////////////////////////////
//
//Temperature deg C. Pressure ratio dimensionless. Airflow kg/s. Power kW. Speeds (Ng, Np) as
//fractions of 100% - 1.0 is 100%. Fuel is "fuel units": kg/s = units * maxFuelFlow.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// TEMPLATE (the AH-64D's T700-GE-701C)
/////////////////////////////////////////////////////////////////////////////////////////////
//
//    class Engines {
//        class Engine01 {
//            name            = "eng01";
//            damageRole      = "engines";
//            damageRoleIndex = 0;
//            engineType      = "turboShaftEngine";
//            designRpm       = 20900;
//            npFly           = 1.01;
//            maxFuelFlow     = 0.033;
//            powerKw         = 1066;
//            maxNg           = 1.10;
//            maxNp           = 1.196;
//            //book limits: oilPsiLimits[], ngMin, ngLimits[], npLimits[], tqLimits[],
//            //tgtLimits[], tqLimitsSe[], tgtLimitsSe[]
//
//            class Compressor {
//                pressureRatio  = 17.0;
//                massFlow       = 4.6;
//                inletDiameter  = 0.396;
//                ramRecovery    = 1.0;
//                compDrag       = 3.4;
//                compDragFloor  = 0.10;
//                airflowTable[] = {
//                     {-40, 0.9430}
//                    ,{ 15, 1.0000}
//                    ,{ 40, 1.2354}
//                };
//                lightOffNg   = 0.15;
//                selfSustNg   = 0.52;
//                idleNg       = 0.679;
//                ngLimitMax   = 1.022;
//                ngLimitBase  = 1.01436;
//                ngLimitSlope = 0.0019091;
//            };
//            class Combustor {
//                fuelLhv             = 43000;
//                combustorEfficiency = 0.99;
//                maxTgt           = 867;
//                maxTgtSe         = 896;
//                startTgt         = 851;
//                startMinTgt      = 80;
//                residualHeatGain = 0.003;
//            };
//            class CompressorTurbine {
//                turbineEfficiency = 0.88;
//                spoolInertia      = 5.0;
//            };
//            class PowerTurbine {
//                ptEfficiency = 0.88;
//                ptInertia    = 0.60;
//                ptDrag       = 0.06;
//                ptDragFloor  = 0.05;
//            };
//            class Governor {
//                fuelIdle        = 0.784;
//                fuelFly         = 3.136;
//                startFuelBase   = 0.50;
//                ffwdGain        = 1.00;
//                leverTravelTime = 8.0;
//                loadShareGain   = 8.0;
//                pid[]           = {80, 40, 0, 0.075};
//                gate[]          = {};
//            };
//            class Starter {
//                type      = "pneumatic";
//                torque    = 0.45;
//                runawayNg = 0.25;
//                gate[]    = {{"PNEU", 0.85}};
//            };
//        };
//        class Engine02 : Engine01 { name = "eng02"; damageRoleIndex = 1; };
//    };
//
/////////////////////////////////////////////////////////////////////////////////////////////
// FIELD REFERENCE
/////////////////////////////////////////////////////////////////////////////////////////////
//
//WHOLE ENGINE
//  engineType          "turboShaftEngine". Picks the assembly - bmkhs_fnc_turboShaftEngine.
//  designRpm           Power turbine RPM at 100% Np. The shaft reference the transmission reads.
//  npFly               Governed Np in FLY, as a fraction of designRpm (1.01 = 101%).
//  powerKw             kW at 100% torque. Sets refTq = P / omega, the torque gauge's 100%.
//                      The torque the engine actually MAKES comes from the physics.
//  maxFuelFlow         kg/s per fuel unit. The combustor burns fuel units * maxFuelFlow kg/s,
//                      so this is physical, not just the gauge's scale.
//  maxNg               Fly-weight trip - shuts the engine down. ALSO the compressor map's
//                      100% point: the map's Ng axis runs 0 to 1 as a fraction of maxNg.
//  maxNp               Electrical overspeed trip - shuts the engine down.
//  fuelSelector        OPTIONAL. The control whose position picks this engine's fuel source -
//                      a FUEL SYS lever, say. Declare none and the engine draws through the
//                      fuel config's CrossfeedModes.
//  fuelSources[]       With fuelSelector: the source for each of that control's positions,
//                      in order - a fuel tank variableName, or "off" for no fuel.
//
//COMPRESSOR - stations 2 to 3
//  pressureRatio       Compressor pressure ratio at Ng 1.0. A spec-sheet number.
//  massFlow            kg/s airflow at Ng 1.0 on a standard day. A spec-sheet number.
//  inletDiameter       m. Not used by any calculation yet - reserved for inlet duct losses.
//  ramRecovery         Share of the ram pressure rise the inlet keeps, 0..1. Applies to the
//                      RISE only, so standing still the inlet sees exactly ambient.
//  compDrag            Spooling down, unfired: drag on the spool as compDrag * Ng^2.
//  compDragFloor       Finishes the stop - Ng^2 alone only asymptotes.
//  airflowTable[]      {FAT, multiplier} pairs, clamped at the ends. THE ONE TUNING TABLE -
//                      see TUNING. Must read 1.0 at 15 so a standard day is the untrimmed
//                      engine.
//  compressorMap[]     OPTIONAL. The engine's own compressor map; declare none and Core's
//                      T700-701C map is used. Rows {Ng / maxNg (corrected), PR / pressureRatio,
//                      flow / massFlow, efficiency, ln(compressor turbine expansion) /
//                      ln(pressureRatio)}, low to high.
//  lightOffNg          Ng at which fuel is introduced.
//  selfSustNg          Ng at which the starter cuts out and the engine reads ON.
//  idleNg              Ng the idle fuel settles at. Also where the start fuel ramp reaches
//                      full, and the floor the governor's minimum flow holds in flight.
//  ngLimitMax          Ng limiter ceiling. The limiter holds Ng under
//  ngLimitBase         ngLimitMax min (ngLimitBase + ngLimitSlope * FAT) - the cold-day,
//  ngLimitSlope        corrected-speed limit. Restricts fuel; Nr droops.
//
//COMBUSTOR - stations 3 to 4
//  fuelLhv             kJ/kg fuel heating value. 43000 for JP-8 / Jet A.
//  combustorEfficiency Share of that heat the gas receives.
//  maxTgt              deg C. TGT limiter, twin engine. Restricts fuel; Nr droops.
//  maxTgtSe            deg C. TGT limiter, single engine.
//  startTgt            deg C. The start's transient limit - over it during a start damages.
//  startMinTgt         deg C. Residual TGT to motor below before moving the power lever.
//  residualHeatGain    How hard an un-purged hot section runs away on a restart.
//
//COMPRESSOR TURBINE - stations 4 to 4.5
//  turbineEfficiency   Isentropic efficiency.
//  spoolInertia        How fast Ng answers a power change.
//
//POWER TURBINE - stations 4.5 to 5
//  ptEfficiency        Isentropic efficiency of the expansion to ambient.
//  ptInertia           The free turbine's own inertia.
//  ptDrag              Drag on a released turbine, as ptDrag * Np^2.
//  ptDragFloor         Finishes the stop - windmilling only.
//
//GOVERNOR
//  fuelIdle            Fuel units at IDLE. Set it so the engine settles at idleNg.
//  fuelFly             Fuel units wide open at FLY - the orifice the governor cuts back from.
//  startFuelBase       Fuel at light-off as a fraction of idle fuel. Sets the start peak.
//  ffwdGain            Collective anticipation - the load demand spindle.
//  leverTravelTime     s, idle to fly. The fuel ramp, the Np reference and the lever animation.
//  loadShareGain       How hard an engine below its matched partners trims up to them.
//  pid[]               Np governor {kp, ki, kd, ki_clamp}. ki * ki_clamp is the integral's
//                      fuel authority - keep it at the fuel range.
//  gate[]              What the ECU needs to keep metering fuel. Empty = always powered.
//
//STARTER
//  type                "pneumatic" | "electric".
//  torque              Stalled torque on the spool, normalised.
//  runawayNg           Ng at which it has no torque left - an air turbine's torque falls with
//                      speed. With no fuel, motoring settles here.
//  gate[]              What it needs available. Empty = always supplied.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// TUNING - putting an engine on its charts
/////////////////////////////////////////////////////////////////////////////////////////////
//
//1. Spec numbers first: pressureRatio, massFlow, powerKw, designRpm, the limits.
//2. Idle: fuelIdle until a lever-at-IDLE engine settles at idleNg.
//3. Start: startFuelBase for the peak TGT, Starter >> torque for the time to light-off,
//   runawayNg for the motoring speed.
//4. Max torque available: on a standard day (15 C) max power is the untrimmed engine. On
//   every other FAT, set that airflowTable row so max torque matches the Maximum Torque
//   Available chart - less air where the engine makes too much, more where too little. The
//   Ng limit and the TGT limit then decide WHICH limit holds, as they do on the real engine.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// WHAT CORE COMPUTES AND YOU NEVER DECLARE
/////////////////////////////////////////////////////////////////////////////////////////////
//
//  compressor map      PR, airflow, efficiency and compressor turbine expansion against
//                      corrected Ng, normalised - the T700-701C's, scaled by each engine's
//                      own pressureRatio, massFlow and maxNg, unless it declares compressorMap[]
//  gas properties      gamma and cp, cold air and hot gas
//  corrected speed     Ng / sqrt(T2 / 288.15) - the hot-day / cold-day behaviour
//  ram                 total T2 and P2 from flight Mach
//  TGT gauge lag       how fast the reading heats, cools, and cools in still air and ram
//  spool power scale   from the compressor's power at Ng 1.0
//  minimum flow        the governor never pulls Ng below idleNg in flight
//  refTq               powerKw at designRpm * npFly
//
/////////////////////////////////////////////////////////////////////////////////////////////

#endif
