"""EXPERIMENT, not a port: the cold section with the compressor's load scaled by air density.
engine.py stays a 1:1 port of Core; this swaps one function in memory and reruns the perf grid.

Run it:  python python/density_experiment.py results.json
         python python/perf_chart.py results.json chart.html
"""
import math
import sys

import engine


def gas_turbine_cold_section(eng, ng, fuelCmd, starterTq, dens, running, spooling, dt):
    fuelGas = fuelCmd * dens if running else 0.0
    airGas = ((ng ** eng['massFlowExp']) * dens) * eng['airCoef']
    compLoad = eng['compressorLoad']
    if running:
        #CHANGED: denser air, more mass through the compressor, more work to turn it.
        absorbed = compLoad * eng['compRunMult'] * (ng ** eng['compRunExp']) * dens
    else:
        absorbed = (compLoad * (eng['compDragMult'] if spooling else 1.0) * ng * ng
                    + (eng['compDragFloor'] if spooling else 0.0))
    #CHANGED: the same density on the work taken out of the gas.
    compWork = compLoad * ng * ng * dens
    ngDot = (fuelGas + starterTq - absorbed) / eng['compressorInertia']
    ng = engine.clamp(ng + ngDot * dt, 0.0, 1.1)
    return ng, fuelGas, compWork, airGas


def gas_turbine_hot_section(eng, tgt, ng, fuelCmd, residualHeat, dens, fat, velY, running, dt):
    currentHeat = 1.0 + (residualHeat - 1.0) * max(1.0 - ng / eng['idleNg'], 0.0)
    #CHANGED: the rise scales with the inlet temperature ratio, not with density.
    theta = (fat + engine.DEG_C_TO_KELVIN) / (engine.STANDARD_TEMP + engine.DEG_C_TO_KELVIN)
    rise = eng['tgtK'] * fuelCmd / max(ng ** eng['massFlowExp'], 0.02) * theta
    tgtHot = fat + currentHeat * rise if running else fat
    ram = max(velY, 0.0) * eng['ramAirCoef']
    coolRate = eng['coolingCoef'] * (ng + eng['stillAirFlow'] + ram)
    rate = eng['thermalMassCoef'] if tgtHot > tgt else coolRate
    return tgt + (tgtHot - tgt) * rate * dt


#CORRECTED PARAMETERS: delta = dens * theta (pressure ratio). Compressor speed is Ng / sqrt(theta);
#mass flow goes as delta / sqrt(theta) and compressor power as delta * sqrt(theta), each times a
#function of corrected speed. On an ISA sea-level day every factor is 1 - nothing moves there.
def _theta(fat):
    return (fat + engine.DEG_C_TO_KELVIN) / (engine.STANDARD_TEMP + engine.DEG_C_TO_KELVIN)


def corrected_cold_section(eng, ng, fuelCmd, starterTq, dens, running, spooling, dt):
    theta = _theta(engine._FAT[0])
    delta = dens * theta
    nc = ng / theta ** 0.5
    fuelGas = fuelCmd if running else 0.0
    airGas = eng['airCoef'] * (nc ** eng['massFlowExp']) * delta / theta ** 0.5
    compLoad = eng['compressorLoad']
    if running and KNEE:
        #knee=a,e1,K,q: the compressor's load stiffening near its top speed - a base term and a
        #high-speed term, solved from the idle, fly and maximum anchors.
        absorbed = (KNEE[0] * nc ** KNEE[1] + KNEE[2] * nc ** KNEE[3]) * delta * theta ** 0.5
    elif running:
        absorbed = compLoad * eng['compRunMult'] * (nc ** eng['compRunExp']) * delta * theta ** 0.5
    else:
        absorbed = (compLoad * (eng['compDragMult'] if spooling else 1.0) * ng * ng
                    + (eng['compDragFloor'] if spooling else 0.0))
    engine._NC[0] = engine.clamp(ng + (fuelGas + starterTq - absorbed) / eng['compressorInertia'] * dt,
                                 0.0, 1.1) / theta ** 0.5
    #cw=<mode>: how the compressor work taken out of the power turbine's gas scales with temperature.
    compWork = compLoad * nc * nc * delta * {'dsqrt': theta ** 0.5, 'dinv': theta ** -0.5, 'd': 1.0}[CW_MODE]
    ngDot = (fuelGas + starterTq - absorbed) / eng['compressorInertia']
    ng = engine.clamp(ng + ngDot * dt, 0.0, 1.1)
    return ng, fuelGas, compWork, airGas


CW_MODE = next((a[3:] for a in sys.argv if a.startswith('cw=')), 'dsqrt')
KNEE = next((list(map(float, a[5:].split(','))) for a in sys.argv if a.startswith('knee=')), None)


#'cycle': the power turbine takes airflow x the absolute temperature it is given, times its share of
#the expansion, which rises with corrected speed. C, n0, b fitted to the port at ISA sea level.
PT_FIT = dict(C=0.34201, n0=0.6540, b=0.750, m=1.0)
#ptfit=C,n0,b,m - a refit tried without editing this file.
for _a in sys.argv:
    if _a.startswith('ptfit='):
        PT_FIT = dict(zip(('C', 'n0', 'b', 'm'), map(float, _a[6:].split(','))))


def cycle_power_turbine(eng, fuelGas, airGas, compWork, np_, nrFrac, dt):
    refTq = eng['refTq']
    mf = max(airGas / eng['airCoef'], 0.02)
    tgtHot = engine._FAT[0] + eng['tgtK'] * fuelGas / mf
    share = max(0.0, (engine._NC[0] - PT_FIT['n0']) / (1.0 - PT_FIT['n0'])) ** PT_FIT['b']
    ptGas = PT_FIT['C'] * mf * share * ((tgtHot + engine.DEG_C_TO_KELVIN)
                                        / (engine.STANDARD_TEMP + engine.DEG_C_TO_KELVIN)) ** PT_FIT['m']
    ptGas = ptGas if fuelGas > 0.0 else 0.0
    shaftTq = ptGas * refTq * eng['ptEfficiency']
    npDrag = eng['ptDrag'] * np_ * np_ + (eng['ptDragFloor'] if shaftTq <= 0.0 else 0.0)
    npDot = ((shaftTq / refTq) - npDrag) / eng['ptInertia']
    npFree = max(np_ + npDot * dt, 0.0)
    npDriven = np_ + ((shaftTq / refTq) / eng['ptInertia']) * dt
    clutch = (npDriven if fuelGas > 0.0 else npFree) >= nrFrac
    return shaftTq, (nrFrac if clutch else npFree), clutch


def corrected_hot_section(eng, tgt, ng, fuelCmd, residualHeat, dens, fat, velY, running, dt):
    theta = _theta(fat)
    massFlow = max((ng / theta ** 0.5) ** eng['massFlowExp'] * dens * theta / theta ** 0.5, 0.02)
    currentHeat = 1.0 + (residualHeat - 1.0) * max(1.0 - ng / eng['idleNg'], 0.0)
    tgtHot = fat + currentHeat * eng['tgtK'] * fuelCmd / massFlow if running else fat
    ram = max(velY, 0.0) * eng['ramAirCoef']
    coolRate = eng['coolingCoef'] * (ng + eng['stillAirFlow'] + ram)
    rate = eng['thermalMassCoef'] if tgtHot > tgt else coolRate
    return tgt + (tgtHot - tgt) * rate * dt


#Wide-open fuel x the accel schedule's ratio to a 60 F day, by FAT (read off the user's chart).
FUEL_RATIO = [[-53.9, 0.88], [-28.9, 0.96], [15.6, 1.0], [60.0, 1.0]]


if __name__ == '__main__':
    #python density_experiment.py results.json [hot | corrected [ratio] [tgtk=<multiplier>]]
    for arg in sys.argv:
        if arg.startswith('tgtk='):
            engine.OVERRIDES['tgtK'] = engine.Heli().H['bmkhs_engines'][0]['tgtK'] * float(arg[5:])
        #set:<engine key>=<value> - any engine config value, tried without editing the .hpp.
        if arg.startswith('set:'):
            k, v = arg[4:].split('=')
            engine.OVERRIDES[k] = float(v)
    if 'ratio' in sys.argv:
        _pulled = engine.pulled
        fuelFly = engine.Heli().H['bmkhs_engines'][0]['fuelFly']

        def pulled(pa, fat, single=False, coll=1.0, secs=100.0):
            ceiling = fuelFly * engine.math_linear_interp(FUEL_RATIO, fat)[1]
            #'delta': the ceiling also scales with the atmosphere's pressure ratio, as the
            #schedule's P3 does - the same barometric expression fn_environment uses.
            if 'delta' in sys.argv:
                ceiling *= math.exp(-engine.GRAVITY * engine.MOLAR_MASS_OF_AIR * pa * engine.FEET_TO_METERS
                                    / (engine.UNIVERSAL_GAS_CONSTANT * (fat + engine.DEG_C_TO_KELVIN)))
            engine.OVERRIDES['fuelFly'] = ceiling
            return _pulled(pa, fat, single, coll, secs)
        engine.pulled = pulled
    if 'corrected' in sys.argv:
        #The cold section has no FAT parameter; the frame's FAT is captured from the environment.
        engine._FAT = [15.0]
        engine._NC = [0.0]
        if 'cycle' in sys.argv:
            engine.turbo_shaft_power_turbine = cycle_power_turbine
        _env = engine.environment

        def environment(H):
            _env(H)
            engine._FAT[0] = H['bmkhs_FAT']
        engine.environment = environment
        engine.gas_turbine_cold_section = corrected_cold_section
        engine.gas_turbine_hot_section = corrected_hot_section
    else:
        engine.gas_turbine_cold_section = gas_turbine_cold_section
        if 'hot' in sys.argv:
            engine.gas_turbine_hot_section = gas_turbine_hot_section
    engine.perf_report(sys.argv[1] if len(sys.argv) > 1 else None)
