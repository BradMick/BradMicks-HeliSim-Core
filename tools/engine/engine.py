"""The HeliSim engine, drivetrain and simple rotors, outside Arma. A 1:1 port of Core.

Run it:  python tools/engine/engine.py

Every function below is named after the SQF function it ports and carries its inputs, outputs
and expressions line for line. The frame runs in fn_coreUpdate's order: environment ->
engineUpdate -> transmissionUpdate -> simpleRotorUpdate. Values are read from the AH-64D's
own config files, so there is no third copy of any number.

Not ported, because nothing on the engine's path reads it: systems circuits (gates are
scenario inputs), damage, torque jitter and the rotor's lift/force path.

If this file and the SQF ever disagree, the SQF is right and this file is the bug.
"""
import copy
import math
import random
import re

AH64_CONFIG = r'E:\AH-64D\addons\fza_ah64_helisim\config\bmkhs_config'

#core.hpp
ISA_STD_DAY_AIR_DENSITY = 1.225
VEL_VNE = 128.611
VEL_VRS = 24.384
VEL_ETL = 12.347
METERS_TO_FEET = 3.28084
FEET_TO_METERS = 0.3048
GRAVITY = 9.806
MOLAR_MASS_OF_AIR = 0.0289644
UNIVERSAL_GAS_CONSTANT = 8.31432
DEG_C_TO_KELVIN = 273.15
SEA_LEVEL_PRESSURE = 29.92
STANDARD_TEMP = 15
IN_MG_TO_HPA = 33.8639
#engine.hpp
GT_OIL_PSI_SCALE = 0.90
GT_SINGLE_ENG_TQ_RATIO = 0.51
GT_TGT_LIMIT_BAND = 40.0
GT_NG_LIMIT_BAND = 0.030
GT_LIMIT_GAIN = 16.0
GT_LIMIT_TRACK = 1.02
#rotor.hpp
MAIN, TAIL = 0, 1
CCW, CW = 0, 1


# ---------------------------------------------------------------------------------------------
# Config - the .hpp files, parsed
# ---------------------------------------------------------------------------------------------

_TOK = re.compile(r'"[^"]*"|-?\d+\.\d*|-?\d*\.\d+|-?\d+|[A-Za-z_][A-Za-z0-9_]*|[{}\[\]=;:,]')


def _parse_value(toks, i):
    t = toks[i]
    if t == '{':
        arr = []
        i += 1
        while toks[i] != '}':
            v, i = _parse_value(toks, i)
            arr.append(v)
            if toks[i] == ',':
                i += 1
        return arr, i + 1
    if t.startswith('"'):
        return t[1:-1], i + 1
    try:
        return float(t), i + 1
    except ValueError:
        return t, i + 1


def _parse_body(toks, i, scope):
    while i < len(toks) and toks[i] != '}':
        if toks[i] == 'class':
            name, i = toks[i + 1], i + 2
            parent = None
            if toks[i] == ':':
                parent, i = toks[i + 1], i + 2
            body = copy.deepcopy(scope[parent]) if parent else {}
            i = _parse_body(toks, i + 1, body) + 1
            if i < len(toks) and toks[i] == ';':
                i += 1
            scope[name] = body
        else:
            key, i = toks[i], i + 1
            if toks[i] == '[':
                i += 2
            i += 1
            val, i = _parse_value(toks, i)
            if i < len(toks) and toks[i] == ';':
                i += 1
            scope[key] = val
    return i


def parse_hpp(path):
    text = re.sub(r'//[^\n]*', '', open(path, encoding='utf-8', errors='ignore').read())
    scope = {}
    _parse_body(_TOK.findall(text), 0, scope)
    return scope


def load_config():
    cfg = {}
    for f in ('helisim_engine.hpp', 'helisim_simpleRotor.hpp'):
        cfg.update(parse_hpp(AH64_CONFIG + '\\' + f))
    return cfg


# ---------------------------------------------------------------------------------------------
# Utilities - BIS / Core helpers
# ---------------------------------------------------------------------------------------------

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def lerp(a, b, t):
    """BIS_fnc_lerp."""
    return a + (b - a) * t


def linear_conversion(lo, hi, v, a, b, clip):
    out = a + (v - lo) * (b - a) / (hi - lo)
    return clamp(out, min(a, b), max(a, b)) if clip else out


def sqf_round(x):
    return math.floor(x + 0.5)


def math_linear_interp(arr, key):
    """fn_mathLinearInterp."""
    upper = next((i for i, r in enumerate(arr) if r[0] > key), -1)
    if upper == 0:
        return arr[0]
    if upper == -1:
        return arr[-1]
    lo, hi = arr[upper - 1], arr[upper]
    return [key] + [lo[i] + (hi[i] - lo[i]) / (hi[0] - lo[0]) * (key - lo[0])
                    for i in range(1, len(lo))]


def math_build_interp_grid(arr):
    """fn_mathBuildInterpGrid - validation omitted, the config is known good."""
    return [arr[0][1:], arr[1:]]


def math_linear_interp_2d(grid, rowKey, colKey):
    """fn_mathLinearInterp2D."""
    colKeys, rows = grid
    row = math_linear_interp(rows, rowKey)
    return math_linear_interp([[k, row[i + 1]] for i, k in enumerate(colKeys)], colKey)[1]


def pid_create(kp, ki, kd, kiClamp):
    return {'kp': kp, 'ki': ki, 'kd': kd, 'ki_clamp': kiClamp, 'prevError': 0.0, 'integral': 0.0}


def pid_run(pid, dt, desired, actual):
    """fn_pidRun."""
    error = desired - actual
    integral = clamp(pid['integral'] + error * dt, -pid['ki_clamp'], pid['ki_clamp'])
    raw = 0.0 if dt == 0 else (error - pid['prevError']) / dt
    dCoef = pid.get('dCoef', 0.3)
    prev = pid.get('derivFilt', raw)
    derivative = prev + dCoef * (raw - prev)
    out = pid['kp'] * error + pid['ki'] * integral + pid['kd'] * derivative
    pid['prevError'], pid['integral'], pid['derivFilt'] = error, integral, derivative
    return out


def pid_reset(pid):
    """fn_pidReset."""
    pid['prevError'], pid['integral'], pid['derivFilt'] = 0.0, 0.0, 0.0


# ---------------------------------------------------------------------------------------------
# Init - fn_environmentVariables, fn_engineVariables, fn_simpleRotorVariables
# ---------------------------------------------------------------------------------------------

NUM_FIELDS = ['designRpm', 'npFly', 'maxFuelFlow', 'powerKw', 'maxNg', 'maxNp']
SECTION_FIELDS = [
    ('ColdSection', ['compressorInertia', 'compressorLoad', 'airCoef', 'compRunMult', 'compRunExp',
                     'compDragMult', 'compDragFloor', 'lightOffNg', 'selfSustNg', 'idleNg',
                     'ngLimitMax', 'ngLimitBase', 'ngLimitSlope']),
    ('HotSection', ['massFlowExp', 'tgtK', 'thermalMassCoef', 'coolingCoef', 'stillAirFlow',
                    'ramAirCoef', 'maxTgt', 'maxTgtSe', 'startTgt', 'startMinTgt', 'residualHeatGain']),
    ('PowerTurbine', ['ptEfficiency', 'ptInertia', 'ptDrag', 'ptDragFloor']),
    ('Governor', ['fuelIdle', 'fuelFly', 'startFuelBase', 'ffwdGain', 'leverTravelTime', 'loadShareGain']),
]
ROTOR_NUM_FIELDS = ['numBlades', 'mastLength', 'gearRatio', 'torqueTau', 'bladeRadius',
                    'bladeChord', 'bladeMass', 'reacTqScalar', 'autoTorque']


def engine_variables(H, cfg, overrides=None):
    engines = []
    for i in range(1, 3):
        e = cfg['Engines']['Engine%02d' % i]
        eng = {k: e[k] for k in NUM_FIELDS}
        eng['name'] = e['name']
        for section, fields in SECTION_FIELDS:
            for f in fields:
                eng[f] = e[section][f]
        eng['pid'] = e['Governor']['pid']
        eng['starterTorque'] = e['Starter']['torque']
        eng['starterGates'] = e['Starter']['gate']
        eng['governorGates'] = e['Governor']['gate']
        eng['refTq'] = (eng['powerKw'] * 1000) / (eng['designRpm'] * eng['npFly'] * 0.10472)
        eng.update(overrides or {})
        engines.append(eng)
    H['bmkhs_engines'] = engines
    H['bmkhs_engPowerLeverState'] = ['OFF', 'OFF']
    H['bmkhs_engState'] = ['OFF', 'OFF']
    H['bmkhs_pid_engine'] = [pid_create(*eng['pid']) for eng in engines]
    z = [0.0, 0.0]
    for k in ('bmkhs_engPctNg', 'bmkhs_engNp', 'bmkhs_engPctNp', 'bmkhs_engPctTq',
              'bmkhs_engOilPsi', 'bmkhs_engFuelFlow', 'bmkhs_engOutputTq', 'bmkhs_engLeverSched'):
        H[k] = list(z)
    H['bmkhs_engClutch'] = [False, False]
    H['bmkhs_engineOverspeed'] = [False, False]
    #The rig models no damage, so these stay healthy - kept so the lines reading them match Core.
    H['bmkhs_engOilHealth'] = [1.0, 1.0]
    H['bmkhs_engFailed'] = [False, False]
    H['bmkhs_engLimFuel'] = [e['fuelFly'] for e in engines]
    H['bmkhs_engClutchSlip'] = [1.0, 1.0]
    H['bmkhs_engTgt'] = [H['bmkhs_FAT'], H['bmkhs_FAT']]
    H['bmkhs_engResidualHeat'] = [1.0, 1.0]
    H['bmkhs_engNpRef'] = [-1.0, -1.0]
    H['bmkhs_engPrevLever'] = ['OFF', 'OFF']


def simple_rotor_variables(H, cfg):
    rotors = []
    for i in range(1, int(cfg['numSimpleRotors']) + 1):
        r = cfg['SimpleRotors']['SimpleRotor%02d' % i]
        rot = {k: r[k] for k in ROTOR_NUM_FIELDS}
        rot['type'] = TAIL if r['type'].lower() == 'tail' else MAIN
        rot['dir'] = CW if r['direction'].lower() == 'cw' else CCW
        rot['dragCoefTable'] = math_build_interp_grid(r['dragCoefTable'])
        rot['liftCoefTable'] = math_build_interp_grid(r['liftCoefTable'])
        rotors.append(rot)
    H['bmkhs_simpleRotors'] = rotors
    H['bmkhs_reqEngTorque'] = [0.0, 0.0]
    H['bmkhs_rtrMoi'] = [0.0, 0.0]


# ---------------------------------------------------------------------------------------------
# fn_environment - ISA_STD base
# ---------------------------------------------------------------------------------------------

#Sea-level temperature; 15 is ISA and what Core's environment uses.
BASE_FAT = 15.0


def environment(H):
    baroAlt = H['baroAltM'] * METERS_TO_FEET
    baseAlt, baseFAT = 0, BASE_FAT
    altitude = sqf_round((baseAlt + baroAlt) / 10) * 10
    altimeter = 29.92
    temperature = baseFAT - sqf_round((baroAlt / 1000) * 2)
    refPressure = altimeter * IN_MG_TO_HPA
    exp_ = (-GRAVITY * MOLAR_MASS_OF_AIR * ((altitude - 0) * FEET_TO_METERS)
            / (UNIVERSAL_GAS_CONSTANT * (temperature + DEG_C_TO_KELVIN)))
    pressure = ((refPressure / 0.01) * math.exp(exp_)) * 0.01
    H['bmkhs_PA'] = altitude
    H['bmkhs_FAT'] = temperature
    H['bmkhs_rho'] = (pressure / 0.01) / (287.05 * (temperature + DEG_C_TO_KELVIN))


# ---------------------------------------------------------------------------------------------
# Engine - fn_engineGovernor, gasTurbine/*, turboShaftEngine/*
# ---------------------------------------------------------------------------------------------

def engine_governor(H, i, eng, ng, np_, tgt, lever, fat, dt):
    idleNg = eng['idleNg']

    tqs = H['bmkhs_engPctTq']
    H['bmkhs_isSingleEng'] = any(t < max(tqs) * GT_SINGLE_ENG_TQ_RATIO for t in tqs)
    target = {'FLY': eng['fuelFly'], 'IDLE': eng['fuelIdle']}.get(lever, 0.0)

    sched = H['bmkhs_engLeverSched'][i]
    if lever == 'FLY' and target > sched:
        sched = min(max(sched, eng['fuelIdle'])
                    + ((eng['fuelFly'] - eng['fuelIdle']) / eng['leverTravelTime']) * dt, target)
    else:
        sched = target
    H['bmkhs_engLeverSched'][i] = sched

    govPowered = H['governorPowered'][i] if eng['governorGates'] else True

    pid = H['bmkhs_pid_engine'][i]
    npRef = H['bmkhs_engNpRef'][i]
    orifice = sched
    if lever == 'FLY' and govPowered:
        if npRef < 0.0:
            pid['integral'] = sched / pid['ki']
            pid['prevError'] = 0.0
            npRef = np_
        npTarget = npRef + (1.0 - npRef) * linear_conversion(eng['fuelIdle'], eng['fuelFly'], sched, 0.0, 1.0, True)
        integral = pid['integral']
        govFuel = pid_run(pid, dt, npTarget, np_) + H['bmkhs_collectiveOutput'] * eng['ffwdGain']
        tgtLim = eng['maxTgtSe'] if H['bmkhs_isSingleEng'] else eng['maxTgt']
        ngLim = min(eng['ngLimitMax'], eng['ngLimitBase'] + eng['ngLimitSlope'] * fat)
        err = min((tgtLim - tgt) / GT_TGT_LIMIT_BAND, (ngLim - ng) / GT_NG_LIMIT_BAND)
        lim = H['bmkhs_engLimFuel'][i] + GT_LIMIT_GAIN * err * dt
        lim = min(lim, min(sched, max(govFuel, 0.0)) * GT_LIMIT_TRACK)
        lim = min(max(lim, eng['fuelIdle']), eng['fuelFly'])
        H['bmkhs_engLimFuel'][i] = lim
        allowed = min(sched, lim)
        if govFuel > allowed:
            pid['integral'] = integral
        elif H['bmkhs_engClutch'][i]:
            #Load sharing: an engine below the average of those matched with it trims up; one above
            #is never trimmed down.
            matched = [k for k, e in enumerate(H['bmkhs_engines'])
                       if H['bmkhs_engPowerLeverState'][k] == 'FLY' and H['bmkhs_engClutch'][k]]
            if len(matched) > 1:
                share = [H['bmkhs_engOutputTq'][k] / H['bmkhs_engines'][k]['refTq'] for k in matched]
                mismatch = sum(share) / len(share) - H['bmkhs_engOutputTq'][i] / eng['refTq']
                mismatch = max(mismatch, 0.0)
                pid['integral'] = clamp(pid['integral'] + eng['loadShareGain'] * mismatch * dt / pid['ki'],
                                        -pid['ki_clamp'], pid['ki_clamp'])
        orifice = min(allowed, max(govFuel, 0.0))
    else:
        pid_reset(pid)
        npRef = -1.0
        H['bmkhs_engLimFuel'][i] = eng['fuelFly']
    H['bmkhs_engNpRef'][i] = npRef

    fuelCmd = orifice
    if ng < idleNg:
        base = eng['startFuelBase']
        fuelCmd = fuelCmd * min(base + (1.0 - base) * ng / idleNg, 1.0)

    rotorTq = sum(H['bmkhs_reqEngTorque'])
    lvrState = H['bmkhs_engPowerLeverState']
    totalEngineTq = sum(e['refTq'] for k, e in enumerate(H['bmkhs_engines']) if lvrState[k] == 'FLY')
    share = rotorTq * (eng['refTq'] / totalEngineTq) if lever == 'FLY' and totalEngineTq > 0 else 0.0
    return fuelCmd, share, orifice


def gas_turbine_starter(H, i, eng, ng):
    sw = H['startSw'][i]
    starting = sw > 0 or H['bmkhs_engState'][i] == 'STARTING'
    override = sw < 0
    if ng >= eng['selfSustNg']:
        if H['bmkhs_engState'][i] == 'STARTING':
            H['bmkhs_engState'][i] = 'ON'
        return 0.0
    if not starting and not override:
        return 0.0
    if H['bmkhs_engineOverspeed'][i]:
        return 0.0
    if H['bmkhs_engFailed'][i]:
        return 0.0
    supplied = H['starterSupplied'][i] if eng['starterGates'] else True
    return eng['starterTorque'] if supplied else 0.0


def gas_turbine_cold_section(eng, ng, fuelCmd, starterTq, dens, running, spooling, dt):
    fuelGas = fuelCmd * dens if running else 0.0
    airGas = ((ng ** eng['massFlowExp']) * dens) * eng['airCoef']
    compLoad = eng['compressorLoad']
    if running:
        absorbed = compLoad * eng['compRunMult'] * (ng ** eng['compRunExp'])
    else:
        absorbed = (compLoad * (eng['compDragMult'] if spooling else 1.0) * ng * ng
                    + (eng['compDragFloor'] if spooling else 0.0))
    compWork = compLoad * ng * ng
    ngDot = (fuelGas + starterTq - absorbed) / eng['compressorInertia']
    ng = clamp(ng + ngDot * dt, 0.0, 1.1)
    return ng, fuelGas, compWork, airGas


def gas_turbine_hot_section(eng, tgt, ng, fuelCmd, residualHeat, dens, fat, velY, running, dt):
    massFlow = max((ng ** eng['massFlowExp']) * dens, 0.02)
    currentHeat = 1.0 + (residualHeat - 1.0) * max(1.0 - ng / eng['idleNg'], 0.0)
    tgtHot = fat + currentHeat * eng['tgtK'] * fuelCmd / massFlow if running else fat
    ram = max(velY, 0.0) * eng['ramAirCoef']
    coolRate = eng['coolingCoef'] * (ng + eng['stillAirFlow'] + ram)
    rate = eng['thermalMassCoef'] if tgtHot > tgt else coolRate
    return tgt + (tgtHot - tgt) * rate * dt


def turbo_shaft_power_turbine(eng, fuelGas, airGas, compWork, np_, nrFrac, dt):
    refTq = eng['refTq']
    ptGas = max(fuelGas + airGas - (compWork if fuelGas > 0.0 else 0.0), 0.0)
    shaftTq = ptGas * refTq * eng['ptEfficiency']
    npDrag = eng['ptDrag'] * np_ * np_ + (eng['ptDragFloor'] if shaftTq <= 0.0 else 0.0)
    npDot = ((shaftTq / refTq) - npDrag) / eng['ptInertia']
    npFree = max(np_ + npDot * dt, 0.0)
    npDriven = np_ + ((shaftTq / refTq) / eng['ptInertia']) * dt
    clutch = (npDriven if fuelGas > 0.0 else npFree) >= nrFrac
    return shaftTq, (nrFrac if clutch else npFree), clutch


def turbo_shaft_engine(H, i, eng):
    dt = H['bmkhs_deltaTime']
    fat = H['bmkhs_FAT']
    dens = H['bmkhs_rho'] / ISA_STD_DAY_AIR_DENSITY
    velY = H['bmkhs_velModelSpace'][1]

    ng = H['bmkhs_engPctNg'][i]
    np_ = H['bmkhs_engNp'][i]
    tgt = H['bmkhs_engTgt'][i]
    residualHeat = H['bmkhs_engResidualHeat'][i]
    lever = H['bmkhs_engPowerLeverState'][i]

    prevLever = H['bmkhs_engPrevLever'][i]
    if prevLever == 'OFF' and lever != 'OFF':
        residualHeat = 1.0 + eng['residualHeatGain'] * tgt
        H['bmkhs_engResidualHeat'][i] = residualHeat
    if prevLever != lever:
        H['bmkhs_engPrevLever'][i] = lever

    tripped = H['bmkhs_engineOverspeed'][i]
    if not tripped and (ng >= eng['maxNg'] or np_ >= eng['maxNp']):
        tripped = True
        H['bmkhs_engineOverspeed'][i] = True

    fuelAvail = H['fuelAvail'][i]
    failed = H['bmkhs_engFailed'][i]
    running = ng > eng['lightOffNg'] and lever != 'OFF' and fuelAvail and not tripped and not failed

    starterTq = gas_turbine_starter(H, i, eng, ng)
    cranking = starterTq > 0.0
    spooling = not running and not cranking

    fuelCmd, share, orifice = engine_governor(H, i, eng, ng, np_, tgt, lever, fat, dt)
    refTq = eng['refTq']

    ngNew, gasPower, compWork, airGas = gas_turbine_cold_section(
        eng, ng, fuelCmd, starterTq, dens, running, spooling, dt)
    tgt = gas_turbine_hot_section(eng, tgt, ngNew, fuelCmd, residualHeat, dens, fat, velY, running, dt)

    xmsnRpm = H['bmkhs_xmsnOutputRpm']
    nrFrac = xmsnRpm / (eng['npFly'] * eng['designRpm'])

    tqOut, npNew, clutch = turbo_shaft_power_turbine(
        eng, gasPower, airGas, compWork, np_, nrFrac, dt)

    if ngNew >= eng['selfSustNg'] and running:
        state = 'ON'
    elif cranking or running:
        state = 'STARTING'
    else:
        state = 'OFF'

    tqOut = tqOut * H['bmkhs_engClutchSlip'][i]
    H['bmkhs_engPctNg'][i] = ngNew
    H['bmkhs_engTgt'][i] = tgt
    H['bmkhs_engOutputTq'][i] = tqOut
    H['bmkhs_engPctTq'][i] = tqOut / refTq
    H['bmkhs_engNp'][i] = npNew
    H['bmkhs_engClutch'][i] = clutch
    H['bmkhs_engPctNp'][i] = npNew * eng['npFly']
    H['bmkhs_engFuelFlow'][i] = fuelCmd * eng['maxFuelFlow']
    H['bmkhs_engOilPsi'][i] = max(ngNew * GT_OIL_PSI_SCALE * H['bmkhs_engOilHealth'][i], 0.0)
    #Rig-only diagnostics, what GTDIAG / GOVDIAG print.
    H['diag'][i] = dict(fuel=fuelCmd, orifice=orifice, starterTq=starterTq, running=running,
                        tripped=tripped, share=share)


def engine_controller(H, cfg):
    """fn_engineUpdate, useSystems path."""
    engState = list(H['bmkhs_engState'])
    for e in (0, 1):
        st = engState[e]
        sw = H['startSw'][e]
        lvr = H['pwrLvr'][e]
        if sw > 0 and st == 'OFF':
            H['bmkhs_engState'][e] = 'STARTING'
        if sw < 0 and st == 'STARTING':
            H['bmkhs_engState'][e] = 'OFF'
        want = 'FLY' if lvr >= 1.0 else 'IDLE' if lvr > 0.0 else 'OFF'
        if want != H['bmkhs_engPowerLeverState'][e]:
            H['bmkhs_engPowerLeverState'][e] = want
            if want == 'OFF' and st == 'ON':
                H['bmkhs_engState'][e] = 'OFF'
    if not H['pneuAvail']:
        for e in (0, 1):
            if engState[e] == 'STARTING':
                H['bmkhs_engState'][e] = 'OFF'
    for e, eng in enumerate(H['bmkhs_engines']):
        turbo_shaft_engine(H, e, eng)
    for e in (0, 1):
        if not H['fuelAvail'][e]:
            H['bmkhs_engState'][e] = 'OFF'


# ---------------------------------------------------------------------------------------------
# fn_transmissionUpdate
# ---------------------------------------------------------------------------------------------

def transmission_update(H):
    rotors = H['bmkhs_simpleRotors']
    jEng = 0.0
    for k, moi in enumerate(H['bmkhs_rtrMoi']):
        gr = rotors[k]['gearRatio']
        if gr > 0.0:
            jEng += moi / (gr * gr)
    outputRpm = H['bmkhs_xmsnOutputRpm']
    totTq = sum(H['bmkhs_reqEngTorque'])
    engInputTq = sum(tq for k, tq in enumerate(H['bmkhs_engOutputTq']) if H['bmkhs_engClutch'][k])
    dt = H['bmkhs_deltaTime']
    alpha = 0.0 if jEng == 0.0 else (engInputTq - totTq) / jEng
    deltaRpm = alpha * dt * (60.0 / (2.0 * math.pi))
    outputRpm = 0.0 if outputRpm < 0.0 else outputRpm + deltaRpm
    H['bmkhs_xmsnOutputRpm'] = outputRpm
    H['bmkhs_xmsnDeltaRpm'] = deltaRpm
    H['xmsnEngInTq'] = engInputTq


# ---------------------------------------------------------------------------------------------
# fn_simpleRotor (torque path), fn_simpleRotorTorque, fn_simpleRotorUpdate
# ---------------------------------------------------------------------------------------------

def simple_rotor(H, idx, rotor):
    dt = H['bmkhs_deltaTime']
    rho = H['bmkhs_rho']
    if rotor['type'] == MAIN:
        collOutput = clamp(H['bmkhs_collectiveOutput'], 0.0, 1.0)
    else:
        collOutput = clamp(H['pedal'], -1.0, 1.0)
    velXY = min(H['hubVelXY'], VEL_VNE)
    velZ = H['hubVelZ']

    rpm = H['bmkhs_xmsnOutputRpm'] / rotor['gearRatio']
    omega = 0.0 if rpm == 0.0 else (2.0 * math.pi) * (rpm / 60.0)
    bladeArea = rotor['bladeRadius'] * rotor['bladeChord']
    bladeRad75 = rotor['bladeRadius'] * 0.75
    bladeVel75 = omega * bladeRad75

    rotorTorque = 0.0
    bladeScalar = rotor['numBlades'] / 4
    dragCoef = math_linear_interp_2d(rotor['dragCoefTable'], collOutput, velXY)
    for _ in range(4):
        bladeDrag = dragCoef * 0.5 * rho * bladeArea * (bladeVel75 * bladeVel75) * bladeScalar
        rotorTorque += bladeDrag * bladeRad75
        if rotor['type'] == MAIN:
            upflow = max(-velZ, 0.0)
            rotorTorque -= rotor['autoTorque'] * upflow * (1.0 - collOutput) * bladeScalar

    simple_rotor_torque(H, idx, rotorTorque, rotor['gearRatio'], rotor['numBlades'],
                        rotor['bladeMass'] if 'bladeMass' in rotor else 0.0,
                        rotor['bladeRadius'], rotor['torqueTau'], dt)


def simple_rotor_torque(H, idx, rotorTorque, gearRatio, numBlades, bladeMass, bladeRadius, torqueTau, dt):
    H['bmkhs_rtrMoi'][idx] = numBlades * ((1.0 / 3.0) * bladeMass * (bladeRadius * bladeRadius))
    req = rotorTorque / gearRatio if gearRatio > 0.0 else 0.0
    smoothed = H['bmkhs_reqEngTorque'][idx]
    alpha = 1.0 - math.exp(-dt / torqueTau)
    H['bmkhs_reqEngTorque'][idx] = smoothed + (req - smoothed) * alpha


def simple_rotor_thrust(H, rotor):
    """fn_simpleRotor's blade lift summed over the four positions - level flight out of ground
    effect (viScalar and gndEffScalar 1.0), no cyclic delta."""
    rho = H['bmkhs_rho']
    collOutput = clamp(H['bmkhs_collectiveOutput'], 0.0, 1.0)
    velXY = min(H['hubVelXY'], VEL_VNE)
    rpm = H['bmkhs_xmsnOutputRpm'] / rotor['gearRatio']
    omega = 0.0 if rpm == 0.0 else (2.0 * math.pi) * (rpm / 60.0)
    bladeVel75 = omega * rotor['bladeRadius'] * 0.75
    velZ = H['hubVelZ']
    viScalarDenom = linear_conversion(-7.62, -19.30, velZ, VEL_VRS, VEL_VRS * 0.1, True)
    viScalar = 0.0 if (velZ < -VEL_VRS and velXY < VEL_ETL) else 1 - (velZ / viScalarDenom)
    liftCoef = math_linear_interp_2d(rotor['liftCoefTable'], collOutput, velXY)
    bladeLift = liftCoef * 0.5 * rho * (rotor['bladeRadius'] * rotor['bladeChord']) * (bladeVel75 * bladeVel75)
    return 4 * bladeLift * (rotor['numBlades'] / 4) * viScalar


def simple_rotor_update(H):
    for idx, rotor in enumerate(H['bmkhs_simpleRotors']):
        simple_rotor(H, idx, rotor)


# ---------------------------------------------------------------------------------------------
# The aircraft, and one frame in fn_coreUpdate's order
# ---------------------------------------------------------------------------------------------

CFG = load_config()


#Engine config overrides every Heli() picks up - how a fit tries values without editing the .hpp.
OVERRIDES = {}


class Heli:
    def __init__(self, overrides=None, baroAltM=0.0):
        H = self.H = {}
        H['baroAltM'] = baroAltM
        environment(H)
        engine_variables(H, CFG, dict(OVERRIDES, **(overrides or {})))
        simple_rotor_variables(H, CFG)
        H['bmkhs_xmsnOutputRpm'] = 0.0
        H['bmkhs_xmsnDeltaRpm'] = 0.0
        H['bmkhs_collectiveOutput'] = 0.0
        H['bmkhs_velModelSpace'] = [0.0, 0.0, 0.0]
        H['hubVelXY'] = H['hubVelZ'] = 0.0
        H['pedal'] = 0.0
        H['startSw'] = [0, 0]
        H['pwrLvr'] = [0.0, 0.0]
        H['pneuAvail'] = True
        H['starterSupplied'] = [True, True]
        H['governorPowered'] = [True, True]
        H['fuelAvail'] = [True, True]
        H['diag'] = [{}, {}]
        self.t = 0.0

    def frame(self, dt, coll=None, pedal=None, velXY=None, velZ=None, baroAltM=None):
        H = self.H
        H['bmkhs_deltaTime'] = dt
        if baroAltM is not None:
            H['baroAltM'] = baroAltM
        environment(H)
        if coll is not None:
            H['bmkhs_collectiveOutput'] = coll
        if pedal is not None:
            H['pedal'] = pedal
        if velXY is not None:
            H['hubVelXY'] = velXY
            H['bmkhs_velModelSpace'][1] = velXY
        if velZ is not None:
            H['hubVelZ'] = velZ
            H['bmkhs_velModelSpace'][2] = velZ
        engine_controller(H, CFG)
        transmission_update(H)
        simple_rotor_update(H)
        self.t += dt

    #Readouts, engine i.
    def ng(self, i=0): return self.H['bmkhs_engPctNg'][i]
    def np(self, i=0): return self.H['bmkhs_engNp'][i]
    def tgt(self, i=0): return self.H['bmkhs_engTgt'][i]
    def tq(self, i=0): return self.H['bmkhs_engPctTq'][i]
    def clutch(self, i=0): return self.H['bmkhs_engClutch'][i]
    def tripped(self, i=0): return self.H['bmkhs_engineOverspeed'][i]

    def nrFrac(self):
        e = self.H['bmkhs_engines'][0]
        return self.H['bmkhs_xmsnOutputRpm'] / (e['npFly'] * e['designRpm'])

    def demand(self):
        """Rotor demand per engine, as a fraction of refTq."""
        return sum(self.H['bmkhs_reqEngTorque']) / 2.0 / self.H['bmkhs_engines'][0]['refTq']


# ---------------------------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------------------------

DT = 1 / 60.0
#Arma's frame runs ~30 ms in the RPTs and drops to 50.
FRAME_DTS = (0.016, 0.032, 0.050)
ARMA_DT = 0.032
JITTER = 0.4
#A normal collective input, 0 -> target over this long. Claude's assumption, not a published rate.
NORMAL_PULL_SEC = 2.0

IDLE, FLY = 0.5, 1.0


def dt_stream(dt, jitter, seed=1):
    rng = random.Random(seed)
    while True:
        yield dt * (1.0 + rng.uniform(-jitter, jitter))


def start_to(a, engines, lever, until, dt=DT, jitter=0.0, **kw):
    """START pressed on each engine at t=0 (it springs back), lever to its detent at the first
    sign of Ng rise, run to `until`."""
    H = a.H
    for i in engines:
        H['startSw'][i] = 1
    dts = dt_stream(dt, jitter)
    first = True
    while a.t < until:
        for i in engines:
            if H['pwrLvr'][i] == 0.0 and a.ng(i) > 0.02:
                H['pwrLvr'][i] = IDLE if lever == 'IDLE' else IDLE
        a.frame(next(dts), **kw)
        if first:
            for i in engines:
                H['startSw'][i] = 0
            first = False


def run_until(a, until, dt=DT, jitter=0.0, each=None, **kw):
    dts = dt_stream(dt, jitter, seed=int(a.t * 1000) + 7)
    while a.t < until:
        a.frame(next(dts), **(each(a) if each else kw))


# ---- the acceptance bench ------------------------------------------------------------------

EXPECTED = dict(light=2.6, cutout=5.5, peakLo=646.0, peakHi=661.0, stop=9.5, below80=6.6,
                motorNg=0.420, hoverTq=0.94, hoverNr=1.01)


def cold_start(tgt0=None, secs=40.0):
    a = Heli()
    if tgt0 is not None:
        a.H['bmkhs_engTgt'][0] = tgt0
    H = a.H
    H['startSw'][0] = 1
    tLight = tCut = None
    peak, tPeak = a.tgt(), 0.0
    trace = []
    wasCranking = False
    while a.t < secs:
        if H['pwrLvr'][0] == 0.0 and a.ng() > 0.02:
            H['pwrLvr'][0] = IDLE
        a.frame(DT)
        H['startSw'][0] = 0
        d = H['diag'][0]
        if tLight is None and d['running']:
            tLight = a.t
        if d['starterTq'] > 0:
            wasCranking = True
        elif wasCranking and tCut is None:
            tCut = a.t
        if a.tgt() > peak:
            peak, tPeak = a.tgt(), a.t
        trace.append((a.t, a.ng(), a.tgt(), a.tq()))
    return dict(light=tLight, cutout=tCut, peak=peak, tPeak=tPeak, ng=a.ng(), tgt=a.tgt(),
                tq=a.tq(), trace=trace, heli=a)


def shutdown(marksAt=(2, 5, 9.5, 30, 300, 1200, 3600)):
    a = cold_start(secs=60.0)['heli']
    t0 = a.t
    a.H['pwrLvr'][0] = 0.0
    marks, stop = {}, None
    for s in marksAt:
        while a.t - t0 < s:
            a.frame(DT)
            if stop is None and a.ng() <= 0.001:
                stop = a.t - t0
        marks[s] = (a.ng(), a.tgt())
    return dict(stop=stop, marks=marks)


def motoring(tgt0=163.0, secs=30.0):
    a = Heli()
    a.H['bmkhs_engTgt'][0] = tgt0
    a.H['startSw'][0] = -1
    m = {}
    while a.t < secs:
        a.frame(DT)
        for lim in (100, 80):
            if lim not in m and a.tgt() < lim:
                m[lim] = a.t
    return dict(below100=m.get(100), below80=m.get(80), ng=a.ng())


def hot_start_abort(tgt0=163.0, abortAt=700.0, ovrDelay=2.0, secs=60.0):
    a = Heli()
    H = a.H
    H['bmkhs_engTgt'][0] = tgt0
    H['startSw'][0] = 1
    peak, tAb, t540 = tgt0, None, None
    first = True
    while a.t < secs:
        if tAb is None and H['pwrLvr'][0] == 0.0 and a.ng() > 0.02:
            H['pwrLvr'][0] = IDLE
        if tAb is None and a.tgt() > abortAt:
            tAb = a.t
            H['pwrLvr'][0] = 0.0
        if tAb is not None and a.t >= tAb + ovrDelay:
            H['startSw'][0] = -1
        a.frame(DT)
        if first:
            H['startSw'][0] = 0
            first = False
        peak = max(peak, a.tgt())
        if tAb is not None and t540 is None and a.tgt() < 540:
            t540 = a.t - tAb
    return dict(abortAt=tAb, peak=peak, t540=t540, tgt=a.tgt())


#What fit_bench scores, and how much of each counts as one unit of error.
FIT_PARAMS = ('starterTorque', 'startFuelBase', 'compDragMult', 'compDragFloor')


def bench_errors():
    """Every bench acceptance number the fit can move, as (name, got, want, unit)."""
    cs = cold_start(secs=30.0)
    sd = shutdown(marksAt=(2, 5, 30))
    mo = motoring()
    peak = cs['peak']
    peakWant = min(max(peak, EXPECTED['peakLo']), EXPECTED['peakHi'])
    return [('light-off s', cs['light'], EXPECTED['light'], 0.1),
            ('cutout s', cs['cutout'] or 99, EXPECTED['cutout'], 0.1),
            ('start peak C', peak, peakWant, 5.0),
            ('stop s', sd['stop'] or 99, EXPECTED['stop'], 0.1),
            ('Ng @2s', sd['marks'][2][0], 0.262, 0.01),
            ('Ng @5s', sd['marks'][5][0], 0.106, 0.01),
            ('TGT @2s', sd['marks'][2][1], 267.0, 5.0),
            ('TGT @5s', sd['marks'][5][1], 191.0, 5.0),
            ('residual @30s', sd['marks'][30][1], 163.0, 5.0),
            ('motor <100 s', mo['below100'] or 99, 5.3, 0.1),
            ('motor <80 s', mo['below80'] or 99, EXPECTED['below80'], 0.1),
            ('motor Ng', mo['ng'], EXPECTED['motorNg'], 0.01)]


def _cost(x):
    OVERRIDES.update(dict(zip(FIT_PARAMS, x)))
    if min(x) <= 0.0:
        return 1e9
    return sum(((got - want) / unit) ** 2 for _, got, want, unit in bench_errors())


def fit_bench(x0, step=0.15, iters=120):
    """Nelder-Mead on FIT_PARAMS against the bench acceptance numbers."""
    n = len(x0)
    simplex = [list(x0)] + [[v * (1 + step) if i == j else v for j, v in enumerate(x0)] for i in range(n)]
    costs = [_cost(p) for p in simplex]
    for _ in range(iters):
        order = sorted(range(n + 1), key=lambda k: costs[k])
        simplex, costs = [simplex[k] for k in order], [costs[k] for k in order]
        cen = [sum(p[j] for p in simplex[:-1]) / n for j in range(n)]
        refl = [cen[j] + (cen[j] - simplex[-1][j]) for j in range(n)]
        cr = _cost(refl)
        if cr < costs[0]:
            exp = [cen[j] + 2 * (cen[j] - simplex[-1][j]) for j in range(n)]
            ce = _cost(exp)
            simplex[-1], costs[-1] = (exp, ce) if ce < cr else (refl, cr)
        elif cr < costs[-2]:
            simplex[-1], costs[-1] = refl, cr
        else:
            con = [cen[j] + 0.5 * (simplex[-1][j] - cen[j]) for j in range(n)]
            cc = _cost(con)
            if cc < costs[-1]:
                simplex[-1], costs[-1] = con, cc
            else:
                for k in range(1, n + 1):
                    simplex[k] = [simplex[0][j] + 0.5 * (simplex[k][j] - simplex[0][j]) for j in range(n)]
                    costs[k] = _cost(simplex[k])
    best = min(range(n + 1), key=lambda k: costs[k])
    OVERRIDES.update(dict(zip(FIT_PARAMS, simplex[best])))
    return dict(zip(FIT_PARAMS, simplex[best])), costs[best]


def equilibrium():
    """The table as the SPEC writes it - algebraic, not flown. loaded_report flies it."""
    e = Heli().H['bmkhs_engines'][0]
    out = []
    for tq, ng, dec in [(0.055, 0.679, 460), (0.18, 0.834, 532), (0.84, 0.930, None),
                        (1.00, 0.951, 810), (1.29, 1.010, 867)]:
        fuel = e['compressorLoad'] * ng ** 2 + tq / e['ptEfficiency'] - ng ** e['massFlowExp'] * e['airCoef']
        out.append((tq, ng, 15 + e['tgtK'] * fuel / ng ** e['massFlowExp'], dec))
    return out


def report():
    a = Heli()
    e = a.H['bmkhs_engines'][0]
    print('=' * 78)
    print('derived: refTq %.1f  idleNg %.4f  fuelIdle %.3f  fuelFly %.3f   rho %.4f  FAT %.0f'
          % (e['refTq'], e['idleNg'], e['fuelIdle'], e['fuelFly'], a.H['bmkhs_rho'], a.H['bmkhs_FAT']))

    print('\n-- equilibrium table, ALGEBRAIC (the spec, not flown) --')
    print('     %TQ     Ng    TGT   declared   err')
    for tq, ng, tgt, dec in equilibrium():
        err = '' if dec is None else '%+d' % round(tgt - dec)
        print('   %5.1f  %.3f   %4.0f  %8s  %4s' % (tq * 100, ng, tgt, dec if dec else '-', err))

    cs = cold_start()
    print('\n-- cold start, engine 1, rotor attached, lever to IDLE at first Ng rise --')
    print('   light-off  %.1fs        (expect %.1f)' % (cs['light'], EXPECTED['light']))
    print('   cutout     %.1fs        (expect %.1f)' % (cs['cutout'], EXPECTED['cutout']))
    print('   peak TGT   %.0fC at %.1fs (expect %.0f-%.0f)'
          % (cs['peak'], cs['tPeak'], EXPECTED['peakLo'], EXPECTED['peakHi']))
    print('   settled    ng %.3f  tgt %.0fC  tq %.1f%%' % (cs['ng'], cs['tgt'], cs['tq'] * 100))
    print('        t     Ng    TGT    %TQ')
    for mark in (2.0, 2.6, 3.0, 4.0, 5.0, 5.5, 7.0, 10.0, 15.0, 20.0, 24.0, 30.0):
        r = min(cs['trace'], key=lambda x: abs(x[0] - mark))
        print('   %6.1f  %.3f   %4.0f  %5.1f' % (r[0], r[1], r[2], r[3] * 100))

    sd = shutdown()
    print('\n-- shutdown, lever OFF from settled idle --')
    print('   spool stops %.1fs      (expect %.1f)' % (sd['stop'] or -1, EXPECTED['stop']))
    for s, (ng, tgt) in sd['marks'].items():
        print('   %7ss  ng %.3f  tgt %.0fC' % (s, ng, tgt))

    mo = motoring()
    print('\n-- motoring from the 163C residual, start sw to ORIDE --')
    print('   <100C %.1fs  <80C %.1fs (expect %.1f)  steady ng %.3f (expect %.3f)'
          % (mo['below100'], mo['below80'], EXPECTED['below80'], mo['ng'], EXPECTED['motorNg']))

    hs = cold_start(163.0)
    ha = hot_start_abort()
    print('\n-- hot start --')
    print('   uncaught peak %.0fC   (over the 851 start limit: %s)'
          % (hs['peak'], 'YES' if hs['peak'] > 851 else 'NO - WRONG'))
    print('   aborted at %.1fs  peak %.0fC  <540 in %.1fs  final %.0fC'
          % (ha['abortAt'], ha['peak'], ha['t540'], ha['tgt']))


# ---- flight: both engines, governed ---------------------------------------------------------

def twin_to_fly(dt=ARMA_DT, jitter=JITTER, flyAt=30.0, until=40.0, **kw):
    """Both engines started together, IDLE, levers to FLY at flyAt."""
    a = Heli()
    start_to(a, (0, 1), 'IDLE', flyAt, dt, jitter, **kw)
    a.H['pwrLvr'] = [FLY, FLY]
    return a


def governed_flight(collTarget, pullSecs=NORMAL_PULL_SEC, dt=ARMA_DT, jitter=JITTER,
                    pullAt=45.0, secs=90.0, velXY=0.0, velZ=0.0):
    a = twin_to_fly(dt, jitter)
    trace = []
    dts = dt_stream(dt, jitter, seed=3)
    while a.t < secs:
        if pullSecs > 0:
            coll = collTarget * clamp((a.t - pullAt) / pullSecs, 0.0, 1.0)
        else:
            coll = collTarget if a.t >= pullAt else 0.0
        a.frame(next(dts), coll=coll, velXY=velXY, velZ=velZ)
        trace.append((a.t, a.ng(), a.np(), a.nrFrac(), a.tq(), a.tgt(), a.clutch(), a.tripped(),
                      a.H['diag'][0]['orifice'], coll, a.demand(), a.H['diag'][0]['fuel']))
    return a, trace


def governed_report():
    print('\n' + '=' * 78)
    print('FLIGHT - both engines, governed, rotor + tail rotor, Arma frame %.0f ms +-%.0f%%'
          % (ARMA_DT * 1000, JITTER * 100))
    print('=' * 78)
    a = Heli()
    start_to(a, (0, 1), 'IDLE', 30.0, ARMA_DT, JITTER)
    print('   twin IDLE at 30 s: Ng %.3f  TGT %.0fC  tq %.1f%%  Np %.1f%%  Nr %.1f%% of governed'
          % (a.ng(), a.tgt(), a.tq() * 100, a.np() * 100, a.nrFrac() * 100))

    a, tr = governed_flight(0.64)
    print('\n   levers FLY at 30 s, collective 0 -> 0.64 over %.0f s at 45 s' % NORMAL_PULL_SEC)
    print('        t     Ng     Np%    Nr%   %TQ  demand   TGT  clutch  orifice')
    for mark in (30.5, 31, 32, 33, 34, 35, 38, 44.5, 45.5, 46, 47, 50, 60, 89.9):
        r = min(tr, key=lambda x: abs(x[0] - mark))
        print('   %6.1f  %.3f  %5.1f  %5.1f  %5.1f  %5.1f  %4.0f   %-5s  %.3f'
              % (r[0], r[1], r[2] * 100, r[3] * 100, r[4] * 100, r[10] * 100, r[5],
                 'lock' if r[6] else 'FREE', r[8]))
    runup = max(r[3] for r in tr if 30.0 <= r[0] < 45.0)
    flat = [r for r in tr if 42.0 <= r[0] < 45.0][-1]
    last = tr[-1]
    e = a.H['bmkhs_engines'][0]
    print('\n   run-up peak Nr %.1f%% of governed' % (runup * 100))
    print('   flat pitch      tq %.1f%%  Ng %.3f  TGT %.0fC   (flight 18.4%% / 0.787-0.800 / 478)'
          % (flat[4] * 100, flat[1], flat[5]))
    print('   FLIGHT ACCEPTANCE, coll 0.64 settled: tq %.1f%% (expect %.0f)  Nr %.1f%% of design (expect %.0f)'
          % (last[4] * 100, EXPECTED['hoverTq'] * 100, last[3] * e['npFly'] * 100, EXPECTED['hoverNr'] * 100))
    print('                                         Ng %.3f  TGT %.0fC   (flight ~94%%: 1.052 / 520)'
          % (last[1], last[5]))

    print('\n   Nr deviation from 45 s     normal pull    panic step')
    for dt in FRAME_DTS:
        dev = []
        for ps in (NORMAL_PULL_SEC, 0.0):
            _, t2 = governed_flight(0.64, pullSecs=ps, dt=dt)
            dev.append(max((r[3] - 1.0 for r in t2 if r[0] >= 45.0), key=abs))
        print('   dt %2.0f ms                   %+6.2f%%       %+6.2f%%' % (dt * 1000, dev[0] * 100, dev[1] * 100))


def collective_for(tqPerEngine):
    """Collective at which main + tail, on governed speed, demand this from each of two engines."""
    lo, hi = 0.0, 1.0
    for _ in range(30):
        mid = (lo + hi) / 2.0
        a = Heli()
        e = a.H['bmkhs_engines'][0]
        a.H['bmkhs_xmsnOutputRpm'] = e['designRpm'] * e['npFly']
        a.H['bmkhs_collectiveOutput'] = mid
        a.H['bmkhs_deltaTime'] = DT
        for _ in range(300):
            simple_rotor_update(a.H)
        if a.demand() < tqPerEngine:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


LOADED_ROWS = [(0.18, 0.834, 532), (0.59, None, None), (0.84, 0.930, None), (0.94, None, None),
               (1.00, 0.951, 810), (1.18, None, None), (1.29, 1.010, 867)]
#What the 15-19-01 flight settled at, for the same torque.
FLIGHT_ROWS = {0.18: (0.787, 478), 0.59: (0.958, 517), 0.94: (1.052, 520), 1.18: (1.08, 570)}


def loaded_report():
    print('\n' + '=' * 78)
    print('LOADED - each torque flown at the collective that demands it (main + tail rotor)')
    print('=' * 78)
    print('   target   coll |  %TQ    Nr%     Ng  spec  flight |  TGT  spec flight | trip')
    for tq, ngSpec, tgtSpec in LOADED_ROWS:
        coll = collective_for(tq)
        a, tr = governed_flight(coll, secs=100.0)
        last = tr[-1]
        fl = FLIGHT_ROWS.get(tq)
        e = a.H['bmkhs_engines'][0]
        print('   %5.0f%%  %.3f | %5.1f  %5.1f  %.3f %5s  %5s | %4.0f  %4s  %4s | %s'
              % (tq * 100, coll, last[4] * 100, last[3] * e['npFly'] * 100, last[1],
                 '%.3f' % ngSpec if ngSpec else '-', '%.3f' % fl[0] if fl else '-', last[5],
                 '%d' % tgtSpec if tgtSpec else '-', '%d' % fl[1] if fl else '-',
                 'YES' if any(r[7] for r in tr) else 'no'))


#Rough fuel burn, both engines, lb/h (user, 2026-09-27): flat pitch on 101 %, and 72 % torque.
FUEL_BURN_LBH = [(0.0, None, 555.0), (None, 0.72, 1080.0)]
LBH_TO_KGS = 0.45359237 / 3600.0


def fuel_flow_report():
    """The engine's dimensionless fuel against the user's fuel burn, per engine - what
    maxFuelFlow has to convert between."""
    print('\n' + '=' * 78)
    print('FUEL FLOW - dimensionless fuel burned vs the stated burn (per engine)')
    print('=' * 78)
    e = Heli().H['bmkhs_engines'][0]
    print('     %TQ    fuel    want kg/s   kg/s now   want/fuel')
    for coll, tq, lbh in FUEL_BURN_LBH:
        last = governed_flight(coll if coll is not None else collective_for(tq), secs=100.0)[1][-1]
        want = lbh / 2.0 * LBH_TO_KGS
        print('   %5.1f   %.3f    %.4f      %.4f      %.4f'
              % (last[4] * 100, last[11], want, last[11] * e['maxFuelFlow'], want / last[11]))


def load_share_run(coll, idleAt=60.0, flyAt=85.0, secs=160.0, dt=ARMA_DT, jitter=JITTER):
    """Both engines governed at FLY; engine 2's lever to IDLE and back, as flown 19-52-59."""
    a = twin_to_fly(dt, jitter)
    dts = dt_stream(dt, jitter, seed=9)
    rows = []
    while a.t < secs:
        if idleAt <= a.t < flyAt:
            a.H['pwrLvr'][1] = IDLE
        elif a.t >= flyAt:
            a.H['pwrLvr'][1] = FLY
        a.frame(next(dts), coll=coll * clamp((a.t - 45.0) / NORMAL_PULL_SEC, 0.0, 1.0))
        rows.append((a.t, a.tq(0), a.tq(1), a.nrFrac(), a.clutch(1)))
    return rows


def load_share_report(coll=None):
    coll = collective_for(0.72) if coll is None else coll
    print('\n' + '=' * 78)
    print('LOAD SHARING - both at FLY, engine 2 to IDLE at 60 s and back to FLY at 85 s')
    print('=' * 78)
    rows = load_share_run(coll)
    print('        t   eng1 %TQ  eng2 %TQ   Nr%  clutch2')
    for mark in (59, 70, 84, 90, 95, 100, 110, 120, 140, 159.9):
        r = min(rows, key=lambda x: abs(x[0] - mark))
        print('   %6.1f    %5.1f     %5.1f   %5.1f  %s' % (r[0], r[1] * 100, r[2] * 100, r[3] * 100, 'lock' if r[4] else 'FREE'))
    even = next((r[0] - 85.0 for r in rows if r[0] > 90 and r[4] and abs(r[1] - r[2]) < 0.01), None)
    print('   within 1%% of each other %s after the lever is selected to FLY (lever travel %.1f s)'
          % ('%.1f s' % even if even is not None else 'NEVER', Heli().H['bmkhs_engines'][1]['leverTravelTime']))


def autorotation_report(velXY=36.0, velZ=-10.0):
    print('\n' + '=' * 78)
    print('POWER-ON AUTOROTATION - governed at 0.40 collective, %.0f m/s; at 60 s collective'
          ' to 0 and %.0f m/s descent' % (velXY, -velZ))
    print('   expected (user): torque ~0-1%, Np ~0.94-0.96, Ng ~idle, TGT not rising')
    print('=' * 78)
    a = twin_to_fly()
    dts = dt_stream(ARMA_DT, JITTER, seed=5)
    rows = []
    while a.t < 110.0:
        if a.t < 60.0:
            kw = dict(coll=0.40 * clamp((a.t - 45.0) / 2.0, 0.0, 1.0), velXY=velXY, velZ=0.0)
        else:
            k = clamp((a.t - 60.0) / 2.0, 0.0, 1.0)
            kw = dict(coll=0.40 * (1.0 - k), velXY=velXY, velZ=velZ * k)
        a.frame(next(dts), **kw)
        rows.append((a.t, a.ng(), a.np(), a.nrFrac(), a.tq(), a.tgt(), a.clutch(), a.demand(),
                     a.H['diag'][0]['orifice']))
    print('        t     Ng     Np%    Nr%   %TQ  demand   TGT  clutch  orifice')
    for mark in (59, 61, 62, 63, 65, 70, 80, 90, 109.9):
        r = min(rows, key=lambda x: abs(x[0] - mark))
        print('   %6.1f  %.3f  %5.1f  %5.1f  %5.1f  %6.1f  %4.0f   %-5s  %.3f'
              % (r[0], r[1], r[2] * 100, r[3] * 100, r[4] * 100, r[7] * 100, r[5],
                 'lock' if r[6] else 'FREE', r[8]))


def shutdown_from_flight_report():
    print('\n' + '=' * 78)
    print('SHUTDOWN FROM GOVERNED FLIGHT - both levers OFF at 60 s, flat pitch')
    print('   Flight 1 (approved): Np stopped in ~2.5 s, leading Ng (~9.6 s)')
    print('=' * 78)
    a = twin_to_fly()
    run_until(a, 60.0, ARMA_DT, JITTER)
    a.H['pwrLvr'] = [0.0, 0.0]
    t0 = a.t
    rows = []
    while a.t < t0 + 30.0:
        a.frame(ARMA_DT)
        rows.append((a.t - t0, a.ng(), a.np(), a.nrFrac(), a.clutch()))
    print('      +t     Ng     Np%   Nr%   clutch')
    for mark in (0.5, 1, 2, 3, 5, 8, 10, 15, 29.9):
        r = min(rows, key=lambda x: abs(x[0] - mark))
        print('   %5.1f  %.3f  %5.1f  %5.1f  %s' % (r[0], r[1], r[2] * 100, r[3] * 100, 'lock' if r[4] else 'FREE'))
    npStop = next((r[0] for r in rows if r[2] < 0.01), None)
    ngStop = next((r[0] for r in rows if r[1] < 0.001), None)
    print('   Np < 1%% at %s   Ng stopped at %s'
          % ('%.1fs' % npStop if npStop else 'never', '%.1fs' % ngStop if ngStop else 'never'))


def engine_state_run():
    """engState and the starter through a start to IDLE, 60 s at idle, lever OFF, 60 s after;
    then START again. Returns when each thing happened."""
    a = Heli()
    H = a.H
    H['startSw'][0] = 1
    ev = {'STARTING': 0.0}
    prev = H['bmkhs_engState'][0]
    crankAfterOff = 0.0
    first, phase = True, 'start'
    while a.t < 200.0:
        if phase == 'start' and H['pwrLvr'][0] == 0.0 and a.ng() > 0.02:
            H['pwrLvr'][0] = IDLE
        if phase == 'start' and a.t >= 70.0:
            H['pwrLvr'][0], phase, tOff = 0.0, 'off', a.t
        if phase == 'off' and a.t >= 130.0:
            H['startSw'][0], phase, tRe = 1, 'restart', a.t
            first = True
        if phase == 'restart' and H['pwrLvr'][0] == 0.0 and a.ng() > 0.02:
            H['pwrLvr'][0] = IDLE
        a.frame(DT)
        if first:
            H['startSw'][0] = 0
            first = False
        st = H['bmkhs_engState'][0]
        if st != prev:
            ev['%s (%s)' % (st, phase)] = round(a.t, 2)
            prev = st
        d = H['diag'][0]
        if phase == 'start' and 'cutout' not in ev and ev.get('crank') and d['starterTq'] == 0:
            ev['cutout'] = round(a.t, 2)
        if d['starterTq'] > 0 and 'crank' not in ev:
            ev['crank'] = round(a.t, 2)
        if phase == 'off' and d['starterTq'] > 0:
            crankAfterOff += DT
        if phase == 'restart' and d['running'] and 'restart light-off' not in ev:
            ev['restart light-off'] = round(a.t - tRe, 2)
    ev['starter cranking after lever OFF, s'] = round(crankAfterOff, 2)
    ev['final'] = '%s  ng %.3f  tgt %.0f' % (H['bmkhs_engState'][0], a.ng(), a.tgt())
    return ev


#The rotor tables' airspeed columns, m/s, and the collective steps flown at each.
ENVELOPE_SPEEDS = (0.00, 10.29, 20.58, 36.01, 51.44, 61.73, 72.02)
ENVELOPE_COLLS = tuple(round(c * 0.1, 1) for c in range(11))


#The design gross weight (user, 2026-09-27): everything is built around it.
GROSS_WEIGHT_LB = 18000.0
LB_TO_N = 4.4482216


#Level-flight pitch attitude (user, 2026-09-27): 0 deg at the hover, -5 deg at 90 kt. Between
#and beyond, fn_mathLinearInterp's own rule - linear, held at the ends.
PITCH_BY_KT = [[0.0, 0.0], [90.0, -5.0]]


def level_flight(v):
    """Hub-axis airflow and thrust tilt for level flight at v m/s, wings level. Nose-down pitch
    puts +v*sin on the disc's up axis."""
    pitch = math.radians(math_linear_interp(PITCH_BY_KT, v * 1.94384)[1])
    return v * math.cos(pitch), -v * math.sin(pitch), pitch


def trim_collective(velXY, velZ=0.0, pitch=0.0, weightLb=GROSS_WEIGHT_LB):
    """Collective at which main rotor thrust, on governed speed, carries the weight."""
    a = Heli()
    e = a.H['bmkhs_engines'][0]
    a.H['bmkhs_xmsnOutputRpm'] = e['designRpm'] * e['npFly']
    a.H['hubVelXY'] = velXY
    a.H['hubVelZ'] = velZ
    main = a.H['bmkhs_simpleRotors'][0]
    need = weightLb * LB_TO_N / math.cos(pitch)
    lo, hi = 0.0, 1.0
    for _ in range(40):
        a.H['bmkhs_collectiveOutput'] = (lo + hi) / 2.0
        if simple_rotor_thrust(a.H, main) < need:
            lo = a.H['bmkhs_collectiveOutput']
        else:
            hi = a.H['bmkhs_collectiveOutput']
    return (lo + hi) / 2.0


def gross_weight_report():
    print('\n' + '=' * 78)
    print('%.0f lb - level flight at the collective that carries it, sea level ISA' % GROSS_WEIGHT_LB)
    print('   both engines governed; wings level, pitch %s by kt; out of ground effect, no fuselage'
          ' drag' % PITCH_BY_KT)
    print('=' * 78)
    print('     kt  pitch    coll   %TQ    Nr%     Ng     TGT   trip')
    for v in ENVELOPE_SPEEDS:
        vXY, vZ, pitch = level_flight(v)
        c = trim_collective(vXY, vZ, pitch)
        a, tr = governed_flight(c, secs=100.0, velXY=vXY, velZ=vZ)
        last = tr[-1]
        e = a.H['bmkhs_engines'][0]
        print('   %4.0f  %5.1f   %.3f  %5.1f  %5.1f  %.3f   %4.0f   %s'
              % (v * 1.94384, math.degrees(pitch), c, last[4] * 100, last[3] * e['npFly'] * 100,
                 last[1], last[5], 'YES' if any(r[7] for r in tr) else 'no'))


def governor_sweep(gains, ffwds=(1.0,)):
    """Governor gains scored on what the rotor must do: settled Nr at every 18,000 lb speed and
    every loaded row, the normal-pull and panic deviation at each frame rate, the run-up."""
    points = [trim_collective(*level_flight(v)[:2], level_flight(v)[2]) for v in ENVELOPE_SPEEDS]
    vels = [level_flight(v) for v in ENVELOPE_SPEEDS]
    rows = [collective_for(tq) for tq, _, _ in LOADED_ROWS]
    print('\n' + '=' * 78)
    print('GOVERNOR SWEEP - worst settled Nr error, worst pull/panic deviation, run-up peak')
    print('=' * 78)
    print('     kp     ki   clamp  ffwd | settled GW  settled rows | pull   panic | run-up')
    out = []
    for kp, ki, clampI in gains:
        for ff in ffwds:
            overrides = dict(pid=[kp, ki, 0.0, clampI], ffwdGain=ff)

            def fly(coll, **kw):
                return governed_flight_with(overrides, coll, **kw)

            gw = max(abs(fly(c, velXY=vx, velZ=vz)[-1][3] - 1.0)
                     for c, (vx, vz, _) in zip(points, vels))
            rw = max(abs(fly(c)[-1][3] - 1.0) for c in rows)
            pull = max(abs(max((r[3] - 1.0 for r in fly(0.64, dt=d) if r[0] >= 45.0), key=abs))
                       for d in FRAME_DTS)
            panic = max(abs(max((r[3] - 1.0 for r in fly(0.64, dt=d, pullSecs=0.0) if r[0] >= 45.0), key=abs))
                        for d in FRAME_DTS)
            runup = max(r[3] for r in fly(0.0) if 30.0 <= r[0] < 45.0) - 1.0
            out.append((max(gw, rw), kp, ki, clampI, ff, gw, rw, pull, panic, runup))
            print('   %5.1f  %5.1f  %.3f  %.2f |   %5.2f%%      %5.2f%%    | %4.2f%%  %4.2f%% | %+5.2f%%'
                  % (kp, ki, clampI, ff, gw * 100, rw * 100, pull * 100, panic * 100, runup * 100))
    return out


def governed_flight_with(overrides, collTarget, pullSecs=NORMAL_PULL_SEC, dt=ARMA_DT,
                         jitter=JITTER, pullAt=45.0, secs=100.0, velXY=0.0, velZ=0.0):
    """governed_flight on an aircraft whose engines carry these config overrides."""
    a = Heli(overrides)
    start_to(a, (0, 1), 'IDLE', 30.0, dt, jitter)
    a.H['pwrLvr'] = [FLY, FLY]
    trace = []
    dts = dt_stream(dt, jitter, seed=3)
    while a.t < secs:
        k = clamp((a.t - pullAt) / pullSecs, 0.0, 1.0) if pullSecs > 0 else float(a.t >= pullAt)
        a.frame(next(dts), coll=collTarget * k, velXY=velXY, velZ=velZ)
        trace.append((a.t, a.ng(), a.np(), a.nrFrac(), a.tq(), a.tgt(), a.clutch(), a.tripped()))
    return trace


def envelope_report():
    print('\n' + '=' * 78)
    print('ENVELOPE - both engines governed, level flight, sea level ISA, settled at 100 s')
    print('   each cell: collective set over %.0f s at 45 s. TRIP = an Ng or Np hard shutdown'
          % NORMAL_PULL_SEC)
    print('=' * 78)
    cells = {}
    for v in ENVELOPE_SPEEDS:
        for c in ENVELOPE_COLLS:
            a, tr = governed_flight(c, secs=100.0, velXY=v)
            last = tr[-1]
            e = a.H['bmkhs_engines'][0]
            cells[(v, c)] = (last[4], last[3] * e['npFly'], last[1], last[5], any(r[7] for r in tr))

    def table(title, fmt, pick):
        print('\n   %s' % title)
        print('    kt \\ coll ' + ''.join('%7.1f' % c for c in ENVELOPE_COLLS))
        for v in ENVELOPE_SPEEDS:
            row = []
            for c in ENVELOPE_COLLS:
                cell = cells[(v, c)]
                row.append('   TRIP' if cell[4] else fmt % pick(cell))
            print('   %5.0f      ' % (v * 1.94384) + ''.join(row))

    table('torque per engine, %', '%7.1f', lambda x: x[0] * 100)
    table('Nr, % of design', '%7.1f', lambda x: x[1] * 100)
    table('Ng', '%7.3f', lambda x: x[2])
    table('TGT, C', '%7.0f', lambda x: x[3])


if __name__ == '__main__':
    report()
    governed_report()
    loaded_report()
    autorotation_report()
    shutdown_from_flight_report()
    load_share_report()
