"""The HeliSim engine, drivetrain and rotors - simple and blade element - outside Arma. A 1:1
port of Core.

Run it:  python python/dev/engine.py          simple rotors
         python python/dev/engine.py bet      blade element rotors

Every function below is named after the SQF function it ports and carries its inputs, outputs
and expressions line for line. The frame runs in fn_coreUpdate's order: environment ->
engineUpdate -> transmissionUpdate -> coreUpdateFlightModel, which runs the rotor model that
bmkhs_rotorModel selects, as Core's setting does. Values are read from the AH-64D's own config
files, so there is no third copy of any number.

Not ported, because nothing on the engine's path reads it: systems circuits (gates are
scenario inputs), damage, torque jitter and the simple rotor's lift/force path. The blade
element rotor's torque comes out of its force path, so that is ported whole; what it hands to
addForce is summed per rotor in rotorForceN / rotorMomentNm for reading.

If this file and the SQF ever disagree, the SQF is right and this file is the bug.
"""
import copy
import math
import random
import re

import os
AH64_CONFIG = os.environ.get('BMKHS_CONFIG', r'E:\AH-64D\addons\fza_ah64_helisim\config\bmkhs_config')

#core.hpp
ISA_STD_DAY_AIR_DENSITY = 1.225
VEL_VNE = 128.611
VEL_VRS = 24.384
VEL_ETL = 12.347
METERS_TO_FEET = 3.28084
FEET_TO_METERS = 0.3048
GRAVITY = 9.806
DEG_C_TO_KELVIN = 273.15
ISA_PRESSURE_HPA = 1013.25
ISA_TEMP_K = 288.15
ISA_LAPSE = 0.0065
ISA_EXPONENT = 5.25588
DRY_ADIABATIC_LAPSE = 0.0098
GAS_CONSTANT_DRY_AIR = 287.05
VAPOUR_DENSITY_DEFICIT = 0.378
VAPOUR_MASS_RATIO = 0.622
LATENT_HEAT_VAPOUR = 2.5e6
SPECIFIC_HEAT_AIR = 1004.0
ENV_WEATHER_PERIOD = 1.0
ENV_CLOUD_DAY = 0.7
ENV_CLOUD_NIGHT = 0.3
ENV_NIGHT_LAYER_A = 0.15
ENV_NIGHT_WIND_MIN = 1.0
ENV_FOG_TOP = 91.44
ENV_RAIN_LIGHT = 2.5
ENV_RAIN_MODERATE = 7.6
ENV_RAIN_TOP = 50.0
ENV_RAIN_RECOVERY = 1800.0
ENV_AGL_TIME = 3.0
ENV_WIND_TIME = 5.0
GUST_HEIGHT_MIN = 3.048
GUST_LOW_TOP = 304.8
GUST_HIGH_BASE = 609.6
GUST_HIGH_SIZE = 533.4
GUST_WIND_STEP = 7.717
GUST_AIRSPEED_MIN = 1.0
#engine.hpp
GT_OIL_PSI_SCALE = 0.90
GT_SINGLE_ENG_TQ_RATIO = 0.51
GT_TGT_LIMIT_BAND = 40.0
GT_NG_LIMIT_BAND = 0.030
GT_LIMIT_GAIN = 16.0
GT_LIMIT_TRACK = 1.02
GT_NG_LEAD = 0.4       # s - the Ng limiter acts on Ng this far ahead at its present rate (engine.hpp)
GT_NG_RATE_TAU = 0.1   # s - filter on that rate
GT_NG_TORQUE_FLOOR = 0.05   # Ng below which power over speed is held, so a stopped spool divides by no zero
GT_GAMMA_COLD = 1.40
GT_CP_COLD = 1.005
GT_GAMMA_HOT = 1.33
GT_CP_HOT = 1.148
GT_R_AIR = 0.28705
GT_STD_TEMP_K = 288.15
GT_STD_PRESSURE_KPA = 101.325
GT_TGT_HEAT_RATE = 0.30
GT_TGT_COOL_RATE = 0.70
GT_TGT_STILL_AIR = 0.0012
GT_TGT_RAM_AIR = 0.00065
GT_PT_STALL_TQ = 2.5
GT_SPOOL_UNIT_LOAD = 2.91244
#rotor.hpp
MAIN, TAIL, PROPELLER, PROPROTOR = 0, 1, 2, 3
#The CBA setting bmkhs_rotorModel (fn_eventPreInit): 0 Simple, 1 BET. fn_coreUpdateFlightModel reads it.
bmkhs_rotorModel = 0


def rotor_type(name):
    """fn_simpleRotorVariables - the type by name, main when not one of them."""
    names = ['main', 'tail', 'propeller', 'proprotor']
    return names.index(name.lower()) if name.lower() in names else MAIN


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
    for f in ('helisim_engine.hpp', 'helisim_simpleRotor.hpp', 'helisim_rotor.hpp', 'helisim_airfoils.hpp'):
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


def util_smooth(previous, new, deltaTime, time):
    """fn_utilSmooth."""
    share = 1 - math.exp(-deltaTime / time)
    if isinstance(new, (list, tuple)):
        return [p + (n - p) * share for p, n in zip(previous, new)]
    return previous + (new - previous) * share


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
    return {'kp': kp, 'ki': ki, 'kd': kd, 'ki_clamp': kiClamp, 'integral': 0.0}


def pid_run(pid, dt, desired, actual):
    """fn_pidRun."""
    error = desired - actual
    integral = clamp(pid['integral'] + error * dt, -pid['ki_clamp'], pid['ki_clamp'])
    raw = 0.0 if dt == 0 else (error - pid.get('prevError', error)) / dt
    dCoef = pid.get('dCoef', 0.3)
    prev = pid.get('derivFilt', raw)
    derivative = prev + dCoef * (raw - prev)
    out = pid['kp'] * error + pid['ki'] * integral + pid['kd'] * derivative
    pid['prevError'], pid['integral'], pid['derivFilt'] = error, integral, derivative
    return out


def pid_reset(pid):
    """fn_pidReset."""
    pid.pop('prevError', None)
    pid['integral'], pid['derivFilt'] = 0.0, 0.0


#Arma's vector commands and trigonometry - degrees, as SQF.
def sqf_sin(d): return math.sin(math.radians(d))
def sqf_cos(d): return math.cos(math.radians(d))
def sqf_acos(x): return math.degrees(math.acos(x))
def vector_add(a, b): return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
def vector_diff(a, b): return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
def vector_multiply(a, k): return [a[0] * k, a[1] * k, a[2] * k]
def vector_dot_product(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def vector_magnitude(a): return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def vector_cross_product(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def vector_normalized(a):
    m = vector_magnitude(a)
    return [0.0, 0.0, 0.0] if m == 0.0 else [a[0] / m, a[1] / m, a[2] / m]


def math_vector_rotate(v, p, r, y):
    """fn_mathVectorRotate."""
    sinP, sinY, sinR = sqf_sin(p), sqf_sin(y), sqf_sin(r)
    cosP, cosY, cosR = sqf_cos(p), sqf_cos(y), sqf_cos(r)
    m = [[cosR * cosY + sinR * sinP * sinY, -cosR * sinY + sinR * sinP * cosY, sinR * cosP],
         [cosP * sinY, cosP * cosY, -sinP],
         [-sinR * cosY + cosR * sinP * sinY, sinR * sinY + cosR * sinP * cosY, cosR * cosP]]
    return [vector_dot_product(row, v) for row in m]


def math_vector_rotate_around_axis(v, axis, angle):
    """fn_mathVectorRotateAroundAxis."""
    s, c = sqf_sin(angle), sqf_cos(angle)
    dot = vector_dot_product(v, axis)
    cross = vector_cross_product(v, axis)
    return [v[i] * c + cross[i] * s + axis[i] * dot * (1 - c) for i in range(3)]


def math_get_area(A, B, C, D):
    """fn_mathGetArea."""
    AB = vector_magnitude(vector_diff(B, A))
    BC = vector_magnitude(vector_diff(B, C))
    CD = vector_magnitude(vector_diff(C, D))
    DA = vector_magnitude(vector_diff(A, D))
    s = (AB + BC + CD + DA) * 0.5
    return math.sqrt((s - AB) * (s - BC) * (s - CD) * (s - DA))


def math_linear_interp_from_center(inMin, inMax, x, outMin, outMid, outMax):
    """fn_mathLinearInterpFromCenter."""
    if x < 0:
        return linear_conversion(inMin, 0, x, outMin, outMid, True)
    return linear_conversion(0, inMax, x, outMid, outMax, True)


# ---------------------------------------------------------------------------------------------
# Init - fn_environmentVariables, fn_engineVariables, fn_simpleRotorVariables
# ---------------------------------------------------------------------------------------------

NUM_FIELDS = ['designRpm', 'npFly', 'maxFuelFlow', 'powerKw', 'maxNg']
SECTION_FIELDS = [
    ('Compressor', ['pressureRatio', 'massFlow', 'inletDiameter', 'ramRecovery', 'compDrag', 'compDragFloor',
                    'lightOffNg', 'selfSustNg', 'idleNg', 'ngLimitMax', 'ngLimitBase', 'ngLimitSlope']),
    ('Combustor', ['fuelLhv', 'combustorEfficiency', 'maxTgt', 'maxTgtSe', 'startTgt', 'startMinTgt',
                   'residualHeatGain']),
    ('CompressorTurbine', ['turbineEfficiency', 'spoolInertia']),
    ('PowerTurbine', ['ptEfficiency', 'ptInertia', 'ptDrag', 'ptDragFloor']),
    ('Governor', ['fuelIdle', 'fuelFly', 'startFuelBase', 'ffwdGain', 'leverTravelTime', 'loadShareGain']),
]
ROTOR_NUM_FIELDS = ['numBlades', 'mastLength', 'gearRatio', 'torqueTau', 'bladeRadius',
                    'bladeChord', 'bladeMass', 'reacTqScalar', 'autoTorque']


#fn_engineVariables' compressor map, the T700-701C - the default for any engine that declares no
#compressorMap[] of its own. Ng / maxNg (corrected), PR / design, flow / design, efficiency,
#compressor turbine expansion as ln(expansion) / ln(design PR).
COMPRESSOR_MAP = [
    [0.0000, 0.0588, 0.0000, 0.544, 0.0000],
    [0.0500, 0.0595, 0.0116, 0.544, 0.0037],
    [0.1000, 0.0618, 0.0329, 0.544, 0.0158],
    [0.1500, 0.0656, 0.0604, 0.544, 0.0348],
    [0.2000, 0.0712, 0.0930, 0.544, 0.0610],
    [0.2500, 0.0789, 0.1300, 0.544, 0.0938],
    [0.3000, 0.0891, 0.1709, 0.544, 0.1326],
    [0.3500, 0.1024, 0.2153, 0.544, 0.1770],
    [0.4000, 0.1194, 0.2631, 0.544, 0.2261],
    [0.4500, 0.1410, 0.3139, 0.544, 0.2792],
    [0.5000, 0.1683, 0.3676, 0.544, 0.3357],
    [0.5500, 0.2026, 0.4241, 0.544, 0.3949],
    [0.6000, 0.2456, 0.4833, 0.544, 0.4564],
    [0.6173, 0.2629, 0.5043, 0.544, 0.4781],
    [0.6768, 0.3329, 0.5891, 0.595, 0.4779],
    [0.7546, 0.4559, 0.7130, 0.642, 0.4882],
    [0.8125, 0.5971, 0.8152, 0.657, 0.5148],
    [0.8369, 0.6835, 0.8609, 0.654, 0.5372],
    [0.8592, 0.7712, 0.9043, 0.656, 0.5534],
    [0.8712, 0.8288, 0.9283, 0.652, 0.5670],
    [0.8772, 0.8571, 0.9413, 0.650, 0.5738],
    [0.9167, 1.0306, 1.0196, 0.653, 0.5938],
    [0.9500, 1.1766, 1.0855, 0.656, 0.6040],
    [1.0000, 1.3959, 1.1845, 0.659, 0.6112],
]


def num_engines(cfg):
    """The engines the pack declares - Engine01, Engine02, ... in order, as Core reads numEngines."""
    n = 0
    while 'Engine%02d' % (n + 1) in cfg['Engines']:
        n += 1
    return n


def engine_variables(H, cfg, overrides=None):
    engines = []
    for i in range(1, num_engines(cfg) + 1):
        e = cfg['Engines']['Engine%02d' % i]
        eng = {k: e[k] for k in NUM_FIELDS}
        eng['name'] = e['name']
        for section, fields in SECTION_FIELDS:
            for f in fields:
                eng[f] = e[section][f]
        eng['pid'] = e['Governor']['pid']
        eng['starterTorque'] = e['Starter']['torque']
        eng['motoringNg'] = e['Starter']['motoringNg']
        eng['starterGates'] = e['Starter']['gate']
        eng['governorGates'] = e['Governor']['gate']
        eng['refTq'] = (eng['powerKw'] * 1000) / (eng['designRpm'] * eng['npFly'] * 0.10472)
        eng['ngOvrspdProt'] = e.get('ngOvrspdProt', eng['maxNg'])
        eng['maxNp'] = e.get('maxNp', 0.0)
        eng['npOvrspdProt'] = e.get('npOvrspdProt', eng['maxNp'])
        eng['compressorMap'] = e['Compressor'].get('compressorMap') or COMPRESSOR_MAP
        eng['airflowTable'] = e['Compressor']['airflowTable']
        eng.update(overrides or {})
        _, prFrac, flowFrac, eff, _ = math_linear_interp(eng['compressorMap'], 1.0 / eng['maxNg'])
        pr = eng['pressureRatio'] * prFrac
        t3 = GT_STD_TEMP_K * (1 + ((pr ** ((GT_GAMMA_COLD - 1) / GT_GAMMA_COLD)) - 1) / eff)
        eng['spoolUnitKw'] = eng['massFlow'] * flowFrac * GT_CP_COLD * (t3 - GT_STD_TEMP_K) / GT_SPOOL_UNIT_LOAD
        #fn_engineVariables: the starter's power limit, solved so that power / Ng balances the unfired
        #spool's torque - the compressor net of its turbine - at motoringNg on a standard day.
        xh = (GT_GAMMA_HOT - 1) / GT_GAMMA_HOT
        mN = max(eng['motoringNg'], GT_NG_TORQUE_FLOOR)
        _, _, mDotM, t3M, _, compPowerM, ctExpansionM, _, _ = gas_turbine_compressor(eng, mN, 15, GT_STD_PRESSURE_KPA, 0)
        ctPowerM = max(mDotM, 0.001) * GT_CP_HOT * eng['turbineEfficiency'] * t3M * (1 - ctExpansionM ** (-xh))
        eng['starterPower'] = (compPowerM - ctPowerM) / eng['spoolUnitKw']
        engines.append(eng)
    H['bmkhs_engines'] = engines
    n = len(engines)
    H['bmkhs_engPowerLeverState'] = ['OFF'] * n
    H['bmkhs_engState'] = ['OFF'] * n
    H['bmkhs_pid_engine'] = [pid_create(*eng['pid']) for eng in engines]
    z = [0.0] * n
    for k in ('bmkhs_engPctNg', 'bmkhs_engNp', 'bmkhs_engPctNp', 'bmkhs_engPctTq',
              'bmkhs_engOilPsi', 'bmkhs_engFuelFlow', 'bmkhs_engOutputTq', 'bmkhs_engLeverSched'):
        H[k] = list(z)
    H['bmkhs_engClutch'] = [False] * n
    H['bmkhs_engineOverspeed'] = [False] * n
    #The rig models no damage, so these stay healthy - kept so the lines reading them match Core.
    H['bmkhs_engOilHealth'] = [1.0] * n
    H['bmkhs_engFailed'] = [False] * n
    H['bmkhs_engLimFuel'] = [e['fuelFly'] for e in engines]
    H['bmkhs_engMinFuel'] = [0.0 for e in engines]
    H['bmkhs_engClutchSlip'] = [1.0] * n
    H['bmkhs_engTgt'] = [H['bmkhs_fat']] * n
    H['bmkhs_engResidualHeat'] = [1.0] * n
    H['bmkhs_engNpRef'] = [-1.0] * n
    H['bmkhs_engNgPrev'] = [0.0] * n
    H['bmkhs_engNgRate'] = [0.0] * n
    H['bmkhs_engPrevLever'] = ['OFF'] * n


def simple_rotor_variables(H, cfg):
    rotors = []
    for i in range(1, int(cfg['numSimpleRotors']) + 1):
        r = cfg['SimpleRotors']['SimpleRotor%02d' % i]
        rot = {k: r[k] for k in ROTOR_NUM_FIELDS}
        rot['type'] = rotor_type(r['type'])
        rot['dir'] = CW if r['direction'].lower() == 'cw' else CCW
        rot['dragCoefTable'] = math_build_interp_grid(r['dragCoefTable'])
        rot['liftCoefTable'] = math_build_interp_grid(r['liftCoefTable'])
        rot['controlMap'] = r.get('controlMap', [])
        rotors.append(rot)
    H['bmkhs_simpleRotors'] = rotors
    H['bmkhs_drivetrainDrag'] = float(CFG.get('drivetrainDrag', 0.0))
    H['bmkhs_reqEngTorque'] = [0.0] * len(rotors)
    H['bmkhs_rtrThrust'] = [0.0] * len(rotors)
    H['bmkhs_rtrMoi'] = [0.0] * len(rotors)


#fn_rotorVariables' config reads - the aircraft variable and the config array it mirrors.
ROTOR_CONFIG = [('bmkhs_rotorType', 'rotorType'), ('bmkhs_rotorDirection', 'rotorDirection'),
                ('bmkhs_rotorNumBlades', 'rotorNumBlades'), ('bmkhs_rotorNumElements', 'rotorNumElements'),
                ('bmkhs_rotorMastLength', 'rotorMastLength'), ('bmkhs_rotorGearRatioArr', 'rotorGearRatio'),
                ('bmkhs_rotorPivot', 'rotorPivot'), ('bmkhs_rotorRotation', 'rotorRotation'),
                ('bmkhs_rotorFlapTimeConst', 'rotorFlapTimeConst'), ('bmkhs_rotorAirfoil', 'rotorAirfoil'),
                ('bmkhs_rotorBladeCutout', 'rotorBladeCutout'), ('bmkhs_rotorBladeLength', 'rotorBladeLength'),
                ('bmkhs_rotorBladeChordArr', 'rotorBladeChord'), ('bmkhs_rotorBladeTwist', 'rotorBladeTwist'),
                ('bmkhs_rotorBladeMassArr', 'rotorBladeMass'), ('bmkhs_rotorDelta3', 'rotorDelta3'),
                ('bmkhs_rotorPitchMin', 'rotorPitchMin'), ('bmkhs_rotorPitchMid', 'rotorPitchMid'),
                ('bmkhs_rotorPitchMax', 'rotorPitchMax'), ('bmkhs_rotorRollMin', 'rotorRollMin'),
                ('bmkhs_rotorRollMid', 'rotorRollMid'), ('bmkhs_rotorRollMax', 'rotorRollMax'),
                ('bmkhs_rotorCollMin', 'rotorCollMin'), ('bmkhs_rotorCollMid', 'rotorCollMid'),
                ('bmkhs_rotorCollMax', 'rotorCollMax'), ('bmkhs_rotorAnimSource', 'rotorAnimSource'),
                ('bmkhs_rotorHitPoint', 'rotorHitPoint')]


def airfoil_variables(H, cfg):
    """fn_airfoilVariables - config errors omitted, the config is known good."""
    airfoils = {}
    for i in range(1, int(cfg['numAirfoils']) + 1):
        a = cfg['Airfoils']['Airfoil%02d' % i]
        airfoils[a['name']] = a['table']
    H['bmkhs_airfoils'] = airfoils


def airfoil_get(H, name):
    """fn_airfoilGet - a name that matches nothing returns an empty table, as Core does after its error."""
    return H['bmkhs_airfoils'].get(name, [])


def rotor_variables(H, cfg):
    """fn_rotorVariables."""
    H['bmkhs_numRotors'] = cfg['numRotors']
    for var, key in ROTOR_CONFIG:
        H[var] = cfg[key]
    for k in ('bmkhs_rotorFlapMoment', 'bmkhs_rotorBladeAzimuth', 'bmkhs_rotorInducedFlow',
              'bmkhs_rotorInducedFlowAccum'):
        H[k] = [[0.0, 0.0, 0.0, 0.0] for _ in range(6)]
    H['bmkhs_rotorReactionTorque'] = [0.0] * 6
    H['bmkhs_rotorThrustAccum'] = [0.0] * 6
    H['bmkhs_rotorRateDampScalar'] = 0.1
    betBands = [0.00, 10.29, 20.58, 36.01, 46.30, 51.44, 61.73, 66.88, 72.02]
    H['bmkhs_betMainLiftTable'] = [[x, 1.0] for x in betBands]
    H['bmkhs_betTailLiftTable'] = [[x, 1.0] for x in betBands]
    for k in ('bmkhs_rotorBeta0', 'bmkhs_rotorA1', 'bmkhs_rotorB1',
              'bmkhs_rotorBeta0Target', 'bmkhs_rotorA1Target', 'bmkhs_rotorB1Target'):
        H[k] = [0.0] * 6
    #Rig only. Arma turns the rotor animation source and holds the hitpoints and the centre of mass;
    #here the phase turns at the rotor's own rpm and nothing is damaged. What the rotor hands to
    #addForce and addTorque is summed per rotor, in N and in Nm about the centre of mass.
    H['rotorAnimPhase'] = [0.0] * 6
    H['hitPointDamage'] = {}
    H['centerOfMass'] = [0.0, 0.0, 0.0]
    H['rotorForceN'] = [[0.0, 0.0, 0.0] for _ in range(6)]
    H['rotorMomentNm'] = [[0.0, 0.0, 0.0] for _ in range(6)]


# ---------------------------------------------------------------------------------------------
# fn_environmentWeather, fn_environment, fn_environmentGusts
# ---------------------------------------------------------------------------------------------

#The CBA environment setting, in its own order - as fn_environmentWeather's table:
#{name: (base altitude ft, mean FAT C, swing C, dew point C, average wind m/s, average cloud 0..1)}
ENVIRONMENTS = {
    'ISA_STD':             (0,    15.0, 0.0,   0.0, 0.0, 0.00),
    'EUROPE_SUMMER':       (800,  18.0, 5.0,  12.0, 2.7, 0.55),
    'EUROPE_WINTER':       (800,  -3.0, 3.0,  -4.0, 3.6, 0.79),
    'MIDDLE_EAST_SUMMER':  (1800, 31.0, 7.0,  14.0, 3.2, 0.25),
    'MIDDLE_EAST_WINTER':  (1800, 14.0, 5.0,   6.0, 2.7, 0.41),
    'CENTRAL_ASIA_SUMMER': (5000, 22.0, 7.0,   8.0, 2.5, 0.31),
    'CENTRAL_ASIA_WINTER': (5000, -4.0, 5.0, -10.0, 2.4, 0.61),
    'ASIA_SUMMER':         (3100, 21.0, 5.0,  16.0, 1.9, 0.63),
    'ASIA_WINTER':         (3100, -2.0, 5.0,  -7.0, 2.5, 0.43),
}
#The setting in use; Standard Day unless a scenario sets another with set_environment().
ENVIRONMENT = 'ISA_STD'
#Its base altitude and mean FAT, which a scenario may move - pulled() puts a FAT at a pressure altitude.
BASE_ALT, BASE_FAT = ENVIRONMENTS[ENVIRONMENT][:2]
#What the environment reads from Arma - the rig's own, there being no Arma here: dayTime, the sunrise
#and sunset of BIS_fnc_sunriseSunsetTime, overcast, wind (m/s, the way it blows), gusts, fog, and the
#settings bmkhs_windDisabled.
WEATHER = {'dayTime': 12.0, 'sunrise': 6.0, 'sunset': 18.0, 'overcast': 0.0, 'wind': [0.0, 0.0, 0.0],
           'gusts': 0.0, 'fog': 0.0, 'windDisabled': False,
           #rain, 0..1, and CBA_missionTime, s - the rain is stepped by it
           'rain': 0.0, 'time': 0.0}
#fn_eventPreInit's globals - the environment's weather, one for every aircraft: the setting's base
#altitude (ft), base FAT, swing and dew point (deg C) and average wind (m/s), where the day is in its
#cycle, what the overcast makes of it, and the rain - how far it has taken the air to saturation
#(0..1), its rate (mm/h) and when it was last stepped (never, until the first solve).
#environment_weather() solves it.
bmkhs_envRain = 0.0
bmkhs_envRainRate = 0.0
bmkhs_envRainT = -1.0
bmkhs_envBaseAlt = 0.0
bmkhs_envBaseFat = 0.0
bmkhs_envSwing = 0.0
bmkhs_envDewPoint = 0.0
bmkhs_envAvgWind = 0.0
bmkhs_envCycle = False
bmkhs_envIsDay = True
bmkhs_envShare = 0.0
bmkhs_envLayerSec = 0.0
bmkhs_envCloudDay = 1.0
bmkhs_envCloudNight = 1.0
#The gusts' own random numbers, seeded so a run repeats.
GUST_RNG = random.Random(1)


def set_environment(name):
    """Selects the setting and puts the base on its own altitude and mean FAT."""
    global ENVIRONMENT, BASE_ALT, BASE_FAT
    ENVIRONMENT = name
    BASE_ALT, BASE_FAT = ENVIRONMENTS[name][:2]


def environment_weather():
    """fn_environmentWeather: the setting's numbers, and what the clock and sky make of the day."""
    global bmkhs_envBaseAlt, bmkhs_envBaseFat, bmkhs_envSwing, bmkhs_envDewPoint, bmkhs_envAvgWind
    global bmkhs_envCycle, bmkhs_envIsDay, bmkhs_envShare, bmkhs_envLayerSec
    global bmkhs_envCloudDay, bmkhs_envCloudNight
    global bmkhs_envRain, bmkhs_envRainRate, bmkhs_envRainT
    #The setting: fn_environmentWeather's switch, with the base a scenario may have moved
    swing, dewPoint, avgWind, avgCloud = ENVIRONMENTS[ENVIRONMENT][2:]
    baseAlt, baseFAT = BASE_ALT, BASE_FAT

    #The day. No sunrise or no sunset, as inside a polar circle, is no daily cycle.
    dayTime, sunrise, sunset = WEATHER['dayTime'], WEATHER['sunrise'], WEATHER['sunset']
    cycle = sunrise >= 0.0 and sunset > sunrise
    isDay = True
    share = 0.0
    layerSec = 0.0
    if cycle:
        dayLength = sunset - sunrise
        nightLength = 24.0 - dayLength
        isDay = dayTime >= sunrise and dayTime < sunset
        if isDay:
            #The sun's heating so far, as a share of the day's (Stull 18.2), burns the night's cold
            #layer off from the ground up: 1 - u + u ln u of its heat is gone when u of the dawn
            #drop is left at the surface.
            heat = (1.0 - math.cos(math.radians(180.0 * (dayTime - sunrise) / dayLength))) / 2.0
            lo, hi = 0.0, 1.0
            for _ in range(20):
                mid = (lo + hi) / 2.0
                if (1.0 - mid) + (mid * math.log(mid)) > heat:
                    lo = mid
                else:
                    hi = mid
            share = (lo + hi) / 2.0
            layerSec = nightLength * 3600.0
        else:
            #The surface cools as the square root of the time since sunset (Stull 18.1, 18.4, 18.5)
            since = dayTime - sunset
            if since < 0.0:
                since = since + 24.0
            share = math.sqrt(since / nightLength)
            layerSec = since * 3600.0

    #Cloud against the region's average: the day's warming by the sunlight it lets through, the
    #night's cooling by the heat it lets out (Stull 2.35, 2.39, overcast taken as middle cloud)
    overcast = WEATHER['overcast']
    cloudDay = (1.0 - (ENV_CLOUD_DAY * overcast)) / (1.0 - (ENV_CLOUD_DAY * avgCloud))
    cloudNight = (1.0 - (ENV_CLOUD_NIGHT * overcast)) / (1.0 - (ENV_CLOUD_NIGHT * avgCloud))

    #Rain. The mission's rain, 0..1, is a rain rate through three even bands - light, moderate and
    #heavy. The rain water that rate holds in the air, and what it evaporates into the air, are the
    #Kessler scheme's (Klemp and Wilhelmson 1978, as the WRF model's module_mp_kessler has them).
    #Evaporating, it takes the air cooler and wetter toward its wet-bulb temperature, while the air
    #mixes back out at ENV_RAIN_RECOVERY: bmkhs_envRain is how far it is there, 0..1. Solved in the
    #setting's own air - its mean FAT and dew point at its base altitude. Standard Day has none.
    rainRate = 0.0
    if ENVIRONMENT == 'ISA_STD':
        bmkhs_envRain = 0.0
        bmkhs_envRainT = -1.0
    else:
        rainRate = math_linear_interp([[0.0, 0.0], [1.0 / 3.0, ENV_RAIN_LIGHT], [2.0 / 3.0, ENV_RAIN_MODERATE],
                                       [1.0, ENV_RAIN_TOP]], WEATHER['rain'])[1]
        #The setting's air, and its wet-bulb: pressure hPa, mixing ratios kg/kg
        pressure = environment_pressure(baseAlt * FEET_TO_METERS)
        vapour = min(environment_saturation(dewPoint), environment_saturation(baseFAT))
        mixing = (VAPOUR_MASS_RATIO * vapour) / (pressure - vapour)
        wetBulb = environment_wet_bulb(baseFAT, vapour, pressure)
        wetBulbVapour = environment_saturation(wetBulb)
        toSaturate = max(((VAPOUR_MASS_RATIO * wetBulbVapour) / (pressure - wetBulbVapour)) - mixing, 1.0e-9)

        def closing(this):
            """The rate, 1/s, at which the rain closes what is left to saturation, with the air
            `this` of the way there."""
            far = min(this, 0.999)
            temp = baseFAT - (far * (baseFAT - wetBulb))
            mix = mixing + (far * toSaturate)
            saturated = environment_saturation(temp)
            mixSat = (VAPOUR_MASS_RATIO * saturated) / (pressure - saturated)
            rho = (pressure * 100.0) / (GAS_CONSTANT_DRY_AIR * (temp + DEG_C_TO_KELVIN))
            #Rain water, g/cm3, for the rate: the rate is the water times its fall speed,
            #36.34 (water)^0.1364 sqrt(sea level density / density) m/s, turned round
            water = 0.001 * ((rainRate / (3600.0 * 36.34 * (0.001 ** 0.1364) * math.sqrt(ISA_STD_DAY_AIR_DENSITY / rho))) ** (1.0 / 1.1364))
            #What it evaporates, kg/kg s - pressure in Pa and density in g/cm3, as the scheme has them
            evaporated = (((1.6 + (124.9 * (water ** 0.2046))) * (water ** 0.525)) / ((2.55e8 / ((pressure * 100.0) * mixSat)) + 5.4e5)) * (max(mixSat - mix, 0.0) / ((0.001 * rho) * mixSat))
            return evaporated / (toSaturate * (1.0 - far))

        if bmkhs_envRainT < 0.0:
            #The first solve: a mission that opens in rain has been in it a while, so the air starts
            #settled, where the rain's closing and the mixing balance
            lo, hi = 0.0, 1.0
            for _ in range(20):
                mid = (lo + hi) / 2.0
                if (closing(mid) * (1.0 - mid)) > (mid / ENV_RAIN_RECOVERY):
                    lo = mid
                else:
                    hi = mid
            bmkhs_envRain = (lo + hi) / 2.0
        else:
            #Toward where the two balance, at the rate of the two together
            rate = closing(bmkhs_envRain) + (1.0 / ENV_RAIN_RECOVERY)
            bmkhs_envRain = util_smooth(bmkhs_envRain, 1.0 - (1.0 / (ENV_RAIN_RECOVERY * rate)),
                                        WEATHER['time'] - bmkhs_envRainT, 1.0 / rate)
        bmkhs_envRainT = WEATHER['time']

    bmkhs_envRainRate = rainRate
    bmkhs_envBaseAlt = baseAlt
    bmkhs_envBaseFat = baseFAT
    bmkhs_envSwing = swing
    bmkhs_envDewPoint = dewPoint
    bmkhs_envAvgWind = avgWind
    bmkhs_envCycle = cycle
    bmkhs_envIsDay = isDay
    bmkhs_envShare = share
    bmkhs_envLayerSec = layerSec
    bmkhs_envCloudDay = cloudDay
    bmkhs_envCloudNight = cloudNight


def environment_saturation(t):
    """fn_environment's _saturation: saturation vapour pressure, hPa, at t deg C (WMO-No. 8)."""
    return 6.112 * math.exp((17.62 * t) / (243.12 + t))


def environment_pressure(altitudeM):
    """fn_environment's _isaPressure: the standard atmosphere's pressure, hPa, at a pressure altitude in m."""
    return ISA_PRESSURE_HPA * ((1.0 - ((ISA_LAPSE * altitudeM) / ISA_TEMP_K)) ** ISA_EXPONENT)


def environment_wet_bulb(temperature, vapour, pressure):
    """fn_environmentWetBulb: the wet-bulb temperature, deg C, of air at a temperature (deg C), vapour
    pressure and pressure (hPa) - where it ends up once rain has saturated it (Stull 4.17, 4.18)."""
    mixing = (VAPOUR_MASS_RATIO * vapour) / (pressure - vapour)
    lo, hi = temperature - 40.0, temperature
    for _ in range(14):
        mid = (lo + hi) / 2.0
        saturated = 6.112 * math.exp((17.62 * mid) / (243.12 + mid))
        if (SPECIFIC_HEAT_AIR * (temperature - mid)) > (LATENT_HEAT_VAPOUR * (((VAPOUR_MASS_RATIO * saturated) / (pressure - saturated)) - mixing)):
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def environment_dew_point(vapour):
    """fn_environment's _dewPointOf: the dew point, deg C, of a vapour pressure in hPa - absolute zero in dry air."""
    if vapour <= 0.0:
        return -DEG_C_TO_KELVIN
    g = math.log(vapour / 6.112)
    return (243.12 * g) / (17.62 - g)


def environment(H):
    #Core solves the weather once every ENV_WEATHER_PERIOD, by bmkhs_envWeatherT, for every aircraft.
    #The rig has no mission clock, so it solves it on every call.
    environment_weather()
    baseAlt = bmkhs_envBaseAlt
    baseFAT = bmkhs_envBaseFat
    swing = bmkhs_envSwing
    dewPoint = bmkhs_envDewPoint
    avgWind = bmkhs_envAvgWind
    cycle = bmkhs_envCycle
    isDay = bmkhs_envIsDay
    share = bmkhs_envShare
    layerSec = bmkhs_envLayerSec
    cloudDay = bmkhs_envCloudDay
    cloudNight = bmkhs_envCloudNight
    rain = bmkhs_envRain
    standardDay = ENVIRONMENT == 'ISA_STD'
    deltaTime = H.get('bmkhs_deltaTime', 0.0)

    #Height above the map's sea level, and above the ground - the sea where there is no ground. The
    #height above ground is eased, so a cliff edge is a drift in temperature and not a step.
    asl = H['baroAltM']
    aglNow = max(asl - max(H.get('terrainAslM', 0.0), 0.0), 0.0)
    agl = util_smooth(H.get('bmkhs_envAgl', aglNow), aglNow, deltaTime, ENV_AGL_TIME)
    H['bmkhs_envAgl'] = agl
    groundAsl = asl - agl

    #The steady wind: the mission's, its jumps eased
    windNow = [WEATHER['wind'][0], WEATHER['wind'][1], 0.0]
    windSteady = util_smooth(H.get('bmkhs_windSteady', windNow), windNow, deltaTime, ENV_WIND_TIME)
    H['bmkhs_windSteady'] = windSteady
    windSteadySpeed = math.sqrt(sum(c * c for c in windSteady))

    #Pressure altitude and the standard atmosphere's pressure there
    altitude = baseAlt + (asl * METERS_TO_FEET)
    pressure = environment_pressure(altitude * FEET_TO_METERS)

    #Free air: the preset's mean, lapsed
    temperature = baseFAT - (ISA_LAPSE * asl)

    #The preset's vapour at its base thins with pressure
    pressureBase = environment_pressure(baseAlt * FEET_TO_METERS)
    vapourBase = environment_saturation(dewPoint)

    if (not standardDay) and cycle:
        #The afternoon's surface above the mean and the dawn's below it are the measured swing at
        #the region's average cloud, each narrowed or widened by the cloud there is. The night's
        #whole drop from the afternoon is deeper in less wind than the region's average and
        #shallower in more (Stull 18.4, 18.5).
        windNight = max(windSteadySpeed, ENV_NIGHT_WIND_MIN)
        rise = swing * cloudDay
        drop = (rise + (swing * cloudNight)) * ((avgWind / windNight) ** 0.75)
        #A calm night cools the surface no further than its dew point - or the average dawn, where
        #that is the colder
        groundFree = baseFAT - (ISA_LAPSE * groundAsl)
        dewGround = environment_dew_point(
            vapourBase * environment_pressure((baseAlt * FEET_TO_METERS) + groundAsl) / pressureBase)
        dropLimit = max((groundFree + rise) - min(dewGround, groundFree - swing), 0.0)
        #The night layer's e-folding height (Stull 18.4)
        layerHeight = max(ENV_NIGHT_LAYER_A * (windNight ** 0.75) * math.sqrt(layerSec), 0.1)
        if isDay:
            dawn = min(drop, dropLimit)
            deficit = min(share * dawn, dawn * math.exp(-agl / layerHeight))
        else:
            deficit = min(drop * share, dropLimit) * math.exp(-agl / layerHeight)
        #The afternoon's air falls at the dry adiabatic rate above the ground until it meets the free air
        temperature = temperature + max(rise - ((DRY_ADIABATIC_LAPSE - ISA_LAPSE) * agl), 0.0) - deficit

    #Vapour: none on a standard day; saturated in fog; never more than the air can hold
    vapour = 0.0
    #What the rain has taken off the temperature, deg C
    rainCooling = 0.0
    if not standardDay:
        vapour = min(vapourBase * pressure / pressureBase, environment_saturation(temperature))
        #Rain: the air here is `rain` of the way to its wet-bulb temperature - cooler, and wetter by
        #the vapour whose evaporation cooled it. Before the fog, so the air is as cool under the
        #fog's top as over it.
        if rain > 0.0:
            wetBulb = environment_wet_bulb(temperature, vapour, pressure)
            wetBulbVapour = environment_saturation(wetBulb)
            mixing = (VAPOUR_MASS_RATIO * vapour) / (pressure - vapour)
            mixing = mixing + (rain * (((VAPOUR_MASS_RATIO * wetBulbVapour) / (pressure - wetBulbVapour)) - mixing))
            rainCooling = rain * (temperature - wetBulb)
            temperature = temperature - rainCooling
            vapour = min((mixing * pressure) / (VAPOUR_MASS_RATIO + mixing), environment_saturation(temperature))
        #Fog: saturated below its top
        if agl < WEATHER['fog'] * ENV_FOG_TOP:
            vapour = environment_saturation(temperature)

    H['bmkhs_rainCooling'] = rainCooling
    H['bmkhs_barAlt'] = altitude
    H['bmkhs_fat'] = temperature
    H['bmkhs_pressure'] = pressure
    H['bmkhs_dewPoint'] = environment_dew_point(vapour)
    H['bmkhs_rho'] = ((pressure - (VAPOUR_DENSITY_DEFICIT * vapour)) * 100.0) / (GAS_CONSTANT_DRY_AIR * (temperature + DEG_C_TO_KELVIN))

    #What the cockpit reads and the airframe feels: the steady wind, none with wind disabled
    windFelt = [0.0, 0.0, 0.0] if WEATHER['windDisabled'] else windSteady
    windSpeed = math.sqrt(sum(c * c for c in windFelt))
    H['bmkhs_windSpeed'] = windSpeed
    H['bmkhs_windDirFrom'] = (sqf_round((math.degrees(math.atan2(windFelt[0], windFelt[1])) + 540.0) % 360.0) % 360
                              if windSpeed > 0.0 else 0)
    velWindWorldSpace = [0.0, 0.0, 0.0]
    velGustWorldSpace = [0.0, 0.0, 0.0]
    if bmkhs_rotorModel == 0:
        velWindWorldSpace = windFelt
        velGustWorldSpace = environment_gusts(H, windFelt, agl, deltaTime)
    H['bmkhs_velWindWorldSpace'] = velWindWorldSpace
    H['bmkhs_velGustWorldSpace'] = velGustWorldSpace


#fn_environmentGusts' table - MIL-F-8785C figure 7, gust strength above 2,000 ft:
#[height ft, light, moderate, severe], ft/s
GUST_INTENSITY = [
    [500.0,    6.6,  8.6, 15.6],
    [1750.0,   6.9,  9.6, 17.6],
    [3750.0,   7.4, 10.6, 23.0],
    [7500.0,   6.7, 10.1, 23.6],
    [15000.0,  4.6,  8.0, 22.1],
    [25000.0,  2.7,  6.6, 20.0],
    [35000.0,  0.4,  5.0, 16.0],
    [45000.0,  0.0,  4.2, 15.1],
    [55000.0,  0.0,  2.7, 12.1],
    [65000.0,  0.0,  0.0,  7.9],
    [75000.0,  0.0,  0.0,  6.2],
    [80000.0,  0.0,  0.0,  5.1],
]


def environment_gusts(H, wind, agl, deltaTime):
    """fn_environmentGusts: the gust on the steady wind, world space m/s."""
    state = H.get('bmkhs_gustState', [0.0, 0.0, 0.0])
    windSpeed = math.sqrt(sum(c * c for c in wind))
    severity = WEATHER['gusts']
    if severity <= 0.0 or windSpeed <= 0.0:
        H['bmkhs_gustState'] = [0.0, 0.0, 0.0]
        return [0.0, 0.0, 0.0]

    #Height above ground, and the speed through the air mass
    h = max(agl, GUST_HEIGHT_MIN)
    vel = H.get('velWorld', [0.0, 0.0, 0.0])
    v = max(math.sqrt(sum((a - b) ** 2 for a, b in zip(vel, wind))), GUST_AIRSPEED_MIN)

    #Below 1,000 ft: strength from the wind, size from the height. The specification's shape is per foot.
    low = min(h, GUST_LOW_TOP)
    shape = 0.177 + (0.000823 * METERS_TO_FEET * low)
    lu = low / (shape ** 1.2)
    lw = low
    sw = 0.1 * windSpeed
    su = sw / (shape ** 0.4)
    if h > GUST_LOW_TOP:
        #Above 2,000 ft: the specification's table, in ft and ft/s, read at this height and between
        #its rows by the wind; from 1,000 to 2,000 ft, between the two
        row = math_linear_interp(GUST_INTENSITY, h * METERS_TO_FEET)
        steps = [0.0, row[1] * FEET_TO_METERS, row[2] * FEET_TO_METERS, row[3] * FEET_TO_METERS]
        at = clamp(windSpeed / GUST_WIND_STEP, 0.0, 3.0)
        lowStep = min(math.floor(at), 2)
        high = steps[lowStep] + ((steps[lowStep + 1] - steps[lowStep]) * (at - lowStep))
        blend = clamp((h - GUST_LOW_TOP) / (GUST_HIGH_BASE - GUST_LOW_TOP), 0.0, 1.0)
        lu = GUST_LOW_TOP + (blend * (GUST_HIGH_SIZE - GUST_LOW_TOP))
        lw = lu
        su = sw + (blend * (high - sw))
        sw = su
    su = su * severity
    sw = sw * severity

    #Each component eases toward zero and is pushed by its own noise, at a rate set by how fast the
    #aircraft crosses a gust: the specification's filters (Yeager, NASA CR-1998-206937, eq. 30-32),
    #stepped exactly so the gusts are the same at any frame rate
    keepU = math.exp(-deltaTime * v / lu)
    keepV = math.exp(-2.0 * deltaTime * v / lu)
    keepW = math.exp(-2.0 * deltaTime * v / lw)
    state = [
        (keepU * state[0]) + (su * math.sqrt(1.0 - (keepU * keepU)) * GUST_RNG.gauss(0.0, 1.0)),
        (keepV * state[1]) + (su * math.sqrt(1.0 - (keepV * keepV)) * GUST_RNG.gauss(0.0, 1.0)),
        (keepW * state[2]) + (sw * math.sqrt(1.0 - (keepW * keepW)) * GUST_RNG.gauss(0.0, 1.0)),
    ]
    H['bmkhs_gustState'] = state

    #Along the wind, across it, and vertical
    along = [wind[0] / windSpeed, wind[1] / windSpeed]
    return [
        (along[0] * state[0]) + (along[1] * state[1]),
        (along[1] * state[0]) - (along[0] * state[1]),
        state[2],
    ]


# ---------------------------------------------------------------------------------------------
# Engine - fn_engineGovernor, gasTurbine/*, turboShaftEngine/*
# ---------------------------------------------------------------------------------------------

def engine_governor(H, i, eng, ng, nc, np_, tgt, lever, fat, dt):
    idleNg = eng['idleNg']

    #Ng's rate of change, filtered - what the Ng limiter anticipates with.
    raw = (ng - H['bmkhs_engNgPrev'][i]) / dt if dt > 0 else 0.0
    H['bmkhs_engNgRate'][i] += (raw - H['bmkhs_engNgRate'][i]) * (1.0 - math.exp(-dt / GT_NG_RATE_TAU))
    H['bmkhs_engNgPrev'][i] = ng

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
        coll = clamp(H['bmkhs_collectiveOutput'] + H.get('bmkhs_fmcAltHoldCollOut', 0.0), 0.0, 1.0)
        govFuel = pid_run(pid, dt, npTarget, np_) + coll * eng['ffwdGain']
        tgtLim = eng['maxTgtSe'] if H['bmkhs_isSingleEng'] else eng['maxTgt']
        ngLim = min(eng['ngLimitMax'], eng['ngLimitBase'] + eng['ngLimitSlope'] * fat)
        ngAhead = ng + H['bmkhs_engNgRate'][i] * GT_NG_LEAD
        err = min((tgtLim - tgt) / GT_TGT_LIMIT_BAND, (ngLim - ngAhead) / GT_NG_LIMIT_BAND)
        lim = H['bmkhs_engLimFuel'][i] + GT_LIMIT_GAIN * err * dt
        lim = min(lim, min(sched, max(govFuel, 0.0)) * GT_LIMIT_TRACK)
        lim = min(max(lim, eng['fuelIdle']), eng['fuelFly'])
        H['bmkhs_engLimFuel'][i] = lim
        allowed = min(sched, lim)
        minFuel = H['bmkhs_engMinFuel'][i] + GT_LIMIT_GAIN * ((eng['idleNg'] - ng) / GT_NG_LIMIT_BAND) * dt
        minFuel = min(max(minFuel, 0.0), eng['fuelIdle'])
        H['bmkhs_engMinFuel'][i] = minFuel
        if govFuel < minFuel:
            pid['integral'] = integral
            govFuel = minFuel
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
        H['bmkhs_engMinFuel'][i] = 0.0
    H['bmkhs_engNpRef'][i] = npRef

    fuelCmd = orifice
    if nc < idleNg:
        base = eng['startFuelBase']
        fuelCmd = fuelCmd * min(base + (1.0 - base) * nc / idleNg, 1.0)

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
    return min(eng['starterTorque'], eng['starterPower'] / max(ng, GT_NG_TORQUE_FLOOR)) if supplied else 0.0


def gas_turbine_compressor(eng, ng, fat, pAmb, velFwd):
    tAmb = fat + DEG_C_TO_KELVIN
    mach2 = max(velFwd, 0.0) ** 2 / (GT_GAMMA_COLD * GT_R_AIR * 1000 * tAmb)
    ram = 1 + (GT_GAMMA_COLD - 1) / 2 * mach2
    t2 = tAmb * ram
    p2 = pAmb + eng['ramRecovery'] * (pAmb * ram ** (GT_GAMMA_COLD / (GT_GAMMA_COLD - 1)) - pAmb)
    theta = t2 / GT_STD_TEMP_K
    nc = ng / math.sqrt(theta)
    _, prFrac, flowFrac, eff, ctFrac = math_linear_interp(eng['compressorMap'], nc / eng['maxNg'])
    designPr = eng['pressureRatio']
    pr = designPr * prFrac
    airflow = math_linear_interp(eng['airflowTable'], fat)[1]
    mDot = eng['massFlow'] * flowFrac * airflow * (p2 / GT_STD_PRESSURE_KPA) / math.sqrt(theta)
    xc = (GT_GAMMA_COLD - 1) / GT_GAMMA_COLD
    t3 = t2 * (1 + ((pr ** xc) - 1) / eff)
    compPower = mDot * GT_CP_COLD * (t3 - t2)
    inletArea = math.pi * (eng['inletDiameter'] / 2) ** 2
    inletVel = mDot / inletArea / (p2 / (GT_R_AIR * t2))
    return nc, pr, mDot, t3, p2 * pr, compPower, designPr ** ctFrac, inletVel, theta


def gas_turbine_combustor(eng, t3, mDot, fuelCmd, running):
    if not running:
        return t3
    fuelKgs = fuelCmd * eng['maxFuelFlow']
    return t3 + fuelKgs * eng['fuelLhv'] * eng['combustorEfficiency'] / (max(mDot, 0.001) * GT_CP_HOT)


def gas_turbine_compressor_turbine(eng, ng, tgt, t4, p3, mDot, compPower, ctExpansion, starterTq,
                                   residualHeat, fat, velY, running, spooling, dt):
    m = max(mDot, 0.001)
    xh = (GT_GAMMA_HOT - 1) / GT_GAMMA_HOT
    ctPower = m * GT_CP_HOT * eng['turbineEfficiency'] * t4 * (1 - ctExpansion ** (-xh))
    t45 = t4 - ctPower / (m * GT_CP_HOT)
    drag = (eng['compDrag'] * ng * ng + eng['compDragFloor']) if spooling else 0.0
    #Power over speed: the turbines' surplus as a torque, in the same units as the starter and the drag.
    #Unchanged at Ng 1.0; below it the same surplus turns the spool harder, as torque does.
    ngDot = (((ctPower - compPower) / eng['spoolUnitKw']) / max(ng, GT_NG_TORQUE_FLOOR)
             + starterTq - drag) / eng['spoolInertia']
    ngNew = clamp(ng + ngDot * dt, 0.0, eng['maxNg'])
    currentHeat = 1.0 + (residualHeat - 1.0) * max(1.0 - ngNew / eng['idleNg'], 0.0)
    tgtHot = fat + currentHeat * (t45 - DEG_C_TO_KELVIN - fat) if running else fat
    coolRate = GT_TGT_COOL_RATE * (ngNew + GT_TGT_STILL_AIR + max(velY, 0.0) * GT_TGT_RAM_AIR)
    rate = GT_TGT_HEAT_RATE if tgtHot > tgt else coolRate
    return ngNew, tgt + (tgtHot - tgt) * rate * dt, t45, p3 / ctExpansion


def turbo_shaft_power_turbine(eng, t45, p45, p2, mDot, running, np_, nrFrac, dt):
    refTq = eng['refTq']
    t5, ptPower = t45, 0.0
    if p45 > p2:
        xh = (GT_GAMMA_HOT - 1) / GT_GAMMA_HOT
        t5 = t45 * (1 - eng['ptEfficiency'] * (1 - (p2 / p45) ** xh))
        ptPower = max(mDot, 0.0) * GT_CP_HOT * (t45 - t5)
    shaftTq = min((ptPower / eng['powerKw']) / max(np_, 1.0 / GT_PT_STALL_TQ), GT_PT_STALL_TQ) * refTq
    npDrag = eng['ptDrag'] * np_ * np_ + (eng['ptDragFloor'] if shaftTq <= 0.0 else 0.0)
    npDot = ((shaftTq / refTq) - npDrag) / eng['ptInertia']
    npFree = max(np_ + npDot * dt, 0.0)
    clutch = npFree >= nrFrac
    return shaftTq, (nrFrac if clutch else npFree), clutch, t5


def turbo_shaft_engine(H, i, eng):
    dt = H['bmkhs_deltaTime']
    fat = H['bmkhs_fat']
    p2 = H['bmkhs_pressure'] * 0.1
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
    if not tripped and ((eng['ngOvrspdProt'] > 0 and ng >= eng['ngOvrspdProt']) or (eng['npOvrspdProt'] > 0 and np_ >= eng['npOvrspdProt'])):
        tripped = True
        H['bmkhs_engineOverspeed'][i] = True

    fuelAvail = H['fuelAvail'][i]
    failed = H['bmkhs_engFailed'][i]
    running = ng > eng['lightOffNg'] and lever != 'OFF' and fuelAvail and not tripped and not failed

    starterTq = gas_turbine_starter(H, i, eng, ng)
    cranking = starterTq > 0.0
    spooling = not running and not cranking

    nc, pr, mDot, t3, p3, compPower, ctExpansion, inletVel, theta = gas_turbine_compressor(eng, ng, fat, p2, velY)

    fuelCmd, share, orifice = engine_governor(H, i, eng, ng, nc, np_, tgt, lever, fat, dt)
    refTq = eng['refTq']

    #Below FLY the fuel control meters open loop, in corrected fuel flow on corrected Ng, with the
    #air the engine passes - the same fuel-air on any day.
    if lever != 'FLY':
        airflow = math_linear_interp(eng['airflowTable'], fat)[1]
        fuelCmd = fuelCmd * ((p3 / pr) / GT_STD_PRESSURE_KPA) * math.sqrt(theta) * airflow
    t4 =gas_turbine_combustor(eng, t3, mDot, fuelCmd, running)
    ngNew, tgt, t45, p45 = gas_turbine_compressor_turbine(
        eng, ng, tgt, t4, p3, mDot, compPower, ctExpansion, starterTq, residualHeat, fat, velY,
        running, spooling, dt)

    xmsnRpm = H['bmkhs_xmsnOutputRpm']
    nrFrac = xmsnRpm / (eng['npFly'] * eng['designRpm'])

    tqOut, npNew, clutch, t5 = turbo_shaft_power_turbine(eng, t45, p45, p2, mDot, running, np_, nrFrac, dt)

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
                        tripped=tripped, share=share, nc=nc, pr=pr, mDot=mDot, t3=t3, t4=t4, t45=t45,
                        t5=t5, inletVel=inletVel)


def engine_controller(H, cfg):
    """fn_engineUpdate, useSystems path."""
    engState = list(H['bmkhs_engState'])
    engs = range(len(H['bmkhs_engines']))
    for e in engs:
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
        for e in engs:
            if engState[e] == 'STARTING':
                H['bmkhs_engState'][e] = 'OFF'
    for e, eng in enumerate(H['bmkhs_engines']):
        turbo_shaft_engine(H, e, eng)
    for e in engs:
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
    dragTq = sum(e['refTq'] for e in H['bmkhs_engines']) * H.get('bmkhs_drivetrainDrag', 0.0)
    if outputRpm <= 0.0:
        dragTq = min(dragTq, max(engInputTq - totTq, 0.0))
    totTq += dragTq
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
    dragCoef = math_linear_interp_2d(rotor['dragCoefTable'], simple_rotor_table_key(rotor, collOutput), velXY)
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
    liftCoef = math_linear_interp_2d(rotor['liftCoefTable'], simple_rotor_table_key(rotor, collOutput), velXY)
    axialVel = H['hubVelZ'] * (-1.0 if liftCoef < 0 else 1.0)
    viScalarDenom = linear_conversion(-7.62, -19.30, axialVel, VEL_VRS, VEL_VRS * 0.1, True)
    viScalar = 0.0 if (axialVel < -VEL_VRS and velXY < VEL_ETL) else 1 - (axialVel / viScalarDenom)
    bladeLift = liftCoef * 0.5 * rho * (rotor['bladeRadius'] * rotor['bladeChord']) * (bladeVel75 * bladeVel75)
    return 4 * bladeLift * (rotor['numBlades'] / 4) * viScalar


def simple_rotor_table_key(rotor, control):
    """fn_simpleRotor's _tableKey - the control through the rotor's optional controlMap."""
    m = rotor.get('controlMap') or []
    return control if not m else math_linear_interp(m, control)[1]


def simple_rotor_update(H):
    for idx, rotor in enumerate(H['bmkhs_simpleRotors']):
        simple_rotor(H, idx, rotor)


# ---------------------------------------------------------------------------------------------
# rotor/ - the blade element rotor: fn_rotorControl, fn_rotorFlapDynamics, fn_rotorBlade, fn_rotor,
# fn_rotorUpdate. The debug drawing and the systemChat readout are not ported.
# ---------------------------------------------------------------------------------------------

def rotor_control(H, type_, pitchMin, pitchMid, pitchMax, rollMin, rollMid, rollMax, collMin, collMid, collMax):
    """fn_rotorControl."""
    pitchInput = H['bmkhs_fltCtrlPitchOut']
    pitchFeather = 0.0
    rollInput = H['bmkhs_fltCtrlRollOut']
    rollFeather = 0.0
    yawInput = H['bmkhs_fltCtrlYawOut']
    collInput = H['bmkhs_fltCtrlCollOut']
    collFeather = 0.0
    if type_ == MAIN:
        pitchFeather = math_linear_interp_from_center(-1, 1, pitchInput, pitchMin, pitchMid, pitchMax)
        rollFeather = math_linear_interp_from_center(-1, 1, rollInput, rollMin, rollMid, rollMax)
        collFeather = linear_conversion(0, 1, collInput, collMin, collMax, True)
    elif type_ == TAIL:
        collFeather = math_linear_interp_from_center(-1, 1, -yawInput, collMin, collMid, collMax)
    return pitchFeather, rollFeather, collFeather


def rotor_flap_dynamics(H, rotorIndex, type_, deltaTime, numBlades, omega, bladeMass, bladeLength,
                        bladeCutout, flapTimeConst, rVec, fVec, uVec):
    """fn_rotorFlapDynamics."""
    flapTimeConstPitch = flapTimeConst[0]
    flapTimeConstRoll = flapTimeConst[1]

    Ib = (1.0 / 3.0) * bladeMass * (bladeLength * bladeLength)
    bladeCGR = bladeCutout + ((bladeLength - bladeCutout) * 0.5)
    M_grav = -bladeMass * 9.806 * bladeCGR
    denom = Ib * (omega * omega) if omega > 1.0 else 0.0

    flapMoments = H['bmkhs_rotorFlapMoment'][rotorIndex]
    bladeAzimuths = H['bmkhs_rotorBladeAzimuth'][rotorIndex]
    M0 = Mlat = Mlon = 0.0
    for bladeIndex in range(int(numBlades)):
        bDir_i = math_vector_rotate_around_axis(rVec, uVec, bladeAzimuths[bladeIndex])
        m = flapMoments[bladeIndex]
        M0 = M0 + m
        Mlat = Mlat + (m * vector_dot_product(bDir_i, rVec))
        Mlon = Mlon + (m * vector_dot_product(bDir_i, fVec))
    M0 = M0 / numBlades
    Mlat = Mlat / (numBlades / 2.0)
    Mlon = Mlon / (numBlades / 2.0)

    beta0Raw = clamp((M0 / denom + M_grav / denom) * (180 / math.pi), -15.0, 15.0) if denom > 0.0 else 0.0
    a1Raw = clamp(Mlat / denom * (180 / math.pi), -15.0, 15.0) if denom > 0.0 else 0.0
    b1Raw = clamp(Mlon / denom * (180 / math.pi), -15.0, 15.0) if denom > 0.0 else 0.0

    isTeetering = (type_ == MAIN) and (numBlades == 2)

    revPeriod = (2.0 * math.pi) / omega if omega > 1.0 else flapTimeConstPitch
    if isTeetering:
        bladePeriod = revPeriod
    else:
        bladePeriod = revPeriod / numBlades if numBlades > 0 else revPeriod
    bladeAlpha = clamp(deltaTime / bladePeriod, 0.0, 1.0) if bladePeriod > 0.0 else 1.0
    beta0Target = beta0Raw if isTeetering else lerp(H['bmkhs_rotorBeta0Target'][rotorIndex], beta0Raw, bladeAlpha)
    a1Target = a1Raw if isTeetering else lerp(H['bmkhs_rotorA1Target'][rotorIndex], a1Raw, bladeAlpha)
    b1Target = b1Raw if isTeetering else lerp(H['bmkhs_rotorB1Target'][rotorIndex], b1Raw, bladeAlpha)
    H['bmkhs_rotorBeta0Target'][rotorIndex] = beta0Target
    H['bmkhs_rotorA1Target'][rotorIndex] = a1Target
    H['bmkhs_rotorB1Target'][rotorIndex] = b1Target

    if type_ == TAIL or isTeetering:
        beta0 = 0.0
    else:
        beta0 = lerp(H['bmkhs_rotorBeta0'][rotorIndex], beta0Target, deltaTime / flapTimeConstPitch)
    if type_ == TAIL:
        a1 = 0.0
        b1 = 0.0
    else:
        a1 = a1Target if isTeetering else lerp(H['bmkhs_rotorA1'][rotorIndex], a1Target, deltaTime / flapTimeConstPitch)
        b1 = b1Target if isTeetering else lerp(H['bmkhs_rotorB1'][rotorIndex], b1Target, deltaTime / flapTimeConstRoll)
    H['bmkhs_rotorBeta0'][rotorIndex] = beta0
    H['bmkhs_rotorA1'][rotorIndex] = a1
    H['bmkhs_rotorB1'][rotorIndex] = b1

    return beta0, a1, b1


def rotor_blade(H, bladeIndex, rotorIndex, deltaTime, numElements, numBlades, pos, uVec, omega, airfoilTable,
                bladeCutout, bladeLength, inflowAlpha, a_rootPos, b_tipPos, c_rootLeadingEdge,
                d_tipLeadingEdge, e_tipTrailingEdge, f_rootTrailingEdge):
    """fn_rotorBlade."""
    rho = H['bmkhs_rho']
    dissymetryOfLift = False
    totalFlapMoment = 0.0
    totalPower = H['bmkhs_rotorReactionTorque'][rotorIndex]

    velBet = vector_magnitude([H['bmkhs_velModelSpace'][0], H['bmkhs_velModelSpace'][1], 0.0])
    isTail = rotorIndex == 1
    liftTbl = H['bmkhs_betTailLiftTable'] if isTail else H['bmkhs_betMainLiftTable']
    bladeScale = 1.0 if liftTbl == [] else math_linear_interp(liftTbl, velBet)[1]
    H['bmkhs_rotorFlapMoment'][rotorIndex][bladeIndex] = 0.0

    numElements = int(numElements)
    for i in range(numElements):
        spanInboard = i / numElements
        spanOutboard = (i + 1) / numElements

        a = vector_add(a_rootPos, vector_multiply(vector_diff(b_tipPos, a_rootPos), spanInboard))
        b = vector_add(a_rootPos, vector_multiply(vector_diff(b_tipPos, a_rootPos), spanOutboard))

        c = vector_add(c_rootLeadingEdge, vector_multiply(vector_diff(d_tipLeadingEdge, c_rootLeadingEdge), spanInboard))
        d = vector_add(c_rootLeadingEdge, vector_multiply(vector_diff(d_tipLeadingEdge, c_rootLeadingEdge), spanOutboard))
        e = vector_add(f_rootTrailingEdge, vector_multiply(vector_diff(e_tipTrailingEdge, f_rootTrailingEdge), spanOutboard))
        f = vector_add(f_rootTrailingEdge, vector_multiply(vector_diff(e_tipTrailingEdge, f_rootTrailingEdge), spanInboard))

        liftFraction = 0.75 if numElements == 1 else 0.5
        liftPos = vector_add(a, vector_multiply(vector_diff(b, a), liftFraction))

        chordLine = vector_diff(vector_add(c, vector_multiply(vector_diff(d, c), 0.5)),
                                vector_add(f, vector_multiply(vector_diff(e, f), 0.5)))
        chordLine = vector_normalized(chordLine)

        #Induced inflow from the previous frame
        vi = H['bmkhs_rotorInducedFlow'][rotorIndex][i]

        velModel = H['bmkhs_velModelSpace']
        if dissymetryOfLift:
            relWind = vector_multiply(velModel, -1.0)
        else:
            relWind = [0.0, 0.0, -velModel[2]]

        spanDir = vector_normalized(vector_diff(b, a))

        fromHub = vector_diff(liftPos, pos)
        radialVec = vector_diff(fromHub, vector_multiply(uVec, vector_dot_product(fromHub, uVec)))
        r = vector_magnitude(radialVec)
        rotRelWind = vector_multiply(vector_cross_product(uVec, spanDir), r * omega * -1.0)

        up = vector_normalized(vector_cross_product(spanDir, chordLine))
        if vector_dot_product(up, uVec) < 0.0:
            up = vector_multiply(up, -1.0)

        bodyRate = H['bmkhs_angVelModelSpace']
        rateDampScalar = H['bmkhs_rotorRateDampScalar']
        bodyRateWind = vector_multiply(vector_cross_product(bodyRate, vector_diff(liftPos, H['centerOfMass'])),
                                       rateDampScalar)

        localWind = vector_add(vector_add(relWind, rotRelWind), bodyRateWind)

        vc = vector_dot_product(uVec, relWind)
        localAxial = vector_multiply(uVec, vc)
        localLatVel = vector_magnitude(vector_diff(relWind, localAxial))

        inducedWind = vector_multiply(uVec, -vi)
        relWind = vector_add(localWind, inducedWind)

        relWindY = vector_dot_product(chordLine, relWind)
        relWindZ = vector_dot_product(up, relWind)
        relWind = vector_add(vector_multiply(chordLine, relWindY), vector_multiply(up, relWindZ))

        relWindNormalized = vector_normalized(relWind)
        AoA = vector_dot_product(chordLine, vector_multiply(relWindNormalized, -1.0))
        AoA = clamp(AoA, -1.0, 1.0)
        AoA = sqf_acos(AoA)

        if vector_dot_product(up, relWindNormalized) < 0.0:
            AoA = AoA * -1.0

        area = math_get_area(c, d, e, f)
        v = vector_magnitude(relWind)
        q = 0.5 * rho * area * (v * v)

        _, CL, CD = math_linear_interp(airfoilTable, AoA)

        lift = CL * q
        drag = min(CD, 0.15) * q

        r_in = bladeCutout + (spanInboard * (bladeLength - bladeCutout))
        r_out = bladeCutout + (spanOutboard * (bladeLength - bladeCutout))
        annularArea = math.pi * ((r_out * r_out) - (r_in * r_in))
        if annularArea > 0.0 and rho > 0.0:
            T_elem = lift * bladeScale
            denom = 2.0 * rho * annularArea * math.sqrt(((vc + vi) * (vc + vi)) + (localLatVel * localLatVel))
            raw = T_elem / denom if denom > 0.0 else 0.0
            viRaw = clamp(raw, -25.0, 25.0)
        else:
            viRaw = 0.0
        H['bmkhs_rotorInducedFlowAccum'][rotorIndex][i] += viRaw

        totalFlapMoment = totalFlapMoment + (lift * r)

        liftVector = vector_cross_product(relWindNormalized, up)
        liftVector = vector_cross_product(liftVector, relWindNormalized)
        liftDir = vector_normalized(liftVector)
        dragDir = vector_normalized(relWind)

        vTangential = r * omega
        P_induced = lift * vi
        P_profile = drag * vTangential
        totalPower = totalPower + (P_induced + P_profile)

        H['bmkhs_rotorThrustAccum'][rotorIndex] += lift * bladeScale

        liftVector = vector_multiply(liftDir, lift * deltaTime * bladeScale)
        dragVector = vector_multiply(dragDir, drag * deltaTime * bladeScale)

        #addForce [liftVector, liftPos] and [dragVector, liftPos] - the rig sums them, per second
        if deltaTime > 0.0:
            arm = vector_diff(liftPos, H['centerOfMass'])
            for impulse in (liftVector, dragVector):
                force = vector_multiply(impulse, 1.0 / deltaTime)
                H['rotorForceN'][rotorIndex] = vector_add(H['rotorForceN'][rotorIndex], force)
                H['rotorMomentNm'][rotorIndex] = vector_add(H['rotorMomentNm'][rotorIndex],
                                                            vector_cross_product(arm, force))

    H['bmkhs_rotorFlapMoment'][rotorIndex][bladeIndex] = totalFlapMoment

    H['bmkhs_rotorReactionTorque'][rotorIndex] = totalPower


def rotor(H, rotorIndex, pivot, rot, type_, dir_, numBlades, numElements, mastLength, gearRatio, flapTimeConst,
          inflowAlpha, delta3, airfoilTable, bladeCutout, bladeLength, bladeChord, bladeTwist, bladeMass,
          pitchMin, pitchMid, pitchMax, rollMin, rollMid, rollMax, collMin, collMid, collMax,
          animSource, hitPoint, dmgThreshold):
    """fn_rotor."""
    deltaTime = H['bmkhs_deltaTime']

    pitchFeather, rollFeather, collFeather = rotor_control(
        H, type_, pitchMin, pitchMid, pitchMax, rollMin, rollMid, rollMax, collMin, collMid, collMax)

    p, r, y = rot

    fVec = math_vector_rotate([0.0, 1.0, 0.0], p, r, y)
    rVec = math_vector_rotate([1.0, 0.0, 0.0], p, r, y)
    uVec = math_vector_rotate([0.0, 0.0, 1.0], p, r, y)

    pos = vector_add(pivot, vector_multiply(uVec, mastLength))
    xmsnRpm = H['bmkhs_xmsnOutputRpm']
    rpm = xmsnRpm / gearRatio
    omega = 0.0 if rpm == 0.0 else (2.0 * math.pi) * (rpm / 60.0)
    #animationSourcePhase animSource - rig: the phase turns at the rotor's own rpm
    H['rotorAnimPhase'][rotorIndex] = (H['rotorAnimPhase'][rotorIndex] + ((rpm / 60.0) * deltaTime)) % 1.0
    rotorAzimuth = H['rotorAnimPhase'][rotorIndex] * 360.0 * (1 if dir_ == CW else -1)

    H['rotorForceN'][rotorIndex] = [0.0, 0.0, 0.0]
    H['rotorMomentNm'][rotorIndex] = [0.0, 0.0, 0.0]

    if H['hitPointDamage'].get(hitPoint, 0.0) < dmgThreshold:

        H['bmkhs_rotorReactionTorque'][rotorIndex] = 0.0
        H['bmkhs_rotorThrustAccum'][rotorIndex] = 0.0
        for ei in range(int(numElements)):
            H['bmkhs_rotorInducedFlowAccum'][rotorIndex][ei] = 0.0

        beta0, a1, b1 = rotor_flap_dynamics(H, rotorIndex, type_, deltaTime, numBlades, omega, bladeMass,
                                            bladeLength, bladeCutout, flapTimeConst, rVec, fVec, uVec)

        if type_ == TAIL:
            beta0 = 0.0
            a1 = 0.0
            b1 = 0.0

        bladeSpacing = 360.0 / numBlades
        for bladeIndex in range(int(numBlades)):
            psi = rotorAzimuth + (bladeSpacing * bladeIndex)

            bladeDir = math_vector_rotate_around_axis(rVec, uVec, psi)
            chordDir = math_vector_rotate_around_axis(fVec, uVec, psi)
            if dir_ == CW:
                chordDir = vector_multiply(chordDir, -1)
                dirSign = -1
            else:
                dirSign = 1

            flapAngle = (beta0 + (a1 * vector_dot_product(bladeDir, fVec))
                         + (b1 * vector_dot_product(bladeDir, rVec)))
            featherAngle = dirSign * (rollFeather * sqf_cos(psi) + pitchFeather * sqf_sin(psi) + collFeather
                                      - (flapAngle * delta3))

            tipTwist = bladeTwist * dirSign

            rootIncidence = -tipTwist * 0.5 if type_ == MAIN else 0.0

            a_rootPos = vector_add(pos, math_vector_rotate_around_axis(
                vector_multiply(bladeDir, bladeCutout), chordDir, flapAngle))
            b_tipPos = vector_add(pos, math_vector_rotate_around_axis(
                vector_multiply(bladeDir, bladeLength), chordDir, flapAngle))
            c_rootLeadingEdge = vector_add(a_rootPos, math_vector_rotate_around_axis(
                vector_multiply(chordDir, bladeChord * 0.25), bladeDir, -(featherAngle + rootIncidence)))
            d_tipLeadingEdge = vector_add(b_tipPos, math_vector_rotate_around_axis(
                vector_multiply(chordDir, bladeChord * 0.25), bladeDir, -(featherAngle + rootIncidence + tipTwist)))
            e_tipTrailingEdge = vector_diff(b_tipPos, math_vector_rotate_around_axis(
                vector_multiply(chordDir, bladeChord * 0.75), bladeDir, -(featherAngle + rootIncidence + tipTwist)))
            f_rootTrailingEdge = vector_diff(a_rootPos, math_vector_rotate_around_axis(
                vector_multiply(chordDir, bladeChord * 0.75), bladeDir, -(featherAngle + rootIncidence)))

            H['bmkhs_rotorBladeAzimuth'][rotorIndex][bladeIndex] = psi

            rotor_blade(H, bladeIndex, rotorIndex, deltaTime, numElements, numBlades, pos, uVec, omega,
                        airfoilTable, bladeCutout, bladeLength, inflowAlpha, a_rootPos, b_tipPos,
                        c_rootLeadingEdge, d_tipLeadingEdge, e_tipTrailingEdge, f_rootTrailingEdge)

        viAccum = H['bmkhs_rotorInducedFlowAccum'][rotorIndex]
        for ei in range(int(numElements)):
            viRawAvg = viAccum[ei]
            viPrev = H['bmkhs_rotorInducedFlow'][rotorIndex][ei]
            viNext = lerp(viPrev, viRawAvg, inflowAlpha)
            H['bmkhs_rotorInducedFlow'][rotorIndex][ei] = viNext

        totalPower = H['bmkhs_rotorReactionTorque'][rotorIndex]
        reactionTorque = totalPower / omega if omega > 0.0 else 0.0
        reqEngTorque = reactionTorque / gearRatio if gearRatio > 0.0 else 0.0

        tqSmoothed = H['bmkhs_reqEngTorque'][rotorIndex]
        tqAlpha = 1.0 - math.exp(-deltaTime / 0.1)
        tqSmoothed = tqSmoothed + (reqEngTorque - tqSmoothed) * tqAlpha
        H['bmkhs_reqEngTorque'][rotorIndex] = tqSmoothed

        rotorThrust = H['bmkhs_rotorThrustAccum'][rotorIndex]
        H['bmkhs_rtrThrust'][rotorIndex] = rotorThrust

        Icm = (1.0 / 3.0) * bladeMass * (bladeLength * bladeLength)
        Iy = (1.0 / 12.0) * bladeMass * (bladeChord * bladeChord)
        Itot = Icm
        Jtot = (Iy + Itot) * numBlades
        H['bmkhs_rtrMoi'][rotorIndex] = Jtot

        #The torque reaction on the fuselage needs no addTorque: the blade forces carry it


def rotor_update(H):
    """fn_rotorUpdate."""
    numRotor = int(H['bmkhs_numRotors'])
    inflowAlpha = [0.05, 0.01]
    airfoilTable = [airfoil_get(H, name) for name in H['bmkhs_rotorAirfoil']]
    dmgThreshold = [0.99, 0.85]

    for rotorIndex in range(numRotor):
        rotor(H, rotorIndex,
              H['bmkhs_rotorPivot'][rotorIndex], H['bmkhs_rotorRotation'][rotorIndex],
              H['bmkhs_rotorType'][rotorIndex], H['bmkhs_rotorDirection'][rotorIndex],
              H['bmkhs_rotorNumBlades'][rotorIndex], H['bmkhs_rotorNumElements'][rotorIndex],
              H['bmkhs_rotorMastLength'][rotorIndex], H['bmkhs_rotorGearRatioArr'][rotorIndex],
              H['bmkhs_rotorFlapTimeConst'][rotorIndex], inflowAlpha[rotorIndex],
              H['bmkhs_rotorDelta3'][rotorIndex], airfoilTable[rotorIndex],
              H['bmkhs_rotorBladeCutout'][rotorIndex], H['bmkhs_rotorBladeLength'][rotorIndex],
              H['bmkhs_rotorBladeChordArr'][rotorIndex], H['bmkhs_rotorBladeTwist'][rotorIndex],
              H['bmkhs_rotorBladeMassArr'][rotorIndex],
              H['bmkhs_rotorPitchMin'][rotorIndex], H['bmkhs_rotorPitchMid'][rotorIndex],
              H['bmkhs_rotorPitchMax'][rotorIndex],
              H['bmkhs_rotorRollMin'][rotorIndex], H['bmkhs_rotorRollMid'][rotorIndex],
              H['bmkhs_rotorRollMax'][rotorIndex],
              H['bmkhs_rotorCollMin'][rotorIndex], H['bmkhs_rotorCollMid'][rotorIndex],
              H['bmkhs_rotorCollMax'][rotorIndex],
              H['bmkhs_rotorAnimSource'][rotorIndex], H['bmkhs_rotorHitPoint'][rotorIndex],
              dmgThreshold[rotorIndex])


# ---------------------------------------------------------------------------------------------
# fn_coreUpdateFlightModel - the rotors; the fuselage and wings are not on the engine's path
# ---------------------------------------------------------------------------------------------

def core_update_flight_model(H):
    if bmkhs_rotorModel == 1:
        rotor_update(H)
    else:
        simple_rotor_update(H)


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
        airfoil_variables(H, CFG)
        simple_rotor_variables(H, CFG)
        rotor_variables(H, CFG)
        H['bmkhs_xmsnOutputRpm'] = 0.0
        H['bmkhs_xmsnDeltaRpm'] = 0.0
        H['bmkhs_collectiveOutput'] = 0.0
        H['bmkhs_velModelSpace'] = [0.0, 0.0, 0.0]
        H['bmkhs_angVelModelSpace'] = [0.0, 0.0, 0.0]
        H['hubVelXY'] = H['hubVelZ'] = 0.0
        H['pedal'] = 0.0
        #fn_inputWrite's sums, which the blade element rotor's control reads. The rig flies no cyclic.
        H['bmkhs_fltCtrlPitchOut'] = H['bmkhs_fltCtrlRollOut'] = 0.0
        H['bmkhs_fltCtrlYawOut'] = H['bmkhs_fltCtrlCollOut'] = 0.0
        n = len(H['bmkhs_engines'])
        H['startSw'] = [0] * n
        H['pwrLvr'] = [0.0] * n
        H['pneuAvail'] = True
        H['starterSupplied'] = [True] * n
        H['governorPowered'] = [True] * n
        H['fuelAvail'] = [True] * n
        H['diag'] = [{} for _ in range(n)]
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
        H['bmkhs_fltCtrlCollOut'] = H['bmkhs_collectiveOutput']
        H['bmkhs_fltCtrlYawOut'] = H['pedal']
        engine_controller(H, CFG)
        transmission_update(H)
        core_update_flight_model(H)
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
        return sum(self.H['bmkhs_reqEngTorque']) / len(self.H['bmkhs_engines']) / self.H['bmkhs_engines'][0]['refTq']

    def all_engines(self):
        return tuple(range(len(self.H['bmkhs_engines'])))


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
                motorNg=0.250, hoverTq=0.94, hoverNr=1.01)   #motorNg: AH-64D dry crank, 25% (user, 2026-10-27)


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
FIT_PARAMS = ('starterTorque', 'startFuelBase', 'compDrag', 'compDragFloor')


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


def report():
    a = Heli()
    e = a.H['bmkhs_engines'][0]
    print('=' * 78)
    print('derived: refTq %.1f  idleNg %.4f  fuelIdle %.3f  fuelFly %.3f   rho %.4f  FAT %.0f'
          % (e['refTq'], e['idleNg'], e['fuelIdle'], e['fuelFly'], a.H['bmkhs_rho'], a.H['bmkhs_fat']))

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
        a.H['bmkhs_collectiveOutput'] = a.H['bmkhs_fltCtrlCollOut'] = mid
        a.H['bmkhs_deltaTime'] = DT
        #The blade element rotor's inflow and coning take longer to settle than the torque filter
        for _ in range(300 if bmkhs_rotorModel == 0 else 900):
            core_update_flight_model(a.H)
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


def main_rotor_thrust(H, main):
    """Main rotor thrust from the rotor model selected. The simple rotor's is its blade lift summed;
    the blade element rotor's is bmkhs_rtrThrust once its inflow and coning have settled."""
    if bmkhs_rotorModel == 0:
        return simple_rotor_thrust(H, main)
    H['bmkhs_velModelSpace'] = [0.0, H['hubVelXY'], H['hubVelZ']]
    H['bmkhs_deltaTime'] = DT
    for _ in range(900):
        rotor_update(H)
    return H['bmkhs_rtrThrust'][0]


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
    for _ in range(40 if bmkhs_rotorModel == 0 else 14):
        a.H['bmkhs_collectiveOutput'] = a.H['bmkhs_fltCtrlCollOut'] = (lo + hi) / 2.0
        if main_rotor_thrust(a.H, main) < need:
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


def pulled(pa, fat, single=False, coll=1.0, secs=100.0):
    """One engine's state settled at collective coll, at pressure altitude pa (ft) and FAT (C).
    Single engine: engine 2 lever OFF before the pull, on an aircraft that has one."""
    global BASE_FAT
    saved = BASE_FAT
    #The environment lapses at the standard rate from its base, so the base that lands on fat.
    BASE_FAT = fat + (ISA_LAPSE * pa * FEET_TO_METERS)
    try:
        alt = pa * FEET_TO_METERS
        a = Heli(baroAltM=alt)
        start_to(a, a.all_engines(), 'IDLE', 30.0, ARMA_DT, JITTER, baroAltM=alt)
        a.H['pwrLvr'] = [FLY] * len(a.all_engines())
        run_until(a, 40.0, ARMA_DT, JITTER, baroAltM=alt)
        if single and len(a.all_engines()) > 1:
            a.H['pwrLvr'][1] = 0.0
        run_until(a, 45.0, ARMA_DT, JITTER, baroAltM=alt)
        run_until(a, secs, ARMA_DT, JITTER,
                  each=lambda h: dict(coll=coll * clamp((h.t - 45.0) / NORMAL_PULL_SEC, 0.0, 1.0),
                                      baroAltM=alt))
    finally:
        BASE_FAT = saved
    return a


#Nr, as a fraction of governed, that still counts as "on speed" (the chart's 101% NR).
PERF_ON_SPEED = 0.995


def max_power(pa, fat, single=False, iters=10):
    """The highest collective that still holds Nr on speed, and what the engine is doing there.
    Held by is whichever limit sits closest: TGT or Ng."""
    lo, hi, best = 0.0, 1.0, None
    for _ in range(iters):
        mid = (lo + hi) / 2.0
        a = pulled(pa, fat, single, mid)
        if a.nrFrac() >= PERF_ON_SPEED:
            lo, best = mid, a
        else:
            hi = mid
    #Nothing holds Nr on speed - no point to report.
    if best is None:
        return None
    a = best
    e = a.H['bmkhs_engines'][0]
    tgtLim = e['maxTgtSe'] if a.H['bmkhs_isSingleEng'] else e['maxTgt']
    ngLim = min(e['ngLimitMax'], e['ngLimitBase'] + e['ngLimitSlope'] * a.H['bmkhs_fat'])
    tgtGap = (tgtLim - a.tgt()) / GT_TGT_LIMIT_BAND
    ngGap = (ngLim - a.ng()) / GT_NG_LIMIT_BAND
    why = 'TGT' if tgtGap < ngGap else 'Ng'
    return a.tq(), a.tgt(), a.ng(), ngLim, why


#Read off the user's charts: the FAT below which the limit is Ng and above which it is TGT.
CHART_SWITCH = {
    False: {0: -10, 2000: -10, 4000: -10, 6000: -10, 8000: -12, 10000: -12, 12000: -14,
            14000: -16, 16000: -16, 18000: -16, 20000: -18},
    True: {0: -4, 2000: -4, 4000: -4, 6000: -6, 8000: -6, 10000: -8, 12000: -8,
           14000: -10, 16000: -12, 18000: -13, 20000: -14},
}
SWITCH_PAS = (0, 2000, 4000, 6000, 8000, 10000)
SWITCH_FATS = tuple(range(-40, 56, 5))


def perf_report(jsonPath=None):
    print('\n' + '=' * 78)
    print('LIMIT SWITCH vs THE CHARTS - hover, highest collective holding Nr >= %.1f%% of governed'
          % (PERF_ON_SPEED * 100))
    print('   per PA: which limit binds at each FAT; the switch is the first FAT held by TGT')
    print('=' * 78)
    results = {}
    for single, name in ((False, 'DUAL ENGINE (10-min, TGT 867)'), (True, 'SINGLE ENGINE (2.5-min, TGT 896)')):
        print('\n   %s' % name)
        print('      PA   chart   rig |  ' + ' '.join('%4d' % f for f in SWITCH_FATS))
        detail = []
        for pa in SWITCH_PAS:
            held, sw = [], None
            for fat in SWITCH_FATS:
                r = max_power(pa, fat, single)
                if r is None:
                    held.append('-')
                    continue
                tq, tgt, ng, ngLim, why = r
                held.append(why)
                detail.append((pa, fat, tq, tgt, ng, ngLim, why))
                if sw is None and why == 'TGT':
                    sw = fat
            print('   %6d  %5d  %4s |  ' % (pa, CHART_SWITCH[single][pa], sw if sw is not None else '-')
                  + ' '.join('%4s' % {'TGT': 'T', 'Ng': 'N'}.get(w, '-') for w in held))
        print('\n      PA   FAT |   %TQ    TGT     Ng  ngLim  held')
        for pa, fat, tq, tgt, ng, ngLim, why in detail:
            print('   %6d  %4d | %5.1f  %5.0f  %.3f  %.3f  %s' % (pa, fat, tq * 100, tgt, ng, ngLim, why))
        results['SE' if single else 'DE'] = [
            dict(pa=pa, fat=fat, tq=tq, tgt=tgt, ng=ng, held=why)
            for pa, fat, tq, tgt, ng, ngLim, why in detail]
    if jsonPath:
        import json
        results['chartSwitch'] = {k: {str(pa): v for pa, v in CHART_SWITCH[s].items()}
                                  for k, s in (('DE', False), ('SE', True))}
        with open(jsonPath, 'w') as f:
            json.dump(results, f, indent=1)


def environment_report():
    """python engine.py env - what each environment setting gives: FAT by hour and height above
    ground on its average day, and its dawn and afternoon under other skies and winds."""
    global ENVIRONMENT
    saved = (ENVIRONMENT, BASE_ALT, BASE_FAT, dict(WEATHER))
    heights = [0, 100, 500, 1000, 2000, 4000, 8000]

    def at(hour, aglFt, terrainFt=0.0):
        WEATHER['dayTime'] = hour
        H = {'baroAltM': (terrainFt + aglFt) * FEET_TO_METERS, 'terrainAslM': terrainFt * FEET_TO_METERS}
        environment(H)
        return H

    try:
        set_environment('ISA_STD')
        H = at(12.0, 0)
        print('ENVIRONMENT')
        print('   Standard Day, sea level: FAT %.2f C, %.2f hPa, rho %.4f, dew point %.1f'
              % (H['bmkhs_fat'], H['bmkhs_pressure'], H['bmkhs_rho'], H['bmkhs_dewPoint']))
        H = at(3.0, 5000)
        print('   Standard Day, 5,000 ft at 03:00: FAT %.2f C, %.2f hPa, rho %.4f' % (H['bmkhs_fat'], H['bmkhs_pressure'], H['bmkhs_rho']))
        for name, row in ENVIRONMENTS.items():
            if name == 'ISA_STD':
                continue
            set_environment(name)
            baseAlt, baseFAT, swing, dewPoint, avgWind, avgCloud = row
            WEATHER.update(sunrise=5.0, sunset=19.0) if name.endswith('SUMMER') else WEATHER.update(sunrise=7.0, sunset=17.0)
            WEATHER.update(overcast=avgCloud, wind=[avgWind, 0.0, 0.0], fog=0.0)
            hours = [0.0, 3.0, WEATHER['sunrise'], 9.0, 12.0, 15.0, WEATHER['sunset'], 21.0]
            print()
            print('   %s - base %d ft, mean %g C, swing %g, dew point %g C, average wind %g m/s and cloud %d%%'
                  % (name, baseAlt, baseFAT, swing, dewPoint, avgWind, round(avgCloud * 100)))
            print('   FAT, C, on the average day, sunrise %02d:00 sunset %02d:00' % (WEATHER['sunrise'], WEATHER['sunset']))
            print('      ft AGL ' + ''.join('  %02d:00' % h for h in hours))
            for ft in heights:
                print('      %6d ' % ft + ''.join('%7.1f' % at(h, ft)['bmkhs_fat'] for h in hours))
            dawn, noon = at(WEATHER['sunrise'], 0), at(15.0, 0)
            print('      surface at dawn: dew point %.1f C, rho %.4f;  at 15:00: dew point %.1f C, rho %.4f'
                  % (dawn['bmkhs_dewPoint'], dawn['bmkhs_rho'], noon['bmkhs_dewPoint'], noon['bmkhs_rho']))
            print('      surface FAT under other skies and winds     dawn   15:00')
            for label, oc, w in (('clear, average wind', 0.0, avgWind), ('overcast, average wind', 1.0, avgWind),
                                 ('average cloud, calm', avgCloud, 0.0), ('average cloud, 5 m/s', avgCloud, 5.0),
                                 ('average cloud, 10 m/s', avgCloud, 10.0)):
                WEATHER.update(overcast=oc, wind=[w, 0.0, 0.0])
                print('         %-38s %6.1f  %6.1f' % (label, at(WEATHER['sunrise'], 0)['bmkhs_fat'], at(15.0, 0)['bmkhs_fat']))
            #Rain on the average afternoon: a mission that opens in it starts settled
            WEATHER.update(overcast=avgCloud, wind=[avgWind, 0.0, 0.0])
            dry = at(15.0, 0)
            print('      rain at 15:00, surface (dry: FAT %.1f, dew point %.1f, rho %.4f)' % (dry['bmkhs_fat'], dry['bmkhs_dewPoint'], dry['bmkhs_rho']))
            print('         settled in rain          mm/h     FAT  dew point     rho   to saturation')
            for label, slider in (('light, 17%', 1.0 / 6.0), ('moderate, 50%', 0.5), ('heavy, 83%', 5.0 / 6.0), ('full, 100%', 1.0)):
                WEATHER.update(rain=slider, time=0.0)
                globals().update(bmkhs_envRainT=-1.0)
                H = at(15.0, 0)
                print('         %-22s %6.1f  %6.1f     %6.1f  %.4f   %3d%%' % (
                    label, bmkhs_envRainRate, H['bmkhs_fat'], H['bmkhs_dewPoint'], H['bmkhs_rho'], round(bmkhs_envRain * 100)))
            #Moderate rain starting on dry air, stepped once a second, then stopping after an hour
            WEATHER.update(rain=0.0, time=0.0)
            globals().update(bmkhs_envRainT=-1.0)
            at(15.0, 0)
            line = '         moderate rain from dry, FAT after'
            for t in range(1, 10801):
                WEATHER.update(rain=0.5 if t <= 3600 else 0.0, time=float(t))
                environment_weather()
                if t in (300, 600, 1200, 1800, 3600):
                    line += '  %d min %.1f' % (t // 60, at(15.0, 0)['bmkhs_fat'])
                if t == 3600:
                    print(line)
                    line = '         and after it stops, FAT after   '
                if t in (4500, 5400, 7200, 9000, 10800):
                    line += '  %d min %.1f' % ((t - 3600) // 60, at(15.0, 0)['bmkhs_fat'])
            print(line)
            WEATHER.update(rain=0.0, time=0.0)
            globals().update(bmkhs_envRainT=-1.0)
    finally:
        ENVIRONMENT = saved[0]
        globals().update(BASE_ALT=saved[1], BASE_FAT=saved[2], bmkhs_envRain=0.0, bmkhs_envRainT=-1.0)
        WEATHER.clear()
        WEATHER.update(saved[3])


if __name__ == '__main__':
    import sys
    if 'bet' in sys.argv:
        #python engine.py bet [...] - the blade element rotors in place of the simple ones
        bmkhs_rotorModel = 1
        print('ROTOR MODEL: BET (bmkhs_rotorModel = 1)')
    if 'env' in sys.argv:
        #python engine.py env
        environment_report()
        sys.exit()
    if 'perf' in sys.argv:
        #python engine.py perf [results.json]
        i = sys.argv.index('perf')
        perf_report(sys.argv[i + 1] if len(sys.argv) > i + 1 else None)
        sys.exit()
    report()
    governed_report()
    loaded_report()
    autorotation_report()
    shutdown_from_flight_report()
    load_share_report()
