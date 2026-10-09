"""The whole airframe's aerodynamic forces, outside Arma - the virtual wind tunnel. A 1:1 port of
fn_coreUpdateFlightModel's force path: every simple rotor (fn_simpleRotor, forward flight and
all), the fuselage (fn_fuselageFront, fn_fuselageTop, fn_fuselageSide) and every wing (fn_wing).

Run it:  set BMKHS_CONFIG=<pack>\\config\\bmkhs_config
         python python/dev/airframe.py trim --kt 134 --gwt 4923 [--pa 0 --fat 15] [--cyc 0.03]
         python python/dev/airframe.py sweep --gwt 4923 [--pa 0 --fat 15]
         python python/dev/airframe.py replay <Arma3.rpt> [--gwt 4923]

trim   - level, wings-level flight at an airspeed: the pitch attitude and collective at which the
         aerodynamic forces carry the weight and cancel the drag. Cyclic is an input (the pitch
         output the FMC and pilot hold, as the flight log records it), unless --bc and --cg are
         given, when it is solved too, for zero pitching moment.
sweep  - trim across the speed range, with the torque it takes.
replay - the steady stretches of a BMKHSLOG flight log, re-flown here at the logged attitude,
         speed and controls: what is left over is how far this file is from the game.

Frames. Rotor pivots are model space as declared; fuselage and wing panels are Object Builder
positions, and Core takes boundingCenter off them at init (fn_fuselageVariables,
fn_wingVariables). Forces do not depend on position at zero rotation rate, so a force trim needs
neither --bc nor --cg; moments do. Both come from the game:
    boundingCenter vehicle player        getCenterOfMass vehicle player

The stabilator (a surface named "stabilator") sits where fn_wing's schedule settles it - trim is
steady, so the 1.5 s lerp has arrived - undamaged and on a live DC bus.

Not ported: ground effect (out of ground effect only), damage, CASUAL's overrides (REALISTIC
only), rotation-rate terms (trim is steady, so they are zero).

If this file and the SQF ever disagree, the SQF is right and this file is the bug.
"""
import argparse
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import engine as E  # noqa: E402 - reads BMKHS_CONFIG at import
import forces as FO  # noqa: E402
import flightlog as FL  # noqa: E402

_add, _sub, _mul, _dot, _cross = FO._add, FO._sub, FO._mul, FO._dot, FO._cross
_sin, _cos = FO._sin, FO._cos
MPS_TO_KNOTS = 1.94384
KNOTS_TO_MPS = 0.51444
FT = E.FEET_TO_METERS


def _norm(v):
    m = math.sqrt(_dot(v, v))
    return [0.0, 0.0, 0.0] if m == 0.0 else _mul(v, 1.0 / m)


def _mag(v): return math.sqrt(_dot(v, v))


def _atan2(a, b):
    """SQF's a atan2 b, in degrees."""
    return math.degrees(math.atan2(a, b))


# ---------------------------------------------------------------------------------------------
# Config - the pack's own files, read the way fn_*Variables reads them
# ---------------------------------------------------------------------------------------------

def _load(name):
    return E.parse_hpp(E.AH64_CONFIG + '\\' + name)


AIRFOILS = {a['name']: a['table'] for a in _load('helisim_airfoils.hpp')['Airfoils'].values()}
_FUS = _load('helisim_fuselage.hpp')
_WINGS = _load('helisim_wings.hpp')


def fuselage_sets(bc):
    """fn_fuselageVariables - panel sets by name, panels less boundingCenter."""
    out = {}
    for p in _FUS['FuselagePanels'].values():
        out[p['name']] = dict(facing=p['facing'].lower(), dragCoefTable=p['dragCoefTable'],
                              panels=[[_sub(v, bc) for v in q] for q in p['panels']])
    return out


def wings(bc):
    """fn_wingVariables."""
    out = []
    for i, w in enumerate(_WINGS['Wings'].values()):
        out.append(dict(name=w.get('name') or 'wing %d' % (i + 1), facing=w['facing'].lower(),
                        numElements=int(w['numElements']), chordLinePos=w['chordLinePos'],
                        airfoilTable=AIRFOILS[w['airfoil']],
                        panels=[[_sub(v, bc) for v in q] for q in w['panels']]))
    return out


def get_area(a, b, c, d):
    """fn_mathGetArea."""
    ab, bc_, cd, da = _mag(_sub(b, a)), _mag(_sub(b, c)), _mag(_sub(c, d)), _mag(_sub(a, d))
    s = (ab + bc_ + cd + da) * 0.5
    return math.sqrt(max((s - ab) * (s - bc_) * (s - cd) * (s - da), 0.0))


def density(paFt, fat):
    """fn_environment's density at a pressure altitude and the temperature there."""
    exp_ = (-E.GRAVITY * E.MOLAR_MASS_OF_AIR * (paFt * FT)
            / (E.UNIVERSAL_GAS_CONSTANT * (fat + E.DEG_C_TO_KELVIN)))
    pressure = ((E.SEA_LEVEL_PRESSURE * E.IN_MG_TO_HPA / 0.01) * math.exp(exp_)) * 0.01
    return (pressure / 0.01) / (287.05 * (fat + E.DEG_C_TO_KELVIN))


# ---------------------------------------------------------------------------------------------
# fn_simpleRotor - the force path in full, and the rotor torque it hands on
# ---------------------------------------------------------------------------------------------

def simple_rotor(r, ctl, vel, rho, xmsnRpm, com):
    """One rotor: total force and moment about com (model space, N and Nm, no dt) and its shaft
    torque (Nm at the rotor, as fn_simpleRotor sums it). vel is bmkhs_velModelSpace."""
    pitchOutput, rollOutput, collOutput = ctl
    isTail = r['type'].lower() == 'tail'
    isCw = r['direction'].lower() == 'cw'

    flapLon = FO.math_linear_interp_from_center(-1, 1, pitchOutput, r['pitchFlapMin'], r['pitchFlapMid'], r['pitchFlapMax'])
    flapLat = FO.math_linear_interp_from_center(-1, 1, rollOutput, r['rollFlapMin'], r['rollFlapMid'], r['rollFlapMax'])
    p, rr, y = r['rotation']
    fVec = FO.math_vector_rotate([0.0, 1.0, 0.0], p, rr, y)
    rVec = FO.math_vector_rotate([1.0, 0.0, 0.0], p, rr, y)
    uVec = FO.math_vector_rotate([0.0, 0.0, 1.0], p, rr, y)
    pos = _add(r['pivot'], _mul(uVec, r['mastLength']))

    velX, velY, velZ = _dot(vel, rVec), _dot(vel, fVec), _dot(vel, uVec)
    velXY = min(math.hypot(velX, velY), E.VEL_VNE)

    rpm = xmsnRpm / r['gearRatio']
    omega = 0.0 if rpm == 0.0 else (2.0 * math.pi) * (rpm / 60.0)
    bladeArea = r['bladeRadius'] * r['bladeChord']
    tipVel = omega * r['bladeRadius']
    bladeRad75 = r['bladeRadius'] * 0.75
    bladeVel75 = omega * bladeRad75
    collCone = collOutput * r['coneAngle']
    tableKey = E.simple_rotor_table_key(r, collOutput)

    advanceRatio = velXY / tipVel if tipVel > 1.0 else 0.0
    windAzimuth = _atan2(velX, velY) if velXY > 0.01 else 0.0
    fbRollAngle = r['flapBackRollMax'] * advanceRatio
    fbPitchAngle = r['flapBackPitchMax'] * advanceRatio

    liftGrid = E.math_build_interp_grid(r['liftCoefTable'])
    dragGrid = E.math_build_interp_grid(r['dragCoefTable'])
    liftCoef = E.math_linear_interp_2d(liftGrid, tableKey, velXY)
    axialVel = velZ * (-1.0 if liftCoef < 0 else 1.0)
    viDenom = E.linear_conversion(-7.62, -19.30, axialVel, E.VEL_VRS, E.VEL_VRS * 0.1, True)
    torqueSign = -1.0 if isCw else 1.0
    q = 0.5 * rho * bladeArea * (bladeVel75 * bladeVel75)
    bladeScalar = r['numBlades'] / 4
    dragCoef = E.math_linear_interp_2d(dragGrid, tableKey, velXY)

    F, M, rotorTorque = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0
    for i in range(4):
        psi = i * 90.0
        locRVec = FO.math_vector_rotate_around_axis(rVec, uVec, psi)
        locFVec = FO.math_vector_rotate_around_axis(fVec, uVec, psi)
        fbRoll = fbRollAngle * _cos(psi - windAzimuth) * (-1.0 if isCw else 1.0)
        fbPitch = fbPitchAngle * _sin(psi - windAzimuth)
        bladeFlap = collCone + (flapLat * _cos(psi)) + fbRoll + (flapLon * _sin(psi)) + fbPitch
        bladeOffset = FO.math_vector_rotate_around_axis(_mul(locRVec, r['bladeRadius']), locFVec, bladeFlap)
        bladeThrustPos = _add(pos, _mul(bladeOffset, 0.75))

        bladeLift = liftCoef * q * bladeScalar
        liftCoefDelta = (rollOutput * _cos(psi) * r['rollLiftCoef']) + (pitchOutput * _sin(psi) * r['pitchLiftCoef'])
        bladeLiftDelta = liftCoefDelta * q * bladeScalar
        bladeDrag = dragCoef * q * bladeScalar
        rotorTorque += bladeDrag * bladeRad75
        if not isTail:
            rotorTorque -= r['autoTorque'] * max(-velZ, 0.0) * (1.0 - collOutput) * bladeScalar

        if axialVel < -E.VEL_VRS and velXY < E.VEL_ETL:
            viScalar = 0.0
        else:
            viScalar = 1 - (axialVel / viDenom)
        gndEffScalar = 1.0  # out of ground effect

        bladeLift = (bladeLift * viScalar * gndEffScalar) + bladeLiftDelta
        liftVec = FO.math_vector_rotate_around_axis(_mul(uVec, bladeLift), locFVec, bladeFlap)
        dragVec = _mul(locFVec, -bladeDrag * torqueSign * r['reacTqScalar'])

        arm = _sub(bladeThrustPos, com)
        for v in (liftVec, dragVec):
            F = _add(F, v)
            M = _add(M, _cross(arm, v))
    return F, M, rotorTorque


# ---------------------------------------------------------------------------------------------
# fn_fuselageFront, fn_fuselageTop, fn_fuselageSide
# ---------------------------------------------------------------------------------------------

def _quad(q):
    a, b, c, d = q
    f = _sub(d, _mul(_sub(d, c), 0.5))
    g = _sub(a, _mul(_sub(a, b), 0.5))
    e = _add(g, _mul(_sub(f, g), 0.5))
    n = _norm(_cross(_sub(c, a), _sub(d, b)))
    return a, b, c, d, e, n


def fuselage_front(s, vel, rho, paFt, com):
    F = [0.0, 0.0, 0.0]
    facing = [0.0, -1.0, 0.0] if s['facing'] == 'backward' else [0.0, 1.0, 0.0]
    for q in s['panels']:
        a, b, c, d, e, vecFwd = _quad(q)
        if _dot(vecFwd, facing) < 0.0:
            vecFwd = _mul(vecFwd, -1.0)
        velFwd = _dot(vel, vecFwd)
        v = E.clamp(velFwd, -E.VEL_VNE, E.VEL_VNE)
        cd = E.math_linear_interp(s['dragCoefTable'], paFt)[1]
        drag = cd * 0.5 * rho * get_area(a, b, c, d) * (v * v)
        F = _add(F, _mul(vecFwd, drag * (1.0 if velFwd < 0.0 else -1.0)))
    return F, [0.0, 0.0, 0.0]  # applied at the CoM


def _lifting_panels(s, vel, rho, com, side):
    """fn_fuselageTop (side False) and fn_fuselageSide (side True) - identical but for the
    facing, which wind components they keep, and where the drag coefficient comes from."""
    F, M = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    if side:
        facing = [-1.0, 0.0, 0.0] if s['facing'] == 'left' else [1.0, 0.0, 0.0]
    else:
        facing = [0.0, 0.0, -1.0] if s['facing'] == 'down' else [0.0, 0.0, 1.0]
    table = AIRFOILS[_FUS['fuselageAirfoil']]
    negVel = _mul(vel, -1.0)
    for q in s['panels']:
        a, b, c, d, e, up = _quad(q)
        if _dot(up, facing) < 0.0:
            up = _mul(up, -1.0)
        chordLine = _norm(_sub([0.0, 1.0, 0.0], _mul(up, up[1])))
        #Zero rotation rate: the local wind terms are zero.
        relWind = [negVel[0], negVel[1], 0.0] if side else [0.0, negVel[1], negVel[2]]
        rwN = _norm(relWind)
        aoa = _atan2(_dot(rwN, up), _dot(rwN, chordLine))
        area = get_area(a, b, c, d)
        cl = E.math_linear_interp(table, aoa)[1]
        vv = min(_mag(relWind), E.VEL_VNE)
        lift = cl * 0.5 * rho * area * (vv * vv)
        #fn_fuselageSide reads its drag coefficient from dragCoefTable keyed by AoA; the top
        #from the airfoil.
        cd = E.math_linear_interp(s['dragCoefTable'], aoa)[1] if side else E.math_linear_interp(table, aoa)[2]
        relWindN = _dot(negVel, up)
        drag = cd * 0.5 * rho * area * (relWindN * relWindN)
        liftVec = _mul(_norm(_cross(_cross(rwN, up), rwN)), lift)
        dragVec = _mul(_norm(relWind), -drag)
        arm = _sub(e, com)
        for v in (liftVec, dragVec):
            F = _add(F, v)
            M = _add(M, _cross(arm, v))
    return F, M


# ---------------------------------------------------------------------------------------------
# fn_wing
# ---------------------------------------------------------------------------------------------

_FACINGS = {'right': [1.0, 0.0, 0.0], 'left': [-1.0, 0.0, 0.0], 'forward': [0.0, 1.0, 0.0],
            'backward': [0.0, -1.0, 0.0], 'up': [0.0, 0.0, 1.0], 'down': [0.0, 0.0, -1.0]}


#heliSimStabTable's columns, as fn_wing keys them: 30 to 180 kt
_STAB_SPEEDS = (15.43, 20.58, 25.72, 29.58, 41.16, 42.44, 51.44, 59.16, 61.73, 72.02, 77.17, 82.31, 84.88, 92.60)


def stabilator_theta(vel, coll):
    """fn_wing's stabilator incidence (deg), settled."""
    row = E.math_linear_interp(_WINGS['heliSimStabTable'], coll)
    vel2D = E.clamp(vel[1], 0.0, 180.0 * KNOTS_TO_MPS)  #fn_stateVelocities
    return E.math_linear_interp([[k, row[i + 1]] for i, k in enumerate(_STAB_SPEEDS)], vel2D)[1]


def wing(w, vel, rho, com, coll):
    F, M = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    facing = _FACINGS[w['facing']]
    n, cp = w['numElements'], w['chordLinePos']
    isStab = w['name'] == 'stabilator'
    stabTheta = stabilator_theta(vel, coll) if isStab else 0.0
    for A, B, C, D in w['panels']:
        if isStab:
            #Trailing edges rotate about the leading edge
            D = _sub(A, FO.math_vector_rotate_around_axis(_sub(A, D), [1.0, 0.0, 0.0], stabTheta))
            C = _sub(B, FO.math_vector_rotate_around_axis(_sub(B, C), [1.0, 0.0, 0.0], stabTheta))
        for j in range(n):
            a = _add(A, _mul(_sub(B, A), j / n))
            b = _add(A, _mul(_sub(B, A), (j + 1) / n))
            c = _add(D, _mul(_sub(C, D), (j + 1) / n))
            d = _add(D, _mul(_sub(C, D), j / n))
            f = _add(d, _mul(_sub(a, d), 1.0 - cp))
            g = _add(c, _mul(_sub(b, c), 1.0 - cp))
            e = _add(f, _mul(_sub(g, f), 0.5))
            chordLine = _norm(_sub(_add(a, _mul(_sub(b, a), 0.5)), _add(d, _mul(_sub(c, d), 0.5))))
            relWind = _mul(vel, -1.0)  # zero rotation rate: no local wind
            up = _norm(_cross(_norm(_sub(g, f)), chordLine))
            if _dot(up, facing) < 0.0:
                up = _mul(up, -1.0)
            relWind = _add(_mul(chordLine, _dot(chordLine, relWind)), _mul(up, _dot(up, relWind)))
            rwN = _norm(relWind)
            aoa = math.degrees(math.acos(E.clamp(_dot(chordLine, _mul(rwN, -1.0)), -1.0, 1.0)))
            if _dot(up, rwN) < 0.0:
                aoa = -aoa
            area = get_area(a, b, c, d)
            row = E.math_linear_interp(w['airfoilTable'], aoa)
            vv = _mag(relWind)
            lift = row[1] * 0.5 * rho * area * (vv * vv)
            drag = row[2] * 0.5 * rho * area * (vv * vv)
            liftVec = _mul(_norm(_cross(_cross(rwN, up), rwN)), lift)
            dragVec = _mul(rwN, -drag)
            arm = _sub(e, com)
            for v in (liftVec, dragVec):
                F = _add(F, v)
                M = _add(M, _cross(arm, v))
    return F, M


# ---------------------------------------------------------------------------------------------
# The aircraft
# ---------------------------------------------------------------------------------------------

class Airframe:
    def __init__(self, bc=(0.0, 0.0, 0.0), cg=(0.0, 0.0, 0.0)):
        self.bc, self.cg = list(bc), list(cg)
        self.fus = fuselage_sets(self.bc)
        self.wings = wings(self.bc)
        self.rotors = FO.rotors()
        e = E.CFG['Engines']['Engine01']
        self.rpm = e['designRpm'] * e['npFly']
        self.refTq = (e['powerKw'] * 1000) / (e['designRpm'] * e['npFly'] * 0.10472)
        self.numEngines = 2

    def forces(self, vel, pitch, roll, yaw, coll, rho, paFt, parts=False):
        """Sum of every aerodynamic force and moment about the CoM, model space, and the gauge
        torque per engine (fraction). parts returns each contributor too."""
        out = {}
        tq = 0.0
        for r in self.rotors:
            t = FO.TAIL if r['type'].lower() == 'tail' else FO.MAIN
            f, m, rt = simple_rotor(r, FO.simple_rotor_control(t, pitch, roll, yaw, coll), vel, rho, self.rpm, self.cg)
            out[r['type'] + ' rotor'] = (f, m)
            tq += rt / r['gearRatio']
        out['fuse front'] = fuselage_front(self.fus['fuselageFront'], vel, rho, paFt, self.cg)
        out['fuse top'] = _lifting_panels(self.fus['fuselageTop'], vel, rho, self.cg, False)
        out['fuse side'] = _lifting_panels(self.fus['fuselageSide'], vel, rho, self.cg, True)
        for w in self.wings:
            out[w['name']] = wing(w, vel, rho, self.cg, coll)
        F, M = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        for f, m in out.values():
            F, M = _add(F, f), _add(M, m)
        gauge = tq / (self.numEngines * self.refTq)
        return (F, M, gauge, out) if parts else (F, M, gauge)


def body_state(v, pitchDeg):
    """Level flight, wings level: model-space velocity and the unit weight direction for an
    airspeed v (m/s) and a pitch attitude (deg, nose up positive - BIS_fnc_getPitchBank)."""
    vel = [0.0, v * _cos(pitchDeg), -v * _sin(pitchDeg)]
    down = [0.0, -_sin(pitchDeg), -_cos(pitchDeg)]
    return vel, down


def trim(af, v, gwt, rho, paFt, cyc=0.0, cycLR=0.0, pedal=0.0, solveCyc=False, guess=(-3.0, 0.6, 0.0)):
    """Pitch attitude and collective (and cyclic pitch, with solveCyc) at which the force along
    and normal to the body, in the longitudinal plane, nets to zero with the weight (and the
    pitching moment, with solveCyc). Newton on a numerical Jacobian."""
    W = gwt * E.GRAVITY
    x = list(guess[:3] if solveCyc else guess[:2])

    def res(x):
        th, c = x[0], x[1]
        cp = x[2] if solveCyc else cyc
        vel, down = body_state(v, th)
        F, M, _ = af.forces(vel, cp, cycLR, pedal, c, rho, paFt)
        r = [(F[1] + W * down[1]) / W, (F[2] + W * down[2]) / W]
        if solveCyc:
            r.append(M[0] / (W * 1.0))
        return r
    for _ in range(60):
        r0 = res(x)
        if max(abs(a) for a in r0) < 1e-6:
            break
        n = len(x)
        J = []
        for k in range(n):
            h = 1e-3 if k == 0 else 1e-5
            xx = list(x)
            xx[k] += h
            J.append([(a - b) / h for a, b in zip(res(xx), r0)])
        #Solve J^T-columns system: sum_k J[k][i] dx_k = -r0[i]
        A = [[J[k][i] for k in range(n)] for i in range(n)]
        dx = _solve(A, [-a for a in r0])
        if dx is None:
            break
        x = [x[0] + dx[0], E.clamp(x[1] + dx[1], 0.0, 1.0)] + ([E.clamp(x[2] + dx[2], -1.0, 1.0)] if solveCyc else [])
    th, c = x[0], x[1]
    cp = x[2] if solveCyc else cyc
    vel, down = body_state(v, th)
    F, M, g = af.forces(vel, cp, cycLR, pedal, c, rho, paFt)
    ok = max(abs(a) for a in res(x)) < 1e-4
    return dict(pitch=th, coll=c, cyc=cp, tq=g, F=F, M=M, ok=ok, bodyKt=vel[1] * MPS_TO_KNOTS)


def _solve(A, b):
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(A)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(m[r][i]))
        if abs(m[p][i]) < 1e-12:
            return None
        m[i], m[p] = m[p], m[i]
        for r in range(n):
            if r != i:
                k = m[r][i] / m[i][i]
                m[r] = [a - k * bb for a, bb in zip(m[r], m[i])]
    return [m[i][n] / m[i][i] for i in range(n)]


# ---------------------------------------------------------------------------------------------
# Reports
# ---------------------------------------------------------------------------------------------

def report_trim(af, a, kts):
    rho = density(a.pa, a.fat)
    print('Level flight, %.0f kg, %.0f ft PA, %.0f C (rho %.4f), %s' % (
        a.gwt, a.pa, a.fat, rho, 'cyclic solved for zero pitch moment' if a.solve else 'cyclic pitch %.3f' % a.cyc))
    print('  kt(TAS)  kt(body)  pitch   coll    cyc    TQ%   ok')
    g = (-1.0, 0.6, 0.0)
    for kt in kts:
        t = trim(af, kt / MPS_TO_KNOTS, a.gwt, rho, a.pa, a.cyc, a.cycLR, a.pedal, a.solve, g)
        g = (t['pitch'], t['coll'], t['cyc'])
        print('  %6.0f   %6.1f   %+5.2f  %.3f  %+.3f  %5.1f   %s' % (
            kt, t['bodyKt'], t['pitch'], t['coll'], t['cyc'], t['tq'] * 100, 'yes' if t['ok'] else 'NO'))
    return t


def report_parts(af, a, kt):
    rho = density(a.pa, a.fat)
    t = trim(af, kt / MPS_TO_KNOTS, a.gwt, rho, a.pa, a.cyc, a.cycLR, a.pedal, a.solve)
    vel, _ = body_state(kt / MPS_TO_KNOTS, t['pitch'])
    F, M, g, parts = af.forces(vel, t['cyc'], a.cycLR, a.pedal, t['coll'], rho, a.pa, parts=True)
    print('\nForces at %.0f kt trim (model space N: x right, y forward, z up):' % kt)
    for k, (f, m) in parts.items():
        print('  %-22s %+9.0f %+9.0f %+9.0f' % (k, f[0], f[1], f[2]))
    print('  %-22s %+9.0f %+9.0f %+9.0f' % ('TOTAL aero', F[0], F[1], F[2]))


def input_interp(current, previous):
    """fn_inputGetInterp - the stick deflected from the force trim position, as a fraction of the
    way from the trim to the stop it points at."""
    target = 1.0 if current > 0.0 else (-1.0 if current < 0.0 else previous)
    return E.clamp(previous + (target - previous) * abs(current), -1.0, 1.0)


def replay(af, path, gwt):
    """Steady stretches of the newest flight log, re-flown at the logged state."""
    rows = [r for _, t in FL.tables(path) for r in t]
    fl = lambda r, k: float(r[k])
    seg, segs = [], []
    for i in range(1, len(rows)):
        r, p = rows[i], rows[i - 1]
        steady = (abs(fl(r, 'kts') - fl(p, 'kts')) < 0.3 and abs(fl(r, 'vsFpm')) < 150 and fl(r, 'radAltFt') > 30
                  and abs(fl(r, 'coll') - fl(p, 'coll')) < 0.003 and fl(r, 'kts') > 30)
        if steady:
            seg.append(r)
        else:
            if len(seg) >= 20:
                segs.append(seg)
            seg = []
    if len(seg) >= 20:
        segs.append(seg)
    W = gwt * E.GRAVITY
    print('%s: %d rows, %d steady stretches (>= 2 s)' % (os.path.basename(path), len(rows), len(segs)))
    print('  t        kt(body) pitch  coll   cycP   TQ%log  TQ%rig  fwd res  up res   (residual, g)')
    for s in segs:
        m = lambda k: sum(fl(r, k) for r in s) / len(s)
        pitch = m('pitch')
        vy = m('kts') / MPS_TO_KNOTS
        vz = -m('vsFpm') / 196.85 * 0.0  # level: vertical speed small, taken as zero
        vel = [0.0, vy, -vy * math.tan(math.radians(pitch))]
        down = [0.0, -_sin(pitch), -_cos(pitch)]
        #fn_simpleRotorControl: the stick moves away from the force trim, then the FMC outputs add
        def ctl(stick, trim, *adds):
            return E.clamp(sum(input_interp(fl(r, stick), fl(r, trim)) + sum(fl(r, k) for k in adds)
                               for r in s) / len(s), -1.0, 1.0)
        cycP = ctl('cyc', 'ftPitch', 'sasP', 'attP', 'mixP')
        cycR = ctl('cycLR', 'ftRoll', 'sasR', 'attR', 'mixR')
        yaw = ctl('pedal', 'ftYaw', 'sasY', 'hdgY', 'mixY')
        paFt = m('altFt')
        rho = density(paFt, 15.0 - 2.0 * paFt / 1000.0)
        F, M, g = af.forces(vel, cycP, cycR, yaw, m('coll'), rho, paFt)
        print('  %5.1f-%5.1f %6.1f  %+5.2f  %.3f  %+.3f  %5.1f   %5.1f   %+.4f  %+.4f' % (
            fl(s[0], 't'), fl(s[-1], 't'), m('kts'), pitch, m('coll'), cycP, m('tq1') * 100, g * 100,
            (F[1] + W * down[1]) / W, (F[2] + W * down[2]) / W))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['trim', 'sweep', 'replay'])
    ap.add_argument('log', nargs='?')
    ap.add_argument('--kt', type=float, default=134.0)
    ap.add_argument('--gwt', type=float, default=4923.0)
    ap.add_argument('--pa', type=float, default=0.0)
    ap.add_argument('--fat', type=float, default=15.0)
    ap.add_argument('--cyc', type=float, default=0.0, help='cyclic pitch output held')
    ap.add_argument('--cycLR', type=float, default=0.0)
    ap.add_argument('--pedal', type=float, default=0.0)
    ap.add_argument('--bc', nargs=3, type=float, default=None)
    ap.add_argument('--cg', nargs=3, type=float, default=None)
    a = ap.parse_args()
    a.solve = a.bc is not None and a.cg is not None
    af = Airframe(a.bc or (0.0, 0.0, 0.0), a.cg or (0.0, 0.0, 0.0))
    if a.mode == 'trim':
        report_trim(af, a, [a.kt])
        report_parts(af, a, a.kt)
    elif a.mode == 'sweep':
        report_trim(af, a, [20, 40, 60, 70, 80, 90, 100, 110, 120, 130, 134, 140, 150, 160])
    else:
        path = a.log or max(glob.glob(os.path.expandvars(r'%LOCALAPPDATA%\Arma 3\*.rpt')), key=os.path.getmtime)
        replay(af, path, a.gwt)
