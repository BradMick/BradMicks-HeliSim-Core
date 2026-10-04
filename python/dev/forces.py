"""The simple rotors' forces and moments, outside Arma, and the control mixing they call for.
A 1:1 port of fn_simpleRotor's force path.

Run it:  set BMKHS_CONFIG=<pack>\\config\\bmkhs_config
         python python/dev/forces.py --cg X Y Z [--gwt KG]

--cg is the centre of mass in model space - `getCenterOfMass vehicle player` in game, REALISTIC.
Moments are taken about it, as fn_simpleRotor's own forces debug does (position - COM).

What it answers: at each collective, in a hover out of ground effect at 100% Nr, the pedal and
cyclic that null the pitch, roll and yaw moments. Those trims, less their value at the bottom of
the collective, are the control mixing a pack declares in ControlMixing (helisim_flightControls).

Not ported: fuselage and wings (no forces in a hover), ground effect, flapback and the induced
velocity terms (all inert at zero airspeed, zero vertical speed, out of ground effect).

If this file and the SQF ever disagree, the SQF is right and this file is the bug.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import engine as E  # noqa: E402 - reads BMKHS_CONFIG at import

MAIN, TAIL = E.MAIN, E.TAIL


# ---------------------------------------------------------------------------------------------
# Math - fn_mathVectorRotate, fn_mathVectorRotateAroundAxis, fn_mathLinearInterpFromCenter
# ---------------------------------------------------------------------------------------------

def _sin(d): return math.sin(math.radians(d))
def _cos(d): return math.cos(math.radians(d))


def math_vector_rotate(v, p, r, y):
    sinP, sinY, sinR = _sin(p), _sin(y), _sin(r)
    cosP, cosY, cosR = _cos(p), _cos(y), _cos(r)
    m = [[cosR * cosY + sinR * sinP * sinY, -cosR * sinY + sinR * sinP * cosY, sinR * cosP],
         [cosP * sinY, cosP * cosY, -sinP],
         [-sinR * cosY + cosR * sinP * sinY, sinR * sinY + cosR * sinP * cosY, cosR * cosP]]
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def math_vector_rotate_around_axis(v, a, ang):
    s, c = _sin(ang), _cos(ang)
    dot = v[0] * a[0] + v[1] * a[1] + v[2] * a[2]
    cross = [v[1] * a[2] - v[2] * a[1], v[2] * a[0] - v[0] * a[2], v[0] * a[1] - v[1] * a[0]]
    return [v[i] * c + cross[i] * s + a[i] * dot * (1 - c) for i in range(3)]


def math_linear_interp_from_center(inMin, inMax, x, outMin, outMid, outMax):
    if x < 0:
        return E.linear_conversion(inMin, 0, x, outMin, outMid, True)
    return E.linear_conversion(0, inMax, x, outMid, outMax, True)


def _add(a, b): return [a[i] + b[i] for i in range(3)]
def _sub(a, b): return [a[i] - b[i] for i in range(3)]
def _mul(a, k): return [a[i] * k for i in range(3)]
def _dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def _cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


# ---------------------------------------------------------------------------------------------
# fn_simpleRotorControl, fn_simpleRotor (force path)
# ---------------------------------------------------------------------------------------------

def simple_rotor_control(rotorType, pitch, roll, yaw, coll):
    """Control outputs for one rotor, from the summed inputs (pilot + trims + FMC + mixing)."""
    pitch = E.clamp(pitch, -1.0, 1.0)
    roll = E.clamp(roll, -1.0, 1.0)
    yaw = E.clamp(yaw, -1.0, 1.0)
    if rotorType == MAIN:
        return E.clamp(pitch, -1.0, 1.0), E.clamp(roll, -1.0, 1.0), E.clamp(coll, 0.0, 1.0)
    return 0.0, 0.0, E.clamp(yaw, -1.0, 1.0)


def simple_rotor_forces(r, ctl, rho, xmsnRpm, com):
    """One rotor's total force and moment about com, model space, N and Nm - the sum of the
    four blades' lift and drag vectors fn_simpleRotor hands to addForce, without the frame's dt.
    Hover: zero velocity, out of ground effect."""
    pitchOutput, rollOutput, collOutput = ctl
    rotorType = TAIL if r['type'].lower() == 'tail' else MAIN
    torqueSign = -1.0 if r['direction'].lower() == 'cw' else 1.0

    flapLon = math_linear_interp_from_center(-1, 1, pitchOutput, r['pitchFlapMin'], r['pitchFlapMid'], r['pitchFlapMax'])
    flapLat = math_linear_interp_from_center(-1, 1, rollOutput, r['rollFlapMin'], r['rollFlapMid'], r['rollFlapMax'])
    p, rr, y = r['rotation']
    fVec = math_vector_rotate([0.0, 1.0, 0.0], p, rr, y)
    rVec = math_vector_rotate([1.0, 0.0, 0.0], p, rr, y)
    uVec = math_vector_rotate([0.0, 0.0, 1.0], p, rr, y)
    pos = _add(r['pivot'], _mul(uVec, r['mastLength']))

    rpm = xmsnRpm / r['gearRatio']
    omega = 0.0 if rpm == 0.0 else (2.0 * math.pi) * (rpm / 60.0)
    bladeArea = r['bladeRadius'] * r['bladeChord']
    bladeRad75 = r['bladeRadius'] * 0.75
    bladeVel75 = omega * bladeRad75
    collCone = collOutput * r['coneAngle']
    q = 0.5 * rho * bladeArea * (bladeVel75 * bladeVel75)
    bladeScalar = r['numBlades'] / 4
    liftGrid = E.math_build_interp_grid(r['liftCoefTable'])
    dragGrid = E.math_build_interp_grid(r['dragCoefTable'])
    tableKey = E.simple_rotor_table_key(r, collOutput)

    F, M = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    for i in range(4):
        psi = i * 90.0
        locRVec = math_vector_rotate_around_axis(rVec, uVec, psi)
        locFVec = math_vector_rotate_around_axis(fVec, uVec, psi)
        #No flapback in a hover - advance ratio 0
        rollFlap = flapLat * _cos(psi)
        pitchFlap = flapLon * _sin(psi)
        bladeFlap = collCone + rollFlap + pitchFlap
        bladeOffset = math_vector_rotate_around_axis(_mul(locRVec, r['bladeRadius']), locFVec, bladeFlap)
        bladeThrustPos = _add(pos, _mul(bladeOffset, 0.75))

        liftCoef = E.math_linear_interp_2d(liftGrid, tableKey, 0.0)
        bladeLift = liftCoef * q * bladeScalar
        liftCoefDelta = (rollOutput * _cos(psi) * r['rollLiftCoef']) + (pitchOutput * _sin(psi) * r['pitchLiftCoef'])
        bladeLiftDelta = liftCoefDelta * q * bladeScalar
        dragCoef = E.math_linear_interp_2d(dragGrid, tableKey, 0.0)
        bladeDrag = dragCoef * q * bladeScalar

        #viScalar 1 (no vertical speed), gndEffScalar 1 (out of ground effect)
        bladeLift = bladeLift + bladeLiftDelta
        liftVec = math_vector_rotate_around_axis(_mul(uVec, bladeLift), locFVec, bladeFlap)
        dragVec = _mul(locFVec, -bladeDrag * torqueSign * r['reacTqScalar'])

        arm = _sub(bladeThrustPos, com)
        for v in (liftVec, dragVec):
            F = _add(F, v)
            M = _add(M, _cross(arm, v))
    return F, M


# ---------------------------------------------------------------------------------------------
# The aircraft in a hover
# ---------------------------------------------------------------------------------------------

def rotors():
    sr = E.CFG['SimpleRotors']
    return [sr['SimpleRotor%02d' % i] for i in range(1, int(E.CFG['numSimpleRotors']) + 1)]


def hover_rpm():
    """100% Nr: the governed power turbine, through the drivetrain (fn_stateRtrRpm's inverse)."""
    eng = E.CFG['Engines']['Engine01'] if 'Engines' in E.CFG else None
    if eng is None:
        raise SystemExit('No Engines >> Engine01 in helisim_engine.hpp')
    return eng['designRpm'] * eng['npFly']


def totals(pitch, roll, yaw, coll, com, rho, rpm):
    F, M = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    for r in rotors():
        rType = TAIL if r['type'].lower() == 'tail' else MAIN
        f, m = simple_rotor_forces(r, simple_rotor_control(rType, pitch, roll, yaw, coll), rho, rpm, com)
        F, M = _add(F, f), _add(M, m)
    return F, M


def trim_cyclic(yaw, coll, com, rho, rpm, guess=(0.0, 0.0), iters=40):
    """Cyclic pitch and roll that null the pitch and roll moments at this pedal and collective,
    held to their travel. Newton on a numerical Jacobian."""
    v = list(guess)

    def pr(x):
        return totals(x[0], x[1], yaw, coll, com, rho, rpm)[1][:2]
    for _ in range(iters):
        m = pr(v)
        if max(abs(c) for c in m) < 1e-3:
            break
        h = 1e-4
        j0 = [(a - b) / h for a, b in zip(pr([v[0] + h, v[1]]), m)]
        j1 = [(a - b) / h for a, b in zip(pr([v[0], v[1] + h]), m)]
        det = j0[0] * j1[1] - j1[0] * j0[1]
        if abs(det) < 1e-12:
            break
        dv = [(-m[0] * j1[1] + j1[0] * m[1]) / det, (-j0[0] * m[1] + m[0] * j0[1]) / det]
        v = [E.clamp(v[0] + dv[0], -1.0, 1.0), E.clamp(v[1] + dv[1], -1.0, 1.0)]
    return v


def trim(coll, com, rho, rpm, guess=(0.0, 0.0, 0.0), iters=50):
    """Pedal, cyclic pitch and cyclic roll that null the moment about com at this collective.
    Pedal by bisection on the yaw moment, cyclic solved inside each step. Everything stays in
    its travel; a residual moment means the controls ran out. Returns (yaw, pitch, roll,
    residual moment, total force)."""
    cyc = list(guess[1:])

    def mz(yaw):
        nonlocal cyc
        cyc = trim_cyclic(yaw, coll, com, rho, rpm, cyc)
        return totals(cyc[0], cyc[1], yaw, coll, com, rho, rpm)[1][2]
    lo, hi = -1.0, 1.0
    mlo, mhi = mz(lo), mz(hi)
    if mlo * mhi > 0:
        yaw = lo if abs(mlo) < abs(mhi) else hi
    else:
        for _ in range(iters):
            mid = 0.5 * (lo + hi)
            mm = mz(mid)
            if (mm > 0) == (mlo > 0):
                lo, mlo = mid, mm
            else:
                hi, mhi = mid, mm
        yaw = 0.5 * (lo + hi)
    cyc = trim_cyclic(yaw, coll, com, rho, rpm, cyc)
    F, M = totals(cyc[0], cyc[1], yaw, coll, com, rho, rpm)
    return yaw, cyc[0], cyc[1], M, F


# ---------------------------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------------------------

COLLS = [round(c * 0.1, 1) for c in range(11)]
PEDALS = [round(p * 0.2 - 1.0, 1) for p in range(11)]


def _table(rows, fmt='%.3f'):
    return '{\n' + ',\n'.join('    {%s, %s}' % ('%.1f' % a, fmt % b) for a, b in rows) + '\n}'


def report(com, gwt=None, fraction=1.0):
    H = {'baroAltM': 0.0}
    E.environment(H)
    rho, rpm = H['bmkhs_rho'], hover_rpm()
    print('Hover, OGE, ISA sea level, 100%% Nr (%.0f rpm at the shaft), COM %s' % (rpm, com))
    print('\n coll |  pedal   pitch    roll |  thrust N   |M| residual Nm')
    trims, g = [], (0.0, 0.0, 0.0)
    for c in COLLS:
        y, p, r, M, F = trim(c, com, rho, rpm, g)
        g = (y, p, r)
        trims.append((c, y, p, r))
        flag = '  <- beyond control travel' if max(abs(y), abs(p), abs(r)) > 1.0 else ''
        print(' %4.1f | %+.3f  %+.3f  %+.3f | %9.0f   %.2f%s' % (c, y, p, r, F[2], math.sqrt(_dot(M, M)), flag))
    if gwt:
        w = gwt * E.GRAVITY
        print('\nWeight %.0f N (%.0f kg)' % (w, gwt))

    #Yaw to pitch first: Core feeds pedal mixes the tail rotor's pitch command (pedal plus the
    #collective yaw mixes), as the -10's mixing unit sees tail rotor pitch. At a mid-hover
    #collective, the pitch cyclic that nulls the pitch moment at each command, roll trimmed.
    cMid = 0.5
    yMid, pMid, rMid, _, _ = trim(cMid, com, rho, rpm)
    y2p, y2r = [], []
    for yaw in PEDALS:
        v = trim_cyclic(yaw, cMid, com, rho, rpm, (pMid, rMid))
        y2p.append([yaw, v[0]])
        y2r.append([yaw, v[1]])
    p0 = [v for k, v in y2p if k == 0.0][0]
    r0 = [v for k, v in y2r if k == 0.0][0]
    y2p = [[k, v - p0] for k, v in y2p]
    y2r = [[k, v - r0] for k, v in y2r]
    y2pAt = lambda cmd: E.math_linear_interp(y2p, cmd)[1]
    y2rAt = lambda cmd: E.math_linear_interp(y2r, cmd)[1]

    f = fraction
    c0 = trims[0]
    print('\nMixing at %.0f%% of full compensation - trims less their value at collective 0:' % (100 * f))
    print('\nCollectiveToYaw (pedal):')
    print(_table([(c, f * (y - c0[1])) for c, y, p, r in trims]))
    print('\nCollectiveToRoll (cyclic, less what YawToRoll gives from the collective yaw mix):')
    print(_table([(c, f * ((r - c0[3]) - (y2rAt(y) - y2rAt(c0[1])))) for c, y, p, r in trims]))
    print('\nYawToPitch (cyclic, by tail rotor command at collective %.1f):' % cMid)
    print(_table([(k, f * v) for k, v in y2p]))
    print('\nYawToRoll (cyclic, by tail rotor command at collective %.1f) - not in the -10:' % cMid)
    print(_table([(k, f * v) for k, v in y2r]))
    print('\nCollective to pitch left over once yaw to pitch has acted - main rotor and CG, not')
    print('stabilator downwash (not modelled). For reference; the -10 mix is downwash:')
    print(_table([(c, (p - c0[2]) - (y2pAt(y) - y2pAt(c0[1]))) for c, y, p, r in trims]))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--cg', nargs=3, type=float, required=True, metavar=('X', 'Y', 'Z'))
    ap.add_argument('--gwt', type=float)
    ap.add_argument('--fraction', type=float, default=1.0,
                    help='share of the coupling the mixes cancel, 0..1')
    a = ap.parse_args()
    report(a.cg, a.gwt, a.fraction)
