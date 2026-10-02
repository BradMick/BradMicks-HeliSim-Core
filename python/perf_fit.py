"""Fast steady-state max-power solver for the corrected-cycle engine, and a fitter that tunes its physics
coefficients against the AH-64D TM Maximum Torque Available charts, held to the ISA sea-level anchors.

Steady state at maximum power: the spool balances (fuel = compressor load), TGT is inlet temperature plus
the fuel-air rise, the power turbine makes airflow x TGT x its share. Max torque is where the first limit
binds - TGT (867 DE / 896 SE), the Ng limit schedule, or the fuel orifice.

Run it:  python python/perf_fit.py [restarts]
"""
import math
import random
import sys

G, M, R = 9.806, 0.0289644, 8.31432
T0 = 288.15
PT_EFF = 0.92
FUEL_FLY = 3.136
NG_LIM = (1.022, 1.01436, 0.0019091)

#TM charts, read off the user's images (+-1 %Q). Reference for SHAPE - the anchors are the hard numbers.
TM = {
    False: {0: {-40: 131, -20: 135, -10: 137, 20: 128, 40: 117},
            2000: {-40: 121, -20: 125, -10: 128, 20: 119, 40: 110},
            4000: {-40: 114, -20: 117, -10: 119, 20: 110, 40: 104},
            8000: {-40: 98, -20: 100, -10: 103, 20: 97, 40: 91},
            10000: {-40: 90, -20: 92, -10: 94, 20: 89, 40: 85}},
    True: {0: {-40: 131, -20: 136, -10: 137, 20: 134, 40: 126},
           2000: {-40: 123, -20: 126, -5: 129, 20: 123, 40: 120},
           4000: {-40: 116, -20: 118, -5: 120, 20: 116, 40: 112},
           8000: {-40: 99, -20: 101, -5: 103, 20: 99, 40: 94},
           10000: {-40: 92, -20: 93, -5: 95, 20: 93, 40: 88}},
}
#Anchors at ISA sea level (user): idle, fly, and the 10-minute maximum at the limiter.
ANCHORS = dict(idleNg=0.679, idleFuel=0.784, idleTgt=460.0, idleTq=0.055,
               flyNg=0.834, flyTgt=532.0, flyTq=0.18, maxTq=1.29, maxTgt=867.0)

NAMES = ['a', 'e1', 'K', 'q', 'mfe', 'tgtK', 'C', 'n0', 'b', 'm', 'v']


def atmos(pa, fat):
    theta = (fat + 273.15) / T0
    delta = math.exp(-G * M * pa * 0.3048 / (R * (fat + 273.15)))
    return theta, delta


def op(p, ng, theta, delta, fat):
    nc = ng / theta ** 0.5
    #Guide vanes scheduled on inlet temperature: the airflow, and the compressor work it costs.
    vg = theta ** p['v']
    fuel = (p['a'] * nc ** p['e1'] + p['K'] * nc ** p['q']) * delta * theta ** 0.5 * vg
    mf = max(nc ** p['mfe'] * delta / theta ** 0.5 * vg, 1e-6)
    tgt = fat + p['tgtK'] * fuel / mf
    share = max(0.0, (nc - p['n0']) / (1.0 - p['n0'])) ** p['b']
    tq = p['C'] * mf * ((tgt + 273.15) / T0) ** p['m'] * share * PT_EFF
    return fuel, tgt, tq


def max_power(p, pa, fat, single):
    theta, delta = atmos(pa, fat)
    tgtLim = 896.0 if single else 867.0
    ngLim = min(NG_LIM[0], NG_LIM[1] + NG_LIM[2] * fat)
    ok = lambda ng: (lambda f, t, q: t <= tgtLim and f <= FUEL_FLY)(*op(p, ng, theta, delta, fat))
    hi = ngLim
    if not ok(hi):
        lo = 0.3
        for _ in range(40):
            mid = (lo + hi) / 2.0
            if ok(mid):
                lo = mid
            else:
                hi = mid
        hi = lo
    f, t, q = op(p, hi, theta, delta, fat)
    held = 'Ng' if hi >= ngLim - 1e-6 else ('TGT' if t >= tgtLim - 0.5 else 'fuel')
    return q, t, hi, held


def anchor_error(p):
    A = ANCHORS
    f, t, q = op(p, A['idleNg'], 1.0, 1.0, 15.0)
    e = ((f - A['idleFuel']) / 0.02) ** 2 + ((t - A['idleTgt']) / 10.0) ** 2 + ((q - A['idleTq']) / 0.02) ** 2
    f, t, q = op(p, A['flyNg'], 1.0, 1.0, 15.0)
    e += ((t - A['flyTgt']) / 10.0) ** 2 + ((q - A['flyTq']) / 0.02) ** 2
    q, t, ng, held = max_power(p, 0, 15.0, False)
    e += ((q - A['maxTq']) / 0.01) ** 2 + ((t - A['maxTgt']) / 2.0) ** 2
    #Ng at the ISA maximum (Phase 3: 129% at Ng 1.010) - the headroom under the 1.022 cap.
    e += ((ng - 1.010) / 0.003) ** 2
    return e


#Max TGT at max power vs inlet temperature, sea level - the user's chart moved up 10 C (user's call).
#Below ~4 C the engine is Ng-limited at these temperatures; above, TGT holds at the limiter.
MAX_TGT = {-40: 731, -30: 771, -20: 811, -10: 849, 0: 864}


def max_tgt_error(p):
    e = 0.0
    for fat, want in MAX_TGT.items():
        q, t, ng, held = max_power(p, 0, fat, False)
        e += ((t - want) / 10.0) ** 2 + (0.0 if held == 'Ng' else 25.0)
    return e


def chart_error(p):
    s, n = 0.0, 0
    for single, tab in TM.items():
        for pa, row in tab.items():
            for fat, want in row.items():
                q = max_power(p, pa, fat, single)[0] * 100
                s += (q - want) ** 2
                n += 1
    return s / n


#Physical ranges, enforced softly so the search can slide along a bound instead of dying on it.
BOUNDS = dict(a=(0.01, 5.0), e1=(0.05, 6.0), K=(0.0, 5.0), q=(3.0, 12.0), mfe=(1.2, 2.2), tgtK=(150.0, 400.0),
              C=(0.001, 2.0), n0=(0.0, 0.66), b=(0.05, 6.0), m=(1.0, 1.0), v=(-1.5, 2.0))


def clamp_params(x):
    return [min(max(v, BOUNDS[k][0]), BOUNDS[k][1]) for k, v in zip(NAMES, x)]


def cost(x):
    pen = sum((max(0.0, BOUNDS[k][0] - v) + max(0.0, v - BOUNDS[k][1])) ** 2 for k, v in zip(NAMES, x)) * 1e4
    p = dict(zip(NAMES, clamp_params(x)))
    if p['q'] < p['e1']:
        pen += (p['e1'] - p['q']) ** 2 * 1e4
    return anchor_error(p) + max_tgt_error(p) + chart_error(p) + pen


def nelder_mead(f, x0, step, iters=3000):
    n = len(x0)
    pts = [list(x0)] + [[x0[j] + (step[j] if j == i else 0.0) for j in range(n)] for i in range(n)]
    vals = [f(x) for x in pts]
    for _ in range(iters):
        order = sorted(range(n + 1), key=lambda i: vals[i])
        pts, vals = [pts[i] for i in order], [vals[i] for i in order]
        cen = [sum(pts[i][j] for i in range(n)) / n for j in range(n)]
        xr = [cen[j] + (cen[j] - pts[-1][j]) for j in range(n)]
        fr = f(xr)
        if fr < vals[0]:
            xe = [cen[j] + 2 * (cen[j] - pts[-1][j]) for j in range(n)]
            fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = [cen[j] + 0.5 * (pts[-1][j] - cen[j]) for j in range(n)]
            fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                pts = [pts[0]] + [[pts[0][j] + 0.5 * (pts[i][j] - pts[0][j]) for j in range(n)] for i in range(1, n + 1)]
                vals = [vals[0]] + [f(x) for x in pts[1:]]
    i = min(range(n + 1), key=lambda k: vals[k])
    return pts[i], vals[i]


#Starting point: the best vane fit, 2026-10-01 (chart rms 7.1).
X0 = [1.154, 1.306, 1.659, 8.198, 1.848, 289.6, 0.3318, 0.6092, 1.563, 1.0, 0.8089]


def report(p):
    print('params: ' + '  '.join('%s %.4g' % (k, p[k]) for k in NAMES))
    print('anchors: err %.2f' % anchor_error(p))
    for nm, ng in (('idle', 0.679), ('fly', 0.834)):
        f, t, q = op(p, ng, 1.0, 1.0, 15.0)
        print('  %-4s Ng %.3f  fuel %.3f  TGT %4.0f  tq %.3f' % (nm, ng, f, t, q))
    q, t, ng, held = max_power(p, 0, 15.0, False)
    print('  max  Ng %.3f  TGT %4.0f  tq %.3f  held %s' % (ng, t, q, held))
    print('max TGT at SL: ' + '  '.join('%d: %3.0f/%d %s' % (f, max_power(p, 0, f, False)[1], w, max_power(p, 0, f, False)[3])
                                         for f, w in MAX_TGT.items()))
    print('chart rms %.2f %%Q' % chart_error(p) ** 0.5)
    for single, tab in TM.items():
        print(' %s' % ('SE' if single else 'DE'))
        for pa, row in tab.items():
            print('  PA%5d ' % pa + '  '.join('%d: %3.0f/%3d %s' % (fat, max_power(p, pa, fat, single)[0] * 100, want,
                                                                  max_power(p, pa, fat, single)[3][0])
                                                for fat, want in row.items()))


if __name__ == '__main__':
    samples = int(sys.argv[1]) if len(sys.argv) > 1 else 3000
    rng = random.Random(7)
    #Broad sweep across the physical box, then refine the best few.
    pool = [(cost(X0), X0)]
    for _ in range(samples):
        x = [rng.uniform(*BOUNDS[k]) for k in NAMES]
        pool.append((cost(x), x))
    pool.sort(key=lambda t: t[0])
    print('sweep best %.1f' % pool[0][0])
    best = pool[0]
    for i, (v0, x0) in enumerate(pool[:6]):
        x, v = nelder_mead(cost, x0, [(BOUNDS[k][1] - BOUNDS[k][0]) * 0.08 + 1e-3 for k in NAMES], iters=1500)
        print('refine %d: %.1f -> %.1f' % (i, v0, v))
        if v < best[0]:
            best = (v, x)
    report(dict(zip(NAMES, clamp_params(best[1]))))
