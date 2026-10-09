"""The main rotor's liftCoefTable and dragCoefTable, generated in the rig - the recipe in
docs/AIRCRAFT_GUIDE.md, "Fitting the main rotor tables", steps 4 to 6.

Run it:  set BMKHS_CONFIG=<pack>\\config\\bmkhs_config
         python python/dev/rotortables.py --cg 0.028 1.787 0.300
                --hover 18000lb:81 22000lb:105 23500lb:115 --me 68:44 --mr 140 [--peak 0.85] [--bc 0 0 0]

--hover  the OGE hover points at sea level, 15 C: weight:torque%, lightest (the mid gross weight)
         first, heaviest last. A weight is kg, or lb with an lb suffix.
--me     max endurance at the mid gross weight: kt:torque%.
--mr     max range at the mid gross weight: kt, where torque is back to the OGE hover torque.
--peak   the collective at which the blade's lift peaks (default 0.85).

Everything is flown at sea level, 15 C, out of ground effect, cyclic solved, through airframe.py
exactly as it is. The pack's own config supplies everything else, including the drag at
collective 0, which is kept so the idle torque does not move.

Prints both tables as helisim_simpleRotor.hpp blocks for the main rotor, then a sweep of the
written (rounded) tables against the power curve. It does not edit the pack.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import airframe as A  # noqa: E402 - reads BMKHS_CONFIG at import
E = A.E

LB_TO_KG = 1.0 / 2.20462
KT_TO_MPS = 0.514444
RHO = A.density(0.0, 15.0)

#The standard power curve: torque / OGE hover torque against airspeed / max-range speed - the mid
#gross weight line (guide step 5). Held as the reference it was drawn from - UH-60, 18,000 lb,
#sea level, 15 C: kt and torque %, max range 140 kt, OGE 81% - so its points divide exactly.
_REF_KT = (0, 10, 20, 30, 40, 50, 60, 68, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160)
_REF_TQ = (81, 74, 63, 54, 48, 46, 44, 44, 44, 44, 46, 50, 55, 62, 70, 81, 98, 114)
STANDARD = [[k / 140.0, t / 81.0] for k, t in zip(_REF_KT, _REF_TQ)]
STD_FLOOR = 44.0 / 81.0
STD_BUCKET = 68.0 / 140.0
PEAK_MARGIN = 1.001         #thrust at the peak over the heaviest hover weight, so it still trims

#The collective shape (guide step 4): CL at collective 0 and past the peak, as fractions of the
#peak; CD past the peak as multiples of the peak's.
CL_ZERO = 0.106
CL_FALL = (0.957, 0.883, 0.766)
CD_STALL = (1.3, 1.8, 2.6)
EXTRA_COL = 92.60           #m/s, the column past the end


STD_ETL = 40.0 / 140.0      #where translational lift is in, as a fraction of max range


def drag_scales(curve, vmr, drop):
    """Torque at a held collective, per column, as a fraction of the hover's (guide step 6):
    falls by `drop` through ETL in the curve's own shape, holds flat to max range, then climbs
    back to 1.0 at the last point so the top of the curve stays below the stall."""
    tq0 = curve[0][1]
    tq_etl = E.math_linear_interp(curve, STD_ETL * vmr)[1]
    last = curve[-1][0]
    out = []
    for kt, tq in curve:
        if kt <= STD_ETL * vmr:
            out.append(1.0 - drop * (tq0 - tq) / (tq0 - tq_etl))
        elif kt <= vmr:
            out.append(1.0 - drop)
        else:
            out.append(1.0 - drop * (last - kt) / (last - vmr))
    return out


def _weight(s):
    return float(s[:-2]) * LB_TO_KG if s.lower().endswith('lb') else float(s)


def power_curve(tqh, vme, tqme, vmr):
    """[[kt, torque]] - the standard placed by the three set points (guide step 5)."""
    r = tqme / tqh
    b = vme / vmr
    out = []
    for x, s in STANDARD:
        if x <= 1.0:
            s = 1.0 - (1.0 - s) * (1.0 - r) / (1.0 - STD_FLOOR)
        #Bucket position: two straight pieces, 0 -> STD_BUCKET onto 0 -> b, STD_BUCKET -> 1 onto b -> 1
        if x <= STD_BUCKET:
            xa = x * b / STD_BUCKET
        elif x <= 1.0:
            xa = b + (x - STD_BUCKET) * (1.0 - b) / (1.0 - STD_BUCKET)
        else:
            xa = x
        out.append([round(xa * vmr, 3), tqh * s])
    return out


class Fit:
    def __init__(self, af, hover, peak):
        self.af = af
        self.main = next(r for r in af.rotors if r['type'].lower() == 'main')
        self.hover = hover
        self.peak = peak
        d = self.main['dragCoefTable']
        self.cd0 = E.math_linear_interp([[r[0], r[1]] for r in d[1:]], 0.0)[1]
        step = (1.0 - peak) / 3.0
        self.fall = [peak + step, peak + 2 * step, 1.0]
        self.vel0, _ = A.body_state(0.01, 0.0)

    def hover_trim(self, w, guess):
        return A.trim(self.af, 0.01, w, RHO, 0.0, solveCyc=True, guess=guess)

    def set_hover_column(self, lrows, lift, drows, drag):
        #Two identical columns: the rig reads only 0 m/s in a hover.
        self.main['liftCoefTable'] = [['A/S', 0.0, 10.29]] + [[r, v, v] for r, v in zip(lrows, lift)]
        self.main['dragCoefTable'] = [['A/S', 0.0, 10.29]] + [[r, v, v] for r, v in zip(drows, drag)]

    def solve_hover_column(self):
        """Guide step 4: CL by the heaviest hover at the peak, then CD by every hover's torque."""
        lrows = [0.0, self.peak] + self.fall
        lift_of = lambda p: [CL_ZERO * p, p] + [f * p for f in CL_FALL]
        wmax = self.hover[-1][0]
        clp = 0.35
        drows, drag = [0.0, 1.0], [self.cd0, 0.1]
        for _ in range(100):
            self.set_hover_column(lrows, lift_of(clp), drows, drag)
            #(drag here is a placeholder - thrust does not read it)
            _, _, _, parts = self.af.forces(self.vel0, 0.0, 0.0, 0.0, self.peak, RHO, 0.0, parts=True)
            thrust = parts['main rotor'][0][2]
            new = clp * wmax * E.GRAVITY * PEAK_MARGIN / thrust
            if abs(new - clp) < 1e-9:
                break
            clp = new
        lift = lift_of(clp)
        colls = []
        for w, _ in self.hover[:-1]:
            t = self.hover_trim(w, (3.0, 0.6, 0.0))
            if not t['ok']:
                raise SystemExit('%.0f kg does not trim in the hover under a %.3f peak' % (w, self.peak))
            colls.append(round(t['coll'], 3))
        drows = [0.0] + colls + [self.peak] + self.fall
        cds = [0.035] * len(self.hover)
        #The heaviest hover trims twice - just below the peak and just past it, stalled. The real one
        #is below, where collective comes up to it: start its trim there.
        guesses = colls + [self.peak - 0.02]
        for _ in range(100):
            drag = [self.cd0] + cds + [m * cds[-1] for m in CD_STALL]
            self.set_hover_column(lrows, lift, drows, drag)
            tq = [self.hover_trim(w, (3.0, c, 0.0))['tq'] for (w, _), c in zip(self.hover, guesses)]
            new = [cd * t / q for cd, (_, t), q in zip(cds, self.hover, tq)]
            if max(abs(a - b) for a, b in zip(new, cds)) < 1e-8:
                break
            cds = new
        drag = [self.cd0] + cds + [m * cds[-1] for m in CD_STALL]
        return lrows, lift, drows, drag, colls

    def set_tables(self, lrows, lift, drows, drag, cols, sl, sd, nd=None):
        """Every airspeed column is the hover column, lift times sl, drag times sd. sd is fixed
        by drag_scales (a collective is a torque, bar ETL); sl is what the column solve finds."""
        rd = (lambda a: round(a, nd)) if nd else (lambda a: a)
        allc = cols + [EXTRA_COL]
        k = (EXTRA_COL - cols[-1]) / (cols[-1] - cols[-2])
        sl = list(sl) + [sl[-1] + (sl[-1] - sl[-2]) * k]
        sd = list(sd) + [sd[-1]]
        lt = [['A/S'] + allc] + [[r] + [rd(v * s) for s in sl] for r, v in zip(lrows, lift)]
        dt = [['A/S'] + allc] + [[r] + [rd(v * s) for s in sd] for r, v in zip(drows, drag)]
        self.main['liftCoefTable'], self.main['dragCoefTable'] = lt, dt
        return lt, dt

    def level(self, kt, w, guess):
        t = A.trim(self.af, max(kt, 0.01) * KT_TO_MPS, w, RHO, 0.0, solveCyc=True, guess=guess)
        if not t['ok']:
            for g in ((0.0, 0.5, 0.0), (-3.0, 0.65, 0.0), (2.0, 0.4, 0.0)):
                t = A.trim(self.af, max(kt, 0.01) * KT_TO_MPS, w, RHO, 0.0, solveCyc=True, guess=g)
                if t['ok']:
                    break
        return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cg', nargs=3, type=float, required=True)
    ap.add_argument('--bc', nargs=3, type=float, default=(0.0, 0.0, 0.0))
    ap.add_argument('--hover', nargs='+', required=True)
    ap.add_argument('--me', required=True)
    ap.add_argument('--mr', type=float, required=True)
    ap.add_argument('--peak', type=float, default=0.85)
    ap.add_argument('--etl-drop', type=float, default=0.05)
    ap.add_argument('--cruise-pitch', type=float, default=None,
                    help='deg, nose up positive: solve the front fuselage drag so max range trims here')
    a = ap.parse_args()

    hover = sorted(((_weight(w), float(t) / 100.0) for w, t in (h.split(':') for h in a.hover)), key=lambda p: p[0])
    vme, tqme = (float(v) for v in a.me.split(':'))
    tqme /= 100.0
    wmid, tqh = hover[0]
    curve = power_curve(tqh, vme, tqme, a.mr)

    fit = Fit(A.Airframe(a.bc, a.cg), hover, a.peak)
    lrows, lift, drows, drag, colls = fit.solve_hover_column()
    ch = colls[0] if colls else a.peak
    print('Hover column (sea level, 15 C, OGE):')
    for (w, t), c in zip(hover, colls + [a.peak]):
        print('  %7.0f kg (%6.0f lb)  %.0f%% at collective %.3f' % (w, w / LB_TO_KG, t * 100, c))

    #Guide step 6: the airspeed columns. Drag is fixed per column by drag_scales, so torque fixes the
    #collective; each column's lift scale is solved so level flight at its speed takes the curve's
    #torque - the sag is in the collective it needs, not in torque drifting with speed.
    cols = [round(kt * KT_TO_MPS, 2) for kt, _ in curve]
    n = len(cols)
    sd = drag_scales(curve, a.mr, a.etl_drop)
    G = [(3.0, ch, 0.0)] * n

    #One column at a time, bracketed: more lift needs less collective, so less torque. A trim that
    #fails is past the stall - too little lift. Later columns ride along on the first pass; then
    #sweep again with every column in place until none moves.
    def column_tq(sl, i):
        fit.set_tables(lrows, lift, drows, drag, cols, sl, sd)
        t = fit.level(curve[i][0], wmid, G[i])
        if t['ok']:
            G[i] = (t['pitch'], t['coll'], t['cyc'])
            return t['tq']
        return float('inf')

    def solve_column(sl, i, ride):
        def with_(s):
            return sl[:i] + [s] * (n - i if ride else 1) + ([] if ride else sl[i + 1:])
        target = curve[i][1]
        lo, hi = sl[i], sl[i]
        while column_tq(with_(lo), i) <= target:
            lo *= 0.8
        while column_tq(with_(hi), i) > target:
            hi *= 1.25
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if column_tq(with_(mid), i) > target:
                lo = mid
            else:
                hi = mid
            if hi - lo < 1e-7:
                break
        return with_(hi)

    def solve_columns(x):
        for i in range(1, n):
            G[i] = G[i - 1]
            x = solve_column(x, i, True)
        for sweep in range(20):
            before = list(x)
            for i in range(1, n):
                x = solve_column(x, i, False)
            moved = max(abs(p - q) for p, q in zip(x, before))
            print('columns, pass %d: largest change %.6f' % (sweep, moved), flush=True)
            if moved < 1e-5:
                break
        return x

    #Guide step 3: with --cruise-pitch, the front fuselage drag (flat across altitude) is solved
    #with the columns so max range trims at that attitude. Each drag value gets its own column
    #solve; a secant on the drag closes the attitude.
    front = fit.af.fus['fuselageFront']
    imr = min(range(n), key=lambda i: abs(curve[i][0] - a.mr))

    def set_front(cd):
        front['dragCoefTable'] = [[alt, cd] for alt in (0, 2000, 4000, 6000, 8000)]

    def cruise_pitch(cd, x):
        set_front(cd)
        x = solve_columns(x)
        fit.set_tables(lrows, lift, drows, drag, cols, x, sd)
        t = fit.level(curve[imr][0], wmid, G[imr])
        print('front drag %.4f: max range %.1f kt trims at %+.2f deg' % (cd, curve[imr][0], t['pitch']), flush=True)
        return t['pitch'], x

    x = [1.0] * n
    if a.cruise_pitch is None:
        x = solve_columns(x)
        cd = None
    else:
        c0 = E.math_linear_interp(front['dragCoefTable'], 0.0)[1]
        c1 = c0 * 1.5
        p0, x = cruise_pitch(c0, x)
        for _ in range(20):
            p1, x = cruise_pitch(c1, x)
            if abs(p1 - a.cruise_pitch) < 0.01:
                break
            c0, c1, p0 = c1, max(0.01, c1 + (a.cruise_pitch - p1) * (c1 - c0) / (p1 - p0)), p1
        cd = round(c1, 3)
        set_front(cd)
        x = solve_columns(x)

    lt, dt = fit.set_tables(lrows, lift, drows, drag, cols, x, sd, nd=4)
    for name, t in (('liftCoefTable', lt), ('dragCoefTable', dt)):
        print('\n            %s[] = {' % name)
        print('                        {"A/S", ' + ', '.join('%.2f' % c for c in t[0][1:]) + '}')
        for row in t[1:]:
            print('                        ,{%.3f, ' % row[0] + ', '.join('%.4f' % v for v in row[1:]) + '}')
        print('                        };')
    if cd is not None:
        print('\nFront fuselage drag (helisim_fuselage.hpp, fuselageFront dragCoefTable): %.3f, flat' % cd)

    print('\nWritten tables, %.0f kg (%.0f lb), sea level, 15 C, cyclic solved:' % (wmid, wmid / LB_TO_KG))
    print('    kt   target   rig   coll   pitch')
    g = (3.0, ch, 0.0)
    for kt, tq in curve:
        t = fit.level(kt, wmid, g)
        if t['ok']:
            g = (t['pitch'], t['coll'], t['cyc'])
        print('  %5.1f  %5.1f%%  %5.1f%%  %.3f  %+5.1f %s' % (kt, tq * 100, t['tq'] * 100, t['coll'], t['pitch'], '' if t['ok'] else 'NO'))
    print('Hovers:')
    for w, tq in hover:
        t = fit.hover_trim(w, (3.0, 0.7, 0.0))
        print('  %7.0f kg (%6.0f lb)  target %5.1f%%  rig %5.1f%% at %.3f %s' % (
            w, w / LB_TO_KG, tq * 100, t['tq'] * 100, t['coll'], '' if t['ok'] else 'NO TRIM'))


if __name__ == '__main__':
    main()
