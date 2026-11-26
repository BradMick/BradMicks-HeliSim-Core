"""Front fuselage dragCoefTable rows above 0 ft, solved against the max range chart (AIRCRAFT_GUIDE,
"Fitting the main rotor tables", step 7).

    set BMKHS_CONFIG=<pack>\\config\\bmkhs_config
    python python/dev/drag_trim.py solve --chart chart.txt --bc X Y Z --cg X Y Z
    python python/dev/drag_trim.py check --chart chart.txt --bc X Y Z --cg X Y Z

solve - every pressure altitude on the chart above the table's first row: the front drag coefficient
        at which the mean, across the chart's FATs, of the rig's torque less the chart's at the
        chart's max range speed is zero. Prints the fuselageFront dragCoefTable block, then flies the
        written (rounded) table against the chart.
check - flies every cell with the pack's front drag as it is declared.

The table's first row - 0 ft, the sea level fit (step 3) - is never solved and never changed: it is
printed back as the pack has it.

The chart file, one FAT per row, max range at each pressure altitude as kt:torque%, true airspeed:

    #FAT_C  0       2000    4000    6000    8000
    -40     111:83  110:81  111:80  111:80  111:79
    -20     116:81  116:80  116:79  116:78  115:78

The first line names the altitude columns, ft. Every cell is flown at --gwt (default: the pack's
fitGrossMass), out of ground effect, cyclic solved, through airframe.py exactly as it is. It does not
edit the pack.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import airframe as A  # noqa: E402 - reads BMKHS_CONFIG at import
E = A.E

KT_TO_MPS = 0.514444
TQ_TOL = 0.005              #a row whose mean torque error is further than this from zero has failed
BRACKET_MAX = 40            #steps the drag bracket may widen before the torque is out of reach
PROBLEMS = []


def problem(msg):
    """Says so the moment something fails or does not converge, and keeps it for the summary."""
    PROBLEMS.append(msg)
    print('  FAILED: ' + msg, flush=True)


def read_chart(path):
    alts, chart = None, {}
    for line in open(path):
        cols = line.split()
        if not cols:
            continue
        if alts is None:
            alts = [int(c) for c in cols[1:]]
            continue
        chart[int(cols[0])] = [(float(k), float(t) / 100.0) for k, t in (c.split(':') for c in cols[1:])]
    return alts, chart


ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('mode', choices=('solve', 'check'))
ap.add_argument('--chart', required=True)
ap.add_argument('--gwt', type=float, default=None, help='kg, the weight the chart is flown at (default: the pack\'s fitGrossMass)')
ap.add_argument('--cg', nargs=3, type=float, required=True)
ap.add_argument('--bc', nargs=3, type=float, default=(0.0, 0.0, 0.0))
args = ap.parse_args()

alts, chart = read_chart(args.chart)
fats = sorted(chart)
af = A.Airframe(bc=tuple(args.bc), cg=tuple(args.cg))
gwt = args.gwt if args.gwt is not None else af.fitGwt
if gwt <= 0.0:
    raise SystemExit('the pack names no fitGrossMass: give --gwt')
front = af.fus['fuselageFront']
#The sea level row, as the pack has it - never solved, never changed
seaAlt, seaCd = front['dragCoefTable'][0][0], front['dragCoefTable'][0][1]


def level(kt, pa, fat):
    """Level flight at a cell - rotortables.py's Fit.level, in the cell's own air. The last two
    starting guesses are for thin air, where the trim sits nose low on a high collective."""
    rho = A.density(pa, fat)
    for g in ((-3.0, 0.6, 0.0), (0.0, 0.5, 0.0), (-3.0, 0.65, 0.0), (2.0, 0.4, 0.0), (-6.0, 0.7, 0.3), (-9.0, 0.85, 0.5)):
        t = A.trim(af, kt * KT_TO_MPS, gwt, rho, pa, solveCyc=True, guess=g)
        if t['ok']:
            break
    return t


def speed_at(tq, kt, pa, fat):
    """The speed, either side of the chart's, at which level flight takes the chart's torque. None
    where the torque is not met within 40 kt below or 60 kt above."""
    lo, hi = kt - 40.0, kt + 60.0
    t = level(lo, pa, fat)
    if not t['ok'] or t['tq'] > tq:
        return None
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        t = level(mid, pa, fat)
        if not t['ok'] or t['tq'] > tq:
            hi = mid
        else:
            lo = mid
    return lo if hi < kt + 59.9 else None


def row_error(pa):
    """Mean, across the FATs, of the rig's torque less the chart's at the chart's speed, at one
    pressure altitude. A cell that does not trim has too much drag."""
    i = alts.index(pa)
    err = 0.0
    for fat in fats:
        kt, tq = chart[fat][i]
        t = level(kt, pa, fat)
        if not t['ok']:
            return float('inf')
        err += t['tq'] - tq
    return err / len(fats)


def solve_row(pa, table, cd):
    """(drag coefficient, mean torque error reached or None, why not). More drag takes more torque.
    Bounded as rotortables.py's solve_column: the bracket widens at most BRACKET_MAX steps each way,
    the bisection 60."""
    def err(c):
        front['dragCoefTable'] = [[a, (c if a == pa else v)] for a, v in table]
        return row_error(pa)
    lo, hi = cd, cd
    for _ in range(BRACKET_MAX):
        if err(lo) <= 0.0:
            break
        lo *= 0.8
    else:
        return cd, None, 'no front drag brings the mean torque down to the chart\'s - the rotor alone takes more'
    for _ in range(BRACKET_MAX):
        if err(hi) > 0.0:
            break
        hi *= 1.25
    else:
        return cd, None, 'no front drag brings the mean torque up to the chart\'s'
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if err(mid) > 0.0:
            hi = mid
        else:
            lo = mid
        if hi - lo < 1e-7:
            break
    got = err(lo)
    if abs(got) > TQ_TOL:
        return lo, got, 'closest is a mean of %+.1f torque points, where the trim gives out' % (got * 100)
    return lo, got, None


def fly():
    """Every cell with the front drag as it stands, against the chart: the rig's torque at the
    chart's speed, and the rig's speed at the chart's torque. Then each altitude's mean."""
    print('   FAT     PA    chart     rig TQ    rig kt         front drag')
    means = {pa: [] for pa in alts}
    for fat in fats:
        for pa, (kt, tq) in zip(alts, chart[fat]):
            t = level(kt, pa, fat)
            if not t['ok']:
                print('  %4d  %5d   %3.0f:%2.0f      NO' % (fat, pa, kt, tq * 100), flush=True)
                problem('%d C %d ft does not trim at %.0f kt' % (fat, pa, kt))
                continue
            v = speed_at(tq, kt, pa, fat)
            means[pa].append(t['tq'] - tq)
            print('  %4d  %5d   %3.0f:%2.0f     %5.1f%%    %s   %.3f' % (
                fat, pa, kt, tq * 100, t['tq'] * 100,
                '%5.1f (%+5.1f)' % (v, v - kt) if v is not None else '   --        ',
                E.math_linear_interp(front['dragCoefTable'], pa)[1]), flush=True)
    print('Mean torque error by pressure altitude, torque points:')
    for pa in alts:
        if means[pa]:
            print('  %5d ft  %+5.1f%s' % (pa, sum(means[pa]) / len(means[pa]) * 100, '   the sea level fit, not solved' if pa <= seaAlt else ''))
    return means


print('%.0f kg, out of ground effect, cyclic solved' % gwt)
if args.mode == 'solve':
    #Every chart altitude above the sea level row gets a row, starting from the pack's own drag there
    rows = [pa for pa in alts if pa > seaAlt]
    table = [[seaAlt, seaCd]] + [[pa, E.math_linear_interp(front['dragCoefTable'], pa)[1]] for pa in rows]
    print('%5d ft: front drag %.3f, the sea level fit - not solved' % (seaAlt, seaCd))
    for i, pa in enumerate(rows):
        cd, got, why = solve_row(pa, table, table[i + 1][1])
        print('%5d ft: front drag %s' % (pa, 'missed' if why else '%.4f' % cd), flush=True)
        if why:
            problem('%d ft: %s' % (pa, why))
        table[i + 1][1] = round(cd, 3)

    print('\n            dragCoefTable[] =')
    print('            {')
    for i, (pa, cd) in enumerate(table):
        print('            %s{%4d, %.3f}' % (' ' if i == 0 else ',', pa, cd))
    print('            };')

    front['dragCoefTable'] = [list(r) for r in table]
    print('\nWritten table:')
    means = fly()
    for pa in rows:
        if means[pa] and abs(sum(means[pa]) / len(means[pa])) > TQ_TOL:
            problem('the written %d ft row leaves a mean of %+.1f torque points' % (pa, sum(means[pa]) / len(means[pa]) * 100))
else:
    print('The pack\'s front drag as declared:')
    fly()

if PROBLEMS:
    print('\n%d PROBLEM%s:' % (len(PROBLEMS), '' if len(PROBLEMS) == 1 else 'S'))
    for p in PROBLEMS:
        print('  - ' + p)
    sys.exit(1)
if args.mode == 'solve':
    print('\nFit OK: every solved row\'s mean within %.1f torque points.' % (TQ_TOL * 100))
