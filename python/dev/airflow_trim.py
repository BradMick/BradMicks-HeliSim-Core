"""Engine airflowTable trim against the OGE hover chart (AIRCRAFT_GUIDE step 3, max torque available).

    set BMKHS_CONFIG=<pack>\\config\\bmkhs_config
    python python/dev/airflow_trim.py solve --chart chart.txt --tot C --rotor-tq F --bc X Y Z --cg X Y Z
    python python/dev/airflow_trim.py check table.json --chart chart.txt --tot C --rotor-tq F --bc X Y Z --cg X Y Z

solve - every engine-limited cell: the torque the hover needs against what the engine delivers,
        untrimmed, and the mean need / delivered ratio per FAT - the airflowTable multiplier.
check - flies every cell with a {FAT, multiplier} table, as JSON: [[-40, 1.14], [-30, 1.093], ...].

The chart file, one FAT per row, the maximum OGE hover weight in kg at each pressure altitude:

    #FAT_C  0     2000  4000  6000  8000  10000
    -40     2500  2500  2500  2500  2475  2280
    -20     2500  2500  2500  2500  2420  2230

The first line names the altitude columns, ft. A cell at --max-gw (default: the chart's largest
weight) is limited by gross weight, not the engine, and is skipped. A cell needing --rotor-tq or
more is drivetrain-limited; the rest are flown at the --tot TGT limit.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import airframe as A  # noqa: E402 - reads BMKHS_CONFIG at import
E = A.E


def read_chart(path):
    alts, chart = None, {}
    for line in open(path):
        cols = line.split()
        if not cols:
            continue
        if alts is None:
            alts = [int(c) for c in cols[1:]]
            continue
        chart[int(cols[0])] = [float(c) for c in cols[1:]]
    return alts, chart


def engines():
    return list(E.CFG['Engines'].values())


def set_engine(table, tot):
    for e in engines():
        e['Combustor']['maxTgt'] = tot
        e['Combustor']['maxTgtSe'] = tot
        e['Compressor']['airflowTable'] = [list(r) for r in table]


ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('mode', choices=('solve', 'check'))
ap.add_argument('table', nargs='?', help='check: the {FAT, multiplier} table, JSON')
ap.add_argument('--chart', required=True)
ap.add_argument('--tot', type=float, required=True, help='C, the TGT the chart is flown at')
ap.add_argument('--rotor-tq', type=float, required=True, help='torque fraction at or above which a cell is drivetrain-limited')
ap.add_argument('--max-gw', type=float, default=None, help='kg, cells at or above it are skipped (default: the chart\'s largest)')
ap.add_argument('--cg', nargs=3, type=float, required=True)
ap.add_argument('--bc', nargs=3, type=float, default=(0.0, 0.0, 0.0))
args = ap.parse_args()

alts, chart = read_chart(args.chart)
maxGw = args.max_gw if args.max_gw is not None else max(w for ws in chart.values() for w in ws)
af = A.Airframe(bc=tuple(args.bc), cg=tuple(args.cg))


def needed(w, pa, fat):
    r = A.trim(af, 0.0, w, A.density(pa, fat), pa, solveCyc=True, guess=(0, 0.6, 0))
    return r['tq'] if r['ok'] else None


#Untrimmed: the engine's own FAT rows, every multiplier 1.0.
table = [[r[0], 1.0] for r in engines()[0]['Compressor']['airflowTable']]
if args.mode == 'check':
    table = json.load(open(args.table))
set_engine(table, args.tot)

ratios = {}
for fat, ws in chart.items():
    for pa, w in zip(alts, ws):
        if w >= maxGw:
            continue
        need = needed(w, pa, fat)
        r = E.max_power(pa, fat, False)
        avail, tgt, ng, why = (r[0], r[1], r[2], r[4]) if r else (None, None, None, '-')
        lim = 'rotor' if need is not None and need >= args.rotor_tq else 'TOT'
        print('%4d C %5d ft %4d kg: need %5.1f%%  engine %5s%% at %4s C N1 %s held %s  -> %s' % (
            fat, pa, w, need * 100 if need else -1, '%.1f' % (avail * 100) if avail else '-',
            '%.0f' % tgt if tgt else '-', '%.3f' % ng if ng else '-', why, lim), flush=True)
        if args.mode == 'solve' and need and lim == 'TOT' and avail and why == 'TGT':
            ratios.setdefault(fat, []).append(need / avail)

if args.mode == 'solve':
    print()
    for fat, rs in sorted(ratios.items()):
        print('%4d C: ratios %s  mean %.4f' % (fat, ' '.join('%.3f' % r for r in rs), sum(rs) / len(rs)))
