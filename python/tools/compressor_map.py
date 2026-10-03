"""Compressor map generator - builds an engine's compressorMap[] from its ratings table.

Run it:  python python/tools/compressor_map.py <bmkhs_config folder> <ratings file>

The config folder is the aircraft's bmkhs_config/ - its helisim_engine.hpp gives the engine's
spec (pressureRatio, massFlow, maxNg, powerKw, efficiencies) and helisim_simpleRotor.hpp the
rotor the check flies. The ratings file is one rating per line, from the engine's limitations
table, all at sea level:

    #name  FAT_C  N1      T45_C  torque
    MCP    5      0.950   894    0.940
    TOP    5      0.968   928    1.006

N1 and torque are fractions (96.8 % = 0.968). Rows below the lowest rating - start, idle, flat
pitch - come from Core's T700-701C map, scaled to this engine's pressure ratio; the rating rows
are solved so each rating lands on its N1, T45 and torque; above the highest rating the map
carries on along the last two ratings. It prints the block to paste into the engine's
Compressor class, then flies every rating in the rig to show it lands.
"""
import math
import os
import sys

if len(sys.argv) < 3:
    print(__doc__)
    sys.exit(1)
os.environ['BMKHS_CONFIG'] = os.path.abspath(sys.argv[1])
#The rating check flies the rig, which lives in dev/ beside this folder.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'dev'))
import engine as E

XC = (E.GT_GAMMA_COLD - 1) / E.GT_GAMMA_COLD
XH = (E.GT_GAMMA_HOT - 1) / E.GT_GAMMA_HOT
T700_PR = 17.0
#Below the lowest rating, T700 rows closer than this are dropped so the map does not step.
BLEND_GAP = 0.05


def read_ratings(path):
    out = []
    for line in open(path):
        line = line.split('#')[0].split()
        if line:
            name, fat, ng, tgt, tq = line
            out.append((name, float(fat), float(ng), float(tgt), float(tq)))
    return sorted(out, key=lambda r: r[2])


def engine_spec():
    e = E.CFG['Engines']['Engine01']
    c = e['Compressor']
    return dict(pr=c['pressureRatio'], mdot=c['massFlow'], maxNg=e['maxNg'], kw=e['powerKw'],
                etaT=e['CompressorTurbine']['turbineEfficiency'], etaPt=e['PowerTurbine']['ptEfficiency'],
                airflowTable=c['airflowTable'])


def scaled_t700_row(row, pr):
    """A T700 row with its compressor turbine column in this engine's pressure ratio: the turbine
    takes the same share of what the compressor makes, held below the first fitted row."""
    base = E.COMPRESSOR_MAP
    _, prs, _, _, ct = E.math_linear_interp(base, max(row[0], 0.6173))
    share = math.log(T700_PR ** ct) / math.log(T700_PR * prs)
    return [row[0], row[1], row[2], row[3], math.log((pr * prs) ** share) / math.log(pr)]


def solve(spec, base, fat, ng, tgt, tq):
    """One rating, steady state at sea level: TGT fixes the compressor turbine expansion,
    torque fixes the airflow. PR and efficiency come from the base map."""
    t2 = fat + E.DEG_C_TO_KELVIN
    theta = t2 / E.GT_STD_TEMP_K
    x = ng / math.sqrt(theta) / spec['maxNg']
    _, prf, _, eff, _ = E.math_linear_interp(base, x)
    pr = spec['pr'] * prf
    t3 = t2 * (1 + (pr ** XC - 1) / eff)
    w = E.GT_CP_COLD * (t3 - t2) / E.GT_CP_HOT
    t45 = tgt + E.DEG_C_TO_KELVIN
    t4 = t45 + w
    er = (1 - w / (spec['etaT'] * t4)) ** (-1 / XH)
    if er >= pr:
        raise SystemExit('%.0f C at N1 %.3f is too cool to turn the compressor - check the rating' % (tgt, ng))
    ptPerKg = E.GT_CP_HOT * t45 * spec['etaPt'] * (1 - (er / pr) ** XH)
    airflow = E.math_linear_interp(spec['airflowTable'], fat)[1]
    mDot = tq * spec['kw'] / ptPerKg
    flow = mDot / (spec['mdot'] * airflow / math.sqrt(theta))
    return [x, prf, flow, eff, math.log(er) / math.log(spec['pr'])]


def build(spec, ratings):
    base = [scaled_t700_row(r, spec['pr']) for r in E.COMPRESSOR_MAP]
    fitted = [solve(spec, base, fat, ng, tgt, tq) for _, fat, ng, tgt, tq in ratings]
    rows = [r for r in base if r[0] <= fitted[0][0] - BLEND_GAP]
    rows += fitted
    if len(fitted) > 1:
        (x1, _, f1, _, c1), (x2, _, f2, _, c2) = fitted[-2], fitted[-1]
        for r in base:
            if r[0] > x2:
                t = (r[0] - x2) / (x2 - x1)
                rows.append([r[0], r[1], f2 + (f2 - f1) * t, r[3], c2 + (c2 - c1) * t])
    return rows


def use(rows):
    for e in E.CFG['Engines'].values():
        if isinstance(e, dict) and 'Compressor' in e:
            e['Compressor']['compressorMap'] = rows


def governed_at(fat, tq):
    lo, hi, a = 0.0, 1.0, None
    for _ in range(12):
        m = (lo + hi) / 2
        a = E.pulled(0, fat, False, m, secs=100.0)
        if a.tq() < tq and a.nrFrac() >= E.PERF_ON_SPEED:
            lo = m
        else:
            hi = m
    return a


def fuel_idle():
    e = E.CFG['Engines']['Engine01']
    idleNg = e['Compressor']['idleNg']
    lo, hi = 0.05, e['Governor']['fuelFly']
    def idle(fuel):
        E.OVERRIDES['fuelIdle'] = fuel
        a = E.Heli()
        E.start_to(a, (0, 1), 'IDLE', 80.0, E.ARMA_DT, E.JITTER)
        return a

    for _ in range(16):
        m = (lo + hi) / 2
        a = idle(m)
        #A trip reads Ng 0 - too rich, not too lean.
        if a.tripped() or a.ng() > idleNg:
            hi = m
        else:
            lo = m
    fuel = round((lo + hi) / 2, 3)
    a = idle(fuel)
    E.OVERRIDES.pop('fuelIdle', None)
    return fuel, a


if __name__ == '__main__':
    ratings = read_ratings(sys.argv[2])
    spec = engine_spec()
    rows = build(spec, ratings)

    print('compressorMap[] = {')
    for i, r in enumerate(rows):
        print('     %s{%.4f, %.4f, %.4f, %.3f, %.4f}' % (' ' if i == 0 else ',', *r))
    print('};')

    use(rows)
    print('\nCheck - each rating flown in the rig, governed to its torque (target in brackets):')
    for name, fat, ng, tgt, tq in ratings:
        a = governed_at(fat, tq)
        print('  %-6s %3.0f C  torque %5.1f %% (%5.1f)  N1 %.4f (%.3f)  T45 %4.0f (%4.0f)'
              % (name, fat, a.tq() * 100, tq * 100, a.ng(), ng, a.tgt(), tgt))
    fuel, a = fuel_idle()
    print('\nfuelIdle that settles at idleNg: %.3f  (Ng %.4f, TGT %.0f C, torque %.1f %%)'
          % (fuel, a.ng(), a.tgt(), a.tq() * 100))
