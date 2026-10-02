# Line-for-line port of fn_engineDamage.sqf. H is the heli's variables; engines are dicts with
# the four limit arrays. Readings are held fixed per scenario (the engine model is not in here).
import random

SYS_ENG_CHIPS_DMG, SYS_ENG_OIL_DROP_DMG, SYS_ENG_OIL_FAST_DMG, SYS_ENG_OIL_ZERO_DMG = 0.50, 0.65, 0.75, 0.85
SYS_ENG_OIL_DIVISOR, SYS_ENG_STARVE_RATE, SYS_ENG_STARVE_REF_NG = 12.1727, 0.00191702, 0.75
SYS_ENG_SHARED_CHIPS_DMG, SYS_ENG_SHARED_FAIL_DMG, SYS_ENG_SHARED_CHIPS2_DMG = 0.25, 0.50, 0.75

LIMITS = {
    'tqLimits':  [(1.00, 6, 10), (1.15, 0, 20)],
    'npLimits':  [(1.05, 12, 10), (1.21, 0, 20)],
    'ngLimits':  [(1.022, 12, 10), (1.051, 0, 20)],
    'tgtLimits': [(810, 1800, 1000), (870, 600, 0), (878, 150, 0), (896, 12, 0), (949, 0, 2000)],
}

def worsen(d):
    p = d / 600.0
    if d > 0.50: p = d / 500.0
    if d > 0.75: p = d / 400.0
    return p

def new_heli(n, use_systems):
    H = dict(useSystems=use_systems, engState=['ON'] * n, damage=[0.0] * n,  # per engine (systems) or [0] shared
             oil=[1.0] * n, lowOil=[False] * n, chips=[False] * n, failed=[False] * n, pick=-1,
             timers=[[-1] * 4 for _ in range(n)], tv={},
             tq=[0.5] * n, np=[1.01] * n, ng=[0.90] * n, tgt=[700.0] * n)
    return H

def dmg_get(H, i):  return H['damage'][i if H['useSystems'] else 0]
def dmg_set(H, i, d):
    if H['useSystems']: H['damage'][i] = d
    else: H['damage'] = [d] * len(H['damage'])

def step(H, dt):
    n = len(H['engState'])
    readings = [('tqLimits', H['tq']), ('npLimits', H['np']), ('ngLimits', H['ng']), ('tgtLimits', H['tgt'])]
    running, accrues = [], []
    for i in range(n):
        on = H['engState'][i] in ('STARTING', 'ON')
        accrue = 0.0; left = []
        for key, values in readings:
            limits = LIMITS[key]; value = values[i]; band = -1; armed = False
            for k, (limit, seconds, _div) in enumerate(limits):
                ceiling = 1e10 if k == len(limits) - 1 else limits[k + 1][0]
                tv = (key, i, k)
                if on and value > limit and value <= ceiling:
                    if seconds <= 0:
                        armed = True; band = 0
                    else:
                        held = H['tv'].get(tv, 0.0) + dt
                        if held >= seconds:
                            held = seconds; armed = True
                        H['tv'][tv] = held
                        band = seconds - held
                else:
                    H['tv'][tv] = 0.0
            if armed:
                for (limit, seconds, divisor) in limits:
                    if divisor > 0 and value > limit:
                        accrue += (value - limit) / divisor
            left.append(band)
        running.append(on); accrues.append(accrue)
        H['timers'][i] = left

    if H['useSystems']:
        for i in range(n):
            on = running[i]; d = dmg_get(H, i); accrue = accrues[i]
            if on and d > 0.25: accrue += worsen(d)
            h = H['oil'][i]
            if on and d > SYS_ENG_OIL_DROP_DMG:
                drain = (d - SYS_ENG_OIL_DROP_DMG) / SYS_ENG_OIL_DIVISOR
                if d > SYS_ENG_OIL_FAST_DMG:
                    drain += (d - SYS_ENG_OIL_FAST_DMG) / (SYS_ENG_OIL_DIVISOR * 2)
                h = max(h - drain * dt, 0.0)
            if d >= SYS_ENG_OIL_ZERO_DMG: h = 0.0
            H['oil'][i] = h
            if on and h <= 0:
                accrue += SYS_ENG_STARVE_RATE * (H['ng'][i] / SYS_ENG_STARVE_REF_NG)
                H['lowOil'][i] = True
            if accrue > 0:
                d = min(d + accrue * dt, 1.0); dmg_set(H, i, d)
            if d >= SYS_ENG_CHIPS_DMG: H['chips'][i] = True
            if d >= 1.0: H['failed'][i] = True
        return

    d = dmg_get(H, 0)
    accrue = max(accrues)
    if (True in running) and d > 0.25: accrue += worsen(d)
    if accrue > 0:
        d = min(d + accrue * dt, 1.0); dmg_set(H, 0, d)
    pick = H['pick']
    if d >= SYS_ENG_SHARED_FAIL_DMG and True not in H['failed']:
        H['failed'][pick if pick >= 0 else random.randrange(n)] = True
    if d >= SYS_ENG_SHARED_CHIPS_DMG and pick < 0 and True not in H['failed']:
        pick = random.randrange(n); H['pick'] = pick; H['chips'][pick] = True
    if d >= SYS_ENG_SHARED_CHIPS2_DMG:
        for i in range(n):
            if not H['failed'][i] and not H['chips'][i]: H['chips'][i] = True
    if d >= 1.0:
        for i in range(n):
            if not H['failed'][i]: H['failed'][i] = True

def engine_update_off(H):
    # fn_engineUpdate: a failed engine is forced OFF after the damage step
    for i, f in enumerate(H['failed']):
        if f: H['engState'][i] = 'OFF'

def run(H, dt=1/60, max_t=20000.0, watch=()):
    t = 0.0; events = []; seen = set()
    while t < max_t:
        step(H, dt); engine_update_off(H); t += dt
        for name, test in watch:
            if name not in seen and test(H):
                seen.add(name); events.append((t, name))
        if all(H['failed']): break
    return events
