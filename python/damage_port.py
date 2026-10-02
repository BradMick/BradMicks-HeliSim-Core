# Line-for-line port of fn_systemTorque.sqf (lines 85-140), worst-first exactly as the code reads it.
# limits: worst-first list of (limit, seconds, divisor). Returns (total_s, armed_at_s) or (None, armed_at).
def run(limits, tq, dt=1/60, max_t=200000.0, running=True, damage=0.0):
    timers = [0.0]*len(limits)
    t = 0.0
    armed_at = None
    while t < max_t:
        accrue = 0.0
        armed = False
        for i, (limit, seconds, divisor) in enumerate(limits):
            ceiling = 1e10 if i == 0 else limits[i-1][0]
            if running and tq > limit and tq <= ceiling:
                if seconds <= 0:
                    armed = True
                else:
                    held = timers[i] + dt
                    if held >= seconds:
                        held = seconds
                        armed = True
                    timers[i] = held
            else:
                timers[i] = 0.0
        if armed:
            if armed_at is None: armed_at = t
            for (limit, seconds, divisor) in limits:
                if divisor > 0 and tq > limit:
                    accrue += (tq - limit) / divisor
        if running and damage > 0.25:
            persistent = damage / 600.0
            if damage > 0.50: persistent = damage / 500.0
            if damage > 0.75: persistent = damage / 400.0
            accrue += persistent
        if accrue > 0:
            damage = min(damage + accrue*dt, 1.0)
        t += dt
        if damage >= 1.0:
            return t, armed_at
    return None, armed_at

def after_arm(limits, v):
    t, a = run(limits, v)
    return None if t is None else t - a

def bisect(f, target, lo=1e-6, hi=1e9, iters=80):
    # f(x) = after-arm time, increasing in x (bigger divisor = slower)
    for _ in range(iters):
        mid = (lo*hi) ** 0.5
        y = f(mid)
        if y is None or y > target: hi = mid
        else: lo = mid
    return (lo*hi) ** 0.5

# ---- Reference: the transmission, unchanged
XMSN = [(2.30, 0, 20), (2.00, 6, 10)]
T_LEFT  = after_arm(XMSN, 2.01)
T_RIGHT = after_arm(XMSN, 2.30)
print(f'REFERENCE transmission: left (201%) {T_LEFT:.1f} s, right (230%) {T_RIGHT:.1f} s  (after arming)\n')

def solve(name, bands_low_to_high, unit, fmt, tests):
    # bands_low_to_high: [(limit, grace), ...]; last one is the max
    lims = [b[0] for b in bands_low_to_high]
    graces = [b[1] for b in bands_low_to_high]
    n = len(lims)
    left_pt, right_pt = lims[0] + unit, lims[-1]

    def build(d0, dm):
        divs = [d0]
        for i in range(1, n-1):
            divs.append(dm * 2**(i-1))
        divs.append(divs[-1] * 2)
        return list(reversed(list(zip(lims, graces, divs)))), divs

    d0 = bisect(lambda x: after_arm(build(x, 1e9)[0], left_pt), T_LEFT)
    note = ''
    if n > 2:
        best_possible = after_arm(build(d0, 1e12)[0], right_pt)
        if best_possible is not None and best_possible < T_RIGHT:
            dm = 0.0
            note = f'  RIGHT END UNREACHABLE: the lowest band alone already gives {best_possible:.1f} s at the max'
        else:
            dm = bisect(lambda x: after_arm(build(d0, x)[0], right_pt), T_RIGHT)
    else:
        dm = 0.0
    cfg, divs = build(d0, dm if dm > 0 else 1e9)
    if n > 2 and dm == 0.0:
        divs = [d0] + [0.0]*(n-2) + [2*d0]
        cfg = list(reversed(list(zip(lims, graces, divs))))
    print(f'=== {name} ===')
    print('  config, worst-first {limit, seconds, divisor}:')
    print('   ', ', '.join('{' + f'{l:g}, {g:g}, {d:.4g}' + '}' for (l, g, d) in cfg))
    for v in tests:
        t, a = run(cfg, v)
        tag = '  <- left' if abs(v-left_pt) < 1e-9 else ('  <- right (max)' if abs(v-right_pt) < 1e-9 else '')
        print(f'    {fmt(v)}: total {t:.1f} s, damage starts {a:.1f} s, {t-a:.1f} s after{tag}')
    if n == 2:
        ra = after_arm(cfg, right_pt)
        if abs(ra - T_RIGHT) > 0.5:
            print(f'  RIGHT END NOT MET: {ra:.1f} s vs {T_RIGHT:.1f} s - two bands, the top band adds nothing at its own limit, so the lowest band\'s divisor sets both ends')
    if note: print(note)
    print()

pct = lambda v: f'{v:g}%'
solve('Nose gearbox SE', [(1.10,150),(1.22,6),(1.25,0)], 0.01, lambda v: f'{v*100:g}%', [1.11,1.15,1.18,1.20,1.22,1.23,1.24,1.25])
solve('Engine TQ', [(100,6),(115,0)], 1, pct, [101,105,108,110,112,115])
solve('Np', [(105,12),(121,0)], 1, pct, [106,110,115,118,121])
solve('Ng', [(102.2,12),(105.1,0)], 1, pct, [103.2,103.5,104,104.5,105.1])
solve('TGT', [(810,1800),(870,600),(878,150),(896,12),(949,0)], 1, lambda v: f'{v:g} C', [811,830,860,875,890,920,949])
