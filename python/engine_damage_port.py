# Port of the per-engine useSystems = 1 path of fn_engineDamage.sqf, same order as the SQF:
# exceedance accrue (none here) + self-worsening from the damage at the top of the frame,
# oil drain from that same damage, forced zero at OIL_ZERO, starvation once oil is 0, then
# the damage update.
OIL_DROP, OIL_FAST, OIL_ZERO = 0.65, 0.75, 0.85
STARVE_REF_NG = 0.75

def step(d, health, ng, D, S, dt, on=True):
    accrue = 0.0
    if on and d > 0.25:
        p = d / 600.0
        if d > 0.50: p = d / 500.0
        if d > 0.75: p = d / 400.0
        accrue += p
    if on and d > OIL_DROP:
        drain = (d - OIL_DROP) / D
        if d > OIL_FAST: drain += (d - OIL_FAST) / (2 * D)
        health = max(health - drain * dt, 0.0)
    if d >= OIL_ZERO:
        health = 0.0
    starving = on and health <= 0.0
    if starving:
        accrue += S * (ng / STARVE_REF_NG)
    if accrue > 0:
        d = min(d + accrue * dt, 1.0)
    return d, health, starving

def run(d0, ng, D, S, dt=1/60, max_t=5000.0):
    d, h, t = d0, 1.0, 0.0
    marks = {}
    while t < max_t:
        d, h, starving = step(d, h, ng, D, S, dt)
        t += dt
        if 'oil0' not in marks and h <= 0.0: marks['oil0'] = (t, d)
        if 'd85' not in marks and d >= OIL_ZERO: marks['d85'] = (t, d)
        if 'starve' not in marks and starving: marks['starve'] = t
        if d >= 1.0:
            marks['fail'] = t
            return marks
    return marks

def bisect(f, lo, hi, iters=80):
    # f(x) < 0 means x too small
    for _ in range(iters):
        mid = (lo * hi) ** 0.5
        if f(mid) < 0: lo = mid
        else: hi = mid
    return (lo * hi) ** 0.5

if __name__ == '__main__':
    # D: with nothing else happening (no starvation yet - S=0), oil reaches 0 exactly as damage reaches 0.85.
    def oil_vs_damage(D):
        m = run(0.6500001, 0.75, D, 0.0)
        t_oil = m['oil0'][0] if 'oil0' in m else 1e9
        # natural (unforced) zero: health hitting 0 strictly before the forced zero at 0.85
        t85 = m['d85'][0]
        return t85 - t_oil if t_oil < t85 - 1e-9 else -(1.0)  # >0 once the drain is strong enough to hit 0 before 0.85
    # find the WEAKEST drain (largest D) that still reaches 0 by 0.85 on its own
    lo, hi = 1e-3, 1e3
    for _ in range(100):
        mid = (lo * hi) ** 0.5
        m = run(0.6500001, 0.75, mid, 0.0)
        natural = 'oil0' in m and m['oil0'][1] < OIL_ZERO
        if natural: lo = mid    # still reaches 0 before 0.85 -> can weaken (raise D)
        else: hi = mid
    D = lo
    m = run(0.6500001, 0.75, D, 0.0)
    print(f'oil divisor D = {D:.6g} (tiers D, 2D)')
    print(f'  from 0.65: oil 0 at {m["oil0"][0]:.1f} s (damage {m["oil0"][1]:.4f}); damage 0.85 at {m["d85"][0]:.1f} s')

    # S: from the moment oil hits 0 at 0.85, at 75% Ng, destroyed in 35.5 s (stacked with self-worsening).
    def time_after_oil0(S, ng):
        m = run(0.6500001, ng, D, S)
        return m['fail'] - m['starve']
    S = bisect(lambda s: 35.5 - time_after_oil0(s, 0.75), 1e-7, 1.0)
    print(f'starvation rate S = {S:.6g} per s at Ng {STARVE_REF_NG}, scaled by Ng/{STARVE_REF_NG}')
    for ng in (0.68, 0.75, 0.85, 0.95, 1.00):
        print(f'  Ng {ng:.2f}: oil 0 -> failure in {time_after_oil0(S, ng):.1f} s')
