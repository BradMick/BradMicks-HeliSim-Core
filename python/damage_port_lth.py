# Line-for-line port of fn_systemTorque.sqf lines 83-140 AS IT NOW READS: low to high, ceiling
# from the next tier, continuous damage gated on _running.
def run(limits, tq, dt=1/60, max_t=200000.0, running=True, damage=0.0):
    timers = [0.0]*len(limits)
    t = 0.0
    armed_at = None
    while t < max_t:
        accrue = 0.0
        armed = False
        for i, (limit, seconds, divisor) in enumerate(limits):
            ceiling = 1e10 if i == len(limits) - 1 else limits[i+1][0]
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

if __name__ == '__main__':
    # Exactly the arrays now in config.
    cases = [
        ('Transmission', [(2.00, 6, 10), (2.30, 0, 20)], [2.01, 2.05, 2.10, 2.15, 2.20, 2.25, 2.30]),
        ('Nose gearbox SE', [(1.10, 150, 10), (1.22, 6, 2), (1.25, 0, 4)],
            [1.11, 1.15, 1.18, 1.20, 1.22, 1.23, 1.24, 1.25]),
        ('Engine TQ', [(1.00, 6, 10), (1.15, 0, 20)], [1.01, 1.05, 1.08, 1.10, 1.12, 1.15]),
        ('Np', [(1.05, 12, 10), (1.21, 0, 20)], [1.06, 1.10, 1.15, 1.18, 1.21]),
        ('Ng', [(1.022, 12, 10), (1.051, 0, 20)], [1.032, 1.035, 1.04, 1.045, 1.051]),
        ('TGT', [(810, 1800, 1000), (870, 600, 0), (878, 150, 0), (896, 12, 0), (949, 0, 2000)],
            [811, 830, 860, 875, 890, 920, 949]),
    ]
    for name, cfg, pts in cases:
        print(name)
        for v in pts:
            t, a = run(cfg, v)
            print(f'  {v:g}: total {t:.1f} s, {t-a:.1f} s after arming')
