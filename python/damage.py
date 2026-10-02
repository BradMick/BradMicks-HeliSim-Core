"""The damage rig - a 1:1 port of fn_systemTorque.sqf and fn_engineDamage.sqf.

Same order, same gating, same expressions as the SQF; only the syntax changes. Engine limits
come from the AH-64D's helisim_engine.hpp through engine.py's own parser, and every SYS_*
constant is read from systems.hpp, so this cannot drift from Core's numbers - only from its
code. When either SQF function changes, change this to match in the same step.

    python damage.py          the standard report
"""
import random
import re

import engine as rig

SYSTEMS_HPP = r'E:\bmkhs_helisim\addons\helisim\functions\systems\systems.hpp'


def load_defines(path):
    out = {}
    for line in open(path, encoding='utf-8'):
        m = re.match(r'\s*#define\s+(\w+)\s+([-+0-9.eE]+)', line)
        if m:
            out[m.group(1)] = float(m.group(2))
    return out


SYS = load_defines(SYSTEMS_HPP)
ENGINE_FIELDS = ('ngLimits', 'npLimits', 'tqLimits', 'tgtLimits', 'tqLimitsSe', 'tgtLimitsSe', 'startTgt')


def load_engines(n=2):
    e = rig.CFG['Engines']['Engine01']
    eng = {k: e[k] for k in ENGINE_FIELDS if k in e}
    eng['startTgt'] = e['HotSection']['startTgt']
    return [dict(eng, damageRole='engines', damageRoleIndex=i) for i in range(n)]


def linear_conversion(a, b, x, c, d, clip=False):
    f = (x - a) / (b - a)
    if clip:
        f = max(0.0, min(1.0, f))
    return c + (d - c) * f


# ---------------------------------------------------------------------------------------------
# The aircraft - just the variables these two functions read and write
# ---------------------------------------------------------------------------------------------

class Heli:
    def __init__(self, use_systems=True, n=2, seed=1):
        self.rnd = random.Random(seed)
        self.v = {}
        self.hp = {}
        engines = load_engines(n)
        v = self.v
        v['bmkhs_useSystems'] = use_systems
        v['bmkhs_engines'] = engines
        v['bmkhs_engState'] = ['ON'] * n
        v['bmkhs_isSingleEng'] = False
        v['bmkhs_engPctNg'] = [0.90] * n
        v['bmkhs_engPctNp'] = [1.01] * n
        v['bmkhs_engPctTq'] = [0.50] * n
        v['bmkhs_engTgt'] = [700.0] * n
        v['isEngineOn'] = True
        #fn_engineVariables seeds
        v['bmkhs_engChips'] = [False] * n
        v['bmkhs_engFailed'] = [False] * n
        v['bmkhs_lowOilPsiFailure'] = [False] * n
        v['bmkhs_engOilHealth'] = [1.0] * n
        v['bmkhs_engLimitTimers'] = [[-1, -1, -1] for _ in range(n)]
        v['bmkhs_engTqTimer'] = [-1] * n
        v['bmkhs_engFailureResult'] = -1
        v['bmkhs_engClutchSlip'] = [1.0] * n
        v['bmkhs_engSlipT'] = [-1] * n
        v['bmkhs_engSlipWait'] = [self.rnd.random() * SYS['SYS_SLIP_WAIT_LOW_DMG'] for _ in range(n)]
        v['bmkhs_engSlipDepth'] = [0.0] * n
        #fn_damageVariables: engine hitpoints, or hitengine once per engine
        v['damagePoints'] = {'engines': ['hitengine%d' % (i + 1) for i in range(n)] if use_systems
                             else ['hitengine'] * n}
        v['bmkhs_sysTorqued'] = []

    def get(self, name, default=None):
        return self.v.get(name, default)

    #fn_damageGet / fn_damageSet
    def damage_get(self, role, index=-1):
        pts = self.v['damagePoints'].get(role, [])
        if not pts:
            return 0.0
        if index >= 0:
            return self.hp.get(pts[index], 0.0) if index < len(pts) else 0.0
        return max(self.hp.get(p, 0.0) for p in pts)

    def damage_set(self, role, damage, index=-1):
        pts = self.v['damagePoints'].get(role, [])
        if not pts:
            return
        for p in ([pts[index]] if index >= 0 else pts):
            self.hp[p] = damage


def component(role, torque_sum, limits_from='', limits_se_from='', index=0, jitters=True, damages=()):
    return dict(damageRole=role, index=index, torqueFrom='bmkhs_engPctTq', torqueSum=torque_sum,
                tqLimitsFrom=limits_from, tqLimitsSeFrom=limits_se_from, jitters=jitters,
                damages=list(damages), breaksVar=[])


def apache_drivetrain(h):
    h.v['damagePoints'].update(transmission=['transmission'], noseGearboxes=['ngb1', 'ngb2'])
    h.v['bmkhs_sysTorqued'] = [component('transmission', True, 'tqLimits'),
                               component('noseGearboxes', False, '', 'tqLimitsSe', index=0),
                               component('noseGearboxes', False, '', 'tqLimitsSe', index=1)]


def no_systems_drivetrain(h):
    h.v['bmkhs_sysTorqued'] = [component('transmission', True, 'tqLimits', 'tqLimitsSe',
                                         jitters=False, damages=('hithrotor', 'hitvrotor'))]


# ---------------------------------------------------------------------------------------------
# fn_systemTorque
# ---------------------------------------------------------------------------------------------

def system_torque(h, dt):
    v = h.v
    torqued = v['bmkhs_sysTorqued']
    if not torqued:
        return
    engines = v['bmkhs_engines']
    tqTimers = [-1] * len(engines)
    driveDmg = [0] * len(engines)
    for comp in torqued:
        role = comp['damageRole']
        index = comp['index']

        def from_engine(key, single):
            if key == '':
                return []
            if comp['torqueSum']:
                n = 1 if single else len(engines)
                return [(x[0] * n, x[1], (x[2] if len(x) > 2 else 0) * n) for x in engines[0][key]]
            return engines[min(index, len(engines) - 1)][key]
        limits = from_engine(comp['tqLimitsFrom'], False)
        seLimits = from_engine(comp['tqLimitsSeFrom'], True)
        if seLimits and v['bmkhs_isSingleEng']:
            limits = seLimits

        val = v.get(comp['torqueFrom'], 0)
        tq = (sum(val) if comp['torqueSum'] else (val[index] if index < len(val) else 0)) \
            if isinstance(val, list) else val

        direct = comp['damages']
        damage = h.damage_get(role, index) if not direct else max(h.hp.get(p, 0.0) for p in direct)
        accrue = 0.0
        running = v['isEngineOn']

        armed = False
        band = -1
        for k, lim in enumerate(limits):
            limit, seconds = lim[0], lim[1]
            ceiling = 1e10 if k == len(limits) - 1 else limits[k + 1][0]
            timerVar = 'bmkhs_tqTimer_%s%d_%d' % (role, index, k)
            if running and tq > limit and tq <= ceiling:
                if seconds <= 0:
                    armed = True
                    band = 0
                else:
                    held = v.get(timerVar, 0) + dt
                    if held >= seconds:
                        held = seconds
                        armed = True
                    v[timerVar] = held
                    band = seconds - held
            else:
                v[timerVar] = 0

        if band >= 0:
            for e, x in enumerate(list(tqTimers)):
                if comp['torqueSum'] or e == index:
                    tqTimers[e] = min(band, x) if x >= 0 else band

        if armed:
            for lim in limits:
                limit, divisor = lim[0], (lim[2] if len(lim) > 2 else 0)
                if divisor > 0 and tq > limit:
                    accrue += (tq - limit) / divisor

        if running and damage > 0.25:
            persistent = damage / 600.0
            if damage > 0.50: persistent = damage / 500.0
            if damage > 0.75: persistent = damage / 400.0
            accrue += persistent

        if accrue > 0:
            damage = min(damage + accrue * dt, 1.0)
            if not direct:
                h.damage_set(role, damage, index)
            else:
                for p in direct:
                    h.hp[p] = damage

        if comp['jitters']:
            for e, x in enumerate(list(driveDmg)):
                if comp['torqueSum'] or e == index:
                    driveDmg[e] = max(x, damage)

    v['bmkhs_engTqTimer'] = tqTimers

    for i, x in enumerate(driveDmg):
        slip = 1.0
        t = v['bmkhs_engSlipT'][i]
        if v['isEngineOn'] and x > 0.25:
            if t < 0:
                wait = v['bmkhs_engSlipWait'][i] - dt
                if wait <= 0:
                    t = 0
                    v['bmkhs_engSlipDepth'][i] = x * SYS['SYS_SLIP_DEPTH'] * (0.5 + h.rnd.random() * 0.5)
                    wait = linear_conversion(0.25, 1.0, x, SYS['SYS_SLIP_WAIT_LOW_DMG'],
                                             SYS['SYS_SLIP_WAIT_HIGH_DMG'], True) * (0.8 + h.rnd.random() * 0.4)
                v['bmkhs_engSlipWait'][i] = wait
            else:
                t = t + dt
            if t >= 0:
                d = v['bmkhs_engSlipDepth'][i]
                if t < SYS['SYS_SLIP_DROP_SEC']:
                    slip = 1.0 - d * (t / SYS['SYS_SLIP_DROP_SEC'])
                elif t < SYS['SYS_SLIP_GRAB_SEC']:
                    slip = 1.0 + linear_conversion(SYS['SYS_SLIP_DROP_SEC'], SYS['SYS_SLIP_GRAB_SEC'], t,
                                                   -d, d * SYS['SYS_SLIP_OVERSHOOT'])
                elif t < SYS['SYS_SLIP_END_SEC']:
                    slip = 1.0 + linear_conversion(SYS['SYS_SLIP_GRAB_SEC'], SYS['SYS_SLIP_END_SEC'], t,
                                                   d * SYS['SYS_SLIP_OVERSHOOT'], 0)
                else:
                    t = -1
        else:
            t = -1
        v['bmkhs_engSlipT'][i] = t
        v['bmkhs_engClutchSlip'][i] = slip


# ---------------------------------------------------------------------------------------------
# fn_engineDamage
# ---------------------------------------------------------------------------------------------

def worsen(damage):
    persistent = damage / 600.0
    if damage > 0.50: persistent = damage / 500.0
    if damage > 0.75: persistent = damage / 400.0
    return persistent


def engine_damage(h, dt):
    v = h.v
    engines = v['bmkhs_engines']
    state = v['bmkhs_engState']
    readings = [('np', 'npLimits', v['bmkhs_engPctNp']),
                ('ng', 'ngLimits', v['bmkhs_engPctNg']),
                ('tgt', 'tgtLimitsSe' if v['bmkhs_isSingleEng'] else 'tgtLimits', v['bmkhs_engTgt'])]

    running, accrues = [], []
    for i, engine in enumerate(engines):
        on = state[i] in ('STARTING', 'ON')
        accrue = 0.0
        left = []
        for name, key, values in readings:
            limits = engine[key]
            value = values[i]
            band = -1
            armed = False
            for k, lim in enumerate(limits):
                limit, seconds = lim[0], lim[1]
                ceiling = 1e10 if k == len(limits) - 1 else limits[k + 1][0]
                timerVar = 'bmkhs_engTimer_%s%d_%d' % (name, i, k)
                if on and value > limit and value <= ceiling:
                    if seconds <= 0:
                        armed = True
                        band = 0
                    else:
                        held = v.get(timerVar, 0) + dt
                        if held >= seconds:
                            held = seconds
                            armed = True
                        v[timerVar] = held
                        band = seconds - held
                else:
                    v[timerVar] = 0
            if armed:
                for lim in limits:
                    limit, divisor = lim[0], (lim[2] if len(lim) > 2 else 0)
                    if divisor > 0 and value > limit:
                        accrue += (value - limit) / divisor
            left.append(band)

        tgtNow = v['bmkhs_engTgt'][i]
        if state[i] == 'STARTING' and tgtNow > engine['startTgt']:
            accrue += (tgtNow - engine['startTgt']) / SYS['SYS_ENG_HOTSTART_DIVISOR']

        running.append(on)
        accrues.append(accrue)
        v['bmkhs_engLimitTimers'][i] = left

    if v['bmkhs_useSystems']:
        for i, engine in enumerate(engines):
            role, index = engine['damageRole'], engine['damageRoleIndex']
            on = running[i]
            damage = h.damage_get(role, index)
            accrue = accrues[i]
            if on and damage > 0.25:
                accrue += worsen(damage)
            health = v['bmkhs_engOilHealth'][i]
            if on and damage > SYS['SYS_ENG_OIL_DROP_DMG']:
                drain = (damage - SYS['SYS_ENG_OIL_DROP_DMG']) / SYS['SYS_ENG_OIL_DIVISOR']
                if damage > SYS['SYS_ENG_OIL_FAST_DMG']:
                    drain += (damage - SYS['SYS_ENG_OIL_FAST_DMG']) / (SYS['SYS_ENG_OIL_DIVISOR'] * 2)
                health = max(health - drain * dt, 0)
            if damage >= SYS['SYS_ENG_OIL_ZERO_DMG']:
                health = 0
            v['bmkhs_engOilHealth'][i] = health
            if on and health <= 0:
                accrue += SYS['SYS_ENG_STARVE_RATE'] * (v['bmkhs_engPctNg'][i] / SYS['SYS_ENG_STARVE_REF_NG'])
                v['bmkhs_lowOilPsiFailure'][i] = True
            if accrue > 0:
                damage = min(damage + accrue * dt, 1.0)
                h.damage_set(role, damage, index)
            if damage >= SYS['SYS_ENG_CHIPS_DMG']:
                v['bmkhs_engChips'][i] = True
            if damage >= 1.0:
                v['bmkhs_engFailed'][i] = True
        return

    role = engines[0]['damageRole']
    damage = h.damage_get(role, 0)
    n = len(engines)
    accrue = max(accrues)
    if (True in running) and damage > 0.25:
        accrue += worsen(damage)
    for e, x in enumerate(v['bmkhs_engOilHealth']):
        if running[e] and x <= 0:
            accrue += SYS['SYS_ENG_STARVE_RATE'] * (v['bmkhs_engPctNg'][e] / SYS['SYS_ENG_STARVE_REF_NG'])
    if accrue > 0:
        damage = min(damage + accrue * dt, 1.0)
        h.damage_set(role, damage)

    def fault(e):
        if h.rnd.random() < 0.5:
            v['bmkhs_engChips'][e] = True
        else:
            v['bmkhs_engOilHealth'][e] = 0
            v['bmkhs_lowOilPsiFailure'][e] = True

    pick = v['bmkhs_engFailureResult']
    if damage >= SYS['SYS_ENG_SHARED_FAIL_DMG'] and True not in v['bmkhs_engFailed']:
        v['bmkhs_engFailed'][pick if pick >= 0 else int(h.rnd.random() * n)] = True
    if damage >= SYS['SYS_ENG_SHARED_CHIPS_DMG'] and pick < 0 and True not in v['bmkhs_engFailed']:
        pick = int(h.rnd.random() * n)
        v['bmkhs_engFailureResult'] = pick
        fault(pick)
    if damage >= SYS['SYS_ENG_SHARED_CHIPS2_DMG']:
        for e in range(n):
            if not v['bmkhs_engFailed'][e] and not v['bmkhs_engChips'][e] and not v['bmkhs_lowOilPsiFailure'][e]:
                fault(e)
    if damage >= 1.0:
        for e in range(n):
            if not v['bmkhs_engFailed'][e]:
                v['bmkhs_engFailed'][e] = True


def engine_update_off(h):
    """fn_engineUpdate: a failed engine is held OFF."""
    for i, f in enumerate(h.v['bmkhs_engFailed']):
        if f:
            h.v['bmkhs_engState'][i] = 'OFF'


# ---------------------------------------------------------------------------------------------
# The standard report
# ---------------------------------------------------------------------------------------------

DT = 1 / 60.0


def drivetrain_time(setup, tq, single=False, use_systems=True, max_t=5000.0):
    """Seconds from damage starting to destroyed, torque held; the part named by setup's first entry."""
    h = Heli(use_systems=use_systems)
    setup(h)
    h.v['bmkhs_engPctTq'] = list(tq)
    h.v['bmkhs_isSingleEng'] = single
    comp = h.v['bmkhs_sysTorqued'][0]
    t, armed_at = 0.0, None
    while t < max_t:
        before = h.damage_get(comp['damageRole'], comp['index']) if not comp['damages'] else \
            max(h.hp.get(p, 0.0) for p in comp['damages'])
        system_torque(h, DT)
        after = h.damage_get(comp['damageRole'], comp['index']) if not comp['damages'] else \
            max(h.hp.get(p, 0.0) for p in comp['damages'])
        t += DT
        if armed_at is None and after > before:
            armed_at = t - DT
        if after >= 1.0:
            return t - armed_at
    return None


def engine_time(use_systems, single=False, **readings):
    h = Heli(use_systems=use_systems)
    h.v['bmkhs_isSingleEng'] = single
    for k, val in readings.items():
        h.v[k] = list(val)
    t = 0.0
    events = {}
    while t < 20000.0:
        engine_damage(h, DT)
        engine_update_off(h)
        t += DT
        v = h.v
        for name, hit in (('chips', any(v['bmkhs_engChips'])), ('oil 0', any(v['bmkhs_lowOilPsiFailure'])),
                          ('first failed', any(v['bmkhs_engFailed'])), ('all failed', all(v['bmkhs_engFailed']))):
            if hit and name not in events:
                events[name] = t
        if all(v['bmkhs_engFailed']):
            break
    return events


def hot_start_damage():
    """The rig's uncaught hot start (restart from residual heat, no motoring), fed frame by frame."""
    a = rig.Heli()
    H = a.H
    rig.start_to(a, [0], 'IDLE', 30.0)
    H['pwrLvr'][0] = 0.0
    rig.run_until(a, a.t + 30.0)
    h = Heli(use_systems=True)
    h.v['bmkhs_engState'] = ['OFF', 'OFF']
    t0 = a.t
    H['startSw'][0] = 1
    first = True
    peak = 0.0
    while a.t < t0 + 20:
        if H['pwrLvr'][0] == 0.0 and a.ng(0) > 0.02:
            H['pwrLvr'][0] = rig.IDLE
        a.frame(rig.DT)
        if first:
            H['startSw'][0] = 0
            first = False
        h.v['bmkhs_engState'][0] = H['bmkhs_engState'][0]
        h.v['bmkhs_engTgt'][0] = a.tgt(0)
        h.v['bmkhs_engPctNg'][0] = a.ng(0)
        h.v['bmkhs_engPctNp'][0] = 0.0
        peak = max(peak, a.tgt(0))
        engine_damage(h, rig.DT)
    return peak, h.damage_get('engines', 0)


def report():
    print('DAMAGE RIG - 1:1 fn_systemTorque / fn_engineDamage, limits from the AH-64D config')
    print()
    print('Drivetrain, seconds from damage starting to destroyed (target 595 s left, 32.3 s max):')
    for label, setup, tq, single in (
            ('Apache transmission, twin, 2.01 sum', apache_drivetrain, (1.005, 1.005), False),
            ('Apache transmission, twin, 2.30 sum', apache_drivetrain, (1.15, 1.15), False),
            ('No-systems rotors, twin, 2.01 sum', no_systems_drivetrain, (1.005, 1.005), False),
            ('No-systems rotors, twin, 2.30 sum', no_systems_drivetrain, (1.15, 1.15), False),
            ('No-systems rotors, single, 1.11', no_systems_drivetrain, (1.11, 0.0), True),
            ('No-systems rotors, single, 1.25', no_systems_drivetrain, (1.25, 0.0), True)):
        print('  %-40s %s' % (label, '%.1f s' % drivetrain_time(setup, tq, single)))
    ngb = lambda h: (apache_drivetrain(h), h.v.__setitem__('bmkhs_sysTorqued', h.v['bmkhs_sysTorqued'][1:]))
    for tq in (1.11, 1.25):
        print('  %-40s %.1f s' % ('Apache nose gearbox 1, single, %.2f' % tq, drivetrain_time(ngb, (tq, 0.0), True)))
    print()
    print('Engine, useSystems = 1, one engine held (events from t = 0):')
    for label, single, readings in (
            ('TGT 949 C twin, Ng 95%', False, dict(bmkhs_engTgt=(949, 700), bmkhs_engPctNg=(0.95, 0.9))),
            ('TGT 949 C single, Ng 95%', True, dict(bmkhs_engTgt=(949, 700), bmkhs_engPctNg=(0.95, 0.9))),
            ('TGT 890 C twin', False, dict(bmkhs_engTgt=(890, 700))),
            ('TGT 890 C single', True, dict(bmkhs_engTgt=(890, 700))),
            ('Np 121%', False, dict(bmkhs_engPctNp=(1.21, 1.01))),
            ('Ng 105.1%', False, dict(bmkhs_engPctNg=(1.051, 0.9))),
            ('Torque 115% - never damages the engine', False, dict(bmkhs_engPctTq=(1.15, 1.15)))):
        ev = engine_time(True, single, **readings)
        print('  %-40s %s' % (label, ', '.join('%s %.1f s' % (k, t) for k, t in ev.items()) or 'no damage'))
    print()
    print('Engine, useSystems = 0 (shared hitengine), engine 1 at 900 C, twin:')
    for seed in (1, 2, 3):
        h = Heli(use_systems=False, seed=seed)
        h.v['bmkhs_engTgt'] = [900.0, 700.0]
        h.v['bmkhs_engPctNg'] = [0.95, 0.95]
        t, log, seen = 0.0, [], set()
        while t < 3000 and not all(h.v['bmkhs_engFailed']):
            engine_damage(h, DT)
            engine_update_off(h)
            t += DT
            v = h.v
            for e in range(2):
                for name, key in (('chips', 'bmkhs_engChips'), ('oil psi', 'bmkhs_lowOilPsiFailure'),
                                  ('failed', 'bmkhs_engFailed')):
                    if v[key][e] and (name, e) not in seen:
                        seen.add((name, e))
                        log.append('%.1f s %s eng %d' % (t, name, e + 1))
        print('  seed %d: %s' % (seed, ' | '.join(log)))
    print()
    peak, dmg = hot_start_damage()
    print('Hot start, uncaught (engine rig trace): peak %.0f C, engine damage %.3f (target 0.60)' % (peak, dmg))


if __name__ == '__main__':
    report()
