"""The HeliSim gas turbine, outside Arma. The spec the SQF is written from.

Run it:  python tools/engine/engine.py

Prints the derived values, the loaded equilibrium, a cold start with its torque trace, the
shutdown marks, motoring, both hot-start branches, and the stationary-rotor case - once with
the power turbine's demand cap and once without, so a change to that term is a diff.

    cap=False   tq = surplus                what the SQF does. All of it goes to the shaft.
    cap=True    tq = min(shaft, surplus)     the original. Deadlocks a stationary rotor:
                                             no Nr -> no drag -> no demand -> no torque.

Nothing else differs between the two runs. If this file and the SQF ever disagree on
BEHAVIOUR, this file is right; on VALUES, the config schema in the plan is authoritative.
"""

ISA_RHO = 1.225


class Engine:
    #The power turbine's demand cap, kept only so a change to that term is a diff rather than
    #an argument. False is what the SQF now does: the turbine passes its surplus to the shaft
    #and the transmission finds equilibrium. True was the original, and it deadlocks a
    #stationary rotor - see the arma_start() driver. Per instance, never a module global.
    cap = False

    #torque = power / omega in the power turbine, bounded by stall. False is what the SQF
    #does today: no speed term at all, so torque is only right at governed Np.
    speedTerm = True
    stallTqMult = 2.5
    ptIdleExtract = 0.05
    #The 701C suppresses torque spikes below this Np - below it the gauge reads the demanded
    #torque, above it the power/omega spike is what the sensor sees.
    spikeNp = 0.39

    designRpm, npFly, powerKw, maxFuelFlow = 20900.0, 1.01, 1066.0, 0.12
    compressorLoad, massFlowExp, tgtK, spoolInertia = 1.7, 1.7, 276.0, 5.0
    unfiredDragMult, unfiredFriction = 3.0, 0.10      # COASTING ONLY
    thermalMassCoef, coolingCoef, stillAirFlow = 0.30, 0.70, 0.0012
    idleTq, flyTq, ptEfficiency = 0.055, 0.18, 0.92
    lightOffNg, selfSustNg = 0.15, 0.52
    startTgt, startMinTgt = 851.0, 80.0
    residualHeatGain, startFuelBase = 0.003, 0.42
    maxTgt, maxNg = 867.0, 1.022
    ngLimitBase, ngLimitSlope = 1.01436, 0.0019091
    starterTorque = 0.30

    # plan aliases
    hotStartCarry = residualHeatGain
    thermalMass = thermalMassCoef
    cooling = coolingCoef
    soak = stillAirFlow

    def __init__(self, fat=15.0, rho=ISA_RHO, cap=False, speedTerm=True):
        self.fat, self.rho, self.cap, self.speedTerm = fat, rho, cap, speedTerm
        self.fuelIdle = self.compressorLoad * 0.679**2 + self.idleTq / self.ptEfficiency
        self.fuelFly = self.compressorLoad * 0.834**2 + self.flyTq / self.ptEfficiency
        self.idleNg = ((self.fuelIdle - self.idleTq / self.ptEfficiency)
                       / self.compressorLoad) ** 0.5
        self.refTq = (self.powerKw * 1000.0) / (self.designRpm * self.npFly * 0.10472)
        self.ng, self.tgt, self.tq = 0.0, fat, 0.0
        self.hotFac = 1.0
        self.lever, self.starting, self.override = 'OFF', False, False

    def setLever(self, pos):
        if self.lever == 'OFF' and pos != 'OFF':
            self.hotFac = 1.0 + self.hotStartCarry * self.tgt
        self.lever = pos

    def step(self, dt, loadTq=0.0, velY=0.0, npFrac=1.0):
        """npFrac: rotor speed as a fraction of governed. 1.0 reproduces the old behaviour,
        which is what every acceptance number was produced at."""
        dens = self.rho / ISA_RHO
        lit = self.ng > self.lightOffNg and self.lever != 'OFF'
        cranking = (self.starting or self.override) and self.ng < self.selfSustNg
        coasting = not lit and not cranking

        floor = {'FLY': self.fuelFly, 'IDLE': self.fuelIdle, 'OFF': 0.0}[self.lever]
        if self.ng < self.idleNg:
            floor *= min(1.0, self.startFuelBase
                         + (1 - self.startFuelBase) * self.ng / self.idleNg)
        fuel = floor

        st = self.starterTorque if cranking else 0.0

        gas = fuel * dens if lit else 0.0
        shaft = (loadTq / self.refTq) / self.ptEfficiency if lit else 0.0
        drag = self.compressorLoad * (self.unfiredDragMult if coasting else 1.0)
        absorbed = drag * self.ng**2 + (self.unfiredFriction if coasting else 0.0)
        self.ng = max(0.0, min(self.ng
                     + ((gas + st - absorbed - shaft) / self.spoolInertia) * dt, 1.1))

        mflow = max((self.ng ** self.massFlowExp) * dens, 0.02)
        fac = 1.0 + (self.hotFac - 1.0) * max(0.0, 1.0 - self.ng / self.idleNg)
        hot = self.fat + fac * self.tgtK * fuel / mflow if lit else self.fat
        rate = (self.thermalMass if hot > self.tgt
                else self.cooling * (self.ng + self.soak + velY / 128.611))
        self.tgt += (hot - self.tgt) * rate * dt

        #The share of gas output the free turbine gets: what the compressor leaves, floored so
        #gas moving over the turbine always turns it, motoring included.
        gasOutput = gas * self.refTq
        share = max(self.ptIdleExtract, (gas - absorbed) / gas) if gas > 0.0 else 0.0

        #Scaled by how fast it is turning, up to ptEfficiency at governed speed.
        self.tq = gasOutput * share * self.ptEfficiency
        self.gaugeTq = min(self.tq / max(npFrac, 1e-6), self.stallTqMult * self.refTq)
        return self.ng, self.tgt, self.tq / self.refTq, fuel


DT = 1 / 60.0


def _load(e):
    return e.idleTq * e.refTq * min(1.0, e.ng / e.idleNg)


def cold_start(tgt0=15.0, secs=40, cap=False):
    e = Engine(cap=cap); e.tgt = tgt0; e.starting = True
    t = 0.0; tLight = tCut = None; peak = tgt0; tPeak = 0.0
    trace = []
    while t < secs:
        if e.lever == 'OFF' and e.ng > 0.02:
            e.setLever('IDLE')
        if e.ng > e.lightOffNg and e.lever != 'OFF' and tLight is None:
            tLight = t
        if e.starting and e.ng >= e.selfSustNg and tCut is None:
            tCut = t; e.starting = False
        e.step(DT, loadTq=_load(e))
        if e.tgt > peak: peak, tPeak = e.tgt, t
        t += DT
        trace.append((t, e.ng, e.tgt, e.tq / e.refTq))
    return dict(light=tLight, cutout=tCut, peak=peak, tPeak=tPeak, ng=e.ng, tgt=e.tgt,
                tq=e.tq / e.refTq, trace=trace)


def shutdown(secs=3700, cap=False):
    e = Engine(cap=cap); e.ng, e.tgt, e.lever = 0.679, 465.0, 'OFF'
    t = 0.0; stop = None; marks = {}
    while t < secs:
        e.step(DT)
        if e.ng <= 0 and stop is None: stop = t
        t += DT
        for s in (2, 5, 9.5, 30, 300, 1200, 3600):
            if abs(t - s) < DT / 2: marks[s] = (e.ng, e.tgt)
    return dict(stop=stop, marks=marks)


def motoring(tgt0=163.0, secs=30, cap=False):
    e = Engine(cap=cap); e.ng, e.tgt, e.lever, e.override = 0.0, tgt0, 'OFF', True
    t = 0.0; m = {}
    while t < secs:
        e.step(DT); t += DT
        for lim in (100, 80):
            if lim not in m and e.tgt < lim: m[lim] = t
    return dict(below100=m.get(100), below80=m.get(80), ng=e.ng)


def hot_start_abort(tgt0=163.0, abortAt=700.0, ovrDelay=2.0, secs=60, cap=False):
    e = Engine(cap=cap); e.tgt = tgt0; e.starting = True
    t = 0.0; peak = tgt0; tAb = None; t540 = None
    while t < secs:
        if e.lever == 'OFF' and e.ng > 0.02 and tAb is None:
            e.setLever('IDLE')
        if tAb is None and e.tgt > abortAt:
            tAb = t; e.lever, e.starting = 'OFF', False
        if tAb is not None and t >= tAb + ovrDelay:
            e.override = True
        e.step(DT, loadTq=_load(e))
        peak = max(peak, e.tgt)
        if tAb and t540 is None and e.tgt < 540: t540 = t - tAb
        t += DT
    return dict(abortAt=tAb, peak=peak, t540=t540, tgt=e.tgt)


def equilibrium():
    e = Engine(); out = []
    for tq, ng, dec in [(0.055, 0.679, 460), (0.18, 0.834, 532), (0.84, 0.930, None),
                        (1.00, 0.951, 810), (1.29, 1.010, 867)]:
        fuel = e.compressorLoad * ng ** 2 + tq / e.ptEfficiency
        tgt = 15 + e.tgtK * fuel / (ng ** e.massFlowExp)
        out.append((tq, ng, tgt, dec))
    return out


#Acceptance numbers from the plan, for automatic comparison rather than eyeballing.
EXPECTED = dict(refTq=482.2, idleNg=0.6790, fuelIdle=0.844, fuelFly=1.378,
                light=2.6, cutout=5.5, peakLo=646.0, peakHi=661.0,
                stop=9.5, residual=163.0, below80=6.6, motorNg=0.420)


def report(cap):
    e = Engine(cap=cap)
    label = 'WITH CAP - the original, deadlocks' if cap else 'WITHOUT CAP - as shipped'
    print('=' * 70)
    print('%s   (cap = %s)' % (label, cap))
    print('=' * 70)
    print('derived: refTq %.1f  idleNg %.4f  fuelIdle %.3f  fuelFly %.3f'
          % (e.refTq, e.idleNg, e.fuelIdle, e.fuelFly))

    print('\n-- loaded equilibrium (algebraic: cap cannot affect it) --')
    print('     %TQ     Ng    TGT   declared   err')
    for tq, ng, tgt, dec in equilibrium():
        err = '' if dec is None else '%+d' % round(tgt - dec)
        print('   %5.1f  %.3f   %4.0f   %8s  %4s'
              % (tq * 100, ng, tgt, dec if dec else '-', err))

    cs = cold_start(cap=cap)
    print('\n-- cold start, lever to IDLE at first Ng rise, FAT 15 --')
    print('   light-off  %.1fs        (expect %.1f)' % (cs['light'], EXPECTED['light']))
    print('   cutout     %.1fs        (expect %.1f)' % (cs['cutout'], EXPECTED['cutout']))
    print('   peak TGT   %.0fC at %.1fs (expect %.0f-%.0f)'
          % (cs['peak'], cs['tPeak'], EXPECTED['peakLo'], EXPECTED['peakHi']))
    print('   settled    ng %.3f  tgt %.0fC  tq %.1f%%' % (cs['ng'], cs['tgt'], cs['tq'] * 100))

    print('\n   torque through the start (what the cap changes):')
    print('        t     Ng    TGT    %TQ')
    for mark in (1.0, 2.0, 2.6, 3.0, 4.0, 5.0, 5.5, 7.0, 10.0, 15.0, 20.0, 24.0, 30.0):
        row = min(cs['trace'], key=lambda r: abs(r[0] - mark))
        print('   %6.1f  %.3f   %4.0f  %5.1f' % (row[0], row[1], row[2], row[3] * 100))

    sd = shutdown(cap=cap)
    print('\n-- shutdown, lever OFF from stabilised idle --')
    print('   spool stops %.1fs      (expect %.1f)' % (sd['stop'], EXPECTED['stop']))
    for s in (2, 5, 9.5, 30, 300, 1200, 3600):
        if s in sd['marks']:
            ng, tgt = sd['marks'][s]
            print('   %7ss  ng %.3f  tgt %.0fC' % (s, ng, tgt))

    mo = motoring(cap=cap)
    print('\n-- motoring from the 163C residual, start sw to ORIDE --')
    print('   <100C %.1fs  <80C %.1fs (expect %.1f)  steady ng %.3f (expect %.3f)'
          % (mo['below100'], mo['below80'], EXPECTED['below80'], mo['ng'], EXPECTED['motorNg']))

    print('\n-- hot start --')
    hs = cold_start(163.0, cap=cap)
    print('   uncaught peak %.0fC   (over the 851 start limit: %s)'
          % (hs['peak'], 'YES' if hs['peak'] > 851 else 'NO - WRONG'))
    ha = hot_start_abort(cap=cap)
    print('   aborted at %.1fs  peak %.0fC  <540 in %.1fs  final %.0fC'
          % (ha['abortAt'], ha['peak'], ha['t540'], ha['tgt']))
    print()


def arma_start(cap=False, secs=30):
    """The Arma case the rig could not previously show.

    In Arma the load is the ROTOR's own drag via bmkhs_reqEngTorque, which is ~0 until Nr
    turns - not the rig's Ng-ramped _load(). With the cap, torque is min(shaft, surplus), so
    zero demand means zero output and the rotor can never start turning: the deadlock.
    """
    e = Engine(cap=cap); e.starting = True
    t = 0.0; first = None
    while t < secs:
        if e.lever == 'OFF' and e.ng > 0.02:
            e.setLever('IDLE')
        if e.starting and e.ng >= e.selfSustNg:
            e.starting = False
        e.step(DT, loadTq=0.0)          # stationary rotor: no drag, no demand
        t += DT
        if first is None and e.tq > 0.0:
            first = t
    return dict(firstTq=first, ng=e.ng, tgt=e.tgt, tq=e.tq / e.refTq)


#xmsnOutputRpm is the Np SHAFT speed, not Nr. Nr = that / gearRatio, so 20900 -> 289 rpm.
ROTOR_MOI, GEAR_RATIO = 5144.6, 72.291

TWO_PI = 6.283185307179586


class SimpleRotor:
    """A port of fn_simpleRotor's torque path and fn_simpleRotorTorque. Same expressions:
    drag over four modelled blade positions scaled by numBlades/4, acting at 0.75 R."""

    def __init__(self, numBlades, gearRatio, bladeRadius, bladeChord, bladeMass,
                 dragCoefTable, torqueTau):
        self.numBlades, self.gearRatio = numBlades, gearRatio
        self.bladeRadius, self.bladeChord, self.bladeMass = bladeRadius, bladeChord, bladeMass
        self.dragCoefTable, self.torqueTau = dragCoefTable, torqueTau
        self.bladeArea = bladeRadius * bladeChord
        self.bladeRad75 = bladeRadius * 0.75
        self.bladeScalar = numBlades / 4.0
        #fn_simpleRotorTorque: numBlades * (1/3) m r^2
        self.moi = numBlades * (1.0 / 3.0) * bladeMass * bladeRadius ** 2
        self.reqEngTorque = 0.0

    def dragCoef(self, collOutput, velXY):
        """mathLinearInterp2D over the declared table, clamped at both ends."""
        hdr = self.dragCoefTable[0][1:]
        rows = self.dragCoefTable[1:]
        keys = [r[0] for r in rows]

        def interp1(vals):
            if velXY <= hdr[0]:
                return vals[0]
            if velXY >= hdr[-1]:
                return vals[-1]
            for i in range(len(hdr) - 1):
                if hdr[i] <= velXY <= hdr[i + 1]:
                    f = (velXY - hdr[i]) / (hdr[i + 1] - hdr[i])
                    return vals[i] + (vals[i + 1] - vals[i]) * f
            return vals[-1]

        if collOutput <= keys[0]:
            return interp1(rows[0][1:])
        if collOutput >= keys[-1]:
            return interp1(rows[-1][1:])
        for i in range(len(keys) - 1):
            if keys[i] <= collOutput <= keys[i + 1]:
                a, b = interp1(rows[i][1:]), interp1(rows[i + 1][1:])
                f = (collOutput - keys[i]) / (keys[i + 1] - keys[i])
                return a + (b - a) * f
        return interp1(rows[-1][1:])

    def step(self, rpm, collOutput, dt, rho=ISA_RHO, velXY=0.0):
        """Returns reqEngTorque - the rotor's drag referred to the engine shaft, filtered."""
        omega = 0.0 if rpm == 0.0 else TWO_PI * (rpm / 60.0)
        bladeVel75 = omega * self.bladeRad75
        cd = self.dragCoef(collOutput, velXY)
        rotorTorque = 0.0
        for _ in range(4):
            drag = cd * 0.5 * rho * self.bladeArea * bladeVel75 ** 2 * self.bladeScalar
            rotorTorque += drag * self.bladeRad75
        req = rotorTorque / self.gearRatio if self.gearRatio > 0 else 0.0
        alpha = 1.0 - pow(2.718281828459045, -dt / self.torqueTau)
        self.reqEngTorque += (req - self.reqEngTorque) * alpha
        return self.reqEngTorque


#Straight off helisim_simpleRotor.hpp.
AH64_MAIN_DRAG = [
    ["A/S", 0.00, 10.29, 20.58, 36.01, 46.30, 51.44, 61.73, 66.88, 72.02],
    [0.00, 0.0078, 0.0078, 0.0060, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005],
    [0.20, 0.0193, 0.0178, 0.0150, 0.0099, 0.0100, 0.0101, 0.0110, 0.0110, 0.0110],
    [0.40, 0.0309, 0.0281, 0.0242, 0.0195, 0.0197, 0.0200, 0.0217, 0.0217, 0.0217],
    [0.64, 0.0447, 0.0401, 0.0350, 0.0307, 0.0310, 0.0314, 0.0341, 0.0341, 0.0341],
    [0.80, 0.0474, 0.0474, 0.0474, 0.0474, 0.0474, 0.0474, 0.0474, 0.0474, 0.0474],
    [1.00, 0.1000, 0.1000, 0.1000, 0.1000, 0.1000, 0.1000, 0.1000, 0.1000, 0.1000],
]


def ah64_main():
    return SimpleRotor(4, 72.291, 7.315, 0.533, 72.108, AH64_MAIN_DRAG, 0.10)


def rotor_start(secs=60, lever='IDLE', coll=0.0, jExtra=0.0):
    """THE GAP THIS RIG HAD: a start that spins a real rotor up. The rotor is the ported
    fn_simpleRotor drag path; Nr is integrated exactly as fn_transmissionUpdate does.

    jExtra: driveline inertia referred to the engine shaft, which the aircraft does not
    model - power turbine, shafts, gearboxes. Blades alone give J_eng ~1.0."""
    e = Engine(); e.starting = True
    main = ah64_main()
    wDesign = e.designRpm * e.npFly
    jEng = main.moi / (main.gearRatio ** 2) + jExtra
    rpm = 0.0
    t = 0.0
    trace = []
    peakShaft = peakGauge = 0.0
    while t < secs:
        if e.lever == 'OFF' and e.ng > 0.02:
            e.setLever(lever)
        if e.starting and e.ng >= e.selfSustNg:
            e.starting = False
        npFrac = rpm / wDesign
        rotorTq = main.step(rpm / main.gearRatio, coll, DT)
        e.step(DT, loadTq=rotorTq, velY=0.0, npFrac=max(npFrac, 1e-9))
        rpm = max(0.0, rpm + ((e.tq - rotorTq) / jEng) * DT * (60.0 / TWO_PI))
        peakShaft = max(peakShaft, e.tq / e.refTq)
        peakGauge = max(peakGauge, e.gaugeTq / e.refTq)
        t += DT
        trace.append((t, e.ng, e.tgt, e.tq, e.gaugeTq / e.refTq, rpm, npFrac,
                      rpm / main.gearRatio, rotorTq))
    return dict(trace=trace, ng=e.ng, tgt=e.tgt, rpm=rpm, jEng=jEng,
                peakShaft=peakShaft, peakGauge=peakGauge, nr=rpm / main.gearRatio)


def rotor_report():
    print('=' * 78)
    print('ROTOR SPIN-UP - a real rotor, integrated as fn_transmissionUpdate does')
    print('=' * 78)
    print('   J_eng = %.3f kg m^2   (moi %.1f / gr %.3f^2)'
          % (ROTOR_MOI / GEAR_RATIO ** 2, ROTOR_MOI, GEAR_RATIO))
    for st in (False, True):
        r = rotor_start(speedTerm=st)
        print('\n   speedTerm=%-5s  peak %%TQ %.0f   settled ng %.3f  tgt %.0f  Np %.0f%% (Nr %.0f rpm)'
              % (st, r['peakTq'] * 100, r['ng'], r['tgt'],
                 100 * r['rpm'] / (Engine().designRpm * Engine().npFly),
                 r['rpm'] / GEAR_RATIO))
        print('        t     Ng    TGT    %TQ    Np%%   Nr rpm')
        for mark in (2, 3, 4, 5, 6, 8, 10, 15, 20, 30, 44):
            row = min(r['trace'], key=lambda x: abs(x[0] - mark))
            print('   %6.1f  %.3f   %4.0f  %5.0f  %4.0f  %6.0f'
                  % (row[0], row[1], row[2], row[3] * 100, row[5] * 100, row[4] / GEAR_RATIO))


if __name__ == '__main__':
    for flag in (True, False):
        report(flag)
    rotor_report()

    print('=' * 70)
    print('THE ARMA CASE - stationary rotor, loadTq = 0 (reqEngTorque at Nr = 0)')
    print('=' * 70)
    for flag in (True, False):
        r = arma_start(cap=flag)
        print('   cap=%-5s  first torque at %-6s  settled ng %.3f  tgt %.0fC  tq %.1f%%'
              % (flag,
                 ('%.1fs' % r['firstTq']) if r['firstTq'] else 'NEVER',
                 r['ng'], r['tgt'], r['tq'] * 100))
    print()
    print('   With the cap and a stationary rotor the engine produces NO torque, ever,')
    print('   so Nr never rises, so the rotor never produces drag. That is the deadlock')
    print('   seen in the aircraft as torque = -0.')
