import sys; sys.path.insert(0, r'E:\bmkhs_helisim\python')
import engine as r
def scenario(label, limiter, gain=1.0, base_fat=15.0, single=False, coll=1.0):
    r.LIMITER, r.LIM_GAIN, r.BASE_FAT = limiter, gain, base_fat
    a = r.Heli(); H = a.H
    r.start_to(a, [0, 1], 'IDLE', 30.0)
    H['pwrLvr'] = [r.FLY, r.FLY]
    r.run_until(a, 60.0, coll=0.30)
    r.run_until(a, 90.0, coll=0.64)                   # hover
    if single: H['pwrLvr'][1] = r.IDLE; r.run_until(a, 110.0, coll=0.55)
    t0 = a.t; tr = []
    while a.t < t0 + 30.0:
        c = min(coll, 0.64 + (coll - 0.64) * (a.t - t0) / 2.0)   # 2 s pull
        a.frame(r.DT, coll=c); tr.append((a.t - t0, a.tgt(0), a.ng(0), a.nrFrac(), a.tq(0)))
    pk = max(tr, key=lambda x: x[1]); ngp = max(tr, key=lambda x: x[2]); nrm = min(tr, key=lambda x: x[3]); end = tr[-1]
    ngLim = min(0.0 + a.H['bmkhs_engines'][0]['ngLimitMax'], a.H['bmkhs_engines'][0]['ngLimitBase'] + a.H['bmkhs_engines'][0]['ngLimitSlope'] * a.H['bmkhs_FAT'])
    print(f'{label}: FAT {a.H["bmkhs_FAT"]:.0f} C, single {a.H["bmkhs_isSingleEng"]}, Ng limit {ngLim:.3f}')
    print(f'    TGT peak {pk[1]:.0f} C at {pk[0]:.1f} s, end {end[1]:.0f} C | Ng peak {ngp[2]:.4f}, end {end[2]:.4f} | Nr min {nrm[3]*100:.1f}% end {end[3]*100:.1f}% | torque end {end[4]*100:.0f}%')
for lim in (False, True):
    tag = 'LIMITER ON ' if lim else 'limiter off'
    scenario(f'{tag} twin, sea level', lim)
    scenario(f'{tag} single engine', lim, single=True)
    scenario(f'{tag} twin, -40 C day', lim, base_fat=-40.0)
