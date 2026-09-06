# Rotor table derivations

Every collapsed table in `helisim_simpleRotor.hpp`, and the arithmetic that
produced it. This is what the spreadsheet has to implement: a designer enters
the physical quantities on the left, the sheet emits the config table on the
right.

Kept current as tables are collapsed. If a collapse happens and this file is
not updated, the spreadsheet cannot be built.

---

## Tail rotor

### `tailRtrThrustVsAirspeed[]`

**Designer enters:** an authority curve vs airspeed (all 1.0 for the AH-64 -
the OGE thrust point, with the fin offloading the rest), `Vbe` in m/s, and the
airspeed exponent.

**Formula, evaluated at each breakpoint:**

```
thrustVsAirspeed(V) = authority(V) * (1 + V/Vbe) ^ airspeedMod
```

**AH-64:** authority flat 1.0, `Vbe` 38.583, `airspeedMod` 0.4.

| V (m/s) | authority | (1+V/Vbe)^0.4 | table |
|---|---|---|---|
| 0.00 | 1.00 | 1.0000 | 1.0000 |
| 10.29 | 1.00 | 1.0992 | 1.0992 |
| 20.58 | 1.00 | 1.1865 | 1.1865 |
| 36.01 | 1.00 | 1.3017 | 1.3017 |
| 46.30 | 1.00 | 1.3708 | 1.3708 |
| 51.44 | 1.00 | 1.4034 | 1.4034 |
| 61.73 | 1.00 | 1.4655 | 1.4655 |
| 66.88 | 1.00 | 1.4951 | 1.4951 |
| 72.02 | 1.00 | 1.5239 | 1.5239 |

**Breakpoints:** the nine the original used. Worst interpolation error against
the continuous form is 0.166%, at 9 kt.

### `tailRtrPitchThrustTable[]`

Not derived - the designer's own curve, pedal to thrust. Asymmetric because a
tail rotor's blade pitch range is not symmetric about zero.

---

## Main rotor

### `mainRtrThrustVsCollective[]` - 2-D, collective x pressure altitude

**Designer enters:** blade pitch min/max (deg), a flat-pitch thrust floor vs
pressure altitude, and thrust per unit Nr vs pressure altitude.

**Formula:**

```
bladePitch(c)   = pitchMin + (pitchMax - pitchMin) * c
floor(PA)       = interp(thrustMinTable, PA)
pitchScalar     = floor + ((1 - floor) / pitchMax) * bladePitch(c)

thrustVsCollective(c, PA) = pitchScalar * interp(thrustMaxTable, PA)
```

The model multiplies the result by Nr fraction separately, as it always did.

**AH-64 inputs:**

| PA (ft) | thrustMin | thrustMax |
|---|---|---|
| 0 | 0.032 | 1.168 |
| 2000 | 0.052 | 1.422 |
| 4000 | 0.037 | 1.745 |
| 6000 | 0.041 | 2.132 |
| 8000 | 0.045 | 2.561 |

pitchMin 1.0 deg, pitchMax 19.0 deg.

**Output:** one row per altitude, two collective points per row - the
relationship is linear in collective at fixed altitude, so two points are
exact along that axis.

```
{   0, {{0.00, 0.0969}, {1.00, 1.1680}}},
{2000, {{0.00, 0.1449}, {1.00, 1.4220}}},
{4000, {{0.00, 0.1530}, {1.00, 1.7450}}},
{6000, {{0.00, 0.1950}, {1.00, 2.1320}}},
{8000, {{0.00, 0.2440}, {1.00, 2.5610}}}
```

**Error:** the PRODUCT of the two altitude tables is not linear between
altitude breakpoints even though each table is, so five rows leave a worst
deviation of 123 N on a 102306 N base - 0.12%. Under 0.18% above 0.3
collective. Nine altitude rows would take it to 0.28%, eleven to 0.19%, if a
future airframe needs it.

**Note:** `mainRtrBladePitchMin`/`Max` were removed from config. The pitch
range is implicit in the surface's endpoints. The spreadsheet still needs them
as INPUTS.

---

## Still to collapse

- `mainRtrThrustVsAirspeed` x `mainRtrVelExponentTable` -> one airspeed curve.
  Same shape as the tail's. 15 breakpoints gives 0.377%, 21 gives 0.123%; the
  original 9 leave 1.45% because the exponent curves sharply around 30 kt.
- The power side. `powerVsAirspeed` x `powerVsCollective` CANNOT reproduce the
  original: collective appears twice in the old chain, once directly and once
  divided by a speed-dependent correction. A separable two-table fit leaves
  47% error at 0.85 collective. This needs its own decomposition, not the
  spec's two tables.
