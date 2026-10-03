"""Aerosurface generator - writes an aircraft's fuselage and wing config from its fm.p3d.

Run it:  python python/tools/fm_generateAeroSurfacePoints.py <fm.p3d> <bmkhs_config folder>

Model every aerosurface as quads in the fm.p3d's first LOD (0.000), one named selection per
quad: the surface's name, optionally numbered - fuselageTop01, fuselageTop02, verticalFin.
Each quad's face points the way its force acts; flip the face to flip the force.

    Fuselage:  fuselageTop  fuselageSide  fuselageFront
    Wings:     leftWing  rightWing  horizontalStabilizer  stabilator
               verticalFin  leftVerticalFin  rightVerticalFin

A stabilator moves on Core's schedule; a horizontalStabilizer is fixed. The tool writes
helisim_fuselage.hpp and helisim_wings.hpp in full, every value other than the geometry and
facing at its default. The .p3d is only read.
"""
import math
import os
import re
import struct
import sys

FUSELAGE = ['fuselageTop', 'fuselageSide', 'fuselageFront']
WINGS    = ['leftWing', 'rightWing', 'horizontalStabilizer', 'stabilator',
            'verticalFin', 'leftVerticalFin', 'rightVerticalFin']

FUSELAGE_AIRFOIL = 'NACA 0012'
WING_AIRFOIL     = 'NACA 4418'
STAB_AIRFOIL     = 'NACA 0012'
NUM_ELEMENTS     = 4
CHORD_LINE_POS   = 0.25

PANEL_DRAG_TABLE = [(0, 0.200), (2000, 0.270), (4000, 0.300), (6000, 0.520), (8000, 0.750)]
FRONT_DRAG_TABLE = [(0, 0.800), (2000, 1.080), (4000, 1.200), (6000, 2.080), (8000, 3.000)]
DRAG_TABLES      = {'fuselageTop': PANEL_DRAG_TABLE, 'fuselageSide': PANEL_DRAG_TABLE,
                    'fuselageFront': FRONT_DRAG_TABLE}

#Collective rows 0..1 against 30-180 kts, deg.
STAB_TABLE = [
    [0.00, -25.00, -15.50,  -6.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00],
    [0.25, -25.00, -17.50, -10.00,  -6.00,  -5.70,  -3.30,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00],
    [0.50, -25.00, -18.30, -11.60, -10.61, -10.28,  -7.63,  -7.30,  -4.98,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00,  -3.00],
    [0.75, -25.00, -21.73, -16.95, -16.00, -15.68, -13.15, -12.83, -10.61,  -8.71,  -8.07,  -5.54,  -4.27,  -3.00,  -3.00,  -3.00],
    [1.00, -25.00, -26.50, -28.00, -23.00, -22.66, -19.97, -19.63, -17.27, -15.24, -14.57, -11.87, -10.52,  -9.17,  -8.50,  -8.50],
]

#{right, forward, up} axis and sign of each facing.
FACINGS  = {'right': (0, 1), 'left': (0, -1), 'forward': (1, 1), 'backward': (1, -1),
            'up': (2, 1), 'down': (2, -1)}
FUSELAGE_AXES = {'fuselageTop': ('up', 'down'), 'fuselageSide': ('left', 'right'),
                 'fuselageFront': ('forward', 'backward')}
OPPOSITE = {'right': 'left', 'left': 'right', 'forward': 'backward', 'backward': 'forward',
            'up': 'down', 'down': 'up'}


class BuildError(Exception):
    pass


def asciiz(b, o):
    e = b.index(b'\0', o)
    return b[o:e].decode('latin-1'), e + 1


def read_lod0(path):
    """Points, faces and named selections of the MLOD's resolution 0 LOD."""
    b = open(path, 'rb').read()
    sig, _, nlods = struct.unpack_from('<4sII', b, 0)
    if sig != b'MLOD':
        raise BuildError('%s is not an unbinarized (MLOD) .p3d' % path)
    o = 12
    for _ in range(nlods):
        _, _, _, npts, nnrm, nfac, _ = struct.unpack_from('<4sIIIIII', b, o)
        o += 28
        pts = [struct.unpack_from('<fff', b, o + i * 16) for i in range(npts)]
        o += npts * 16 + nnrm * 12
        faces = []
        for _ in range(nfac):
            nv = struct.unpack_from('<I', b, o)[0]
            faces.append([struct.unpack_from('<I', b, o + 4 + k * 16)[0] for k in range(nv)])
            o += 72
            _, o = asciiz(b, o)
            _, o = asciiz(b, o)
        o += 4
        sels = []
        while True:
            o += 1
            name, o = asciiz(b, o)
            size = struct.unpack_from('<I', b, o)[0]
            data = b[o + 4:o + 4 + size]
            o += 4 + size
            if name == '#EndOfFile#':
                break
            if not name.startswith('#'):
                sels.append((name, data))
        res = struct.unpack_from('<f', b, o)[0]
        o += 4
        if res == 0.0:
            return pts, faces, sels
    raise BuildError('%s has no 0.000 LOD' % path)


def sub(a, b):   return [p - q for p, q in zip(a, b)]
def dot(a, b):   return sum(p * q for p, q in zip(a, b))
def cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def facing_of(quad):
    n = cross(sub(quad[1], quad[0]), sub(quad[2], quad[0]))
    k = max(range(3), key=lambda i: abs(n[i]))
    sign = 1 if n[k] > 0 else -1
    return next(w for w, (ax, sg) in FACINGS.items() if ax == k and sg == sign)


def ordered(quad, facing):
    """Clockwise seen from the facing side, leading edge (furthest forward) first."""
    ax, sg = FACINGS[facing]
    n = [0.0, 0.0, 0.0]
    n[ax] = float(sg)
    u = [0.0, 0.0, 0.0]
    u[(ax + 1) % 3] = 1.0
    v = cross(n, u)
    c = [sum(p[i] for p in quad) / 4 for i in range(3)]
    quad = sorted(quad, key=lambda p: -math.atan2(dot(sub(p, c), v), dot(sub(p, c), u)))
    def along_y(i):
        d = sub(quad[(i + 1) % 4], quad[i])
        return abs(d[1]) / (math.sqrt(dot(d, d)) or 1.0)
    #Spanwise pair: the two opposite edges running most across the airflow.
    pair = (0, 2) if along_y(0) + along_y(2) <= along_y(1) + along_y(3) else (1, 3)
    mid = lambda i: (round(quad[i][1] + quad[(i + 1) % 4][1], 6), round(quad[i][2] + quad[(i + 1) % 4][2], 6))
    first = max(pair, key=mid)
    return quad[first:] + quad[:first]


def build(path):
    pts, faces, sels = read_lod0(path)
    npts = len(pts)
    surfaces = {}
    for name, data in sels:
        m = re.fullmatch(r'([A-Za-z]+?)(\d*)', name)
        surface = m.group(1) if m else None
        if surface not in FUSELAGE + WINGS:
            raise BuildError("you didn't name %s correctly - it must be one of: %s, optionally numbered"
                             % (name, ', '.join(FUSELAGE + WINGS)))
        sel_pts = {i for i, w in enumerate(data[:npts]) if w}
        sel_faces = [i for i, w in enumerate(data[npts:]) if w]
        if len(sel_faces) != 1 or len(faces[sel_faces[0]]) != 4 or sel_pts != set(faces[sel_faces[0]]):
            raise BuildError('%s must be exactly one 4-sided face' % name)
        #MLOD points are (right, up, forward).
        quad = [[pts[i][0], pts[i][2], pts[i][1]] for i in faces[sel_faces[0]]]
        surfaces.setdefault(surface, []).append((m.group(2), name, quad))

    missing = [s for s in FUSELAGE if s not in surfaces]
    if missing:
        raise BuildError('the fuselage needs all three surfaces - %s missing' % ', '.join(missing))

    report = []
    built = {}
    for surface, quads in surfaces.items():
        quads.sort(key=lambda q: (q[0] != '', int(q[0] or 0)))
        votes = {}
        for _, name, quad in quads:
            votes.setdefault(facing_of(quad), []).append(name)
        ranked = sorted(votes.items(), key=lambda kv: -len(kv[1]))
        facing = ranked[0][0]
        if len(ranked) > 1 and len(ranked[1][1]) == len(ranked[0][1]):
            raise BuildError('quad facing mismatch, please ensure all quads are facing the same direction '
                             'on surface %s' % surface)
        for other, names in ranked[1:]:
            if other != OPPOSITE[facing]:
                raise BuildError('quad facing mismatch, please ensure all quads are facing the same direction '
                                 'on surface %s' % surface)
            report += ['%s was flipped to conform to its neighbours' % n for n in names]
        if surface in FUSELAGE_AXES and facing not in FUSELAGE_AXES[surface]:
            raise BuildError('%s faces %s - it must face %s' % (surface, facing, ' or '.join(FUSELAGE_AXES[surface])))
        built[surface] = (facing, [ordered(q, facing) for _, _, q in quads])
    return built, report


def fmt_quad(quad):
    return '{' + ','.join('{%6.3f,%6.3f,%6.3f}' % tuple(0.0 + round(c, 3) for c in p) for p in quad) + '}'


def fmt_rows(rows, indent):
    return '\n'.join(indent + (' ' if i == 0 else ',') + r for i, r in enumerate(rows))


def fuselage_hpp(built):
    out = ['//Generated by fm_generateAeroSurfacePoints.py from fm.p3d. Re-running resets every value.',
           '',
           '    fuselageAirfoil     = "%s";' % FUSELAGE_AIRFOIL,
           '']
    names = [s for s in FUSELAGE if s in built]
    out += ['    class FuselagePanels {']
    for i, s in enumerate(names, 1):
        facing, quads = built[s]
        out += ['        class FuselagePanel%02d {' % i,
                '            name          = "%s";' % s,
                '            facing        = "%s";' % facing,
                '            dragCoefTable[] =',
                '            {',
                fmt_rows(['{%4d, %.3f}' % r for r in DRAG_TABLES[s]], '            '),
                '            };',
                '            panels[] =',
                '            {',
                fmt_rows([fmt_quad(q) for q in quads], '             '),
                '            };',
                '        };']
        if i < len(names):
            out.append('')
    out += ['    };', '']
    return '\n'.join(out)


def wings_hpp(built):
    out = ['//Generated by fm_generateAeroSurfacePoints.py from fm.p3d. Re-running resets every value.', '']
    names = [s for s in WINGS if s in built]
    out += ['    class Wings {']
    for i, s in enumerate(names, 1):
        facing, quads = built[s]
        out += ['        class Wing%02d {' % i,
                '            name           = "%s";' % s,
                '            facing         = "%s";' % facing,
                '            numElements    = %d;' % NUM_ELEMENTS,
                '            airfoil        = "%s";' % (STAB_AIRFOIL if s in ('stabilator', 'horizontalStabilizer') else WING_AIRFOIL),
                '            chordLinePos   = %.2f;' % CHORD_LINE_POS,
                '            panels[] =',
                '            {',
                fmt_rows([fmt_quad(q) for q in quads], '             '),
                '            };',
                '        };']
    out += ['    };', '']
    if 'stabilator' in built:
        out += ['    //Stabilator incidence, deg. Rows are collective 0..1; columns 30-180 kts.',
                '    heliSimStabTable[] =',
                '    {',
                fmt_rows(['{' + ','.join('%6.2f' % v for v in r) + '}' for r in STAB_TABLE], '     '),
                '    };',
                '']
    return '\n'.join(out)


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    p3d, config = sys.argv[1], sys.argv[2]
    try:
        built, report = build(p3d)
    except BuildError as e:
        print('Nothing written: %s' % e)
        sys.exit(1)
    for line in report:
        print(line)
    for s in FUSELAGE + WINGS:
        if s in built:
            print('  %-22s %-8s %d quad%s' % (s, built[s][0], len(built[s][1]), '' if len(built[s][1]) == 1 else 's'))
    if input('\nAll data will be reset to default. Continue? (y/n) ').strip().strip('﻿').lower() != 'y':
        print('Nothing written.')
        sys.exit(0)
    for name, text in (('helisim_fuselage.hpp', fuselage_hpp(built)), ('helisim_wings.hpp', wings_hpp(built))):
        with open(os.path.join(config, name), 'w', newline='\n') as f:
            f.write(text)
        print('wrote %s' % os.path.join(config, name))
