"""Builds the AH-64D's fm.p3d from its old dimension config - the quads exactly where Core put them.

Run it:  python python/dev/ah64_fm_extract.py

Fuselage corners: fuselagePosition + fuselageRotation applied as fn_fuselageTop/Side/Front did
(fn_mathRotateVector: roll about Y, pitch about X, yaw about Z, by fn_quaternion). Wing corners:
fn_wing.sqf's dimension block before 1.1.1 (pos/pitch/roll/span/chord/sweep/twist/tipWidthScalar,
stabilator at theta 0). Facing is the normal that code used. The MLOD layout copies the Tiger's.
"""
import math
import os
import struct
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import engine

AH64   = r'E:\AH-64D\addons\fza_ah64_helisim'
CONFIG = AH64 + r'\config\bmkhs_config'
OUT    = AH64 + r'\fm.p3d'
TIGER  = r'E:\bmkhs_ec665\addons\helisim\fm.p3d'
#The last AH-64D commit with the dimension config; the working copy is generated now.
OLD_CONFIG_COMMIT = '05c5a90be'

WING_NAMES = {'right wing': 'rightWing', 'left wing': 'leftWing', 'vertical fin': 'verticalFin',
              'stabilator': 'stabilator'}
FUSE_NAMES = {'top': 'fuselageTop', 'side': 'fuselageSide', 'front': 'fuselageFront'}


def add(a, b):   return [p + q for p, q in zip(a, b)]
def sub(a, b):   return [p - q for p, q in zip(a, b)]
def mul(a, s):   return [p * s for p in a]
def dot(a, b):   return sum(p * q for p, q in zip(a, b))
def cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
def norm(a):
    l = math.sqrt(dot(a, a))
    return [p / l for p in a] if l else a
sind = lambda d: math.sin(math.radians(d))
cosd = lambda d: math.cos(math.radians(d))


def q_mul(a, b):
    """fn_quaternionMultiply."""
    return [a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
            a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
            a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]]


def quaternion(v, axis, ang):
    """fn_quaternion."""
    n = norm(axis + [0.0])
    s = sind(ang / 2)
    r = [n[0] * s, n[1] * s, n[2] * s, cosd(ang / 2)]
    res = q_mul(q_mul(r, v + [0.0]), [-r[0], -r[1], -r[2], r[3]])
    return res[:3]


def rotate_vector(v, x_ang, y_ang, z_ang):
    """fn_mathRotateVector."""
    v = quaternion(v, [0.0, 1.0, 0.0], y_ang)
    v = quaternion(v, [1.0, 0.0, 0.0], x_ang)
    return quaternion(v, [0.0, 0.0, 1.0], z_ang)


def vector_rotate(v, p, r, y):
    """fn_mathVectorRotate."""
    m = [[cosd(r) * cosd(y) + sind(r) * sind(p) * sind(y), -cosd(r) * sind(y) + sind(r) * sind(p) * cosd(y), sind(r) * cosd(p)],
         [cosd(p) * sind(y), cosd(p) * cosd(y), -sind(p)],
         [-sind(r) * cosd(y) + cosd(r) * sind(p) * sind(y), sind(r) * sind(y) + cosd(r) * sind(p) * cosd(y), cosd(r) * cosd(p)]]
    return [dot(row, v) for row in m]


def rotate_around_axis(v, axis, ang):
    """fn_mathVectorRotateAroundAxis."""
    c, s = cosd(ang), sind(ang)
    d = dot(v, axis)
    cr = [v[1] * axis[2] - v[2] * axis[1], v[2] * axis[0] - v[0] * axis[2], v[0] * axis[1] - v[1] * axis[0]]
    return [v[i] * c + cr[i] * s + axis[i] * d * (1 - c) for i in range(3)]


def fuselage(cfg):
    pos = cfg['fuselagePosition']
    pitch, roll, yaw = cfg['fuselageRotation']
    right = rotate_vector([1.0, 0.0, 0.0], pitch, roll, yaw)
    fwd   = rotate_vector([0.0, 1.0, 0.0], pitch, roll, yaw)
    up    = rotate_vector([0.0, 0.0, 1.0], pitch, roll, yaw)
    normals = {'top': up, 'side': right, 'front': fwd}
    out = []
    for p in cfg['FuselagePanels'].values():
        facing = p['facing']
        quads = [[add(add(add(pos, mul(right, v[0])), mul(fwd, v[1])), mul(up, v[2])) for v in q] for q in p['panels']]
        out.append((FUSE_NAMES[facing], quads, normals[facing]))
    return out


def wing(w):
    pos, pitch, roll = w['pos'], w['pitch'], w['roll']
    span, chord, sweep, twist, taper = w['span'], w['chord'], w['sweep'], w['twist'], w['tipWidthScalar']
    if w['isStabilator'] > 0:
        right, fwd = [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]
        a = sub(pos, mul(right, span * 0.5))
        b = add(pos, mul(right, span * 0.5))
        c = sub(b, mul(fwd, chord))
        d = sub(a, mul(fwd, chord))
    else:
        right = vector_rotate([1.0, 0.0, 0.0], pitch, roll, 0.0)
        fwd   = vector_rotate([0.0, 1.0, 0.0], pitch, roll, 0.0)
        root = sub(pos, mul(right, span * 0.5))
        tip  = add(add(pos, mul(right, span * 0.5)), mul(fwd, sweep))
        a = add(root, mul(fwd, chord * 0.5))
        b = add(tip, mul(fwd, chord * 0.5 * taper))
        c = sub(tip, mul(fwd, chord * 0.5 * taper))
        d = sub(root, mul(fwd, chord * 0.5))
        t = rotate_around_axis(sub(b, c), right, twist)
        b = add(tip, mul(t, 0.5))
        c = sub(tip, mul(t, 0.5))
    #fn_wing's normal: spanwise x chord line, flipped for a negative span.
    clp = w['chordLinePos']
    f = add(d, mul(sub(a, d), 1.0 - clp))
    g = add(c, mul(sub(b, c), 1.0 - clp))
    chord_line = norm(sub(add(a, mul(sub(b, a), 0.5)), add(d, mul(sub(c, d), 0.5))))
    n = norm(cross(norm(sub(g, f)), chord_line))
    if span < 0:
        n = mul(n, -1.0)
    return WING_NAMES[w['name']], [[a, b, c, d]], n


def wound(quad, n):
    """Corner order whose face points along n, as the generator reads it."""
    return quad if dot(cross(sub(quad[1], quad[0]), sub(quad[2], quad[0])), n) > 0 else quad[::-1]


def asciiz(b, o):
    e = b.index(b'\0', o)
    return b[o:e], e + 1


def tiger_layout():
    b = open(TIGER, 'rb').read()
    _, ver, _ = struct.unpack_from('<4sII', b, 0)
    _, major, minor, npts, nnrm, nfac, flags = struct.unpack_from('<4sIIIIII', b, 12)
    pflags = struct.unpack_from('<I', b, 40 + 12)[0]
    o = 40 + npts * 16 + nnrm * 12
    fflags = struct.unpack_from('<I', b, o + 68)[0]
    tex, o2 = asciiz(b, o + 72)
    mat, _ = asciiz(b, o2)
    return ver, major, minor, flags, pflags, fflags, tex, mat


def write_mlod(surfaces, path):
    ver, major, minor, lflags, pflags, fflags, tex, mat = tiger_layout()
    pts, faces, sels = [], [], []
    for name, quad in surfaces:
        base = len(pts)
        #MLOD points are (right, up, forward).
        pts += [(p[0], p[2], p[1]) for p in quad]
        faces.append([base, base + 1, base + 2, base + 3])
        sels.append((name, len(faces) - 1))
    npts, nfac = len(pts), len(faces)
    out = bytearray(struct.pack('<4sII', b'MLOD', ver, 1))
    out += struct.pack('<4sIIIIII', b'P3DM', major, minor, npts, nfac, nfac, lflags)
    for p in pts:
        out += struct.pack('<fffI', p[0], p[1], p[2], pflags)
    for f in faces:
        n = norm(cross(sub(pts[f[1]], pts[f[0]]), sub(pts[f[2]], pts[f[0]])))
        out += struct.pack('<fff', *n)
    for i, f in enumerate(faces):
        out += struct.pack('<I', 4)
        for v in f:
            out += struct.pack('<IIff', v, i, 0.0, 0.0)
        out += struct.pack('<I', fflags) + tex + b'\0' + mat + b'\0'
    out += b'TAGG'
    def tagg(name, data):
        return b'\x01' + name + b'\0' + struct.pack('<I', len(data)) + data
    for name, fi in sels:
        data = bytearray(npts + nfac)
        for v in faces[fi]:
            data[v] = 1
        data[npts + fi] = 1
        out += tagg(name.encode(), bytes(data))
    out += tagg(b'#UVSet#', struct.pack('<I', 0) + bytes(nfac * 4 * 8))
    out += tagg(b'#EndOfFile#', b'')
    out += struct.pack('<f', 0.0)
    open(path, 'wb').write(out)


def old_config(name):
    text = subprocess.run(['git', '-C', r'E:\AH-64D', 'show',
                           '%s:addons/fza_ah64_helisim/config/bmkhs_config/%s' % (OLD_CONFIG_COMMIT, name)],
                          capture_output=True, text=True, check=True).stdout
    with tempfile.NamedTemporaryFile('w', suffix='.hpp', delete=False, encoding='utf-8') as f:
        f.write(text)
    try:
        return engine.parse_hpp(f.name)
    finally:
        os.remove(f.name)


if __name__ == '__main__':
    fcfg = old_config('helisim_fuselage.hpp')
    wcfg = old_config('helisim_wings.hpp')
    surfaces = []
    for name, quads, n in fuselage(fcfg):
        surfaces += [('%s%02d' % (name, i), wound(q, n)) for i, q in enumerate(quads, 1)]
    for w in wcfg['Wings'].values():
        name, quads, n = wing(w)
        surfaces += [(name, wound(q, n)) for q in quads]
    write_mlod(surfaces, OUT)
    for name, quad in surfaces:
        print('%-16s %s' % (name, ' '.join('(%.3f %.3f %.3f)' % tuple(p) for p in quad)))
    print('wrote %s' % OUT)
