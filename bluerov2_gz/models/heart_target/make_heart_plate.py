#!/usr/bin/env python3
"""Generate the torpedo target: a square plate with a heart-shaped hole.

SDF has no boolean geometry, so a hole cannot be expressed by subtracting
one primitive from another - it has to be baked into a mesh. This writes
meshes/heart_plate.obj, which the world then references for both the
visual and the collision, so a torpedo genuinely passes THROUGH the
cutout instead of through the plate.

Construction. The plate lies in the local XZ plane with its thickness
along local Y, matching the box it replaces (size 0.61 0.02 0.61), so the
existing pose in the world file still lands it correctly.

The region between the square rim and the heart hole is triangulated
radially: for each of N angles about the centre, take the heart's radius
and the square's radius, and join consecutive pairs into quads. That is
only valid if the heart is star-shaped about the chosen centre - i.e. a
ray from the centre crosses the boundary exactly once - otherwise the
notch between the two lobes would be filled in and the hole would come
out looking like a spade. The heart is star-shaped about its bounding-box
centre (verified over a usable band of centres), so the notch survives.
"""

import math
import os

HOLE_WIDTH_M = 0.34      # heart width; plate is 0.61 square
PLATE_HALF = 0.305
THICKNESS_HALF = 0.01
SEGMENTS = 256


def heart(t):
    return (16 * math.sin(t) ** 3,
            13 * math.cos(t) - 5 * math.cos(2 * t)
            - 2 * math.cos(3 * t) - math.cos(4 * t))


def build():
    dense = [heart(2 * math.pi * i / 4000) for i in range(4000)]
    xs = [p[0] for p in dense]
    ys = [p[1] for p in dense]
    bbcx, bbcy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    scale = HOLE_WIDTH_M / (max(xs) - min(xs))

    # Heart samples in plate coordinates, centred on the plate.
    curve = [(((x - bbcx) * scale), ((y - bbcy) * scale)) for x, y in dense]

    # Radius of the heart as a function of angle, by nearest sample.
    by_angle = {}
    for px, pz in curve:
        a = math.atan2(pz, px) % (2 * math.pi)
        r = math.hypot(px, pz)
        key = int(a / (2 * math.pi) * SEGMENTS) % SEGMENTS
        # Star-shaped, so every angular bin holds one crossing; keep the
        # sample nearest the bin centre.
        target = (key + 0.5) / SEGMENTS * 2 * math.pi
        prev = by_angle.get(key)
        if prev is None or abs(a - target) < prev[0]:
            by_angle[key] = (abs(a - target), r)

    verts, faces = [], []

    def v(x, y, z):
        verts.append((x, y, z))
        return len(verts)          # OBJ is 1-based

    inner_f, inner_b, outer_f, outer_b = [], [], [], []
    for k in range(SEGMENTS):
        a = (k + 0.5) / SEGMENTS * 2 * math.pi
        r_in = by_angle[k][1]
        # Square boundary along the same ray.
        r_out = PLATE_HALF / max(abs(math.cos(a)), abs(math.sin(a)))

        ix, iz = r_in * math.cos(a), r_in * math.sin(a)
        ox, oz = r_out * math.cos(a), r_out * math.sin(a)

        inner_f.append(v(ix, -THICKNESS_HALF, iz))
        inner_b.append(v(ix, THICKNESS_HALF, iz))
        outer_f.append(v(ox, -THICKNESS_HALF, oz))
        outer_b.append(v(ox, THICKNESS_HALF, oz))

    for k in range(SEGMENTS):
        j = (k + 1) % SEGMENTS
        # Front and back faces, opposite winding so both point outward.
        faces += [(inner_f[k], outer_f[k], outer_f[j]),
                  (inner_f[k], outer_f[j], inner_f[j])]
        faces += [(inner_b[k], outer_b[j], outer_b[k]),
                  (inner_b[k], inner_b[j], outer_b[j])]
        # Hole wall - this is the surface a torpedo flies past.
        faces += [(inner_f[k], inner_b[j], inner_b[k]),
                  (inner_f[k], inner_f[j], inner_b[j])]
        # Outer rim.
        faces += [(outer_f[k], outer_b[k], outer_b[j]),
                  (outer_f[k], outer_b[j], outer_f[j])]

    return verts, faces, (max(ys) - min(ys)) * scale


def main():
    verts, faces, hole_h = build()
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, 'meshes', 'heart_plate.obj')
    os.makedirs(os.path.dirname(path), exist_ok=True)

    # Per-face normals are NOT optional. Without them the OBJ loads with a
    # normal count of zero, dartsim's CustomMeshShape rejects the submesh
    # ("normal count [0] does not match vertex count"), and the mesh then
    # reaches ODE empty, where OdeMesh::fillArrays walks off the end and
    # segfaults the whole simulator at the first physics step.
    normals = []
    for a, b, c in faces:
        (ax, ay, az), (bx, by, bz), (cx_, cy_, cz) = verts[a-1], verts[b-1], verts[c-1]
        ux, uy, uz = bx-ax, by-ay, bz-az
        vx, vy, vz = cx_-ax, cy_-ay, cz-az
        nx, ny, nz = uy*vz - uz*vy, uz*vx - ux*vz, ux*vy - uy*vx
        n = math.sqrt(nx*nx + ny*ny + nz*nz) or 1.0
        normals.append((nx/n, ny/n, nz/n))

    with open(path, 'w') as f:
        f.write('# Torpedo target: 0.61 m square plate, heart-shaped hole\n')
        f.write(f'# hole {HOLE_WIDTH_M:.3f} x {hole_h:.3f} m, generated by make_heart_plate.py\n')
        f.write('o heart_plate\n')
        for x, y, z in verts:
            f.write(f'v {x:.6f} {y:.6f} {z:.6f}\n')
        for nx, ny, nz in normals:
            f.write(f'vn {nx:.6f} {ny:.6f} {nz:.6f}\n')
        for i, (a, b, c) in enumerate(faces, start=1):
            f.write(f'f {a}//{i} {b}//{i} {c}//{i}\n')

    print(f'wrote {path}')
    print(f'  {len(verts)} vertices, {len(faces)} triangles, {len(normals)} normals')
    print(f'  hole {HOLE_WIDTH_M:.3f} x {hole_h:.3f} m in a '
          f'{2*PLATE_HALF:.2f} x {2*PLATE_HALF:.2f} m plate')


if __name__ == '__main__':
    main()
