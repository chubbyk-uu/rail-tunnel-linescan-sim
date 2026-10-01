"""Light-leak audit of the Stage B optical meshes (panels, joints, filler, gap).

Any hole in the lining is bounded by edges that only one triangle uses. Every such edge away
from the tunnel ends is probed: points just outside the edge (in its triangle's plane, away
from the triangle) at several offsets and positions are viewed from the tunnel axis, radially
and as obliquely as the camera field edge. A probe leaks when the nearest surface along the
ray is the opaque backing cylinder (or nothing): the camera would see through the lining there.
Deliberate overlaps (groove walls below the filler, filler edges against walls) do not leak,
because the ray meets the overlapping surface first. Float64 intersections with a 1e-9
barycentric tolerance, so a probe exactly on a shared edge is not reported. Renderer float32
cracks (instance seams, collinear T-junctions) are outside this audit; the renderer overlaps
its 2 m chunks and the backing catches any remaining sub-micrometre slip.
"""
import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

NAMES = ('panels', 'joints', 'filler', 'gap')
WELD_M = 1e-7                       # below float32 resolution at the lining radius
OFFSETS_M = (1e-6, 3e-6, 1e-5, 3e-5)
POSITIONS = (.1, .3, .5, .7, .9)
OBLIQUE_M = .42                     # half camera field along x at the wall
CELL_X, CELL_A = .01, math.radians(.25)
BARY_EPS = 1e-9                    # a ray exactly on a shared edge counts as a hit (float64 test)


def load(folder):
    vertices, triangles, owner = [], [], []
    for index, name in enumerate(NAMES):
        base = len(vertices)
        for line in (Path(folder)/f'{name}.obj').read_text().splitlines():
            if line.startswith('v '):
                vertices.append([float(v) for v in line.split()[1:4]])
            elif line.startswith('f '):
                triangles.append([base+int(v.split('/')[0])-1 for v in line.split()[1:]])
                owner.append(index)
    return np.array(vertices), np.array(triangles, dtype=np.int64), np.array(owner)


_STATE = {}


def _probe_edges(work):
    st = _STATE; points, grid, bins = st['points'], st['grid'], st['bins']; axis_z = st['axis_z']
    leaks = []
    for u, v, t, w in work:
        a, b, c = points[u], points[v], points[w]
        d = b-a; length = float(np.linalg.norm(d))
        if length < 1e-9: continue
        inward = c-a-np.dot(c-a, d)/np.dot(d, d)*d
        outward = -inward/np.linalg.norm(inward)
        grid_s, grid_o = np.meshgrid(POSITIONS, OFFSETS_M, indexing='ij')
        targets = (a+np.outer(grid_s.ravel(), d)+np.outer(grid_o.ravel(), outward))
        offsets = np.repeat(grid_o.ravel(), 3)
        targets = np.repeat(targets, 3, axis=0)
        origins = np.column_stack([targets[:, 0]+np.tile([0., -OBLIQUE_M, OBLIQUE_M], len(targets)//3),
                                   np.zeros(len(targets)), np.full(len(targets), axis_z)])
        dirs = targets-origins; dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        to_backing = (st['backing_radius']-1e-6)/np.hypot(dirs[:, 1], dirs[:, 2])
        ang = np.arctan2(targets[:, 1], targets[:, 2]-axis_z)
        xs = range(int(math.floor(targets[:, 0].min()/CELL_X))-2, int(math.floor(targets[:, 0].max()/CELL_X))+3)
        a_lo, a_hi = int(math.floor(ang.min()/CELL_A))-1, int(math.floor(ang.max()/CELL_A))+1
        if a_hi-a_lo > bins/2: a_lo, a_hi = a_hi-bins, a_lo   # wrapped near +/- pi
        cand = sorted({k for ix in xs for ia in range(a_lo, a_hi+1) for k in grid.get((ix, ia % bins), ())})
        hit = np.full(len(targets), math.inf)
        if cand:
            cand = np.array(cand, dtype=np.int64)
            A0, E1, E2 = st['A0'][cand], st['E1'][cand], st['E2'][cand]
            pv = np.cross(dirs[:, None, :], E2[None]); det = np.einsum('ck,rck->rc', E1, pv)
            rel = origins[:, None, :]-A0[None]
            with np.errstate(divide='ignore', invalid='ignore'):
                uu = np.einsum('rck,rck->rc', rel, pv)/det; q = np.cross(rel, E1[None])
                vv = np.einsum('rck,rk->rc', q, dirs)/det; tt = np.einsum('ck,rck->rc', E2, q)/det
            with np.errstate(invalid='ignore'):
                ok = (np.abs(det) > 1e-18) & (uu >= -BARY_EPS) & (vv >= -BARY_EPS) & (uu+vv <= 1+BARY_EPS) & (tt > 0)
            hit = np.where(ok, tt, math.inf).min(axis=1)
        leaking = hit >= to_backing
        if leaking.any():
            leaks.append(dict(mesh=NAMES[st['owner'][t]], a=a.round(6).tolist(), b=b.round(6).tolist(), length_m=length,
                              leak_offset_m=float(offsets[leaking].max()), leaking_probes=int(leaking.sum()),
                              x_m=float((a[0]+b[0])/2),
                              angle_deg=math.degrees(math.atan2((a[1]+b[1])/2, (a[2]+b[2])/2-axis_z))))
    return leaks


def audit(folder, radius=2.75, axis_z=2.015):
    vertices, triangles, owner = load(folder)
    keys = np.round(vertices/WELD_M).astype(np.int64)
    _, first, weld = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    points = vertices[first]; tris = weld.reshape(-1)[triangles]
    P = points[tris]
    radial = np.hypot(P[..., 1], P[..., 2]-axis_z)
    backing_radius = float(radial.max())
    backing = radial.min(axis=1) > backing_radius-1e-6
    lo, hi = points[:, 0].min(), points[:, 0].max()
    # Uniform (x, angle) grid of all non-backing triangles.
    angle = np.arctan2(P[..., 1], P[..., 2]-axis_z)
    grid = defaultdict(list)
    for t in np.where(~backing)[0]:
        a = angle[t]
        if a.max()-a.min() > math.pi: a = np.where(a < 0, a+2*math.pi, a)
        for ix in range(int(math.floor(P[t, :, 0].min()/CELL_X)), int(math.floor(P[t, :, 0].max()/CELL_X))+1):
            for ia in range(int(math.floor(a.min()/CELL_A)), int(math.floor(a.max()/CELL_A))+1):
                grid[(ix, ia % int(round(2*math.pi/CELL_A)))].append(t)
    bins = int(round(2*math.pi/CELL_A))
    A0 = P[:, 0]; E1 = P[:, 1]-A0; E2 = P[:, 2]-A0

    edges = defaultdict(list)
    for t, (a, b, c) in enumerate(tris):
        if backing[t]: continue
        for u, v, w in ((a, b, c), (b, c, a), (c, a, b)):
            edges[(u, v) if u < v else (v, u)].append((t, w))
    work, ends = [], 0
    for (u, v), users in edges.items():
        if len(users) != 1: continue
        a, b = points[u], points[v]
        if (abs(a[0]-lo) < 1e-9 and abs(b[0]-lo) < 1e-9) or (abs(a[0]-hi) < 1e-9 and abs(b[0]-hi) < 1e-9):
            ends += 1; continue
        work.append((u, v, users[0][0], users[0][1]))
    global _STATE
    _STATE = dict(points=points, grid=grid, bins=bins, A0=A0, E1=E1, E2=E2, owner=owner, axis_z=axis_z,
                  backing_radius=backing_radius)
    # fork() shares the grid without copying, but is unsafe in a multi-threaded process.
    import multiprocessing, os
    if len(os.listdir('/proc/self/task')) == 1:
        with multiprocessing.get_context('fork').Pool() as pool:
            results = pool.map(_probe_edges, [work[i::64] for i in range(64)])
    else:
        results = [_probe_edges(work)]
    leaks = [r for part in results for r in part]
    probed = len(work)
    by_mesh = {name: sum(r['mesh'] == name for r in leaks) for name in NAMES}
    import hashlib
    meshes = {f'{name}.obj': hashlib.sha256((Path(folder)/f'{name}.obj').read_bytes()).hexdigest() for name in NAMES}
    return dict(folder=str(folder), meshes=meshes, triangles=int(len(tris)), backing_triangles=int(backing.sum()),
                backing_radius_m=backing_radius, boundary_edges_probed=probed, tunnel_end_edges=ends,
                probes_per_edge=len(POSITIONS)*len(OFFSETS_M)*3, offsets_m=list(OFFSETS_M),
                leaks=dict(edges=len(leaks), by_mesh=by_mesh, examples=sorted(leaks, key=lambda r: -r['leak_offset_m'])[:20],
                           all=leaks))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('geometry', help='folder with panels/joints/filler/gap.obj')
    p.add_argument('--output')
    a = p.parse_args()
    report = audit(a.geometry)
    text = json.dumps(report, indent=2)
    if a.output: Path(a.output).write_text(text+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'leaks'} | {'leak_edges': report['leaks']['edges'],
                                                                          'by_mesh': report['leaks']['by_mesh']}, indent=2))
    return 0 if not report['leaks']['edges'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
