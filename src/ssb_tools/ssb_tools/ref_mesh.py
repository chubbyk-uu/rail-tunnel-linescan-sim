"""Truth-side optical mesh reference for evaluation only; reconstruction never imports it.

Rays are intersected with the archived scene's float64 source triangles (panels,
groove walls, filler), not with the renderer's float local chunks and not with an
ideal cylinder. Triangles are indexed by angular strips about the tunnel axis; a ray
only tests triangles whose (theta, x) bounds meet its path between the innermost and
outermost mesh radii.
"""
import math
from pathlib import Path
import numpy as np
import yaml
from .session import read_json, sha256_file


def read_obj(path):
    vertices, faces = [], []
    with open(path) as stream:
        for line in stream:
            if line.startswith('v '):
                vertices.append(line.split()[1:4])
            elif line.startswith('f '):
                items = line.split()[1:]
                if len(items) != 3: raise ValueError(f'only triangle faces are supported: {path}')
                faces.append([int(item.split('/')[0]) for item in items])
    vertices = np.asarray(vertices, np.float64).reshape(-1, 3)
    faces = np.asarray(faces, np.int64).reshape(-1, 3)-1
    if len(faces) and (faces.min() < 0 or faces.max() >= len(vertices)):
        raise ValueError(f'invalid OBJ indices: {path}')
    return vertices, faces


def session_scene(session, override=None):
    """Archived optical scene of a session, identity-checked against its backend record.

    `override` relocates a moved scene file; its hash must still equal the record.
    """
    root = Path(session.root)
    source = yaml.safe_load((root/'evaluation/config_source.yaml').read_text())
    path = Path(override or source['render']['optical_scene'])
    if not path.is_absolute() and override is None:
        config = read_json(root/'config/provenance.json')['inputs']['config']['path']
        path = Path(config).parent/path
    expected = read_json(root/'config/backend.json')['describe']['optical_scene_sha256']
    if not Path(path).is_file() or sha256_file(path) != expected:
        raise ValueError(f'optical scene differs from the capture backend record: {path}')
    return path, read_json(path)


class OpticalMesh:
    def __init__(self, vertices, faces, material, axis_z, bin_rad=math.radians(.25)):
        self.vertices, self.faces = np.asarray(vertices, np.float64), np.asarray(faces, np.int64)
        self.material = np.asarray(material, np.int16)
        if not len(self.faces) or len(self.material) != len(self.faces):
            raise ValueError('nonempty mesh with one material per triangle required')
        self.axis_z = float(axis_z)
        triangles = self.vertices[self.faces]
        self.a = triangles[:, 0]; self.e1 = triangles[:, 1]-self.a; self.e2 = triangles[:, 2]-self.a
        radius = np.hypot(triangles[..., 1], triangles[..., 2]-self.axis_z)
        theta = np.arctan2(triangles[..., 1], triangles[..., 2]-self.axis_z)
        # Flat facets dip inside their vertices' radius; 1 mm margins bound the path.
        self.r_inner = float(radius.min())-1e-3; self.r_outer = float(radius.max())+1e-3
        self.x_min, self.x_max = triangles[..., 0].min(1), triangles[..., 0].max(1)
        lo, hi = theta.min(1), theta.max(1)
        wrapped = hi-lo > math.pi  # the bottom seam at +-pi; few, always tested
        self.always = np.flatnonzero(wrapped)
        self.bin_rad = bin_rad; self.bins = int(math.ceil(2*math.pi/bin_rad))
        first = np.floor((lo[~wrapped]+math.pi)/bin_rad).astype(np.int64)
        last = np.floor((hi[~wrapped]+math.pi)/bin_rad).astype(np.int64)
        ids = np.flatnonzero(~wrapped)
        counts = last-first+1
        members = np.repeat(ids, counts)
        bins = np.repeat(first, counts)+(np.arange(counts.sum())-np.repeat(np.cumsum(counts)-counts, counts))
        order = np.argsort(bins, kind='stable')
        self.members = members[order]
        self.offsets = np.searchsorted(bins[order], np.arange(self.bins+1))

    @classmethod
    def from_session(cls, session, scene_path=None):
        path, scene = session_scene(session, scene_path)
        vertices, faces, material = [], [], []
        base = 0; sources = [path]
        for mesh in scene['meshes']:
            file = Path(mesh['file'])
            file = file if file.is_absolute() else path.parent/file
            if sha256_file(file) != mesh['sha256']: raise ValueError(f'optical mesh identity mismatch: {file}')
            v, f = read_obj(file); sources.append(file)
            vertices.append(v); faces.append(f+base); material.append(np.full(len(f), int(mesh['material'])))
            base += len(v)
        truth = session.truth()
        mesh = cls(np.concatenate(vertices), np.concatenate(faces), np.concatenate(material),
                   truth['tunnel']['axis_z_m'])
        mesh.sources = sources
        return mesh

    def _radius_distance(self, origin, direction, radius):
        y, z = origin[1], origin[2]-self.axis_z
        a = direction[1]**2+direction[2]**2; b = 2*(y*direction[1]+z*direction[2]); c = y*y+z*z-radius*radius
        return (-b+math.sqrt(b*b-4*a*c))/(2*a)

    def intersect(self, origins, directions):
        """Nearest double-sided hit of each ray; returns points (n,3), triangle ids, materials."""
        origins = np.asarray(origins, np.float64); directions = np.asarray(directions, np.float64)
        points = np.empty_like(origins); hit_ids = np.empty(len(origins), np.int64)
        for i, (origin, direction) in enumerate(zip(origins, directions)):
            near, far = (self._radius_distance(origin, direction, r) for r in (self.r_inner, self.r_outer))
            path = origin+np.outer([near, far], direction)
            theta = np.arctan2(path[:, 1], path[:, 2]-self.axis_z)
            if abs(theta[1]-theta[0]) > math.pi: raise ValueError('ray path crosses the bottom seam')
            b0, b1 = np.clip(np.floor((np.sort(theta)+math.pi)/self.bin_rad).astype(int), 0, self.bins-1).tolist()
            candidates = np.concatenate([self.members[self.offsets[b]:self.offsets[b+1]] for b in range(b0, b1+1)]+
                                        [self.always])
            x0, x1 = path[:, 0].min()-1e-6, path[:, 0].max()+1e-6
            candidates = candidates[(self.x_min[candidates] <= x1) & (self.x_max[candidates] >= x0)]
            e1, e2 = self.e1[candidates], self.e2[candidates]
            p = np.cross(direction, e2); det = np.einsum('ij,ij->i', e1, p)
            with np.errstate(divide='ignore', invalid='ignore'):
                inverse = 1/det; rel = origin-self.a[candidates]
                u = np.einsum('ij,ij->i', rel, p)*inverse
                q = np.cross(rel, e1); v = q@direction*inverse
                t = np.einsum('ij,ij->i', e2, q)*inverse
            valid = (np.abs(det) > 1e-14) & (u >= 0) & (v >= 0) & (u+v <= 1) & (t > 0)
            if not valid.any(): raise ValueError(f'ray {i} misses the optical mesh')
            best = np.flatnonzero(valid)[np.argmin(t[valid])]
            points[i] = origin+t[best]*direction; hit_ids[i] = candidates[best]
        return points, hit_ids, self.material[hit_ids]
