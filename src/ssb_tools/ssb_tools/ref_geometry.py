"""Independent float64 reference for the stage A optics (DESIGN.md §4.1, §4.4).

Written from the design, not from the C++ code: head pose from the true axis angle
and mount offsets, pixel rays, analytic cylinder hit and the procedural wall albedo.
"""
import numpy as np

MASK64 = np.uint64(0xFFFFFFFFFFFFFFFF)


def focal_length(camera):
    return camera['pixel_pitch_m'] * camera['nominal_distance_m'] * camera['width'] / camera['fov_at_nominal_m']


def pixel_tangents(camera, columns=None):
    width = camera['width']
    u = np.arange(width, dtype=np.float64) if columns is None else np.asarray(columns, np.float64)
    return (u - 0.5 * (width - 1)) * camera['pixel_pitch_m'] / focal_length(camera)


def _rot(axis, angle):
    angle = np.asarray(angle, np.float64)
    c, s = np.cos(angle), np.sin(angle)
    one, zero = np.ones_like(c), np.zeros_like(c)
    if axis == 'x':
        m = [[one, zero, zero], [zero, c, -s], [zero, s, c]]
    elif axis == 'y':
        m = [[c, zero, s], [zero, one, zero], [-s, zero, c]]
    else:
        m = [[c, -s, zero], [s, c, zero], [zero, zero, one]]
    return np.moveaxis(np.array(m), (0, 1), (-2, -1))


def head_pose(theta, x, truth, body=None):
    """Optical centre, optical axis and line direction in the world frame.

    Scan angle theta is measured from the top towards +y, so the nominal optical axis
    is (0, sin theta, cos theta): a rotation of -theta about +x.
    """
    m = truth['mount']
    tilt = _rot('z', m['tilt_z_rad']) @ _rot('y', m['tilt_y_rad'])
    head = tilt @ _rot('x', -np.asarray(theta, np.float64))
    axis_point = np.stack([np.asarray(x, np.float64) + truth['head_mount_x_m'],
                           np.full(np.shape(x), m['dy_m']),
                           np.full(np.shape(x), truth['tunnel']['axis_z_m'] + m['dz_m'])], axis=-1)
    if body is not None and 'body_valid' in body.dtype.names and np.any(body['body_valid']):
        rotation=_rot('z',body['yaw']) @ _rot('y',body['pitch']) @ _rot('x',body['roll'])
        nominal=np.array([truth['head_mount_x_m'],m['dy_m'],truth['tunnel']['axis_z_m']-.3+m['dz_m']])
        axis_point=np.stack([np.asarray(x),body['y'],body['z']],axis=-1)+(rotation @ nominal)
        head=rotation @ head
    origin = axis_point + head @ np.array([0.0, m['tangential_m'], m['e_m']])
    optical = head @ np.array([0.0, 0.0, 1.0])
    line = head @ np.array([np.cos(m['twist_rad']), np.sin(m['twist_rad']), 0.0])
    return origin, optical, line


def wall_hits(origin, optical, line, tangents, truth):
    """Cylinder hit (x, q) for rays origin + t * normalize(optical + tan * line).

    origin/optical/line: (n, 3); tangents: (m,). Returns (n, m) arrays.
    """
    R, zc = truth['tunnel']['radius_m'], truth['tunnel']['axis_z_m']
    d = optical[:, None, :] + tangents[None, :, None] * line[:, None, :]
    d = d / np.linalg.norm(d, axis=-1, keepdims=True)
    oy = origin[:, 1][:, None]
    oz = origin[:, 2][:, None] - zc
    a = d[..., 1] ** 2 + d[..., 2] ** 2
    b = 2 * (oy * d[..., 1] + oz * d[..., 2])
    c = oy ** 2 + oz ** 2 - R ** 2
    t = (-b + np.sqrt(b * b - 4 * a * c)) / (2 * a)
    x = origin[:, 0][:, None] + t * d[..., 0]
    q = R * np.arctan2(oy + t * d[..., 1], oz + t * d[..., 2])
    return x, q


def _hash_cell(a, b):
    with np.errstate(over='ignore'):
        z = (a.astype(np.int64).astype(np.uint64) * np.uint64(0x9E3779B97F4A7C15)) ^ \
            (b.astype(np.int64).astype(np.uint64) * np.uint64(0xC2B2AE3D27D4EB4F))
        z = z + np.uint64(0x9E3779B97F4A7C15)
        z = (z ^ (z >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
        z = (z ^ (z >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
        return z ^ (z >> np.uint64(31))


def wall_albedo(x, q):
    """50 mm checker (0.35 / 0.65) plus 1 mm hashed noise of +-0.1."""
    cx, cq = np.floor(x / 0.05).astype(np.int64), np.floor(q / 0.05).astype(np.int64)
    base = np.where((cx + cq) & 1, 0.65, 0.35)
    h = _hash_cell(np.floor(x / 0.001), np.floor(q / 0.001))
    noise = (h >> np.uint64(40)).astype(np.float64) / 16777216.0 - 0.5
    return base + 0.2 * noise


def albedo_code(albedo):
    return np.clip(np.rint(albedo * 255.0), 0, 255).astype(np.uint8)
