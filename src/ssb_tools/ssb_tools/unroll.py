"""Ideal geometric unrolling from observable data only (DESIGN.md §9.1).

Inputs are the observable configuration (nominal geometry, T_hat) and recorded
metadata: row exposure times, scan-encoder edges, odometer edges and gate events.
Nothing here may read `evaluation/` (true poses, truth.json).
"""
import math

import numpy as np
from scipy.interpolate import PchipInterpolator


def edge_positions(edges):
    """Encoder position (in counts) at each edge: the level that was crossed."""
    return edges['count'] + (edges['dir'] < 0)


def encoder_position(edges, t, start=None):
    """Continuous encoder position (counts) at times t.

    Monotone cubic (PCHIP) through the edge crossings: linear interpolation in time
    reached 0.24 px on rows exposed while the axis decelerated to rest; PCHIP follows
    the acceleration and stays below 0.02 px there. `start` optionally adds a known
    (time, position) point before the first edge.
    """
    t_e, p_e = edges['t'], edge_positions(edges).astype(np.float64)
    if start is not None:
        t_e, p_e = np.concatenate([[start[0]], t_e]), np.concatenate([[start[1]], p_e])
    if len(t_e) < 2:
        raise ValueError('need at least two encoder positions')
    return PchipInterpolator(t_e, p_e, extrapolate=True)(t)


def row_coordinates(config, rows, scan_edges, odo_edges, gates, columns):
    """Unrolled (x, q) of the given pixel columns for every row.

    Scan angle: each segment is referenced to its own gate-start event (nominal gate
    angle); within it the angle follows the interpolated encoder position. Track
    position: odometer distance with the calibrated wheel diameter.
    """
    cal, cam = config['calibration'], config['camera']
    counts_per_rev = config['scan_encoder']['ppr'] * config['scan_encoder']['edges_per_cycle']
    odo = config['odometer']
    metres_per_count = math.pi * cal['wheel_diameter_m'] / (odo['ppr'] * odo['edges_per_cycle'] * odo['gear_ratio'])
    R = cal['radius_m']
    f = cam['pixel_pitch_m'] * cam['nominal_distance_m'] * cam['width'] / cam['fov_at_nominal_m']
    tangents = (np.asarray(columns, np.float64) - 0.5 * (cam['width'] - 1)) * cam['pixel_pitch_m'] / f

    t = rows['t_center']
    starts = gates[(gates['kind'] == 0) & (gates['dir'] > 0)]
    ref_time = {int(g['revolution']): g['t'] for g in starts}
    missing = sorted(set(np.unique(rows['segment']).tolist()) - set(ref_time))
    if missing:
        raise ValueError(f'segments without a gate-start event: {missing}')
    seg_ref_t = np.array([ref_time[int(k)] for k in rows['segment']])
    pos = encoder_position(scan_edges, t)
    ref_pos = encoder_position(scan_edges, seg_ref_t)
    theta = config['gate']['start_rad'] + 2 * math.pi * rows['segment'] + (pos - ref_pos) * 2 * math.pi / counts_per_rev

    # The odometer starts at count 0 on a level at power-on (t = 0); an unknown
    # sub-count start phase would be a constant x offset of up to one count.
    odo_pos = encoder_position(odo_edges, t, start=(0.0, 0.0))
    x_axis = config['motion']['start_x_m'] + odo_pos * metres_per_count + cal['head_mount_x_m']
    x = x_axis[:, None] + R * tangents[None, :]
    q = np.broadcast_to((R * theta)[:, None], x.shape)
    return x, q, theta


def wrap_angle(theta):
    return (theta + math.pi) % (2 * math.pi) - math.pi
