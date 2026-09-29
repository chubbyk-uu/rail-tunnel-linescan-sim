"""Independent reference of the hardware timing model (DESIGN.md §4.2, §8.1).

Re-implemented from the specification, with different numerics than the C++ engine:
lattice crossings are solved with numpy polynomial roots instead of bisection, and
the rescaler is computed per encoder interval instead of as a streaming schedule.

Specification (stage A):
- Encoder counts are floor((theta - offset) / spacing), relative to the first sample.
- A forward edge beyond the high-water count c is sub-tick 64c. If the previous edge
  was forward with count c-1 and T = t_c - t_{c-1} <= max_period, sub-ticks 64c+j
  (j = 1..63) fall at t_c + jT/64 when strictly earlier than the next edge; otherwise
  they are dropped (early edge, or reverse if that next edge is not a new forward one).
- Without a valid T the lattice up to the next new forward edge is dropped (no period).
- Sub-ticks that are multiples of 7 are row triggers; exposure centre is
  t + delay + exposure/2, and a trigger is only resolved once t + delay + exposure is
  inside the stream.
- The gate is open while the axis is inside [start, end) of a revolution, judged by
  gate events at the exposure centre (start inclusive, end exclusive).
- A gated trigger closer than 1/max_line_rate - 1 ns to the last accepted one is an overrun.
"""
import math

import numpy as np

EARLY_EDGE, NO_PERIOD, REVERSE, OVERRUN, STREAM_END = 1, 2, 3, 4, 5
TWO_PI = 2 * math.pi


def hermite_coefficients(t0, t1, p0, p1, m0, m1):
    """p(u) - p0 = b u + c u^2 + d u^3, u = (t - t0) / (t1 - t0)."""
    H, dp = t1 - t0, p1 - p0
    return H * m0, 3 * dp - 2 * H * m0 - H * m1, -2 * dp + H * m0 + H * m1


def hermite_value(samples, t, value, rate):
    """Interpolate field `value` (with derivative `rate`) of the pose stream at times t."""
    ts = samples['t']
    t = np.asarray(t, np.float64)
    i = np.clip(np.searchsorted(ts, t, side='left') - 1, 0, len(ts) - 2)
    t0, t1 = ts[i], ts[i + 1]
    b, c, d = hermite_coefficients(t0, t1, samples[value][i], samples[value][i + 1],
                                   samples[rate][i], samples[rate][i + 1])
    u = (t - t0) / (t1 - t0)
    H = t1 - t0
    return samples[value][i] + ((d * u + c) * u + b) * u, (b + (2 * c + 3 * d * u) * u) / H


def cubic_roots(b, c, d, target):
    """Roots of d u^3 + c u^2 + b u = target via numpy eigenvalues, polished by Newton.

    Leading coefficients negligible against the others are dropped first: a near-zero
    cubic term makes the companion matrix ill-conditioned (constant-speed segments).
    """
    scale = max(abs(b), abs(c), abs(d))
    coeffs = [d, c, b, -target]
    while len(coeffs) > 2 and abs(coeffs[0]) <= 1e-9 * scale:
        coeffs = coeffs[1:]
    roots = np.roots(coeffs)
    polished = []
    for r in roots:
        if abs(r.imag) > 1e-6:
            polished.append(r)
            continue
        u = r.real
        for _ in range(4):
            f = ((d * u + c) * u + b) * u - target
            g = (3 * d * u + 2 * c) * u + b
            if g == 0:
                break
            u -= f / g
        polished.append(complex(u, 0.0))
    return polished


def lattice_crossings(samples, value, rate, offset, spacing):
    """All changes of floor((p - offset) / spacing) as (t, index_after, dir) arrays."""
    ts, ps, ms = samples['t'], samples[value], samples[rate]
    times, index, dirs = [], [], []
    cur = math.floor((ps[0] - offset) / spacing)
    for i in range(len(ts) - 1):
        t0, t1, p0, p1 = ts[i], ts[i + 1], ps[i], ps[i + 1]
        b, c, d = hermite_coefficients(t0, t1, p0, p1, ms[i], ms[i + 1])
        crit = [r.real for r in np.roots([3 * d, 2 * c, b]) if abs(r.imag) < 1e-12 and 0 < r.real < 1] \
            if (d != 0 or c != 0) else []
        knots = [0.0] + sorted(crit) + [1.0]
        for a, e in zip(knots[:-1], knots[1:]):
            pe = p1 if e == 1.0 else p0 + ((d * e + c) * e + b) * e
            end = math.floor((pe - offset) / spacing)
            step = 1 if end > cur else -1
            while cur != end:
                level = cur + 1 if step > 0 else cur
                target = offset + level * spacing - p0
                roots = [r.real for r in cubic_roots(b, c, d, target) if abs(r.imag) < 1e-9]
                # Levels can coincide with a sample (constant speed): allow rounding at the ends.
                inside = [r for r in roots if a - 1e-9 <= r <= e + 1e-9]
                if not inside:
                    raise RuntimeError(f'no root for level {level} in interval {i}')
                u = min(inside, key=lambda r: abs(r - (a + e) / 2)) if len(inside) > 1 else inside[0]
                u = min(max(u, a), e)
                times.append(t1 if u >= 1.0 else t0 + u * (t1 - t0))
                cur += step
                index.append(cur)
                dirs.append(step)
    return np.array(times), np.array(index, np.int64), np.array(dirs, np.int64)


class GateTimeline:
    def __init__(self, t0, open0, rev0, events):
        self.t = np.array([t0] + [e[0] for e in events])
        self.open = np.array([open0] + [e[1] for e in events], bool)
        self.rev = np.array([rev0] + [e[2] for e in events], np.int64)

    def at(self, t):
        i = np.searchsorted(self.t, t, side='right') - 1
        i = np.maximum(i, 0)
        return self.open[i], self.rev[i]


def reference(samples, config, truth):
    cam, res = config['camera'], config['rescaler']
    counts_per_rev = config['scan_encoder']['ppr'] * config['scan_encoder']['edges_per_cycle']
    M, D = res['multiply'], res['divide']
    spacing = TWO_PI / counts_per_rev
    offset = truth['start_theta_rad'] + truth['scan_encoder_zero_rad']
    odo = config['odometer']
    odo_spacing = TWO_PI / (odo['ppr'] * odo['edges_per_cycle'] * odo['gear_ratio'])
    t_end = samples['t'][-1]

    et, ei, ed = lattice_crossings(samples, 'theta', 'omega', offset, spacing)
    initial = math.floor((samples['theta'][0] - offset) / spacing)
    ec = ei - initial
    ot, oi, od = lattice_crossings(samples, 'wheel', 'wheel_omega', 0.0, odo_spacing)
    oc = oi - math.floor(samples['wheel'][0] / odo_spacing)

    gs_off = config['gate']['start_rad'] + truth['gate_start_offset_rad']
    ge_off = config['gate']['end_rad'] + truth['gate_end_offset_rad']
    gst, gsi, gsd = lattice_crossings(samples, 'theta', 'omega', gs_off, TWO_PI)
    get_, gei, ged = lattice_crossings(samples, 'theta', 'omega', ge_off, TWO_PI)
    gates = []
    for t, i, d in zip(gst, gsi, gsd):
        gates.append((t, int(i if d > 0 else i + 1), 0, int(d)))
    for t, i, d in zip(get_, gei, ged):
        gates.append((t, int(i if d > 0 else i + 1), 1, int(d)))
    gates.sort(key=lambda g: g[0])  # stable: starts before ends at equal times
    th0 = samples['theta'][0]
    s0, e0 = math.floor((th0 - gs_off) / TWO_PI), math.floor((th0 - ge_off) / TWO_PI)
    timeline = GateTimeline(samples['t'][0], s0 == e0 + 1, s0,
                            [(g[0], (g[2] == 0) == (g[3] > 0), g[1]) for g in gates])

    triggers, dropped = [], []

    def drop_lattice(first, last, t_lo, t_hi, reason):
        s = -(-first // D) * D
        while s <= last:
            dropped.append([s // D, t_lo, t_hi, reason])
            s += D

    high, awaiting, awaiting_since = 0, 0, samples['t'][0]
    for k in range(len(et)):
        t, c, d = et[k], int(ec[k]), int(ed[k])
        if not (d > 0 and c > high):
            continue
        high = c
        if awaiting is not None:
            drop_lattice(awaiting, M * c - 1, awaiting_since, t, NO_PERIOD)
            awaiting = None
        if (M * c) % D == 0:
            triggers.append(((M * c) // D, t))
        valid = k > 0 and ed[k - 1] > 0 and ec[k - 1] == c - 1 and 0 < t - et[k - 1] <= res['max_period_s']
        if not valid:
            awaiting, awaiting_since = M * c + 1, t
            continue
        period = t - et[k - 1]
        nxt = et[k + 1] if k + 1 < len(et) else None
        next_is_new_forward = nxt is not None and ed[k + 1] > 0 and ec[k + 1] > high
        for j in range(1, M):
            s = M * c + j
            if s % D:
                continue
            tau = t + j * period / M
            if nxt is not None and tau >= nxt:
                drop_lattice(s, s, t, nxt, EARLY_EDGE if next_is_new_forward else REVERSE)
            elif nxt is None and tau > t_end:
                drop_lattice(s, s, t, t_end, STREAM_END)
            else:
                triggers.append((s // D, tau))
    if awaiting is not None:
        drop_lattice(awaiting, M * (high + 1) - 1, awaiting_since, t_end, STREAM_END)

    delay, exposure = cam['trigger_delay_s'], cam['exposure_s']
    min_period = 1.0 / cam['max_line_rate_hz'] - 1e-9
    triggers.sort(key=lambda r: r[1])
    rows, last_accepted = [], None
    for row, t in triggers:
        if t + delay + exposure > t_end:
            dropped.append([row, t, t_end, STREAM_END])
            continue
        centre = t + delay + 0.5 * exposure
        is_open, rev = timeline.at(centre)
        if not is_open:
            continue
        if last_accepted is not None and t - last_accepted < min_period:
            dropped.append([row, centre, centre, OVERRUN])
            continue
        last_accepted = t
        rows.append((row, int(rev), t, centre))

    drops = np.array(dropped, dtype=object).reshape(-1, 4)
    gated = []
    for row, t_lo, t_hi, reason in drops:
        if reason == OVERRUN:
            gated.append(1)
        else:
            gated.append(int(timeline.at(min(t_lo, t_end))[0] or timeline.at(min(t_hi, t_end))[0]))
    return {
        'scan_edges': (et, ec, ed),
        'odometer_edges': (ot, oc, od),
        'gates': gates,
        'rows': rows,
        'dropped': {int(r[0]): (int(r[3]), g) for r, g in zip(drops, gated)},
    }
