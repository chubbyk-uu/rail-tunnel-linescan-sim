"""Bounded preparation of the Stage B Gazebo geometry and its resource plan.

Optical meshes are evaluation/scene data, never reconstruction inputs. This module
does not turn on a textured OptiX renderer or assert wheel/rail contact fidelity.
"""
import argparse
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path
import resource
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def peak_rss_bytes():
    # Linux VmHWM belongs to this executable. ru_maxrss can retain a pre-exec
    # launcher's high-water mark, which is misleading for a short preparation tool.
    status = Path('/proc/self/status')
    if status.exists():
        for line in status.read_text().splitlines():
            if line.startswith('VmHWM:'):
                return int(line.split()[1])*1024
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def load_spec(path):
    spec = yaml.safe_load(Path(path).read_text())
    if spec.get('schema') != 'ssb.scene_spec.v1':
        raise ValueError('unsupported scene spec')
    p = spec['panels']
    values = [p['ring_width_m'], p['angular_step_deg'], p['joint_chamfer_m'], p['joint_groove_half_width_m'], p['joint_depth_m'],
              p['joint_contact_gap_m'], p['joint_gap_depth_m']]
    if not all(math.isfinite(v) and v > 0 for v in values):
        raise ValueError('invalid panel dimensions')
    recess = p['joint_filler_recess_m']; damage = p['joint_damage']; states = p['joint_states']
    if not 0 <= recess[0] <= recess[1] or p['joint_chamfer_m']+recess[1]+damage['loss_depth_m'][1] >= p['joint_depth_m']:
        raise ValueError('joint filler/damage depths must stay above the groove floor')
    rho = p['joint_edge_radius_m']
    if not 0 < rho or 2*rho*math.tan(math.pi/8) >= p['joint_chamfer_m']*math.sqrt(2) or p['joint_edge_fillet_steps'] < 1:
        raise ValueError('edge fillets must fit on the chamfer')
    if p['joint_edge_wiggle']['rms_m']*3 >= p['joint_chamfer_m']/2 or p['joint_along_step_m'] <= 0:
        raise ValueError('edge wiggle too large for the chamfer')
    if p['joint_contact_gap_m'] >= 2*p['joint_groove_half_width_m'] or p['joint_depth_m']+p['joint_gap_depth_m'] >= .09:
        raise ValueError('contact gap must fit the groove floor and the lining stay within 0.1 m')
    if set(states) != {'filled','unfilled','damaged'} or abs(sum(states.values())-1) > 1e-9 or min(states.values()) < 0:
        raise ValueError('joint state ratios must cover filled/unfilled/damaged and sum to 1')
    if len(p['angles_deg']) != 6 or not all(v > 0 for v in p['angles_deg']) or abs(sum(p['angles_deg']) - 360) > 1e-9:
        raise ValueError('six panel angles must cover 360 degrees')
    if 2*(p['joint_chamfer_m']+p['joint_groove_half_width_m']) >= p['ring_width_m']:
        raise ValueError('joint wider than a ring')
    for name in ('max_mesh_triangles', 'max_scene_vertices', 'asset_working_set_bytes'):
        if not isinstance(spec['resources'][name], int) or spec['resources'][name] <= 0:
            raise ValueError('invalid resource limit: ' + name)
    return spec


def resource_plan(config, spec):
    """Estimates only. No full wall array or raw image allocation is performed."""
    cam, enc, res = config['camera'], config['scan_encoder'], config['rescaler']
    r, lo, hi = (config['tunnel'][k] for k in ('radius_m', 'x_min_m', 'x_max_m'))
    n = enc['ppr'] * enc['edges_per_cycle'] * res['multiply'] / res['divide']
    speed = config['motion']['advance_per_rev_m'] * config['motion']['line_rate_hz'] / n
    roi = config['acceptance']['valid_x_m']
    arc = r * math.radians(config['gate']['end_deg'] - config['gate']['start_deg'])
    texel = spec['materials']['background_texel_m']
    if texel <= 0 or r <= 0 or hi <= lo or roi[1] <= roi[0]:
        raise ValueError('invalid resource planning domain')
    side = spec['materials']['tile_core_pixels'] + 2 * spec['materials']['tile_gutter_pixels']
    # Linear mono16 + rough8 + defect guard8 + two-component signed normal16.
    # The current renderer uses bilinear sampling without a mip chain.
    tile_bytes = side * side * 8
    tex_budget = spec['resources']['gpu_texture_budget_bytes']
    # Only the current batch's x/q footprint must be resident. Across a revolution
    # all angles must be accessible; retaining every angle simultaneously is wasteful.
    across = math.ceil(2 * math.pi * r / (spec['materials']['tile_core_pixels'] * texel))
    budget_tiles = tex_budget // tile_bytes
    batch = config['render']['batch_rows']
    duration = batch/config['motion']['line_rate_hz']+cam['exposure_s']
    tile_m = spec['materials']['tile_core_pixels']*texel
    active_x = math.ceil((cam['fov_at_nominal_m']+speed*duration)/tile_m)+1
    active_q = math.ceil((2*math.pi*r*config['motion']['line_rate_hz']/n*duration)/tile_m)+1
    active_tiles = active_x*active_q
    if budget_tiles < active_tiles:
        raise ValueError(f'texture budget cannot fit one active batch: {budget_tiles} < {active_tiles}')
    stride = config['render']['debug_column_stride']
    debug_columns = 0 if stride == 0 else len(set(range(0, cam['width'], stride)) | {cam['width'] - 1})
    per_batch = batch * (cam['width'] + debug_columns * 16)
    rows20 = (roi[1] - roi[0]) / config['motion']['advance_per_rev_m'] * n * arc / (2 * math.pi * r)
    return dict(schema='ssb.resource_plan.v1', rows_per_rev=n, nominal_speed_m_s=speed,
                longitudinal_pixel_m=cam['fov_at_nominal_m'] / cam['width'],
                circumferential_row_m=2 * math.pi * r / n,
                roi_raw_bytes_estimate=math.ceil(rows20 * cam['width']),
                roi_mono8_unwrap_bytes=math.ceil((roi[1]-roi[0])/texel)*math.ceil(arc/texel),
                full_scene_mono8_background_bytes=math.ceil((hi-lo)/texel)*math.ceil(2*math.pi*r/texel),
                texture_tile_bytes=tile_bytes, texture_tiles_in_budget=budget_tiles,
                circumference_tiles=across, active_batch_tiles=active_tiles,
                spare_prefetch_tiles=budget_tiles-active_tiles,
                rendered_batch_bytes=per_batch, render_job_queue_bytes=batch*200*config['render']['max_queued_batches'],
                write_queue_limit_bytes=config['storage']['write_queue_bytes'],
                storage_block_bytes=config['storage']['block_rows']*cam['width'],
                pose_record_bytes=160, row_job_bytes=200,
                pose_stream_bytes_estimate=160*math.ceil(config['motion']['profile'][-1][0]/config['motion']['sample_period_s']),
                budgets=spec['resources'],
                note='Estimates for nominal constant-speed geometry, not measured peaks. Runtime texture cache '
                     'uses bounded CPU/GPU LRU stores. Cache follows the batch footprint in both x and periodic q; '
                     'it does not retain an entire circumference. Actual hit bounds, prefetch/eviction '
                     'and GPU geometry/driver allocations need independent measurement.')


class Mesh:
    def __init__(self, path, counters, limits, uv_domain=None):
        self.out = Path(path).open('w')
        self.vertices = self.faces = 0
        self.counters, self.limits = counters, limits
        self.uv_domain=uv_domain
        self.out.write('# Stage B metric geometry, inward tunnel surfaces\n')

    def quad(self, points):
        if self.counters['vertices'] + 4 > self.limits['max_scene_vertices'] or self.counters['triangles'] + 2 > self.limits['max_mesh_triangles']:
            raise ValueError('scene geometry resource budget exceeded')
        first = self.vertices + 1
        for point in points:
            self.out.write('v %.9f %.9f %.9f\n' % tuple(point))
        # Ogre2 needs explicit normals to construct a textured HLMS material.
        # OptiX still derives face normals from these same triangle positions.
        normal = newell(points)   # robust when one quad edge has zero length (triangle)
        length = math.sqrt(sum(n*n for n in normal))
        if length == 0:
            raise ValueError('degenerate mesh quad')
        for _ in points:
            self.out.write('vn %.9f %.9f %.9f\n' % tuple(n/length for n in normal))
        if self.uv_domain:
            lo,hi,zc=self.uv_domain[:3]
            angles=[math.atan2(p[1],p[2]-zc) for p in points]
            a0,b0=self.uv_domain[3:] if len(self.uv_domain)==5 else (-math.pi,math.pi)
            anchor=(a0+b0)/2 if len(self.uv_domain)==5 else angles[0]
            angles=[a+2*math.pi*round((anchor-a)/(2*math.pi)) for a in angles]
            for p,a in zip(points,angles):
                self.out.write('vt %.9f %.9f\n' % ((p[0]-lo)/(hi-lo),(a-a0)/(b0-a0)))
        # Winding chosen by the caller; explicit inward winding on cylindrical faces.
        # A quad with one zero-length edge is written as its single non-degenerate triangle.
        index=lambda i:f'{i}/{i}/{i}' if self.uv_domain else f'{i}//{i}'
        same=lambda a,b:all(abs(points[a][k]-points[b][k])<1e-12 for k in range(3))
        corners=[k for k in range(4) if not same(k,(k+1)%4)]
        faces=[(0,1,2),(0,2,3)] if len(corners)==4 else [tuple(corners)] if len(corners)==3 else []
        for f in faces:
            self.out.write('f '+' '.join(index(first+k) for k in f)+'\n')
        self.vertices += 4
        self.faces += len(faces)
        self.counters['vertices'] += 4
        self.counters['triangles'] += len(faces)

    def close(self):
        self.out.close()


def point(x, angle, radius, zc):
    sine,cosine=math.sin(angle),math.cos(angle)
    if abs(sine)<1e-14: sine=0.
    if abs(cosine)<1e-14: cosine=0.
    return (x, radius*sine, zc+radius*cosine)


def cylinder_strip(mesh, x0, x1, a, b, radius, zc, step):
    count = max(1, math.ceil((b-a)/step))
    for i in range(count):
        u, v = a+(b-a)*i/count, a+(b-a)*(i+1)/count
        mesh.quad([point(x0,u,radius,zc), point(x0,v,radius,zc),
                   point(x1,v,radius,zc), point(x1,u,radius,zc)])


def newell(points):
    """Polygon normal (area-weighted) from all vertices."""
    n=[0.,0.,0.]
    for a,b in zip(points,points[1:]+points[:1]):
        n[0]+=(a[1]-b[1])*(a[2]+b[2]);n[1]+=(a[2]-b[2])*(a[0]+b[0]);n[2]+=(a[0]-b[0])*(a[1]+b[1])
    return tuple(n)


def oriented(mesh, points, toward):
    """Add a quad whose normal faces the reference point (inward/into the groove)."""
    n=newell(points)
    centre=[sum(pt[i] for pt in points)/4 for i in range(3)]
    mesh.quad(points if sum(n[i]*(toward[i]-centre[i]) for i in range(3)) >= 0 else points[::-1])


def joint_sections(rng, p, length):
    """State of one joint section and, if damaged, its filler-loss stretches along the joint."""
    states = p['joint_states']; names = ['filled','unfilled','damaged']
    state = names[int(rng.choice(3,p=[states[n] for n in names]))]
    recess = float(rng.uniform(*p['joint_filler_recess_m']))
    losses = []
    if state == 'damaged':
        d = p['joint_damage']; target = rng.uniform(*d['loss_fraction'])*length; pos = 0.
        while pos < length and sum(b-a for a,b,_ in losses) < target:
            a = pos+rng.uniform(0, (length-target)/3)          # intact filler before the next loss
            b = min(length, a+rng.uniform(*d['stretch_m']))
            if b-a > 1e-3: losses.append((a,b,float(rng.uniform(*d['loss_depth_m']))))
            pos = b
    return dict(state=state, filler_recess_m=recess, losses_m=[list(l) for l in losses])


def filler_levels(section, length):
    """Piecewise filler depth below the chamfer bottom along a section: [(s0, s1, depth)]."""
    if section['state'] == 'unfilled': return []
    cuts, pos = [], 0.
    for a,b,depth in section['losses_m']:
        if a > pos: cuts.append((pos,a,section['filler_recess_m']))
        cuts.append((a,b,depth)); pos = b
    if pos < length: cuts.append((pos,length,section['filler_recess_m']))
    return cuts


def lip_profile(g, c, rho, steps):
    """Half cross-section of a joint lip, outer -> inner, as (s, depth, follows_inner_edge).

    s: lateral distance from the joint centre line; depth: outward from the lining radius.
    Surface -> fillet (radius rho) -> 45 deg chamfer -> fillet -> groove wall top at (g, c+t),
    t = rho*tan(22.5 deg). The inner-edge points move with the along-joint wiggle.
    """
    h, t = g+c, rho*math.tan(math.pi/8)
    points = []
    for k in range(steps+1):   # outer fillet, centre (h+t, rho), -90 -> -135 deg
        phi = math.radians(-90-45*k/steps)
        points.append((h+t+rho*math.cos(phi), rho+rho*math.sin(phi), False))
    for k in range(steps+1):   # inner fillet, centre (g+rho, c+t), -135 -> -180 deg
        phi = math.radians(-135-45*k/steps)
        points.append((g+rho+rho*math.cos(phi), c+t+rho*math.sin(phi), True))
    return points, t


def wiggle(rng, spec):
    """Smooth along-joint edge offset (m) as a function of arc length: a few sinusoids."""
    w = spec['joint_edge_wiggle']; k = len(w['wavelengths_m']); a = w['rms_m']*math.sqrt(2/k)
    phases = rng.uniform(0, 2*math.pi, k)
    return lambda s: sum(a*math.sin(2*math.pi*s/lam+ph) for lam,ph in zip(w['wavelengths_m'],phases))


def make_meshes(out, config, spec):
    """Lining panels plus rounded, chamfered caulking grooves with mortar filler.

    Cross-section across a joint (depth outward from the lining radius r): panel -> fillet ->
    45 deg chamfer -> fillet -> groove wall at half width g; filler top at depth c + recess.
    The chamfer's inner edge wiggles slightly along the joint. Ring joints run around the
    circumference, split into sections at the next ring's segment boundaries; longitudinal
    joints run between ring joints and end in walls at the ring-joint lip. Filled sections
    omit the hidden groove floor; unfilled/damaged sections (kept for future use) include
    floor, contact gap and gasket.
    """
    p = spec['panels']
    t_ = config['tunnel']
    r, zc, lo, hi = (t_[k] for k in ('radius_m', 'axis_z_m', 'x_min_m', 'x_max_m'))
    step = math.radians(p['angular_step_deg'])
    c, g, D = p['joint_chamfer_m'], p['joint_groove_half_width_m'], p['joint_depth_m']
    w, G = p['joint_contact_gap_m']/2, p['joint_gap_depth_m']
    rho, fsteps = p['joint_edge_radius_m'], p['joint_edge_fillet_steps']
    profile, t = lip_profile(g, c, rho, fsteps)
    h = g+c; outer = h+t          # panel boundary distance from the joint centre line
    seal = 2*(r+D+G)*(1-math.cos(step/2))+1e-5
    edge_radius = r+D+G+seal
    if 2*outer/r >= math.radians(min(p['angles_deg'])):
        raise ValueError('joint wider than a panel')
    rng = np.random.default_rng(spec['seed']+7)
    counters = dict(vertices=0, triangles=0)
    # panels.obj (material 0) also carries the rounded lips/chamfers: they are the segment's own
    # concrete, continuous in texture, normal map and cracks. joints.obj: groove walls/floor (1);
    # filler.obj: mortar (2); gap.obj: dark gasket/void behind the contact gap (3).
    top, groove, fill, gap = (Mesh(out/name, counters, spec['resources'],(lo,hi,zc))
                              for name in ('panels.obj','joints.obj','filler.obj','gap.obj'))
    panels, joints = [], []
    INF = float('inf')
    ring_lip_nodes, long_lip_nodes = defaultdict(set), defaultdict(set)

    def lip_points(n):
        """Lip polyline (s, depth) for an inner-edge offset n."""
        return [(q+(n if inner else 0.), d) for q,d,inner in profile]

    def lip_depth(points, u):
        """Depth of the lip surface at lateral distance u (INF inside the groove)."""
        if u >= points[0][0]: return 0.
        if u <= points[-1][0]: return INF
        for (s0,d0),(s1,d1) in zip(points[:-1],points[1:]):
            if s1 <= u <= s0: return d0+(d1-d0)*(s0-u)/(s0-s1) if s0 > s1 else d1
        return INF

    FILLER_OVERLAP = 5e-5   # filler tucks behind the wall: no seam along the joint (hidden)
    UNDERLAY_DEPTH, UNDERLAY_HALF = 1e-4, 3e-4   # hidden strip under crossing-patch seams

    def wall_half_width(n, depth):
        """Lateral position of the lip/wall polyline (the actual faceted surface) at a depth."""
        pts = lip_points(n)+[(g+n, D+seal)]
        for (s0,d0),(s1,d1) in zip(pts[:-1],pts[1:]):
            if d0 <= depth <= d1:
                return s1 if d1 == d0 else s0+(s1-s0)*(depth-d0)/(d1-d0)
        return g+n

    def cross_section(section, depth_top, n_left, n_right):
        """Per side: lip points plus the wall down to where it is hidden, and filler half widths."""
        filled = section['state'] == 'filled'
        sides = {}
        for side, n in ((-1,n_left),(1,n_right)):
            pts = lip_points(n)
            bottom = (c+max(t,depth_top)+1e-3) if filled else D+seal
            pts.append((g+n, bottom))
            sides[side] = pts
        widths = {side:wall_half_width(n,c+depth_top)+FILLER_OVERLAP for side,n in ((-1,n_left),(1,n_right))}
        return sides, widths

    def sweep(mesh, curve_a, curve_b, to_world, toward):
        for (sa,da),(sb,db),(sc,dc),(sd,dd) in zip(curve_a[:-1],curve_a[1:],curve_b[1:],curve_b[:-1]):
            oriented(mesh,[to_world(*q) for q in ((sa,da,0),(sb,db,0),(sc,dc,1),(sd,dd,1))],toward)

    # Phase 1: joint records (states, recess, edge wiggles) so crossings know both joints.
    width = p['ring_width_m']
    rings = {}
    for ring in range(math.floor(lo/width), math.ceil(hi/width)):
        left, right = ring*width, (ring+1)*width
        # A ring boundary within 1e-9 m of a tunnel end is that end (ring*width rounding).
        x0 = left+outer if lo+1e-9 < left else lo
        x1 = right-outer if right < hi-1e-9 else hi
        if x1 <= x0: continue
        a = math.radians(-p['angles_deg'][0]/2 + (ring % 2)*p['alternating_stagger_deg'])
        edges = [a]
        for span in p['angles_deg']: edges.append(edges[-1]+math.radians(span))
        longs = []
        for index,A in enumerate(edges[:-1]):
            section = dict(kind='longitudinal', ring=ring, index=index, x_m=[x0,x1], angle_rad=A, **joint_sections(rng,p,x1-x0))
            joints.append(section);longs.append(dict(A=A, section=section, wig={side:wiggle(rng,p) for side in (-1,1)}))
        rings[ring] = dict(left=left, right=right, x0=x0, x1=x1, edges=edges, longs=longs)
    ring_joints = {}
    for ring, R in rings.items():
        if not lo+1e-9 < R['left'] < hi-1e-9: continue
        recess = float(rng.uniform(*p['joint_filler_recess_m']))   # one level per ring joint
        sections = []
        for index,(u,v) in enumerate(zip(R['edges'][:-1],R['edges'][1:])):
            section = dict(kind='ring', ring=ring, index=index, x_m=R['left'], angle_rad=[u,v], **joint_sections(rng,p,(v-u)*r))
            if section['state'] == 'filled': section['filler_recess_m'] = recess
            joints.append(section);sections.append(section)
        ring_joints[ring] = dict(X=R['left'], sections=sections, recess=recess, wig={side:wiggle(rng,p) for side in (-1,1)})

    def crossing_ok(RJ, L):
        return all(sec['state']=='filled' for sec in RJ['sections']) and L['section']['state']=='filled'

    def openings(ring, side):
        """Angular intervals of longitudinal grooves that open into ring joint `ring` on `side`."""
        other = rings.get(ring if side > 0 else ring-1)
        if other is None: return []
        return [(L['A']-outer/r, L['A']+outer/r, L) for L in other['longs'] if crossing_ok(ring_joints[ring], L)]

    def ring_joint(ring):
        RJ = ring_joints[ring]; X = RJ['X']; wig = RJ['wig']
        holes = {side:openings(ring, side) for side in (-1,1)}
        edge = {-1:[], 1:[]}   # ring filler edge polyline per side: (angle, half width)
        previous = None
        for section in RJ['sections']:
            u, v = section['angle_rad']
            levels = filler_levels(section,(v-u)*r) or [(0.,(v-u)*r,None)]
            for a,b,depth in levels:
                ua, ub = u+a/r, u+b/r; n = max(1, math.ceil((ub-ua)/step))
                # Hole edges mapped into this level's angle range (joint angles wrap at 2 pi).
                wrapped = lambda e: ua+math.fmod(math.fmod(e-ua,2*math.pi)+2*math.pi,2*math.pi)
                angles = sorted({ua+(ub-ua)*i/n for i in range(n+1)} |
                                {wrapped(e) for side in (-1,1) for lo_a,hi_a,_ in holes[side] for e in (lo_a,hi_a) if ua < wrapped(e) < ub})
                shapes = [cross_section(section, depth if depth is not None else 0., wig[-1](q*r), wig[1](q*r)) for q in angles]
                for side in (-1,1):
                    for q0,q1,(s0,_),(s1,_) in zip(angles[:-1],angles[1:],shapes[:-1],shapes[1:]):
                        mid = (q0+q1)/2
                        if any(math.fmod(math.fmod(mid-lo_a,2*math.pi)+2*math.pi,2*math.pi) <= hi_a-lo_a for lo_a,hi_a,_ in holes[side]):
                            continue   # junction patch instead
                        world = lambda s_,d_,end,q0=q0,q1=q1,side=side: point(X+side*s_, q1 if end else q0, r+d_, zc)
                        sweep(top, s0[side][:len(profile)], s1[side][:len(profile)], world, (X,0.,zc))
                        ring_lip_nodes[(X, side)].update((q0, q1))
                        sweep(groove, s0[side][len(profile)-1:], s1[side][len(profile)-1:], world, (X,0.,zc))
                if depth is not None:
                    rad = r+c+depth
                    for q0,q1,(_,w0),(_,w1) in zip(angles[:-1],angles[1:],shapes[:-1],shapes[1:]):
                        oriented(fill,[point(X-w0[-1],q0,rad,zc),point(X-w1[-1],q1,rad,zc),point(X+w1[1],q1,rad,zc),point(X+w0[1],q0,rad,zc)],(X,0.,zc))
                    for q,(_,wd) in zip(angles,shapes):
                        for sd in (-1,1): edge[sd].append((q,wd[sd]))
                    if previous is not None and abs(previous-rad) > 1e-9:   # step between filler levels
                        lo_r, hi_r = sorted((previous, rad)); wl, wr = shapes[0][1][-1], shapes[0][1][1]
                        oriented(fill,[point(X-wl,ua,lo_r,zc),point(X+wr,ua,lo_r,zc),point(X+wr,ua,hi_r,zc),point(X-wl,ua,hi_r,zc)],
                                 point(X,ua+(1 if rad>previous else -1)*1e-3,r,zc))
                    previous = rad
                else:
                    previous = None
                if section['state'] != 'filled':
                    for q0,q1 in zip(angles[:-1],angles[1:]):
                        for side in (-1,1):
                            oriented(groove,[point(X+side*g,q0,r+D,zc),point(X+side*g,q1,r+D,zc),point(X+side*w,q1,r+D,zc),point(X+side*w,q0,r+D,zc)],(X,0.,zc))
                            oriented(groove,[point(X+side*w,q0,r+D,zc),point(X+side*w,q1,r+D,zc),point(X+side*w,q1,r+D+G+seal,zc),point(X+side*w,q0,r+D+G+seal,zc)],(X,0.,zc))
                        oriented(gap,[point(X-w,q0,r+D+G,zc),point(X-w,q1,r+D+G,zc),point(X+w,q1,r+D+G,zc),point(X+w,q0,r+D+G,zc)],(X,0.,zc))
        a0 = RJ['sections'][0]['angle_rad'][0]
        for side in (-1,1):
            poly = sorted(edge[side]); qs = np.array([q for q,_ in poly]); ws = np.array([v for _,v in poly])
            for lo_a,hi_a,L in holes[side]:
                junction(X, side, lo_a, hi_a, RJ, L, a0, qs, ws)

    def junction(X, side, lo_a, hi_a, RJ, L, a0, qs, ws):
        """Corner where a longitudinal groove opens into a ring groove: height field
        depth = max(ring lip depth, longitudinal lip depth), clipped at the longitudinal filler."""
        A, sec = L['A'], L['section']; fl = c+sec['filler_recess_m']; fr = c+RJ['recess']
        x_of = lambda u: X+side*u
        n_ring = RJ['wig'][side](A*r)                   # ring inner-edge offset at this crossing
        u_ref = wall_half_width(n_ring,fr)+FILLER_OVERLAP   # ring filler edge at the crossing centre
        # Nodes as fractions between the ring filler edge (per angle, watertight with the ring
        # filler) and the panel boundary; include the lip facet and filler-level positions.
        cand = {q for q,_ in lip_points(n_ring)} | {wall_half_width(n_ring,fl)+FILLER_OVERLAP}
        # Dense near the ring filler edge, where the lip/filler boundary cells would otherwise
        # form short steep facets.
        taus = sorted({round(v,9) for v in {i/4 for i in range(5)} | {.015,.03,.06} |
                       {(q-u_ref)/(outer-u_ref) for q in cand if u_ref < q < outer}})
        # Ring filler edge: the same polyline the ring filler quads use (watertight join).
        unwrap = lambda q: a0+math.fmod(math.fmod(q-a0,2*math.pi)+2*math.pi,2*math.pi)
        u_min_at = lambda q: float(np.interp(unwrap(q), qs, ws))
        Aw = unwrap(A)
        ring_nodes = {(q-Aw)*r for q in qs if abs(q-Aw)*r < outer}
        xm = x_of((u_ref+outer)/2)
        hw = {sd:wall_half_width(L['wig'][sd](xm),fl)+FILLER_OVERLAP for sd in (-1,1)}
        s_nodes = sorted({round(v,9) for v in {-outer, outer, -hw[-1], hw[1]} |
                         {sd*q for sd in (-1,1) for q,_ in lip_points(L['wig'][sd](xm)) if q < outer} |
                         {-outer+2*outer*i/12 for i in range(13)} | ring_nodes |
                         {sd*(hw[sd]+e) for sd in (-1,1) for e in (-2e-4,-1e-4,1e-4,2e-4)} if abs(v) <= outer+1e-12})
        def depth(u, s, tau):
            # Filler level blends from the ring's (at its filler edge) to the longitudinal
            # joint's (at the panel boundary): continuous mortar, no step at the crossing.
            fl = fr+(c+sec['filler_recess_m']-fr)*tau
            sd = 1 if s >= 0 else -1
            dr = lip_depth(lip_points(RJ['wig'][side]((A+s/r)*r)), u)
            dl = lip_depth(lip_points(L['wig'][sd](x_of(u))), abs(s))
            removal = max(dr, dl)
            return (fl, True) if removal >= fl-1e-12 else (removal, False)
        U = [[u_min_at(A+q/r)+(outer-u_min_at(A+q/r))*tau for q in s_nodes] for tau in taus]
        grid = [[depth(U[i][j], q, taus[i]) for j,q in enumerate(s_nodes)] for i in range(len(taus))]
        toward = (X, 0., zc)
        node = lambda a,b: point(x_of(U[a][b]), A+s_nodes[b]/r, r+grid[a][b][0], zc)
        def underlay(mesh, ends, across_u, extend=(0, 0)):
            """Hidden strip just below a seam of this height-field patch, spanning the seam on
            both sides. The patch's edge vertices cannot all match its neighbours' (chords vs
            arcs, differently spaced nodes), so a micrometre slit may remain; rays through it
            meet the strip in the patch's own material instead of the backing. Both sides of
            the seam lie above the strip, so it is never directly visible."""
            (u0,s0,d0),(u1,s1,d1) = ends; w = UNDERLAY_HALF
            # At patch corners the strip also runs on past the corner (three surfaces meet there).
            if across_u: s0, s1 = s0-extend[0]*w*math.copysign(1, s1-s0), s1+extend[1]*w*math.copysign(1, s1-s0)
            else: u0, u1 = u0-extend[0]*w*math.copysign(1, u1-u0), u1+extend[1]*w*math.copysign(1, u1-u0)
            du, ds = (w, 0.) if across_u else (0., w)
            pts = [point(x_of(u+k*du), A+(q+k*ds)/r, r+d+UNDERLAY_DEPTH, zc)
                   for (u,q,d),k in (((u0,s0,d0),-1),((u1,s1,d1),-1),((u1,s1,d1),1),((u0,s0,d0),1))]
            oriented(mesh, pts, toward)
        seam = lambda a,b: (U[a][b], s_nodes[b], grid[a][b][0])
        last_i, last_j = len(taus)-1, len(s_nodes)-1
        stepped = [not (abs(grid[0][j][0]-fr) < 1e-9 and abs(grid[0][j+1][0]-fr) < 1e-9) for j in range(last_j)]
        for i in range(len(taus)-1):
            for j in range(len(s_nodes)-1):
                corners = [(i,j),(i+1,j),(i+1,j+1),(i,j+1)]
                is_fill = sum(grid[a][b][1] for a,b in corners) >= 3
                mesh = fill if is_fill else top
                oriented(mesh, [node(a,b) for a,b in corners], toward)
                row_ends, col_ends = (j == 0, j+1 == last_j), (i == 0, i+1 == last_i)
                if i == 0 and not stepped[j]: underlay(mesh, (seam(0,j), seam(0,j+1)), True, row_ends)
                if i+1 == last_i: underlay(mesh, (seam(last_i,j), seam(last_i,j+1)), True, row_ends)
                if j == 0: underlay(mesh, (seam(i,0), seam(i+1,0)), False, col_ends)
                if j+1 == last_j: underlay(mesh, (seam(i,last_j), seam(i+1,last_j)), False, col_ends)
        # Step down/up to the ring filler along the ring filler edge.
        for j in range(len(s_nodes)-1):
            if not stepped[j]: continue
            d0, d1 = grid[0][j][0], grid[0][j+1][0]
            q0, q1 = A+s_nodes[j]/r, A+s_nodes[j+1]/r; u0, u1 = U[0][j], U[0][j+1]
            mesh = fill if grid[0][j][1] and grid[0][j+1][1] else groove
            inside = point(x_of(u0-1e-3),(q0+q1)/2,r,zc)
            oriented(mesh,[point(x_of(u0),q0,r+d0,zc),point(x_of(u1),q1,r+d1,zc),point(x_of(u1),q1,r+fr,zc),point(x_of(u0),q0,r+fr,zc)],inside)
            underlay(mesh, ((u0,s_nodes[j],fr),(u1,s_nodes[j+1],fr)), True, (j == 0, j+1 == last_j))

    def longitudinal_joint(R, L, closed):
        A, section, wig = L['A'], L['section'], L['wig']; x0, x1 = R['x0'], R['x1']
        above = lambda x: point(x, A, r-1., zc)   # a point inside the tunnel above the joint
        previous = None
        levels = filler_levels(section, x1-x0) or [(0., x1-x0, None)]
        for a,b,depth in levels:
            xa, xb = x0+a, x0+b; n = max(1, math.ceil((xb-xa)/p['joint_along_step_m']))
            xs = [xa+(xb-xa)*i/n for i in range(n+1)]
            shapes = [cross_section(section, depth if depth is not None else 0., wig[-1](x), wig[1](x)) for x in xs]
            for side in (-1,1):
                for xa_,xb_,(s0,_),(s1,_) in zip(xs[:-1],xs[1:],shapes[:-1],shapes[1:]):
                    world = lambda s_,d_,end,xa_=xa_,xb_=xb_,side=side: point(xb_ if end else xa_, A+side*s_/r, r+d_, zc)
                    sweep(top, s0[side][:len(profile)], s1[side][:len(profile)], world, above((xa_+xb_)/2))
                    long_lip_nodes[(section['ring'], section['index'], side)].update((xa_, xb_))
                    sweep(groove, s0[side][len(profile)-1:], s1[side][len(profile)-1:], world, point((xa_+xb_)/2,A,r+c,zc))
            if depth is not None:
                rad = r+c+depth
                for xa_,xb_,(_,w0),(_,w1) in zip(xs[:-1],xs[1:],shapes[:-1],shapes[1:]):
                    oriented(fill,[point(xa_,A-w0[-1]/r,rad,zc),point(xb_,A-w1[-1]/r,rad,zc),point(xb_,A+w1[1]/r,rad,zc),point(xa_,A+w0[1]/r,rad,zc)],above((xa_+xb_)/2))
                if previous is not None and abs(previous-rad) > 1e-9:
                    lo_r, hi_r = sorted((previous, rad)); wl, wr = shapes[0][1][-1], shapes[0][1][1]
                    oriented(fill,[point(xa,A-wl/r,lo_r,zc),point(xa,A+wr/r,lo_r,zc),point(xa,A+wr/r,hi_r,zc),point(xa,A-wl/r,hi_r,zc)],point(xa+(1 if rad>previous else -1)*1e-3,A,r,zc))
                previous = rad
            else:
                previous = None
            if section['state'] != 'filled':
                aw, ag = w/r, g/r
                for side in (-1,1):
                    oriented(groove,[point(xa,A+side*ag,r+D,zc),point(xb,A+side*ag,r+D,zc),point(xb,A+side*aw,r+D,zc),point(xa,A+side*aw,r+D,zc)],above((xa+xb)/2))
                    oriented(groove,[point(xa,A+side*aw,r+D,zc),point(xb,A+side*aw,r+D,zc),point(xb,A+side*aw,r+D+G+seal,zc),point(xa,A+side*aw,r+D+G+seal,zc)],point((xa+xb)/2,A,r+D,zc))
                oriented(gap,[point(xa,A-aw,r+D+G,zc),point(xb,A-aw,r+D+G,zc),point(xb,A+aw,r+D+G,zc),point(xa,A+aw,r+D+G,zc)],above((xa+xb)/2))
        # End walls only where the groove meets a ring joint without a junction patch.
        for x, inward, is_closed in ((x0,1,closed[0]),(x1,-1,closed[1])):
            if not is_closed: continue
            sides, _ = cross_section(section, (filler_levels(section,x1-x0) or [(0,0,0.)])[0 if inward>0 else -1][2] or 0., wig[-1](x), wig[1](x))
            Lp, Rp = sides[-1], sides[1]
            if section['state'] != 'filled':
                Lp = Lp+[(w, D), (w, D+G)]; Rp = Rp+[(w, D), (w, D+G)]
            ref = point(x+inward*1e-3, A, r+c, zc)
            for (sl,dl),(sl2,dl2),(sr2,dr2),(sr,dr) in zip(Lp[:-1],Lp[1:],Rp[1:],Rp[:-1]):
                oriented(groove,[point(x,A-sl/r,r+dl,zc),point(x,A-sl2/r,r+dl2,zc),point(x,A+sr2/r,r+dr2,zc),point(x,A+sr/r,r+dr,zc)],ref)

    def unique_sorted(values, key=lambda v: v):
        out = []
        for v in sorted(values, key=key):
            if not out or key(v)-key(out[-1]) > 1e-12: out.append(v)
        return out

    def panel(ring, R, index, L):
        """Panel face between its lips, without T-junctions: its boundary vertices are exactly the
        lips' outer-edge vertices (ring lips at x0/x1, longitudinal lips along both sides).
        A zipper joins the x0 and x1 node rows; the two side edges fan out to the longitudinal
        lip nodes. Angular spans stay within the node spacing (<= the panel step)."""
        a, b = L['A']+outer/r, R['edges'][index+1]-outer/r
        mid = (a+b)/2
        def row(X, side):
            if (X, side) not in ring_lip_nodes:     # tunnel end: plain subdivision
                n = max(1, math.ceil((b-a)/step))
                return [(a+(b-a)*i/n,)*2 for i in range(n+1)]
            nodes = []
            for q in ring_lip_nodes[(X, side)]:
                unwrapped = q+2*math.pi*round((mid-q)/(2*math.pi))
                if a-1e-9 <= unwrapped <= b+1e-9: nodes.append((unwrapped, q))
            nodes = unique_sorted(nodes, key=lambda v: v[0])
            if not nodes or abs(nodes[0][0]-a) > 1e-9: nodes.insert(0, (a, a))
            if abs(nodes[-1][0]-b) > 1e-9: nodes.append((b, b))
            return nodes
        x0, x1 = R['x0'], R['x1']
        bottom = row(R['left'], 1) if ring in ring_joints else row(None, 0)
        top_row = row(R['right'], -1) if ring+1 in ring_joints else row(None, 0)
        nxt = R['longs'][index+1] if index+1 < len(R['longs']) else R['longs'][0]
        def chain(key):
            xs = unique_sorted(long_lip_nodes.get(key, ()))
            xs = [x for x in xs if x0+1e-9 < x < x1-1e-9]
            return [x0]+xs+[x1]
        left, right = chain((ring, index, 1)), chain((ring, nxt['section']['index'], -1))
        # Vertices are (x, sort angle, exact angle); point() gets the lips' exact float inputs.
        B = [(x0, q, e) for q, e in bottom]; T = [(x1, q, e) for q, e in top_row]
        tris = []; i = j = 0
        while i < len(B)-1 or j < len(T)-1:
            if j == len(T)-1 or (i < len(B)-1 and B[i+1][1] <= T[j+1][1]):
                tris.append((B[i], B[i+1], T[j])); i += 1
            else:
                tris.append((B[i], T[j+1], T[j])); j += 1
        def emit(v):
            centre = point(sum(p[0] for p in v)/3, sum(p[1] for p in v)/3, r-1., zc)
            pts = [point(p[0], p[2], r, zc) for p in v]
            oriented(top, pts+[pts[-1]], centre)
        def fan(tri, corner_b, corner_t, xs, angle):
            """Split the side edge corner_b-corner_t at the longitudinal lip's x nodes."""
            apex = next(p for p in tri if p is not corner_b and p is not corner_t)
            chain = [corner_b]+[(x, angle, angle) for x in xs[1:-1]]+[corner_t]
            for p0, p1 in zip(chain[:-1], chain[1:]): emit((p0, p1, apex))
        fan(tris[0], B[0], T[0], left, a)
        for tri in tris[1:-1]: emit(tri)
        fan(tris[-1], B[-1], T[-1], right, b)
        panels.append(dict(ring=ring, index=index, x_m=[x0, x1], angle_rad=[a, b]))

    try:
        # Opaque lining behind all slot floors: closes crossings, no renderer fallback.
        cylinder_strip(groove,lo,hi,-math.pi,math.pi,edge_radius,zc,step)
        for ring, R in rings.items():
            if ring in ring_joints: ring_joint(ring)
            for index, L in enumerate(R['longs']):
                patched = lambda rj: rj in ring_joints and crossing_ok(ring_joints[rj], L)
                longitudinal_joint(R, L, (ring in ring_joints and not patched(ring), ring+1 in ring_joints and not patched(ring+1)))
        for ring, R in rings.items():
            for index, L in enumerate(R['longs']):
                panel(ring, R, index, L)
    finally:
        top.close()
        groove.close()
        fill.close()
        gap.close()
    counts = {k:sum(j['state']==k for j in joints) for k in ('filled','unfilled','damaged')}
    return dict(**counters, panels=panels, joints=joints, joint_state_counts=counts,
                max_chord_sag_m=r*(1-math.cos(step/2)),
                joint_profile=dict(chamfer_m=c, groove_half_width_m=g, depth_m=D, outer_width_m=2*outer, contact_gap_m=2*w, gap_depth_m=G,
                                   edge_radius_m=rho, edge_fillet_steps=fsteps, edge_wiggle=p['joint_edge_wiggle'],
                                   filler_recess_m=p['joint_filler_recess_m'], states=p['joint_states'], damage=p['joint_damage'],
                                   assumption='Chamfer, edge radius/wiggle, filler recess and damage are engineering assumptions; groove width/depth within Tianjin DB/T 29-272 ranges. Filler material: grey mortar (user choice).'),
                hidden_edge_overlap_m=seal, opaque_backing_radius_m=edge_radius)


def sub(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def box(link, name, pose, size, color, collision=False):
    visual = sub(link, 'visual', name=name)
    sub(visual, 'pose', pose)
    sub(sub(sub(visual, 'geometry'), 'box'), 'size', size)
    material = sub(visual, 'material')
    sub(material, 'diffuse', color)
    sub(material, 'ambient', color)
    if collision:
        col = sub(link, 'collision', name=name)
        sub(col, 'pose', pose)
        sub(sub(sub(col, 'geometry'), 'box'), 'size', size)


def inertial(link, mass, inertia):
    node = sub(link, 'inertial')
    sub(node, 'mass', mass)
    mat = sub(node, 'inertia')
    for tag, value in zip(('ixx','iyy','izz'), inertia):
        sub(mat, tag, value)
    for tag in ('ixy','ixz','iyz'):
        sub(mat, tag, 0)


def cylinder(link, name, pose, radius, length, color):
    visual = sub(link,'visual',name=name)
    sub(visual,'pose',pose)
    geometry = sub(sub(visual,'geometry'),'cylinder')
    sub(geometry,'radius',radius); sub(geometry,'length',length)
    material = sub(visual,'material')
    sub(material,'diffuse',color); sub(material,'ambient',color)


def make_world(out, config, spec, previews=None):
    root = ET.Element('sdf', version='1.9')
    world = sub(root, 'world', name='stage_b')
    physics = sub(world, 'physics', name='1ms', type='dart')
    sub(physics, 'max_step_size', config['motion']['sample_period_s'])
    sub(physics, 'real_time_factor', 1)
    for filename, name in (('gz-sim-physics-system','Physics'), ('gz-sim-user-commands-system','UserCommands'), ('gz-sim-scene-broadcaster-system','SceneBroadcaster')):
        sub(world, 'plugin', filename=filename, name='gz::sim::systems::'+name)
    sub(world, 'gravity', '0 0 -9.81')
    # GUI-only diffuse surroundings; no fixed point or directional lights.
    scene = sub(world,'scene')
    ambient = spec.get('preview',{}).get('ambient_rgb',[.65,.65,.65])
    sub(scene,'ambient',' '.join(map(str,[*ambient,1])))
    sub(scene,'background','0.18 0.20 0.23 1')
    sub(scene,'shadows','false')
    sub(scene,'grid','false')
    tunnel = sub(world, 'model', name='tunnel')
    sub(tunnel, 'static', 'true')
    link = sub(tunnel, 'link', name='lining')
    lining = [('joints','joints.obj',None,'0.28 0.28 0.28 1'),('filler','filler.obj',None,'0.42 0.42 0.41 1'),('gap','gap.obj',None,'0.05 0.05 0.05 1')]
    if previews:
        lining.extend((p['name'],p['mesh'],p['texture'],'1 1 1 1') for p in previews)
    else:
        texture='wall_preview.png' if (out/'wall_preview.png').exists() else None
        lining.append(('panels','panels.obj',texture,'1 1 1 1' if texture else '0.48 0.48 0.48 1'))
    for name, mesh, texture, color in lining:
        vis = sub(link, 'visual', name=name)
        sub(sub(sub(vis, 'geometry'), 'mesh'), 'uri', str((out/mesh).resolve()))
        mat = sub(vis, 'material')
        sub(mat, 'diffuse', color)
        sub(mat, 'ambient', color)
        if texture:
            metal=sub(sub(mat,'pbr'),'metal')
            sub(metal,'albedo_map',str((out/texture).resolve()))
            sub(metal,'roughness',.9)
            sub(metal,'metalness',0)
    from .stage_b_track import replace_track
    replace_track(world,out,config,spec)
    from .stage_b_robot import make_robot
    world.append(make_robot(out,config,spec))
    from .stage_b_lighting import apply_work_light_environment
    apply_work_light_environment(world,spec)
    ET.indent(root)
    ET.ElementTree(root).write(out/'world.sdf', encoding='unicode', xml_declaration=True)


def make_joint(model, name, kind, parent, child, axis):
    joint = sub(model, 'joint', name=name, type=kind)
    sub(joint, 'parent', parent)
    sub(joint, 'child', child)
    node = sub(joint, 'axis')
    sub(node, 'xyz', axis)
    limit = sub(node, 'limit')
    for tag, value in (('lower',-1e16),('upper',1e16),('effort',1e9),('velocity',1e9)):
        sub(limit, tag, value)


def prepare(config_path, spec_path, output, surface_path=None):
    start = time.monotonic()
    config = yaml.safe_load(Path(config_path).read_text())
    spec = load_spec(spec_path)
    plan = resource_plan(config, spec)
    output = Path(output).resolve()
    if output.exists():
        raise ValueError('output already exists: '+str(output))
    output.mkdir(parents=True)
    try:
        geometry = make_meshes(output, config, spec)
        # The lining must be closed: no ray from inside may reach the backing (ssb_tools.mesh_audit).
        from .mesh_audit import audit
        report = audit(output, config['tunnel']['radius_m'], config['tunnel']['axis_z_m'])
        (output/'mesh_audit.json').write_text(json.dumps(report, indent=2)+'\n')
        if report['leaks']['edges'] and geometry['joint_state_counts']['filled'] == len(geometry['joints']):
            raise ValueError(f"optical lining leaks at {report['leaks']['edges']} edges; see mesh_audit.json")
        geometry['light_leak_edges'] = report['leaks']['edges']
        previews = None
        if surface_path:
            from .stage_b_preview import prepare_preview
            previews, preview_plan = prepare_preview(surface_path,output,geometry,config,spec)
            plan['gui_preview'] = preview_plan
        make_world(output, config, spec, previews)
    except Exception:
        (output/'FAILED').write_text('Preparation incomplete; do not use this scene.\n')
        raise
    manifest = dict(schema='ssb.stage_b_geometry.v1', implementation='geometry_and_timing_foundation',
                    config=dict(path=str(Path(config_path).resolve()), sha256=digest(config_path)),
                    spec=dict(path=str(Path(spec_path).resolve()), sha256=digest(spec_path)),
                    geometry=geometry, resource_plan=plan,
                    assets={str(p.relative_to(output)):digest(p) for p in output.rglob('*') if p.is_file()},
                    preparation_seconds=time.monotonic()-start,
                    preparation_peak_rss_bytes=peak_rss_bytes(),
                    limitations=['Bind these hashed optical meshes with stage_b_optics for capture.',
                                 'Handholes remain to implement; joint states are recorded in geometry.joints.',
                                 ('Rigid wheel/rail friction contact; front drive, rear free running wheels, encoders on two spring-loaded measuring wheels. '
                                  'Contact fidelity requires dynamics validation.' if config.get('contact',{}).get('enabled') else
                                  'Prismatic guide and velocity-commanded wheels with no wheel contacts; '
                                  'not a validated wheel/rail contact model.')])
    if surface_path: manifest['preview_surface']=dict(path=str(Path(surface_path).resolve()),sha256=digest(surface_path))
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(output=str(output), triangles=geometry['triangles'],
                         peak_rss_bytes=manifest['preparation_peak_rss_bytes'], preparation_seconds=manifest['preparation_seconds'])))
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--spec', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--surface', type=Path, help='baked surface manifest for detailed Gazebo panel textures')
    args = parser.parse_args()
    prepare(args.config, args.spec, args.output,args.surface)


if __name__ == '__main__':
    main()
