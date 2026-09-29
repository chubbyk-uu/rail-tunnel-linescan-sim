"""Bounded preparation of the Stage B Gazebo geometry and its resource plan.

Optical meshes are evaluation/scene data, never reconstruction inputs. This module
does not turn on a textured OptiX renderer or assert wheel/rail contact fidelity.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import resource
import time
import xml.etree.ElementTree as ET

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
    values = [p['ring_width_m'], p['angular_step_deg'], p['joint_width_m'], p['joint_depth_m']]
    if not all(math.isfinite(v) and v > 0 for v in values):
        raise ValueError('invalid panel dimensions')
    if len(p['angles_deg']) != 6 or not all(v > 0 for v in p['angles_deg']) or abs(sum(p['angles_deg']) - 360) > 1e-9:
        raise ValueError('six panel angles must cover 360 degrees')
    if p['joint_width_m'] >= p['ring_width_m']:
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
                rendered_batch_bytes=per_batch, render_job_queue_bytes=batch*96*config['render']['max_queued_batches'],
                write_queue_limit_bytes=config['storage']['write_queue_bytes'],
                storage_block_bytes=config['storage']['block_rows']*cam['width'],
                pose_stream_bytes_estimate=56*math.ceil(config['motion']['profile'][-1][0]/config['motion']['sample_period_s']),
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
        u = [points[1][i]-points[0][i] for i in range(3)]
        v = [points[2][i]-points[0][i] for i in range(3)]
        normal = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
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
        index=lambda i:f'{i}/{i}/{i}' if self.uv_domain else f'{i}//{i}'
        self.out.write(f'f {index(first)} {index(first+1)} {index(first+2)}\nf {index(first)} {index(first+2)} {index(first+3)}\n')
        self.vertices += 4
        self.faces += 2
        self.counters['vertices'] += 4
        self.counters['triangles'] += 2

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


def make_meshes(out, config, spec):
    p = spec['panels']
    t = config['tunnel']
    r, zc, lo, hi = (t[k] for k in ('radius_m', 'axis_z_m', 'x_min_m', 'x_max_m'))
    step = math.radians(p['angular_step_deg'])
    half = p['joint_width_m']/2
    da = half/r
    # Slot floors and edge walls use different angular partitions. Extend walls
    # slightly behind the floor to seal facet gaps without altering the visible rim.
    seal=2*(r+p['joint_depth_m'])*(1-math.cos(step/2))+1e-5
    edge_radius=r+p['joint_depth_m']+seal
    if 2*da >= math.radians(min(p['angles_deg'])):
        raise ValueError('joint wider than a panel')
    counters = dict(vertices=0, triangles=0)
    top, groove = (Mesh(out/name, counters, spec['resources'],(lo,hi,zc)) for name in ('panels.obj','joints.obj'))
    panels = []
    try:
        # Opaque lining behind all slot floors. This closes staggered joint
        # crossings where differently tessellated floor strips meet, with no
        # extra collision geometry and no analytic/pixel fallback in the renderer.
        cylinder_strip(groove,lo,hi,-math.pi,math.pi,edge_radius,zc,step)
        width = p['ring_width_m']
        for ring in range(math.floor(lo/width), math.ceil(hi/width)):
            left, right = ring*width, (ring+1)*width
            x0, x1 = max(lo, left+half), min(hi, right-half)
            if x1 <= x0:
                continue
            if lo < left < hi:
                cylinder_strip(groove, max(lo,left-half), min(hi,left+half), -math.pi, math.pi, r+p['joint_depth_m'], zc, step)
            a = math.radians(-p['angles_deg'][0]/2 + (ring % 2)*p['alternating_stagger_deg'])
            for index, span in enumerate(p['angles_deg']):
                b = a+math.radians(span)
                cylinder_strip(top, x0, x1, a+da, b-da, r, zc, step)
                cylinder_strip(groove, x0, x1, a-da, a+da, r+p['joint_depth_m'], zc, step)
                # Circumferential edge walls, extending outward into the lining.
                for angle, reverse in ((a+da, False), (b-da, True)):
                    pts = [point(x0,angle,r,zc), point(x0,angle,edge_radius,zc),
                           point(x1,angle,edge_radius,zc), point(x1,angle,r,zc)]
                    groove.quad(pts[::-1] if reverse else pts)
                count = max(1, math.ceil((b-a-2*da)/step))
                for edge, reverse in ((x0, True), (x1, False)):
                    for i in range(count):
                        u = a+da+(b-a-2*da)*i/count
                        v = a+da+(b-a-2*da)*(i+1)/count
                        pts = [point(edge,u,r,zc), point(edge,v,r,zc),
                               point(edge,v,edge_radius,zc), point(edge,u,edge_radius,zc)]
                        groove.quad(pts[::-1] if reverse else pts)
                panels.append(dict(ring=ring, index=index, x_m=[x0,x1], angle_rad=[a+da,b-da]))
                a = b
    finally:
        top.close()
        groove.close()
    return dict(**counters, panels=panels, max_chord_sag_m=r*(1-math.cos(step/2)),
                joint_state='unfilled', joint_width_m=2*half, joint_depth_m=p['joint_depth_m'],
                hidden_edge_overlap_m=seal,opaque_backing_radius_m=edge_radius)


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
    lining = [('joints','joints.obj',None,'0.28 0.28 0.28 1')]
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
    track = sub(world, 'model', name='track')
    sub(track, 'static', 'true')
    rails = sub(track, 'link', name='rails')
    t = spec['track']
    lo, hi = config['tunnel']['x_min_m'], config['tunnel']['x_max_m']
    length, centre = hi-lo, (hi+lo)/2
    head_y = (t['gauge_m']+t['head_width_m'])/2
    for side, y in (('left',head_y), ('right',-head_y)):
        # Rail top is z=0; 38/121.5/16.5 mm simplified rail sections total 176 mm.
        for part, width, height, z in (('head',t['head_width_m'],.038,-.019),
                                      ('web',t['web_width_m'],.1215,-.09875),
                                      ('foot',t['foot_width_m'],.0165,-.16775)):
            box(rails, side+'_'+part, f'{centre} {y} {z} 0 0 0', f'{length} {width} {height}', '0.32 0.34 0.36 1', collision=part=='head')
    for side, y in (('left',head_y), ('right',-head_y)):
        box(rails, side+'_support', f'{centre} {y} -0.2075 0 0 0', f'{length} 0.25 0.063', '0.25 0.25 0.25 1')
    box(rails, 'bed', f'{centre} 0 -0.339 0 0 0', f'{length} {t["bed_width_m"]} 0.200', '0.3 0.3 0.3 1', collision=True)
    box(rails, 'foundation', f'{centre} 0 -0.587 0 0 0', f'{length} {t["bed_width_m"]} 0.296', '0.35 0.35 0.35 1')
    from .stage_b_robot import make_robot
    world.append(make_robot(out,config,spec))
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
                                 'Filled/damaged seams and handholes remain to implement.',
                                 'Prismatic guide and velocity-commanded wheels with no wheel contacts; '
                                 'not a validated wheel/rail contact model.'])
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
