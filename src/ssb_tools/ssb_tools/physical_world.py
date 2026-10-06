"""Physical-world manifest and strict consistency check (rails, wheels, springs).

The capture configuration holds the simulation truth; the world SDF holds what Gazebo actually
simulates. A manifest written when the world is generated records the physical assets: rail
heightmap segments (pose, size, samples, file hash) and the vehicle's wheel radii and spring
joints. Before a capture, and again on the archived snapshot afterwards, the world is checked
against the configuration directly (complete two-rail coverage, placement, orientation, size,
decoded profile per segment, wheel radii, springs) and against the manifest (file hashes).
A configuration edited without regenerating the world is therefore rejected, and comparing
hashes alone is never enough. The car's start pose is not part of the manifest (missions move
it); the Gazebo plugin checks it separately. Evaluation truth only; never a reconstruction input.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from .package_paths import share_file

SCHEMA = 'ssb.physical_manifest.v2'
NAME = 'physical_manifest.json'
RUNNING_WHEELS = ('odometer_wheel', 'wheel_1', 'wheel_2', 'wheel_3')
MEASURING = {'left': 'measure_left', 'right': 'measure_right'}
SUSPENSION = ('odometer_suspension', 'wheel_joint_1_suspension', 'wheel_joint_2_suspension', 'wheel_joint_3_suspension')
# Adjacent segments share their overlap sample positions (one global grid); their heights differ
# only by each segment's own 16-bit quantization.


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _floats(text):
    return [float(v) for v in text.split()]


def _resolve(uri, base, snapshot=None):
    uri = uri.strip().removeprefix('file://')
    if snapshot is not None:
        path = Path(snapshot)/Path(uri).name
        if not path.is_file():
            raise FileNotFoundError(f'missing archived physical asset: {path}')
        return path
    return Path(uri) if Path(uri).is_absolute() else Path(base)/uri


def describe(world_path, snapshot=None):
    """What the SDF actually contains: rail heightmap models and the car's physical parameters."""
    from PIL import Image
    world_path = Path(world_path)
    root = ET.parse(world_path).getroot()
    rails = []
    for model in root.iter('model'):
        if not model.get('name', '').startswith('rail_surface_'): continue
        shape = model.find('link/collision/geometry/heightmap')
        path = _resolve(shape.findtext('uri'), world_path.parent, snapshot)
        width, height = Image.open(path).size
        rails.append(dict(name=model.get('name'), pose=_floats(model.findtext('pose', '0 0 0 0 0 0')),
                          size=_floats(shape.findtext('size')), samples=[width, height],
                          file=Path(shape.findtext('uri')).name, path=str(path), sha256=sha256(path)))
    car = root.find(".//model[@name='scan_car']")
    wheels, joints, guides = {}, {}, {}
    if car is not None:
        for link in car.findall('link'):
            guide=link.find("collision[@name='guide_contact']/geometry")
            if guide is not None:
                shape=next(iter(guide)).tag
                guides[link.get('name')]=dict(shape=shape,pose=_floats(link.findtext('pose','0 0 0 0 0 0')),
                    radius=float(guide.findtext(f'{shape}/radius')),length=float(guide.findtext(f'{shape}/length','0')))
            geometry = link.find("collision[@name='tread_contact']/geometry")
            if geometry is None: continue
            kind = next(iter(geometry)).tag
            wheels[link.get('name')] = dict(shape=kind, radius=float(geometry.findtext(f'{kind}/radius')))
        for joint in car.findall('joint'):
            name = joint.get('name')
            if name in SUSPENSION or name.endswith('_slide'):
                axis = joint.find('axis')
                joints[name] = dict(stiffness=float(axis.findtext('dynamics/spring_stiffness', '0')),
                                    damping=float(axis.findtext('dynamics/damping', '0')),
                                    reference=float(axis.findtext('dynamics/spring_reference', '0')),
                                    lower=float(axis.findtext('limit/lower')), upper=float(axis.findtext('limit/upper')))
    boxes = []
    track = root.find("world/model[@name='track']")
    link = track.find("link[@name='rails']") if track is not None else None
    if link is not None:
        for collision in link.findall('collision'):
            box = collision.find('geometry/box')
            boxes.append(dict(name=collision.get('name'), shape='box' if box is not None else 'other',
                              model_pose=_floats(track.findtext('pose', '0 0 0 0 0 0')),
                              link_pose=_floats(link.findtext('pose', '0 0 0 0 0 0')),
                              pose=_floats(collision.findtext('pose', '0 0 0 0 0 0')),
                              size=_floats(box.findtext('size')) if box is not None else []))
    assembly = None
    if car is not None:
        base = car.find("link[@name='base']"); head = car.find("link[@name='head']")
        scan = car.find("joint[@name='scan']")
        if base is not None and head is not None and scan is not None:
            assembly = dict(base_pose=_floats(base.findtext('pose', '0 0 0 0 0 0')),
                            head_pose=_floats(head.findtext('pose', '0 0 0 0 0 0')),
                            axis=_floats(scan.findtext('axis/xyz', '0 0 0')),
                            axis_frame=scan.find('axis/xyz').get('expressed_in', ''))
    return dict(rails=sorted(rails, key=lambda r: r['name']), rail_boxes=sorted(boxes, key=lambda b: b['name']),
                wheels=wheels, joints=joints, scanner_assembly=assembly,guide_bearings=guides)


def expected(config, spec):
    """What the configuration (truth) and scene spec require."""
    from .rail_irregularity import settings, segments, SDF_SIZE_FACTOR, SEGMENT_SAMPLES, rails as rail_profiles, guide_box_top
    s = settings(config)
    track, robot = spec['track'], spec['robot']
    head_y = (track['gauge_m']+track['head_width_m'])/2
    x0, x1 = float(config['tunnel']['x_min_m']), float(config['tunnel']['x_max_m'])
    rails = []
    if s is not None:
        for k, (a, b) in enumerate(segments(x0, x1)):
            for side, y in (('left', head_y), ('right', -head_y)):
                rails.append(dict(name=f'rail_surface_{side}_{k:02d}', rail=side, x_m=[a, b], y=y,
                                  length=(b-a)*SDF_SIZE_FACTOR, width=track['head_width_m']*SDF_SIZE_FACTOR))
    profile = rail_profiles(config) if s is not None else None
    box_top = guide_box_top(np.minimum(profile[1], profile[2])) if profile is not None else 0.
    height = .038+box_top
    boxes = [dict(name=side+'_head', shape='box', model_pose=[0.]*6, link_pose=[0.]*6,
                  pose=[(x0+x1)/2, sign*head_y, box_top-height/2, 0., 0., 0.],
                  size=[x1-x0, track['head_width_m'], height])
             for side, sign in (('left', 1), ('right', -1))]
    truth = config['truth']
    wheels = {name: dict(shape='cylinder', radius=truth['wheel_diameter_m']/2) for name in RUNNING_WHEELS}
    joints, guides = {}, {}
    compliance = truth.get('wheel_compliance')
    if compliance:
        for name in SUSPENSION:
            joints[name] = dict(stiffness=compliance['stiffness_n_m'], damping=compliance['damping_n_s_m'], reference=0.)
    if config.get('contact', {}).get('enabled'):
        clearance=robot.get('guide_bearing_clearance_m',.0002)
        radius=robot.get('guide_bearing_radius_m',.025)
        length=robot.get('guide_bearing_width_m',.024)
        for side,sign in (('left',1),('right',-1)):
            for x in (-robot['wheelbase_m']/2,robot['wheelbase_m']/2):
                guides[f'{side}_guide_{x}']=dict(shape='cylinder',radius=radius,length=length,
                    pose=[x,sign*(track['gauge_m']/2-radius-clearance),-.019,0.,0.,0.])
        m = robot['measuring_wheel']
        for side, prefix in MEASURING.items():
            wheels[prefix+'_wheel'] = dict(shape='sphere', radius=truth[f'odo_{side}_diameter_m']/2)
            joints[prefix+'_slide'] = dict(stiffness=m['spring_rate_n_m'], damping=m['damping_n_s_m'],
                                          reference=-m['preload_n']/m['spring_rate_n_m'],
                                          lower=-m['travel_m'], upper=m['travel_m'])
    return dict(settings=s, x_range_m=[x0, x1], head_y_m=head_y, head_width_m=track['head_width_m'],
                segment_samples=SEGMENT_SAMPLES, rails=rails, rail_boxes=boxes, wheels=wheels, joints=joints,guide_bearings=guides)


def write_manifest(world_path, config, spec):
    """Record the generated physical assets beside the world; returns the manifest path."""
    world_path = Path(world_path)
    actual = describe(world_path)
    for rail in actual['rails']: rail.pop('path')
    manifest = dict(schema=SCHEMA, expected=expected(config, spec), actual=actual,
                    role='simulation truth: physical world assets; evaluation only')
    path = world_path.parent/NAME
    path.write_text(json.dumps(manifest, indent=2)+'\n')
    return path


def _close(a, b, tol=1e-9):
    return all(abs(x-y) <= tol*max(1., abs(y)) for x, y in zip(a, b)) and len(a) == len(b)


def check(config, spec, world_path, manifest_path=None, snapshot=None, decode=True):
    """Strict check of the world (SDF) against the configuration and the manifest."""
    from PIL import Image
    from .rail_irregularity import rails as rail_profiles
    world_path = Path(world_path)
    manifest_path = Path(manifest_path) if manifest_path else world_path.parent/NAME
    want = json.loads(json.dumps(expected(config, spec)))
    have = describe(world_path, snapshot)
    checks = {}
    def record(name, ok, **details):
        checks[name] = dict(passed=bool(ok), **details)
    # Manifest: present, written for this configuration, and describing these files.
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else None
    record('manifest_present', manifest is not None and manifest.get('schema') == SCHEMA, path=str(manifest_path))
    if manifest is not None:
        record('manifest_matches_configuration', manifest['expected'] == want)
        listed = {r['name']: r for r in manifest['actual']['rails']}
        bad = [r['name'] for r in have['rails'] if r['name'] not in listed or r['sha256'] != listed[r['name']]['sha256']
               or not _close(r['pose'], listed[r['name']]['pose']) or not _close(r['size'], listed[r['name']]['size'])]
        record('rail_files_match_manifest', not bad and len(listed) == len(have['rails']), mismatched=bad)
    def boxes_match(a, b):
        return (a['name'] == b['name'] and a['shape'] == b['shape'] and
                all(_close(a[k], b[k]) for k in ('model_pose', 'link_pose', 'pose', 'size')))
    def box_set_matches(a, b):
        return len(a) == len(b) and all(boxes_match(x, y) for x, y in zip(a, b))
    record('rail_boxes_match_configuration', box_set_matches(have['rail_boxes'], want['rail_boxes']),
           actual=have['rail_boxes'], expected=want['rail_boxes'])
    record('rail_boxes_match_manifest', manifest is not None and
           box_set_matches(have['rail_boxes'], manifest['actual'].get('rail_boxes', [])))
    # Rails: complete set, both rails, coverage, placement, orientation, size, samples.
    names_want = {r['name'] for r in want['rails']}; names_have = {r['name'] for r in have['rails']}
    record('rail_models_complete', names_want == names_have, missing=sorted(names_want-names_have),
           unexpected=sorted(names_have-names_want))
    by_name = {r['name']: r for r in have['rails']}
    placement = []
    for r in want['rails']:
        h = by_name.get(r['name'])
        if h is None: continue
        x, y, z, roll, pitch, yaw = h['pose']
        problems = []
        if abs(x-sum(r['x_m'])/2) > 1e-9: problems.append('x')
        if abs(y-r['y']) > 1e-9: problems.append('lateral position')
        if max(abs(roll), abs(pitch), abs(yaw)) > 1e-12: problems.append('orientation')
        if abs(h['size'][0]-r['length']) > 1e-9 or abs(h['size'][1]-r['width']) > 1e-9: problems.append('size')
        if h['samples'] != [want['segment_samples']]*2: problems.append('samples')
        if problems: placement.append(dict(name=r['name'], problems=problems))
    record('rail_placement', not placement, problems=placement)
    coverage = {}
    for side in ('left', 'right'):
        spans = sorted([(h['pose'][0]-h['size'][0]/2*(1-1/want['segment_samples']),
                         h['pose'][0]+h['size'][0]/2*(1-1/want['segment_samples'])) for h in have['rails']
                        if h['name'].startswith(f'rail_surface_{side}_')])
        ok = bool(spans) == bool(want['rails'])
        if spans:
            ok = ok and spans[0][0] <= want['x_range_m'][0]+1e-9 and spans[-1][1] >= want['x_range_m'][1]-1e-9
            ok = ok and all(b0 <= b1+1e-12 and a1 <= b0+1e-12 for (a0, b0), (a1, b1) in zip(spans, spans[1:]))
        coverage[side] = dict(passed=ok, spans=len(spans))
    record('rail_coverage', all(v['passed'] for v in coverage.values()), rails=coverage)
    # Decoded collision surface against the profile regenerated from the configuration.
    if decode and want['rails']:
        x, left, right, _ = rail_profiles(config)
        segs, surfaces = [], {}
        for h in have['rails']:
            rows = np.asarray(Image.open(h['path']), dtype=float)
            n = rows.shape[1]; extent = h['size'][0]*(n-1)/n
            xs = np.linspace(h['pose'][0]-extent/2, h['pose'][0]+extent/2, n)
            # Decoded as gz loads it: the image's own min..max pixel spans the size height.
            full_range = rows.min() == 0 and rows.max() == 65535
            zs = h['pose'][2]+(rows[0]-rows.min())/max(np.ptp(rows), 1)*h['size'][2]
            profile = left if '_left_' in h['name'] else right
            error = float(np.abs(zs-np.interp(xs, x, profile)).max())
            tolerance = h['size'][2]/65535/2+1e-9
            segs.append(dict(name=h['name'], max_error_m=error, tolerance_m=tolerance, passed=error <= tolerance,
                             uniform_rows=bool(np.ptp(rows, axis=0).max() == 0), full_range=bool(full_range)))
            surfaces[h['name']] = (xs, zs, tolerance)
        good = lambda s: s['passed'] and s['uniform_rows'] and s['full_range']
        record('rail_profile_matches_configuration', bool(segs) and all(good(s) for s in segs),
               segments=[s for s in segs if not good(s)], count=len(segs),
               worst_error_m=max((s['max_error_m'] for s in segs), default=None))
        overlaps = []
        for side in ('left', 'right'):
            names = sorted(n for n in surfaces if n.startswith(f'rail_surface_{side}_'))
            for n0, n1 in zip(names, names[1:]):
                (x0, z0, q0), (x1, z1, q1) = surfaces[n0], surfaces[n1]
                lo, hi = max(x0[0], x1[0]), min(x0[-1], x1[-1])
                if hi <= lo: continue
                grid = np.linspace(lo, hi, 2001)
                difference = float(np.abs(np.interp(grid, x0, z0)-np.interp(grid, x1, z1)).max())
                overlaps.append(dict(pair=[n0, n1], difference_m=difference, limit_m=q0+q1))
        worst = max(overlaps, key=lambda o: o['difference_m']/o['limit_m'], default=None)
        record('rail_overlap_surfaces', all(o['difference_m'] <= o['limit_m'] for o in overlaps),
               max_difference_m=max((o['difference_m'] for o in overlaps), default=0.), worst=worst)
    # Vehicle: wheel radii and spring joints exactly as the truth requires.
    wheel_bad = [n for n, w in want['wheels'].items()
                 if n not in have['wheels'] or have['wheels'][n]['shape'] != w['shape']
                 or abs(have['wheels'][n]['radius']-w['radius']) > 1e-12]
    record('wheel_radii_match_truth', not wheel_bad, mismatched=wheel_bad)
    joint_bad = [n for n, j in want['joints'].items()
                 if n not in have['joints'] or any(abs(have['joints'][n][k]-v) > 1e-9*max(1., abs(v)) for k, v in j.items())]
    extra = [n for n in have['joints'] if n not in want['joints']]
    record('springs_match_configuration', not joint_bad and not extra, mismatched=joint_bad, unexpected=extra)
    def guides_match(a,b):
        return (set(a)==set(b) and all(a[n]['shape']==b[n]['shape'] and _close(a[n]['pose'],b[n]['pose']) and
            abs(a[n]['radius']-b[n]['radius'])<1e-9 and abs(a[n]['length']-b[n]['length'])<1e-9 for n in a))
    record('guide_bearings_match_configuration',guides_match(have['guide_bearings'],want['guide_bearings']))
    listed_guides=manifest['actual'].get('guide_bearings') if manifest is not None else None
    if listed_guides is not None:
        record('guide_bearings_match_manifest',guides_match(have['guide_bearings'],listed_guides))
    from .robot_geometry import assembly_pose, mount_geometry
    base_z, _ = mount_geometry(config)
    assembly = dict(base_pose=[0.,0.,base_z,0.,0.,0.], head_pose=assembly_pose(config),
                    axis=[-1.,0.,0.], axis_frame='')
    actual = have['scanner_assembly']
    assembly_ok = (actual is not None and actual['axis_frame']=='' and
                   all(_close(actual[k], assembly[k]) for k in ('base_pose','head_pose','axis')))
    record('scanner_assembly_matches_truth', assembly_ok, expected=assembly, actual=actual)
    # Legacy nominal manifests lack this optional entry; the direct configuration
    # comparison above is mandatory, including for those old bundles.
    listed = manifest['actual'].get('scanner_assembly') if manifest is not None else None
    if listed is not None:
        record('scanner_assembly_matches_manifest', actual == listed)
    return dict(passed=all(c['passed'] for c in checks.values()), checks=checks, world=str(world_path))


def spec_for(config_path):
    config_path = Path(config_path)
    local = config_path.parent/'spec.yaml'
    return yaml.safe_load((local if local.exists() else
                           share_file('config/stage_b_scene.yaml')).read_text())


def snapshot_inputs(root):
    """Strict archived inputs, independent of the original world/config directories."""
    snapshot = Path(root)/'evaluation/physical'
    if not snapshot.is_dir():
        raise FileNotFoundError(f'physical snapshot missing: {snapshot}; recapture legacy sessions')
    for name in (NAME, 'spec.yaml'):
        if not (snapshot/name).is_file():
            raise FileNotFoundError(f'missing archived physical input: {snapshot/name}')
    worlds = list(snapshot.glob('*.sdf'))
    if len(worlds) != 1:
        raise ValueError(f'physical snapshot requires exactly one SDF: {snapshot}')
    return snapshot, worlds[0], yaml.safe_load((snapshot/'spec.yaml').read_text())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=('check', 'write'))
    p.add_argument('--config', required=True); p.add_argument('--world', required=True)
    p.add_argument('--spec'); p.add_argument('--manifest'); p.add_argument('--snapshot')
    a = p.parse_args()
    config = yaml.safe_load(Path(a.config).read_text())
    spec = yaml.safe_load(Path(a.spec).read_text()) if a.spec else spec_for(a.config)
    if a.command == 'write':
        print(write_manifest(a.world, config, spec)); return 0
    if not config.get('contact', {}).get('enabled'):
        print(json.dumps(dict(passed=True, skipped='ideal mode has no rail/wheel contact')))
        return 0
    report = check(config, spec, a.world, a.manifest, a.snapshot)
    print(json.dumps(report, indent=2))
    if not report['passed']:
        failed = [n for n, c in report['checks'].items() if not c['passed']]
        print('physical world differs from the configuration: '+', '.join(failed), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
