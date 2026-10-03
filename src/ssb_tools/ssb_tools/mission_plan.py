"""Private generation inputs for straight-rail missions; no reconstruction input."""
import copy
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def start_values(command):
    """Reject malformed external inputs before they can change a task's state."""
    try:
        values = [command[key] for key in ('start_m', 'distance_m')]
        if any(isinstance(value, bool) for value in values):
            raise ValueError('Boolean is not a distance')
        start, distance = map(float, values)
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        raise ValueError('Start and distance must be finite numbers') from error
    if not all(math.isfinite(value) for value in (start, distance)):
        raise ValueError('Start and distance must be finite numbers')
    return start, distance


def mission_limits(config):
    """Declared inspection/vehicle envelope, never inferred from captured truth."""
    try:
        m=config['mission']; lo,hi=map(float,m['inspection_x_m'])
        half=float(m['vehicle_half_length_m']); margin=float(m['safety_margin_m'])
        minimum=float(m['minimum_distance_m'])
        build_lo=float(config['tunnel']['x_min_m']); build_hi=float(config['tunnel']['x_max_m'])
        fov=float(config['camera']['fov_at_nominal_m'])
        mount=float(config.get('calibration',{}).get('head_mount_x_m',0.))
    except (KeyError,TypeError,ValueError) as error:
        raise ValueError('Explicit mission inspection bounds and vehicle envelope are required') from error
    if not all(math.isfinite(v) for v in (lo,hi,half,margin,minimum,build_lo,build_hi,fov,mount)):
        raise ValueError('Mission limits must be finite')
    if not (build_lo<=lo<hi<=build_hi and half>0 and margin>=0 and minimum>=1 and hi-lo>=minimum and fov>0):
        raise ValueError('Invalid inspection domain, vehicle envelope or minimum distance')
    return dict(inspection_x_m=[lo,hi], construction_x_m=[build_lo,build_hi],
                minimum_distance_m=minimum, vehicle_half_length_m=half, safety_margin_m=margin,
                clearance_m=max(half,fov/2+abs(mount))+margin)


def _travel_plan(config, start, distance, inspection_domain=True):
    start, distance = start_values(dict(start_m=start, distance_m=distance))
    if not all(math.isfinite(x) for x in (start, distance)) or distance <= 0:
        raise ValueError('Start and distance must be finite; distance must be positive')
    limits=mission_limits(config)
    if distance < limits['minimum_distance_m']:
        raise ValueError(f"Minimum capture distance is {limits['minimum_distance_m']:g} m")
    t = config['tunnel']
    fov = config['camera']['fov_at_nominal_m']
    lo,hi=limits['inspection_x_m']
    if inspection_domain and (start < lo or start + distance > hi + 1e-9):
        raise ValueError(f'Mission must stay within the {lo:g}–{hi:g} m range')
    clearance=limits['clearance_m']
    if start-clearance < t['x_min_m'] or start+distance+clearance > t['x_max_m']:
        raise ValueError('Insufficient track or camera field-of-view margin')
    c = copy.deepcopy(config)
    c.pop('inspection', None)
    m = c['motion']
    e, rescaler = c['scan_encoder'], c['rescaler']
    counts = e['ppr'] * e['edges_per_cycle'] * rescaler['multiply'] / rescaler['divide']
    speed = m['advance_per_rev_m'] * m['line_rate_hz'] / counts
    ramp = min(1., distance / speed)
    cruise = max(0., distance / speed - ramp)
    m['start_x_m'] = start
    profile = [[0., 0.], [ramp, 1.]]
    if cruise > 1e-9:
        profile.append([ramp+cruise, 1.])
    profile.extend([[2*ramp+cruise, 0.], [2*ramp+cruise+1., 0.]])
    m['profile'] = profile
    pitch = m['advance_per_rev_m']
    gate = c['gate']
    open_deg = (gate['end_deg']-gate['start_deg']) % 360.
    initial_phase = (m['start_theta_deg']-gate['start_deg']) % 360.
    entry_deg = 0. if initial_phase < open_deg else 360.-initial_phase
    ramp_distance = speed*ramp/2
    # Predetermined nominal head-x interval, not a crop inferred from recorded rows.
    # A full bottom arc bounds the unexposed tail for any requested stop phase.
    # 10 mm covers nominal servo/encoder lag and avoids testing exact gate boundaries.
    guard = .01
    valid = [start+max(pitch*entry_deg/360., ramp_distance)+guard,
             start+distance-max(pitch*(1.-open_deg/360.), ramp_distance)-guard]
    if valid[0] >= valid[1]:
        raise ValueError('Travel is too short for a nonempty exposure acceptance interval')
    c['acceptance']['valid_x_m'] = valid
    return c, dict(start_m=start, distance_m=distance, end_m=start+distance,
                   speed_m_s=speed, duration_s=profile[-1][0],
                   exposure_acceptance_x_m=valid,
                   image_extent_m=[start-fov/2, start+distance+fov/2],
                   conservative_full_angle_m=([start+pitch, start+distance-pitch]
                                              if distance > 2*pitch else None),
                   coverage_note='Conservative estimate; verify full-angle coverage from captured row geometry')


def plan(config, start, distance):
    """Legacy vehicle-travel task; never claims a complete target wall interval."""
    return _travel_plan(config, start, distance)


def wall_plan(config, start, length, calibration, guard=.01, grid_pitch=.0002):
    """Plan a wall target using nominal pitch, calibrated usable FOV and ramp margins.

    Cover the target at every scan phase, and keep any exposure whose calibrated
    FOV intersects it outside the ramps. Pixel coverage alone permits the last
    overlapping band to decelerate while still seeing the target; the resulting
    unequal encoder-row spacing weakens double-band seam support.
    """
    from .wall_coverage import calibrated_spans, calibrated_row_footprint
    start, length = start_values(dict(start_m=start, distance_m=length))
    limits=mission_limits(config);lo,hi=limits['inspection_x_m']
    if length < limits['minimum_distance_m'] or start < lo or start+length > hi+1e-9:
        raise ValueError(f'Wall target must be at least {limits["minimum_distance_m"]:g} m within {lo:g}–{hi:g} m')
    if not all(math.isfinite(x) and x > 0 for x in (guard, grid_pitch)):
        raise ValueError('Wall planning guard and grid pitch must be positive')
    spans = calibrated_spans(config, calibration)
    left, right = max(spans, key=lambda span: span[1]-span[0])
    pitch = config['motion']['advance_per_rev_m']
    if right-left <= pitch+2*guard:
        raise ValueError('Calibrated usable FOV is too narrow for the nominal helical pitch and guard')
    e, r = config['scan_encoder'], config['rescaler']
    rows_per_rev = e['ppr']*e['edges_per_cycle']*r['multiply']/r['divide']
    row_step = 2*math.pi/rows_per_rev
    footprint = calibrated_row_footprint(config, calibration)
    if footprint < row_step:
        raise ValueError('Nominal perpendicular pixel footprint is narrower than the row sampling step')
    speed = pitch*config['motion']['line_rate_hz']/rows_per_rev
    # At most one second ramps, so this upper bound also works for short/faster tasks.
    ramp_margin = speed/2
    mount = config['calibration']['head_mount_x_m']
    vehicle_start = min(start-pitch-left, start-right)-mount-ramp_margin-guard
    vehicle_end = max(start+length+pitch-right, start+length-left)-mount+ramp_margin+guard
    c, task = _travel_plan(config, vehicle_start, vehicle_end-vehicle_start, inspection_domain=False)
    gate = config['gate']
    # The requested wall stays fixed in tunnel coordinates even when the head
    # captures a wider angular guard for body roll/pitch. Never shrink the target.
    theta_start = math.radians(-120.)
    arc = 240.
    capture_arc = (gate['end_deg']-gate['start_deg']) % 360.
    offset = (-120.-gate['start_deg']) % 360.
    if not (240. <= capture_arc <= 260. and offset+arc <= capture_arc+1e-9):
        raise ValueError('Acquisition gate must contain the fixed upper 240 degree wall target (at most 260 degrees)')
    c['inspection'] = dict(schema='ssb.wall_target.v1', target_x_m=[start, start+length],
                           theta_rad=[theta_start, theta_start+math.radians(arc)],
                           grid_pitch_m=grid_pitch, guard_m=guard)
    task.update(mode='wall', target_x_m=[start, start+length], target_length_m=length,
                capture_gate_deg=[gate['start_deg'], gate['end_deg']], output_arc_deg=arc,
                nominal_usable_span_m=[left, right], ramp_margin_m=ramp_margin, guard_m=guard,
                ramp_policy='all nominal target-intersecting calibrated exposures inside cruise',
                nominal_row_step_m=row_step*config['calibration']['radius_m'],
                minimum_row_footprint_m=footprint*config['calibration']['radius_m'],
                coverage_note='Nominal full-angle target; acceptance uses recorded encoders and measured calibration. '
                              'Actual body motion and wheel scale errors are not known to this planner.')
    return c, task


def prepare(template, output, start, distance, mode='travel'):
    template, output = Path(template).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('Mission input directory already exists')
    source = template/'capture.yaml'
    if mode not in ('travel', 'wall'): raise ValueError('Unknown task mode')
    if mode == 'wall':
        from .optical_identity import check_calibration
        check_calibration(source, template/'calibration.json')
    config = yaml.safe_load(source.read_text())
    c, task = (wall_plan(config, start, distance, json.loads((template/'calibration.json').read_text()))
               if mode == 'wall' else plan(config, start, distance))
    scene = Path(c['render']['optical_scene'])
    c['render']['optical_scene'] = str((source.parent/scene).resolve())
    world = ET.parse(template/'world/world.sdf')
    car = world.getroot().find("world/model[@name='scan_car']")
    pose = list(map(float, car.findtext('pose', '0 0 0 0 0 0').split()))
    pose[0] = task['start_m']
    car.find('pose').text = ' '.join(map(str, pose))
    # SDF assets may be relative to the template world directory.
    for element in world.getroot().iter():
        if element.tag in ('uri', 'albedo_map', 'normal_map', 'roughness_map') and element.text:
            value = element.text.strip()
            if '://' not in value:
                element.text = str((template/'world'/value).resolve())
    output.mkdir(parents=True)
    (output/'capture.yaml').write_text(yaml.safe_dump(c, sort_keys=False))
    (output/'task.json').write_text(json.dumps(task, indent=2)+'\n')
    world.write(output/'world.sdf', encoding='unicode', xml_declaration=True)
    # Physical-world manifest and scene spec travel with the world (car pose is not in them).
    for name, source_path in (('physical_manifest.json', template/'world/physical_manifest.json'),
                              ('spec.yaml', template/'spec.yaml')):
        if source_path.exists(): (output/name).write_bytes(source_path.read_bytes())
    return c, task


def main():
    parser = argparse.ArgumentParser(description='Prepare a full-angle wall target with nominal overscan buffers.')
    parser.add_argument('--demo', required=True); parser.add_argument('--output', required=True)
    parser.add_argument('--target-start-m', type=float, required=True)
    parser.add_argument('--target-length-m', type=float, required=True)
    args = parser.parse_args()
    _, task = prepare(args.demo, args.output, args.target_start_m, args.target_length_m, mode='wall')
    print(json.dumps(task, indent=2))


if __name__ == '__main__': main()
