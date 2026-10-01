"""Private generation inputs for straight-rail missions; no reconstruction input."""
import copy
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


def plan(config, start, distance):
    start, distance = start_values(dict(start_m=start, distance_m=distance))
    if not all(math.isfinite(x) for x in (start, distance)) or distance <= 0:
        raise ValueError('Start and distance must be finite; distance must be positive')
    if distance < 1.:
        raise ValueError('Minimum capture distance is 1 m')
    t = config['tunnel']
    fov = config['camera']['fov_at_nominal_m']
    # Effective inspection domain excludes the 1.5 m construction buffers.
    if start < 0 or start + distance > 20 + 1e-9:
        raise ValueError('Mission must stay within the 0–20 m range')
    clearance = max(.65, fov / 2)
    if start-clearance < t['x_min_m'] or start+distance+clearance > t['x_max_m']:
        raise ValueError('Insufficient track or camera field-of-view margin')
    c = copy.deepcopy(config)
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


def prepare(template, output, start, distance):
    template, output = Path(template).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('Mission input directory already exists')
    source = template/'capture.yaml'
    c, task = plan(yaml.safe_load(source.read_text()), start, distance)
    scene = Path(c['render']['optical_scene'])
    c['render']['optical_scene'] = str((source.parent/scene).resolve())
    world = ET.parse(template/'world/world.sdf')
    car = world.getroot().find("world/model[@name='scan_car']")
    pose = list(map(float, car.findtext('pose', '0 0 0 0 0 0').split()))
    pose[0] = start
    car.find('pose').text = ' '.join(map(str, pose))
    # SDF assets may be relative to the template world directory.
    for element in world.getroot().iter():
        if element.tag in ('uri', 'albedo_map', 'normal_map', 'roughness_map') and element.text:
            value = element.text.strip()
            if '://' not in value:
                element.text = str((template/'world'/value).resolve())
    output.mkdir(parents=True)
    (output/'capture.yaml').write_text(yaml.safe_dump(c, sort_keys=False))
    world.write(output/'world.sdf', encoding='unicode', xml_declaration=True)
    # Physical-world manifest and scene spec travel with the world (car pose is not in them).
    for name, source_path in (('physical_manifest.json', template/'world/physical_manifest.json'),
                              ('spec.yaml', template/'spec.yaml')):
        if source_path.exists(): (output/name).write_bytes(source_path.read_bytes())
    return c, task
