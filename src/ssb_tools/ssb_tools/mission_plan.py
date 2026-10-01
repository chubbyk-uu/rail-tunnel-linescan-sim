"""Private generation inputs for straight-rail missions; no reconstruction input."""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def plan(config, start, distance):
    if not all(math.isfinite(x) for x in (start, distance)) or distance <= 0:
        raise ValueError('起点和距离必须为有限值，距离必须大于零')
    if distance < .05:
        raise ValueError('当前采集任务距离至少为 0.05 m，以留出初始非采集角的行程')
    t = config['tunnel']
    fov = config['camera']['fov_at_nominal_m']
    # Effective inspection domain excludes the 1.5 m construction buffers.
    if start < 0 or start + distance > 20 + 1e-9:
        raise ValueError('当前任务范围必须在 0–20 m 内')
    clearance = max(.65, fov / 2)
    if start-clearance < t['x_min_m'] or start+distance+clearance > t['x_max_m']:
        raise ValueError('轨道或相机视场余量不足')
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
    c['acceptance']['valid_x_m'] = [start, start+distance]
    pitch = m['advance_per_rev_m']
    return c, dict(start_m=start, distance_m=distance, end_m=start+distance,
                   speed_m_s=speed, duration_s=profile[-1][0],
                   image_extent_m=[start-fov/2, start+distance+fov/2],
                   conservative_full_angle_m=([start+pitch, start+distance-pitch]
                                              if distance > 2*pitch else None),
                   coverage_note='保守估计，完整内壁覆盖须采集后按行几何核验')


def prepare(template, output, start, distance):
    template, output = Path(template).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('任务输入目录已存在')
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
    return c, task
