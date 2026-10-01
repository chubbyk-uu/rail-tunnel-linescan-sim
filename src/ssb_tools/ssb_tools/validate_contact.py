"""Independent straight-track contact/encoder/scan checks from archived diagnostics.

Travel, slip and reversal are evaluated at the rear (encoder) axle centres, not at the base
origin 0.3 m above the rail: on an irregular track, body pitch alone moves that origin
fore/aft by millimetres. Vertical irregularity and wheel compliance are simulation truth from
the capture configuration; the body is compared with a rigid-disc quasi-static reference.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .robot_geometry import mount_geometry

STARTUP_S = 1.5   # settle pre-stress from placing a flat-posed vehicle on the track releases here


def speed_factor(profile, t):
    """Commanded quintic smootherstep speed factor (same knots as the capture config)."""
    k = np.asarray(profile, float); t = np.asarray(t, float); out = np.full(t.shape, k[-1, 1])
    for (ta, fa), (tb, fb) in zip(k[:-1], k[1:]):
        inside = (t >= ta) & (t < tb); u = (t[inside]-ta)/(tb-ta)
        out[inside] = fa+(fb-fa)*u**3*(10+u*(-15+6*u))
    out[t < k[0, 0]] = 0.
    return out


def disc_centre(x, z, centre, radius):
    """Height of a rigid disc resting on profile z(x), relative to its nominal height."""
    u = np.linspace(-.03, .03, 121)
    return np.max(np.interp(centre[:, None]+u, x, z)+np.sqrt(radius*radius-u*u), axis=1)-radius


def rail_profile_matches_world(config, world):
    """Decode every rail-top heightmap of the world and compare it with the regenerated truth."""
    from .rail_irregularity import decode_heightmap, profile
    import xml.etree.ElementTree as ET
    expected = profile(config)
    uris = [e.text for e in ET.parse(world).getroot().iter('uri')
            if e.text and Path(e.text).name.startswith('rail_top_')]
    if expected is None:
        return dict(passed=not uris, heightmaps=len(uris))
    x, z, _ = expected
    record = json.loads((Path(uris[0]).parent/'rail_irregularity.json').read_text())
    worst = 0.
    for entry in record['heightmaps']:
        xs, zs = decode_heightmap(Path(uris[0]).parent/entry['file'], entry)
        worst = max(worst, float(np.abs(zs-np.interp(xs, x, z)).max()))
    tolerance = max(e['quantization_m'] for e in record['heightmaps'])+1e-9
    return dict(passed=bool(uris) and worst <= tolerance, heightmaps=len(uris),
                max_error_m=worst, tolerance_m=tolerance)


def validate(root, config, spec=None, world=None):
    root = Path(root); config = Path(config); c = yaml.safe_load(config.read_text())
    spec_path = Path(spec) if spec else (config.parent/'spec.yaml' if (config.parent/'spec.yaml').exists()
                                          else Path(__file__).resolve().parents[1]/'config/stage_b_scene.yaml')
    robot = yaml.safe_load(spec_path.read_text())['robot']
    a = np.genfromtxt(root/'evaluation/contact.csv', delimiter=',', names=True)
    settle = a[(a['t'] >= -.5) & (a['t'] < 0)]; a = a[a['t'] >= 0]
    b = np.genfromtxt(root/'metadata/encoders.csv', delimiter=',', names=True)
    summary = json.loads((root/'summary.json').read_text())
    count = c['odometer']['ppr']*c['odometer']['edges_per_cycle']*c['odometer']['gear_ratio']
    dl = c['calibration'].get('odo_left_diameter_m', c['calibration']['wheel_diameter_m'])
    dr = c['calibration'].get('odo_right_diameter_m', c['calibration']['wheel_diameter_m'])
    tl = c['truth'].get('odo_left_diameter_m', c['truth']['wheel_diameter_m'])
    tr = c['truth'].get('odo_right_diameter_m', c['truth']['wheel_diameter_m'])
    base_z, _ = mount_geometry(c)
    half = robot['wheelbase_m']/2
    rail_y = (yaml.safe_load(spec_path.read_text())['track']['gauge_m']+yaml.safe_load(spec_path.read_text())['track']['head_width_m'])/2
    estimated = np.pi/count*(b['count_left']*dl+b['count_right']*dr)/2
    target = np.deg2rad(c['motion']['start_theta_deg'])+2*np.pi*estimated/c['motion']['advance_per_rev_m']
    # Rear encoder axle centres from the true body pose.
    rotation = Rotation.from_euler('ZYX', np.column_stack([a['yaw'], a['pitch'], a['roll']]))
    position = np.column_stack([a['x'], a['y'], a['z']])
    axles = {}
    for side, sign, diameter, rate in (('left', 1, tl, a['left_rate']), ('right', -1, tr, a['right_rate'])):
        centre = position+rotation.apply([-half, sign*rail_y, diameter/2-base_z])
        vx = np.gradient(centre[:, 0], a['t'])
        axles[side] = dict(x=centre[:, 0], vx=vx, slip=np.abs(vx-rate*diameter/2))
    axle_x = (axles['left']['x']+axles['right']['x'])/2
    travel = axle_x[-1]-axle_x[0]; turns = (a['scan'][-1]-a['scan'][0])/(2*np.pi)
    pitch = travel/turns if turns > 0 else None
    expected_pitch = c['motion']['advance_per_rev_m']/((dl/tl+dr/tr)/2)
    slip = np.maximum(axles['left']['slip'], axles['right']['slip'])
    vx = np.minimum(axles['left']['vx'], axles['right']['vx'])
    commanded = speed_factor(c['motion']['profile'], a['t'])
    reverse = float(-np.clip(vx, None, 0).sum()*np.median(np.diff(a['t'])))
    compliance = c['truth'].get('wheel_compliance')
    deflection = compliance['static_deflection_m'] if compliance else 0.
    after = a['t'] >= STARTUP_S
    checks = dict(complete=bool(summary['complete']), positive_travel=bool(travel > 1),
        no_reverse=bool(reverse < 5e-4 and not (vx[commanded > .05] < -1e-4).any()),
        lateral_guidance=bool(abs(a['y']).max() < .001),
        attitude=bool(max(abs(a['roll']).max(), abs(a['pitch']).max()) < .02),
        small_longitudinal_slip=bool(np.percentile(slip, 99) < .001 and slip.max() < .005),
        encoder_distance=bool(np.max(abs(estimated-b['s_hat'])) < 1e-10),
        encoder_scan_target=bool(np.max(abs(target-b['theta_target'])) < 1e-9),
        scan_tracking=bool(np.max(abs(b['theta_target']-b['scan'])) < .012),
        diameter_error_pitch=bool(pitch and abs(pitch/expected_pitch-1) < .001))
    report = dict(travel_m=float(travel), estimated_m=float(estimated[-1]),
        encoder_minus_travel_m=float(estimated[-1]-travel), measured_pitch_m=pitch,
        expected_pitch_m=expected_pitch, slip_p99_m_s=float(np.percentile(slip, 99)),
        max_slip_speed_m_s=float(slip.max()), reverse_displacement_m=reverse,
        max_lateral_m=float(abs(a['y']).max()), max_scan_error_rad=float(abs(b['theta_target']-b['scan']).max()))
    from .rail_irregularity import profile
    truth = profile(c)
    if truth is None:
        heave_ref = np.zeros(len(a)); pitch_ref = np.zeros(len(a))
    else:
        x, z, record = truth
        front = disc_centre(x, z, a['x']+half, tl/2); rear = disc_centre(x, z, a['x']-half, tl/2)
        heave_ref = (front+rear)/2; pitch_ref = (rear-front)/(2*half)
        report['track_irregularity'] = record['metrics']
    heave_error = a['z']-(base_z+heave_ref-deflection)
    pitch_error = a['pitch']-pitch_ref
    checks['supported'] = bool(abs(heave_error[after]).max() < (3e-4 if truth else 3e-3))
    report.update(heave_error_max_m=float(abs(heave_error[after]).max()),
                  pitch_error_max_rad=float(abs(pitch_error[after]-pitch_error[after].mean()).max()),
                  heave_range_m=float(np.ptp(a['z'][after])), pitch_range_rad=float(np.ptp(a['pitch'][after])))
    if truth is not None:
        checks['body_follows_track'] = bool(report['pitch_error_max_rad'] < 1e-3)
    names = ('suspension_rear_left', 'suspension_front_left', 'suspension_rear_right', 'suspension_front_right')
    if compliance:
        if not all(n in a.dtype.names for n in names):
            checks['wheel_compliance_logged'] = False
        else:
            loads = np.column_stack([a[n] for n in names])/deflection
            static = np.column_stack([settle[n] for n in names]).mean()
            checks['wheels_always_loaded'] = bool(loads.min() > 0)
            checks['static_deflection'] = bool(abs(static/deflection-1) < .02)
            report.update(static_deflection_m=float(static), load_ratio_range=[float(loads.min()), float(loads.max())])
    if world:
        match = rail_profile_matches_world(c, world)
        checks['rail_profile_matches_truth'] = match['passed']; report['rail_profile'] = match
    report = dict(passed=all(checks.values()), checks=checks, **report,
        limitation='straight-track check with common-mode vertical irregularity; not a derailment, curve or twist certification')
    (root/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('diagnostics')
    p.add_argument('--config', required=True); p.add_argument('--spec'); p.add_argument('--world')
    a = p.parse_args(); r = validate(a.diagnostics, a.config, a.spec, a.world)
    print(json.dumps(r, indent=2)); return 0 if r['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
