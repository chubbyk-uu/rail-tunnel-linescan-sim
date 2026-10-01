"""Independent straight-track contact/encoder/scan checks from archived diagnostics.

Travel, slip and reversal are evaluated at the centres of the two spring-loaded measuring
wheels that carry the encoders (body pose plus logged slide position), not at the base origin
0.3 m above the rail: on an irregular track, body pitch alone moves that origin fore/aft by
millimetres. Each measuring-wheel centre must stay on its rail (rigid disc on the rail profile). Rail irregularity and wheel compliance are simulation truth from the
capture configuration; the body is compared with a quasi-static reference: a rigid disc per wheel
on its own rail, and the body on the least-squares plane of the four wheels (equal springs).
The residual twist e = (rl - fl - rr + fr)/4 is taken by the springs as +/-e; where |e| exceeds
the static deflection a wheel unloads and the chassis rocks on a diagonal (physical, reported).
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


def physical_world_report(root, config_path, config, spec, world=None):
    """Strict physical-world check (ssb_tools.physical_world) on the run's archived snapshot when
    present (evaluation/physical), else on the given world beside its manifest."""
    from .physical_world import check, snapshot_inputs
    snapshot = Path(root)/'evaluation/physical'
    if snapshot.exists():
        snapshot, archived_world, archived_spec = snapshot_inputs(root)
        return check(config, archived_spec, archived_world, snapshot/'physical_manifest.json', snapshot=snapshot)
    return check(config, spec, world) if world else None


def validate(root, config, spec=None, world=None):
    root = Path(root); config = Path(config); c = yaml.safe_load(config.read_text())
    spec_path = Path(spec) if spec else (config.parent/'spec.yaml' if (config.parent/'spec.yaml').exists()
                                          else Path(__file__).resolve().parents[1]/'config/stage_b_scene.yaml')
    from .physical_world import snapshot_inputs
    physical_spec = (snapshot_inputs(root)[2] if (root/'evaluation/physical').exists()
                     else yaml.safe_load(spec_path.read_text()))
    robot = physical_spec['robot']; measure = robot['measuring_wheel']
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
    rail_y = (physical_spec['track']['gauge_m']+physical_spec['track']['head_width_m'])/2
    estimated = np.pi/count*(b['count_left']*dl+b['count_right']*dr)/2
    target = np.deg2rad(c['motion']['start_theta_deg'])+2*np.pi*estimated/c['motion']['advance_per_rev_m']
    # Measuring-wheel centres from the true body pose and the slide displacement (body z).
    rotation = Rotation.from_euler('ZYX', np.column_stack([a['yaw'], a['pitch'], a['roll']]))
    position = np.column_stack([a['x'], a['y'], a['z']])
    axles = {}
    for side, sign, diameter, rate in (('left', 1, tl, a['left_rate']), ('right', -1, tr, a['right_rate'])):
        slide = a[f'measure_slide_{side}']
        local = np.column_stack([np.full(len(a), measure['x_m']), np.full(len(a), sign*rail_y), diameter/2-base_z+slide])
        centre = position+rotation.apply(local)
        vx = np.gradient(centre[:, 0], a['t'])
        axles[side] = dict(x=centre[:, 0], z=centre[:, 2], vx=vx, slide=slide, slip=np.abs(vx-rate*diameter/2), radius=diameter/2)
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
    # Encoder wheel surface travel minus axle travel, at cruise and elsewhere (start/stop/parked).
    dt = np.gradient(a['t']); cruise = speed_factor(c['motion']['profile'], a['t']) >= .999
    for side, diameter, rate in (('left', tl, a['left_rate']), ('right', tr, a['right_rate'])):
        excess = (rate*diameter/2-axles[side]['vx'])*dt
        report[f'encoder_{side}_excess_m'] = dict(cruise=float(excess[cruise].sum()), other=float(excess[~cruise].sum()))
    from .rail_irregularity import rails
    truth = rails(c)
    differential = bool(truth and truth[3]['settings']['cross_level_tier_m'] > 0)
    twist_ref = np.zeros(len(a))
    rail_top = (lambda side, cx, r: np.zeros(len(cx))) if truth is None else \
        (lambda side, cx, r: disc_centre(truth[0], truth[1] if side == 'left' else truth[2], cx, r))
    on_rail = max(float(abs(axles[side]['z']-axles[side]['radius']-rail_top(side, axles[side]['x'], axles[side]['radius'])).max())
                  for side in ('left', 'right'))
    slide_margin = measure['travel_m']-max(float(abs(axles[side]['slide']).max()) for side in ('left', 'right'))
    checks['measuring_wheels_on_rail'] = bool(on_rail < 1e-4 and slide_margin > 1e-3)
    report.update(measuring_wheel_height_error_max_m=on_rail, measuring_slide_margin_m=slide_margin,
                  measuring_slide_range_m={side: [float(axles[side]['slide'].min()), float(axles[side]['slide'].max())]
                                           for side in ('left', 'right')})
    if truth is None:
        heave_ref = np.zeros(len(a)); pitch_ref = np.zeros(len(a)); roll_ref = np.zeros(len(a))
    else:
        x, left, right, record = truth
        running = c['truth']['wheel_diameter_m']/2     # load-carrying wheels, not the measuring wheels
        rl, fl = (disc_centre(x, left, a['x']+dx, running) for dx in (-half, half))
        rr, fr = (disc_centre(x, right, a['x']+dx, running) for dx in (-half, half))
        heave_ref = (rl+fl+rr+fr)/4; pitch_ref = (rl+rr-fl-fr)/(4*half); roll_ref = (rl+fl-rr-fr)/(4*rail_y)
        twist_ref = (rl-fl-rr+fr)/4
        report['track_irregularity'] = record['metrics']
    heave_error = a['z']-(base_z+heave_ref-deflection)
    pitch_error = a['pitch']-pitch_ref; roll_error = a['roll']-roll_ref
    names = ('suspension_rear_left', 'suspension_front_left', 'suspension_rear_right', 'suspension_front_right')
    logged = compliance and all(n in a.dtype.names for n in names)
    # The plane reference holds while all four wheels carry load; a lifted wheel leaves the
    # body rocking on a diagonal, which no quasi-static reference predicts.
    reference = after & (np.column_stack([a[n] for n in names]).min(axis=1) > 0 if differential and logged else True)
    centred = lambda e: float(abs(e[reference]-e[reference].mean()).max())
    checks['supported'] = bool(abs(heave_error[reference]).max() < (3e-4 if truth else 3e-3))
    report.update(reference_fraction=float(reference.sum()/after.sum()), heave_error_max_m=float(abs(heave_error[reference]).max()),
                  pitch_error_max_rad=centred(pitch_error), roll_error_max_rad=centred(roll_error),
                  heave_range_m=float(np.ptp(a['z'][after])), pitch_range_rad=float(np.ptp(a['pitch'][after])),
                  roll_range_rad=float(np.ptp(a['roll'][after])),
                  wheelbase_twist_max_m=float(4*abs(twist_ref[after]).max()))
    if truth is not None:
        checks['body_follows_track'] = bool(report['pitch_error_max_rad'] < 1e-3 and report['roll_error_max_rad'] < 1e-3)
    # Start-up: exposures begin as soon as the scan head enters the gate (about 1 s), before the
    # settle pre-stress has released at STARTUP_S. Reported separately, against the steady mean.
    gate = c.get('gate', {}); lo_g, hi_g = np.deg2rad(gate.get('start_deg', -180.)), np.deg2rad(gate.get('end_deg', 180.))
    wrapped = np.remainder(a['scan']+np.pi, 2*np.pi)-np.pi
    in_gate = np.where((a['t'] >= 0) & (wrapped >= lo_g) & (wrapped <= hi_g))[0]
    if len(in_gate):
        first = float(a['t'][in_gate[0]]); start = (a['t'] >= first) & ~after
        steady = lambda e: e[reference].mean()
        report['startup'] = dict(first_exposure_s=first, until_s=STARTUP_S, samples=int(start.sum()),
                                 heave_error_max_m=float(abs(heave_error[start]).max()) if start.any() else 0.,
                                 pitch_error_max_rad=float(abs(pitch_error[start]-steady(pitch_error)).max()) if start.any() else 0.,
                                 roll_error_max_rad=float(abs(roll_error[start]-steady(roll_error)).max()) if start.any() else 0.,
                                 note='settle pre-stress release; reported, not a pass/fail criterion')
    if compliance:
        if not logged:
            checks['wheel_compliance_logged'] = False
        else:
            loads = np.column_stack([a[n] for n in names])/deflection
            static = np.column_stack([settle[n] for n in names]).mean()
            unloaded = loads[after] <= 0
            if differential:
                # Twist larger than the static deflection unloads a wheel of a rigid chassis.
                predicted = abs(twist_ref[after]) >= deflection
                report.update(unloaded_fraction=dict(zip(names, unloaded.mean(axis=0).round(5).tolist())),
                              predicted_unloaded_fraction=float(predicted.mean()))
            else:
                checks['wheels_always_loaded'] = bool(not unloaded.any())
            checks['static_deflection'] = bool(abs(static/deflection-1) < .02)
            report.update(static_deflection_m=float(static), load_ratio_range=[float(loads.min()), float(loads.max())])
    physical = physical_world_report(root, config, c, physical_spec, world)
    if physical is not None:
        checks['physical_world_matches_truth'] = physical['passed']; report['physical_world'] = physical
    report = dict(passed=all(checks.values()), checks=checks, **report,
        limitation='straight-track quasi-static check of vertical and cross-level irregularity; not a derailment or curve certification')
    (root/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('diagnostics')
    p.add_argument('--config', required=True); p.add_argument('--spec'); p.add_argument('--world')
    a = p.parse_args(); r = validate(a.diagnostics, a.config, a.spec, a.world)
    print(json.dumps(r, indent=2)); return 0 if r['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
