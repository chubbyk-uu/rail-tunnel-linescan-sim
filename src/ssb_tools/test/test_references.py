import inspect
import math

import numpy as np

from ssb_tools import ref_geometry, ref_timing, unroll, validate_stage_a

POSE = np.dtype([('t', '<f8'), ('x', '<f8'), ('v', '<f8'), ('theta', '<f8'), ('omega', '<f8'),
                 ('wheel', '<f8'), ('wheel_omega', '<f8')])

CONFIG = {
    'camera': {'width': 4096, 'pixel_pitch_m': 7.04e-6, 'fov_at_nominal_m': 0.8, 'nominal_distance_m': 2.77,
               'exposure_s': 8e-6, 'trigger_delay_s': 0.0, 'max_line_rate_hz': 50000},
    'rescaler': {'multiply': 64, 'divide': 7, 'max_period_s': 0.01},
    'scan_encoder': {'ppr': 2500, 'edges_per_cycle': 4},
    'odometer': {'ppr': 2500, 'edges_per_cycle': 4, 'gear_ratio': 1.0},
    'gate': {'start_rad': -2 * math.pi / 3, 'end_rad': 2 * math.pi / 3},
}
TRUTH = {'start_theta_rad': math.pi, 'scan_encoder_zero_rad': 0.0, 'gate_start_offset_rad': 0.0,
         'gate_end_offset_rad': 0.0, 'head_mount_x_m': 0.0, 'tunnel': {'radius_m': 2.77, 'axis_z_m': 1.97},
         'mount': {'e_m': 0, 'tangential_m': 0, 'dy_m': 0, 'dz_m': 0, 'tilt_y_rad': 0, 'tilt_z_rad': 0,
                   'twist_rad': 0}}


def constant_speed(duration, rate=50000.0, dt=1e-3, phase=0.0):
    omega = 2 * math.pi * rate / (640000 / 7)
    t = np.arange(0, duration + 1e-12, dt)
    s = np.zeros(len(t), POSE)
    s['t'], s['theta'], s['omega'] = t, math.pi + phase + omega * t, omega
    s['x'], s['v'] = 0.6 * omega / (2 * math.pi) * t, 0.6 * omega / (2 * math.pi)
    s['wheel_omega'] = s['v'] / 0.1
    s['wheel'] = s['wheel_omega'] * t
    return s


def test_linear_crossings_are_exact():
    s = constant_speed(0.01, phase=1e-5)
    spacing = 2 * math.pi / 10000
    t, idx, d = ref_timing.lattice_crossings(s, 'theta', 'omega', math.pi, spacing)
    assert (d == 1).all() and (np.diff(idx) == 1).all()
    np.testing.assert_allclose(t, (idx * spacing - 1e-5) / s['omega'][0], atol=1e-13)


def test_constant_speed_rows_are_uniform_and_complete():
    ref = ref_timing.reference(constant_speed(2.2), CONFIG, TRUTH)
    rows = np.array(ref['rows'])
    seg = rows[rows[:, 1] == 1]
    assert np.allclose(np.diff(seg[:, 2]), 20e-6, atol=1e-11)
    assert np.all(np.diff(seg[:, 0]) == 1)
    assert math.floor(640000 / 7 * 2 / 3) <= len(seg) <= math.ceil(640000 / 7 * 2 / 3)
    assert not any(g for reason, g in ref['dropped'].values() if reason != ref_timing.STREAM_END)


def test_nominal_head_points_up_at_zero_and_sideways_at_ninety_degrees():
    origin, optical, line = ref_geometry.head_pose(np.array([0.0, math.pi / 2]), np.array([1.0, 1.0]), TRUTH)
    np.testing.assert_allclose(origin, [[1, 0, 1.97], [1, 0, 1.97]], atol=1e-15)
    np.testing.assert_allclose(optical, [[0, 0, 1], [0, 1, 0]], atol=1e-15)
    np.testing.assert_allclose(line, [[1, 0, 0], [1, 0, 0]], atol=1e-15)
    x, q = ref_geometry.wall_hits(origin, optical, line, np.array([0.0]), TRUTH)
    np.testing.assert_allclose(q[:, 0], [0, 2.77 * math.pi / 2], atol=1e-12)


def test_reconstruction_code_never_touches_truth():
    source = inspect.getsource(unroll)
    for forbidden in ('.evaluation(', '.truth(', 'row_truth', 'pose_stream', 'Session'):
        assert forbidden not in source


def test_accounting_catches_a_silently_missing_row():
    rows = np.zeros(4, [('row', '<i8'), ('segment', '<i8'), ('t_trigger', '<f8')])
    rows['row'], rows['segment'], rows['t_trigger'] = [10, 11, 13, 14], 1, [1, 2, 3, 4]
    drops = np.zeros(0, [('row', '<i8'), ('gated', '<i4')])
    assert validate_stage_a.accounting(rows, drops)['state'] == 'fail'
    drops = np.array([(12, 1)], [('row', '<i8'), ('gated', '<i4')])
    assert validate_stage_a.accounting(rows, drops)['state'] == 'pass'


def test_valid_region_separates_buffer_drops_from_missing_rows():
    poses = constant_speed(1.0)
    dt = np.dtype([('row', '<i8'), ('t_lo', '<f8'), ('t_hi', '<f8'), ('reason', '<i4'), ('gated', '<i4')])
    truth = np.zeros(3, [('x', '<f8')])
    truth['x'] = [0.0, 0.1, 0.2]
    cfg = {'acceptance': {'valid_x_m': [0.05, 0.15]}}
    speed = poses['v'][0]
    buffer_drop = np.array([(1, 0.01 / speed, 0.01 / speed, 2, 1)], dt)    # x = 0.01, outside
    inside_drop = np.array([(2, 0.1 / speed, 0.1 / speed, 1, 1)], dt)      # x = 0.10, inside
    ungated = np.array([(3, 0.1 / speed, 0.1 / speed, 1, 0)], dt)          # not an exposure
    assert validate_stage_a.valid_region(cfg, poses, None, truth, np.concatenate([buffer_drop, ungated]))['state'] == 'pass'
    result = validate_stage_a.valid_region(cfg, poses, None, truth, np.concatenate([buffer_drop, inside_drop]))
    assert result['state'] == 'fail' and result['missing_in_region'] == 1 and result['buffer_drops_outside_region'] == 1
    assert validate_stage_a.valid_region({}, poses, None, truth, buffer_drop)['state'] == 'unmeasurable'


def test_planned_motion_distinguishes_drained_from_finished():
    assert validate_stage_a.planned_motion({'motion': {'complete': False}})['state'] == 'fail'
    assert validate_stage_a.planned_motion({'motion': {'complete': True}})['state'] == 'pass'
    assert validate_stage_a.planned_motion({'motion': {'complete': None}})['state'] == 'unmeasurable'


def test_valid_region_needs_a_real_interval_with_exposures():
    poses = constant_speed(1.0)
    dt = np.dtype([('row', '<i8'), ('t_lo', '<f8'), ('t_hi', '<f8'), ('reason', '<i4'), ('gated', '<i4')])
    none = np.zeros(0, dt)
    truth = np.zeros(2, [('x', '<f8')])
    truth['x'] = [0.0, 0.3]  # spans the band but no row lies inside it
    band = {'acceptance': {'valid_x_m': [0.1, 0.2]}}
    assert validate_stage_a.valid_region(band, poses, None, truth, none)['state'] == 'unmeasurable'
    for bad in ([0.2, 0.1], [0.1, float('nan')], [0.1], 'x'):
        result = validate_stage_a.valid_region({'acceptance': {'valid_x_m': bad}}, poses, None, truth, none)
        assert result['state'] == 'fail', bad


def test_truth_record_must_derive_from_the_config_source():
    src = {'tunnel': {'radius_m': 2.77, 'axis_z_m': 1.97}, 'motion': {'start_theta_deg': 180.0},
           'truth': {'wheel_diameter_m': 0.2, 'scan_encoder_zero_deg': 0.0, 'gate_start_offset_deg': 0.0,
                     'gate_end_offset_deg': 0.0, 'head_mount_x_m': 0.0,
                     'mount': {k: 0.0 for k in ('e_m', 'tangential_m', 'dy_m', 'dz_m', 'tilt_y_rad',
                                                'tilt_z_rad', 'twist_rad')}}}
    truth = {'tunnel': {'radius_m': 2.77, 'axis_z_m': 1.97}, 'start_theta_rad': math.pi, 'wheel_diameter_m': 0.2,
             'scan_encoder_zero_rad': 0.0, 'gate_start_offset_rad': 0.0, 'gate_end_offset_rad': 0.0,
             'head_mount_x_m': 0.0, 'mount': dict(src['truth']['mount']), 'config_sha256': 'abc'}
    expected = validate_stage_a.truth_from_source(src, 'abc')
    assert validate_stage_a.field_mismatches(truth, expected) == []
    truth['wheel_diameter_m'] = 0.3
    assert validate_stage_a.field_mismatches(truth, expected) == ['wheel_diameter_m']
    del truth['mount']['e_m']
    assert 'mount.e_m' in validate_stage_a.field_mismatches(truth, expected)
