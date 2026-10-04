import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from ssb_tools.mission_plan import wall_plan, plan
from ssb_tools.session import sha256_file
from ssb_tools.unroll import row_axis_coordinates
from ssb_tools.wall_coverage import (calibrated_spans, calibrated_row_footprint, target_grid,
                                     coverage_runs, inspect_session, RUN_DTYPE)


@pytest.fixture
def nominal():
    width = 4096; fov = .8522592711111112
    config = dict(camera=dict(width=width, fov_at_nominal_m=fov, nominal_distance_m=2.75,
                              optical_signature='measured-rig'),
                  tunnel=dict(x_min_m=-2., x_max_m=22.),
                  robot=dict(scan_axis_height_m=1.715),
                  mission=dict(inspection_x_m=[0.,20.],vehicle_half_length_m=.56,safety_margin_m=.09,minimum_distance_m=1.),
                  motion=dict(start_x_m=3., start_theta_deg=180., advance_per_rev_m=.6,
                              line_rate_hz=10000*128/15/3),
                  calibration=dict(radius_m=2.75, head_mount_x_m=0., wheel_diameter_m=.2,
                                   odo_left_diameter_m=.08, odo_right_diameter_m=.08),
                  scan_encoder=dict(ppr=2500, edges_per_cycle=4), rescaler=dict(multiply=128, divide=15),
                  odometer=dict(ppr=2500, edges_per_cycle=4, gear_ratio=1),
                  gate=dict(start_deg=-120., end_deg=120., start_rad=-2*math.pi/3, end_rad=2*math.pi/3),
                  acceptance={}, contact=dict(enabled=True))
    valid = np.ones(width, bool); valid[:30] = False; valid[-30:] = False
    calibration = dict(optical_signature='measured-rig',
                       geometry=dict(output_fov_m=fov, coefficients=[0., fov/2, 0., 0.],
                                     valid=valid.tolist(), lookup=np.arange(width).tolist()),
                       flat=dict(valid=np.ones(width, bool).tolist()))
    return config, calibration


def nominal_lines(config, task):
    """Independent spatial lattice; omit both ramps to test the conservative cruise guarantee."""
    pitch = config['motion']['advance_per_rev_m']; phase = math.radians(config['motion']['start_theta_deg'])
    n = 10000*128/15; lattice = np.arange(1, math.ceil(task['distance_m']/pitch*n), dtype=np.int64)
    theta = phase+lattice*2*math.pi/n
    x = task['start_m']+lattice*pitch/n
    wrapped = (theta+math.pi) % (2*math.pi)-math.pi
    selected = ((x >= task['start_m']+task['ramp_margin_m']) & (x <= task['end_m']-task['ramp_margin_m']) &
                (wrapped >= -2*math.pi/3) & (wrapped < 2*math.pi/3))
    return x[selected], theta[selected]


def test_relative_scale_planner_reserves_public_uncertainty_and_requires_real_buffer(nominal):
    config, calibration = nominal
    config['tunnel'].update(x_min_m=-1.5, x_max_m=21.5)
    with pytest.raises(ValueError, match='margin'):
        wall_plan(config, 0., 20., calibration, relative_encoder_scale=True)
    config['tunnel'].update(x_min_m=-2.5, x_max_m=22.5)
    plain, old = wall_plan(config, 0., 20., calibration)
    planned, task = wall_plan(config, 0., 20., calibration, relative_encoder_scale=True)
    assert planned['inspection']['relative_encoder_scale_bound_fraction'] == .03
    assert 'relative_encoder_scale_bound_fraction' not in plain['inspection']
    assert task['start_m'] < old['start_m'] and task['end_m'] > old['end_m']
    assert task['relative_scale_margin_m'] > .03*task['distance_m']/2
    changed = copy.deepcopy(config); changed['truth'] = dict(odo_left_diameter_m=.079, odo_right_diameter_m=.081)
    assert wall_plan(changed, 0., 20., calibration, relative_encoder_scale=True)[1] == task


@pytest.mark.parametrize('margin', [0., .5, 5., 10.])
def test_capture_guard_keeps_the_full_fixed_wall_target(nominal, margin):
    config, calibration = nominal
    config['gate'].update(start_deg=-120.-margin, end_deg=120.+margin,
                          start_rad=math.radians(-120.-margin), end_rad=math.radians(120.+margin))
    planned, task = wall_plan(config, 3., 3., calibration)
    np.testing.assert_allclose(planned['inspection']['theta_rad'], [-2*math.pi/3, 2*math.pi/3])
    assert task['output_arc_deg'] == 240.
    assert task['capture_gate_deg'] == [-120.-margin, 120.+margin]
    assert planned['gate'] == config['gate']


@pytest.mark.parametrize('start,end', [(-110.,130.),(-131.,131.),(-120.,119.)])
def test_wall_target_cannot_be_shrunk_or_outside_capture_gate(nominal,start,end):
    config, calibration = nominal
    config['gate'].update(start_deg=start,end_deg=end)
    with pytest.raises(ValueError,match='fixed upper 240'):
        wall_plan(config,3.,3.,calibration)


@pytest.mark.parametrize('start,length', [(0., 1.), (3., 3.), (0., 20.)])
@pytest.mark.parametrize('phase', [180., -130., 0., 120.])
def test_wall_plan_covers_requested_target_for_all_initial_phases(nominal, start, length, phase):
    config, calibration = nominal; config['motion']['start_theta_deg'] = phase
    before = copy.deepcopy(config)
    c, task = wall_plan(config, start, length, calibration)
    assert config == before
    assert c['inspection']['target_x_m'] == [start, start+length]
    assert task['start_m'] < start and task['end_m'] > start+length
    profile = np.array(c['motion']['profile'])
    travel = np.sum(np.diff(profile[:, 0])*(profile[:-1, 1]+profile[1:, 1])/2)*task['speed_m_s']
    assert travel == pytest.approx(task['distance_m'])
    x, theta = nominal_lines(c, task)
    grid = target_grid([start, start+length], 2.75, [-2*math.pi/3, 2*math.pi/3], .001)
    tiles = coverage_runs(x, theta, config, calibrated_spans(config, calibration), grid, q_tile_rows=333)
    assert all(np.all(tile['count'] > 0) for tile in tiles)


def test_wall_plan_ignores_private_truth_and_rejects_bad_margins(nominal):
    config, calibration = nominal
    _, expected = wall_plan(config, 0., 20., calibration)
    config['truth'] = dict(wheel_diameter_m=20., lens_k1=.4, track_irregularity=dict(seed=1))
    assert wall_plan(config, 0., 20., calibration)[1] == expected
    config['tunnel']['x_min_m'] = -.8
    with pytest.raises(ValueError, match='margin'): wall_plan(config, 0., 20., calibration)
    config['camera']['nominal_distance_m'] *= 2
    with pytest.raises(ValueError, match='narrow'): wall_plan(config, 3., 3., calibration)


@pytest.mark.parametrize('start,length', [(0., 1.), (3., 3.), (0., 20.)])
@pytest.mark.parametrize('phase', [180., -130., 0., 120.])
def test_partial_first_and_last_turns_have_public_outside_target_proofs(nominal, start, length, phase):
    from ssb_tools.initial_unroll import BandSampler, PROJECTION
    from ssb_tools.evaluate_global_geometry import seam_plan
    config, calibration = nominal; config['motion']['start_theta_deg'] = phase
    c, task = wall_plan(config, start, length, calibration)
    # Independent nominal spatial encoder trajectory, with ramps omitted. Use
    # a coarse analytic row lattice; no rendered pixels or simulated poses.
    x, theta = nominal_lines(c, task); x, theta = x[::100], theta[::100]
    projection = np.zeros(len(x), PROJECTION)
    projection['sequence'] = np.arange(len(x)); projection['lattice_row'] = np.arange(len(x))
    projection['x_axis_m'] = x
    projection['theta_rad'] = theta
    projection['segment'] = np.floor((theta+math.pi)/(2*math.pi)).astype(int)
    offsets = np.array(task['nominal_usable_span_m'])
    sampler = BandSampler(projection, np.zeros((len(x), 2), np.float32), offsets, offsets,
                          np.ones(2, bool), .02)
    grid = dict(theta_rad=[-2*math.pi/3, 2*math.pi/3], radius_m=2.75,
                target_x_m=[start, start+length], dx_m=.0002)
    windows = seam_plan(sampler, grid, .2, task['correction_reach_m'])
    assert any(w['status'] == 'planned' for w in windows)
    assert not any(w['status'] == 'unmeasurable' for w in windows)
    assert task['seam_margin_m'] > task['correction_reach_m']


@pytest.mark.parametrize('start,length', [(0., 1.), (3., 3.), (0., 20.)])
def test_every_target_intersecting_nominal_exposure_is_outside_the_ramps(nominal, start, length):
    config, calibration = nominal
    _, task = wall_plan(config, start, length, calibration)
    left, right = task['nominal_usable_span_m']
    head_mount = config['calibration']['head_mount_x_m']
    first_intersection = start-right-head_mount
    last_intersection = start+length-left-head_mount
    assert task['start_m']+task['ramp_margin_m'] < first_intersection
    assert task['end_m']-task['ramp_margin_m'] > last_intersection


def test_bad_column_is_not_silently_bridged(nominal):
    config, calibration = nominal
    calibration['flat']['valid'][2000] = False
    spans = calibrated_spans(config, calibration)
    assert len(spans) == 2 and spans[0][1] < spans[1][0]


def dense_counts(tiles, grid):
    dense = np.zeros(grid['shape'], np.int32)
    for tile in tiles:
        for run in tile:
            dense[run['q_bin'], run['x_begin']:run['x_end']] = run['count']
    return dense


def test_interval_sweep_matches_independent_dense_raster_and_tile_sizes():
    config = dict(gate=dict(start_rad=0., end_rad=.4), scan_encoder=dict(ppr=4, edges_per_cycle=4),
                  rescaler=dict(multiply=1, divide=1))
    x = np.array([.1, .5, .9, .55]); theta = np.array([.1, .15, .3, .25]); spans = [(-.12, -.02), (.01, .14)]
    grid = target_grid([0, 1], 1., [0, .4], .05)
    xs = (np.arange(grid['shape'][1])+.5)*grid['dx_m']
    qs = (np.arange(grid['shape'][0])+.5)*grid['dq_m']
    want = np.zeros(grid['shape'], int)
    for centre, angle in zip(x, theta):
        angular = abs(qs-angle) <= math.pi/16
        for a, b in spans:
            along = (xs >= centre+a-1e-12) & (xs <= centre+b+1e-12)
            want += angular[:, None]*along[None, :]
    for tile_size in (1, 3, 512):
        actual = dense_counts(coverage_runs(x, theta, config, spans, grid, tile_size), grid)
        np.testing.assert_array_equal(actual, want)


def test_pixel_footprint_is_independent_of_rescaler_and_does_not_bridge_missing_rows(nominal):
    c, calibration = nominal
    footprint = calibrated_row_footprint(c, calibration)
    assert footprint == pytest.approx(2*math.atan(calibration['geometry']['output_fov_m']/(2*4096*2.75)))
    c['rescaler']['multiply'] *= 2
    assert calibrated_row_footprint(c, calibration) == footprint
    config = dict(gate=dict(start_rad=0., end_rad=.4), scan_encoder=dict(ppr=100, edges_per_cycle=4),
                  rescaler=dict(multiply=1, divide=1))
    step = 2*math.pi/400
    theta = np.array([.1, .1+step*1.01, .1+step*2])
    grid = target_grid([0, 1], 1., [theta[0], theta[-1]], .0005)
    x = np.full(3, .5); spans = [(-.5, .5)]
    tiles = coverage_runs(x, theta, config, spans, grid, angular_footprint_rad=step*1.02)
    assert np.all(dense_counts(tiles, grid) > 0)
    tiles = coverage_runs(x[[0, 2]], theta[[0, 2]], config, spans, grid,
                          angular_footprint_rad=step*1.02)
    assert np.any(dense_counts(tiles, grid) == 0)


EDGE = np.dtype([('t', '<f8'), ('count', '<i8'), ('dir', '<i8')])
GATE = np.dtype([('t', '<f8'), ('revolution', '<i8'), ('kind', '<i4'), ('dir', '<i4')])
ROW = np.dtype([('sequence', '<i8'), ('row', '<i8'), ('segment', '<i8'), ('t_trigger', '<f8'), ('t_center', '<f8')])


def test_two_calibrated_measuring_wheels_replace_running_wheel_scale(nominal):
    config, _ = nominal
    edges = np.array([(0.5, 100, 1), (1., 200, 1), (2., 400, 1)], EDGE)
    right = np.array([(0.5, 150, 1), (1., 300, 1), (2., 600, 1)], EDGE)
    scan = np.array([(0., 0, 1), (1., 1000, 1), (2., 2000, 1)], EDGE)
    gates = np.array([(0., 0, 0, 1)], GATE)
    rows = np.array([(0, 1, 0, .5, .5), (1, 2, 0, 1., 1.)], ROW)
    x, _ = row_axis_coordinates(config, rows, scan, edges, gates, right)
    np.testing.assert_allclose(x, 3+math.pi*.08/10000*np.array([125., 250.]), atol=1e-12)
    config['calibration']['wheel_diameter_m'] = 100.
    np.testing.assert_allclose(row_axis_coordinates(config, rows, scan, edges, gates, right)[0], x)
    with pytest.raises(ValueError, match='both'): row_axis_coordinates(config, rows, scan, edges, gates)


def public_session(folder, nominal):
    config, calibration = nominal; c, task = wall_plan(config, 3., 3., calibration)
    # Constant-speed public sensor fixture: no private configuration, scene, TF or trajectory file exists.
    x, theta = nominal_lines(c, dict(task, ramp_margin_m=0.))
    times = (x-task['start_m'])/.2; phase = math.pi; omega = 2*math.pi/3
    rows = np.zeros(len(times), ROW)
    rows['sequence'] = np.arange(len(rows)); rows['row'] = np.rint((theta-phase)/(2*math.pi)*(10000*128/15))
    rows['segment'] = np.floor((theta+2*math.pi/3)/(2*math.pi)).astype(int)
    rows['t_trigger'] = times; rows['t_center'] = times
    scan_count = np.arange(5001, math.floor((phase+omega*task['distance_m']/.2)/(2*math.pi)*10000)+1)
    scan = np.zeros(len(scan_count), EDGE); scan['count'] = scan_count; scan['dir'] = 1
    scan['t'] = (scan_count*2*math.pi/10000-phase)/omega
    odo_count = np.arange(1, math.floor(task['distance_m']/(math.pi*.08/10000))+1)
    odo = np.zeros(len(odo_count), EDGE); odo['count'] = odo_count; odo['dir'] = 1
    odo['t'] = odo_count*math.pi*.08/10000/.2
    revolutions = np.arange(1, rows['segment'].max()+1)
    gates = np.zeros(len(revolutions), GATE); gates['revolution'] = revolutions; gates['dir'] = 1
    gates['t'] = (-2*math.pi/3+revolutions*2*math.pi-phase)/omega
    (folder/'config').mkdir(parents=True); (folder/'metadata').mkdir()
    (folder/'config/observable_config.json').write_text(json.dumps(c))
    manifest = {}
    for name, table in dict(rows=rows, scan_edges=scan, odometer_edges=odo, odometer_right_edges=odo, gate_events=gates).items():
        path = folder/'metadata'/(name+'.bin'); table.tofile(path)
        manifest[name] = dict(file=path.name, sha256=sha256_file(path), count=len(table), dtype=table.dtype.descr, record_size=table.dtype.itemsize)
    (folder/'metadata/manifest.json').write_text(json.dumps(manifest))
    summary = dict(status='complete', motion=dict(complete=True), rows=len(rows), files={
        name: sha256_file(folder/name) for name in ('config/observable_config.json', 'metadata/manifest.json')})
    (folder/'session.json').write_text(json.dumps(summary))
    path = folder/'calibration.json'; path.write_text(json.dumps(calibration))
    return path, rows


def test_public_only_reconstruction_and_missing_row_detection(nominal, tmp_path):
    root = tmp_path/'public_only'; calibration, rows = public_session(root, nominal)
    assert not (root/'evaluation').exists() and not (root/'raw').exists()
    report = inspect_session(root, calibration, tmp_path/'complete', q_tile_rows=257)
    assert report['nominal_complete'] and report['missing_pixels'] == 0
    assert Path(tmp_path/'complete/counts.bin').stat().st_size < 20*1024**2
    # Delete a cruise row near a target grid angular centre. Neighbours must not fill its gap.
    sample = next(i for i, row in enumerate(rows) if 7 < row['t_center'] < 8 and row['segment'] == 3)
    rows = np.delete(rows, sample); rows['sequence'] = np.arange(len(rows))
    path = root/'metadata/rows.bin'; rows.tofile(path)
    manifest_path = root/'metadata/manifest.json'; manifest = json.loads(manifest_path.read_text())
    manifest['rows'].update(count=len(rows), sha256=sha256_file(path)); manifest_path.write_text(json.dumps(manifest))
    summary = json.loads((root/'session.json').read_text()); summary['rows'] = len(rows)
    summary['files']['metadata/manifest.json'] = sha256_file(manifest_path)
    (root/'session.json').write_text(json.dumps(summary))
    report = inspect_session(root, calibration, tmp_path/'missing')
    assert not report['nominal_complete'] and report['missing_pixels'] > 0
    with pytest.raises(ValueError, match='target'): inspect_session(root, calibration, tmp_path/'crop', [3.1, 5.9])
    with pytest.raises(ValueError, match='pitch'): inspect_session(root, calibration, tmp_path/'coarse', pitch=.001)


def test_legacy_vehicle_travel_is_not_full_wall_coverage(nominal):
    config, calibration = nominal
    c, task = plan(config, 3., 3.)
    task['ramp_margin_m'] = .1
    x, theta = nominal_lines(c, task)
    grid = target_grid([3, 6], 2.75, [-2*math.pi/3, 2*math.pi/3], .001)
    assert any(np.any(tile['count'] == 0) for tile in coverage_runs(x, theta, c, calibrated_spans(c, calibration), grid))


def test_public_metadata_cannot_redirect_to_external_inputs(nominal, tmp_path):
    root = tmp_path/'public_only'; calibration, _ = public_session(root, nominal)
    manifest_path = root/'metadata/manifest.json'; manifest = json.loads(manifest_path.read_text())
    manifest['scan_edges']['file'] = '../../forbidden.bin'
    manifest_path.write_text(json.dumps(manifest))
    summary_path = root/'session.json'; summary = json.loads(summary_path.read_text())
    summary['files']['metadata/manifest.json'] = sha256_file(manifest_path)
    summary_path.write_text(json.dumps(summary))
    with pytest.raises(ValueError, match='escapes'): inspect_session(root, calibration, tmp_path/'rejected')
    assert not (tmp_path/'rejected').exists()
