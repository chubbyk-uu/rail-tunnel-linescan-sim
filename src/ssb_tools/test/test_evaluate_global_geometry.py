import numpy as np
import pytest

from ssb_tools.evaluate_band_matches import mesh_points
from ssb_tools.evaluate_global_geometry import (sources_at, endpoint_drift, boundary_coordinates,
    boundary_support, verify_session, run, seam_plan, shared_seam_plan, match_membership, stratum_gate,
    map_points)
from ssb_tools.global_geometry import Trajectory, GeometrySettings
from ssb_tools.initial_unroll import BandSampler, PROJECTION
from ssb_tools.native_rows import MemoryRows
from ssb_tools.ref_mesh import OpticalMesh


def test_cli_passes_the_frozen_seam_spacing_to_evaluation(monkeypatch):
    import sys
    import ssb_tools.evaluate_global_geometry as module
    received = {}
    def evaluate(*args, **kwargs):
        received.update(kwargs)
        return {k: {} for k in ('seam', 'seam_strata', 'mapping', 'boundary', 'gates', 'performance')}
    monkeypatch.setattr(module, 'run', evaluate)
    monkeypatch.setattr(sys, 'argv', ['evaluate_global_geometry', '--session', 'capture',
        '--unroll', 'd1', '--trajectory', 'fit', '--output', 'evaluation/new',
        '--spacing-m', '.1', '--workers', '3'])
    module.main()
    assert received['spacing_m'] == .1 and received['workers'] == 3


@pytest.mark.parametrize('gate', ['pass', 'fail', 'unmeasurable'])
def test_strict_geometry_cli_returns_failure_for_failed_or_unmeasurable_gates(monkeypatch, gate):
    import sys
    import ssb_tools.evaluate_global_geometry as module
    report = {k: {} for k in ('seam', 'seam_strata', 'mapping', 'boundary', 'performance')}
    report['gates'] = dict(strict_seam=dict(status=gate), perimeter=dict(status='pass'))
    monkeypatch.setattr(module, 'run', lambda *a, **kw: report)
    monkeypatch.setattr(sys, 'argv', ['evaluate_global_geometry', '--session', 'capture', '--unroll', 'd1',
        '--trajectory', 'fit', '--output', 'evaluation/new', '--strict'])
    if gate == 'pass': module.main()
    else:
        with pytest.raises(SystemExit) as error: module.main()
        assert error.value.code == 1


def independent_fixture(depth=0.):
    phases = np.linspace(-.2, .2, 101)
    p = np.zeros(202, PROJECTION)
    for band in range(2):
        sl = slice(101*band, 101*(band+1))
        p['sequence'][sl] = np.arange(sl.start, sl.stop)
        p['lattice_row'][sl] = np.arange(101)+band*1000
        p['segment'][sl] = band
        p['theta_rad'][sl] = phases+band*2*np.pi
        p['x_axis_m'][sl] = .4+.2*band+.1*phases
    width=513
    raw = np.full((202,width),150,np.uint8)
    native = MemoryRows(raw,dict(offset=np.zeros(width),gain=np.ones(width),valid=np.ones(width,bool)))
    offsets=(np.arange(width)-256)*.001
    sampler=BandSampler(p,native,offsets,offsets,np.ones(width,bool),.00401)
    model=Trajectory(sampler,1.,.7,GeometrySettings(attitude_spacing_m=.1))
    coefficients=np.zeros(model.size)
    # Independent injected axial scale: true vehicle x = 1.01*nominal x - .005.
    # Greville coordinates reproduce the affine correction exactly in cubic splines.
    k=model.knots[0]
    greville=np.array([k[i+1:i+4].mean() for i in range(model.sizes[0])])
    coefficients[:model.sizes[0]]=.01*(greville-.5)/model.scale
    rows=np.zeros(len(p),dtype=[('sequence','<i8'),('theta','<f8'),('x','<f8')])
    rows['sequence']=p['sequence'];rows['theta']=p['theta_rad']
    rows['x']=1.01*p['x_axis_m']-.005
    camera=dict(width=width,pixel_pitch_m=.001,nominal_distance_m=1.,fov_at_nominal_m=.513)
    truth=dict(head_mount_x_m=0.,robot=dict(base_reference_z_m=.3,scan_axis_height_m=.7),
               tunnel=dict(radius_m=1.,axis_z_m=1.),mount=dict(
                   dy_m=0.,dz_m=0.,e_m=0.,tangential_m=0.,tilt_y_rad=0.,tilt_z_rad=0.,twist_rad=0.))
    v=np.array([[-1.,-.5,2.+depth],[-1.,.5,2.+depth],[3.,.5,2.+depth],[3.,-.5,2.+depth]])
    mesh=OpticalMesh(v,[[0,1,2],[0,2,3]],[2,2],1.)
    return model,coefficients,rows,camera,truth,mesh


@pytest.mark.parametrize('depth,expected_after', [(0.,0.),(.02,-.00404)])
def test_actual_seam_uses_inverse_native_footprints_and_recessed_optical_mesh(depth,expected_after):
    model,c,rows,camera,truth,mesh=independent_fixture(depth)
    def seam(coefficients):
        points=[]
        for band in (0,1):
            table,valid,_=sources_at(model,coefficients,band,np.array([.5]),np.array([0.]))
            assert valid.all()
            point,material=mesh_points(table,'a',rows,camera,truth,mesh)
            assert material[0]==2
            points.append(point[0])
        return points[1]-points[0]
    before=seam(np.zeros(model.size))
    after=seam(c)
    np.testing.assert_allclose(after,[expected_after,0.],atol=1e-12)
    assert abs(before[0])>.001
    # The 20 mm recess leaves a real parallax error that an ideal-cylinder
    # evaluator would hide even with the correct injected vehicle trajectory.
    if depth:assert abs(after[0])>.004


def test_shared_radial_surface_removes_parallax_without_changing_pose_or_discarding_sources():
    from ssb_tools.surface_relief import SurfaceRelief
    model, c, rows, camera, truth, mesh = independent_fixture(.02)
    original = c.copy()
    model.relief = SurfaceRelief([dict(grid=[.25, -.1, .5, .2],
                                      depth=np.full((2, 2), .02, np.float32))])
    points = []
    for band in (0, 1):
        table, valid, _ = sources_at(model, c, band, np.array([.5]), np.array([0.]))
        assert valid.all()
        point, material = mesh_points(table, 'a', rows, camera, truth, mesh)
        assert material[0] == 2
        points.append(point[0])
    np.testing.assert_allclose(points[1]-points[0], [0., 0.], atol=1e-9)
    np.testing.assert_array_equal(c, original)


def test_relief_is_used_even_for_zero_pose_coefficients_and_nominal_diagnostic_stays_unchanged():
    from ssb_tools.surface_relief import SurfaceRelief
    model, _, *_ = independent_fixture()
    zero = np.zeros(model.size)
    before = sources_at(model, zero, 0, np.array([.5]), np.array([0.]))[0]
    model.relief = SurfaceRelief([dict(grid=[.25, -.1, .5, .2],
                                      depth=np.full((2, 2), .02, np.float32))])
    optimized = sources_at(model, zero, 0, np.array([.5]), np.array([0.]))[0]
    nominal = sources_at(model, zero, 0, np.array([.5]), np.array([0.]), use_relief=False)[0]
    np.testing.assert_array_equal(nominal, before)
    assert optimized['a_lower_column'][0] != before['a_lower_column'][0]


def test_nominal_mapping_drift_and_boundary_ignore_optimized_relief():
    from ssb_tools.surface_relief import SurfaceRelief
    model, _, rows, camera, truth, mesh = independent_fixture(.02)
    zero = np.zeros(model.size)
    grid = dict(shape=[11, 9], target_x_m=[.35, .65], theta_rad=[-.15, .15],
                radius_m=1., dx_m=.3/9, dq_m=.3/11)
    xs = np.linspace(.35, .65, 9); qs = np.linspace(-.1, .1, 3)
    x, q = np.meshgrid(xs, qs)
    before = map_points(model, zero, x.ravel(), q.ravel(), rows, camera, truth, mesh, use_relief=False)
    boundary = boundary_support(model, zero, grid, use_relief=False)
    model.relief = SurfaceRelief([dict(grid=[.25, -.2, .5, .4], depth=np.full((2, 2), .02, np.float32))])
    after = map_points(model, zero, x.ravel(), q.ravel(), rows, camera, truth, mesh, use_relief=False)
    for actual, expected in zip(after, before):
        np.testing.assert_array_equal(actual, expected)
    assert boundary_support(model, zero, grid, use_relief=False) == boundary
    optimized = map_points(model, zero, x.ravel(), q.ravel(), rows, camera, truth, mesh, use_relief=True)
    assert not np.allclose(optimized[0], before[0])
    assert endpoint_drift(after[0].reshape(x.shape+(2,)), after[2].reshape(x.shape) >= 0, xs, qs) == \
           endpoint_drift(before[0].reshape(x.shape+(2,)), before[2].reshape(x.shape) >= 0, xs, qs)


def test_raw_exposure_gap_remains_in_real_mesh_scoring_and_fails_gate():
    from ssb_tools.evaluate_global_geometry import score_window, residual_summary
    model, c, rows, camera, truth, mesh = independent_fixture()
    old = model.sampler; p = old.projection
    keep = ~((p['segment'] == 0) & (abs(p['theta_rad']) < .012))
    native = MemoryRows(old.native.raw_rows[keep], dict(offset=np.zeros(513), gain=np.ones(513), valid=np.ones(513, bool)))
    sampler = BandSampler(p[keep], native, old.offsets, old.output_offsets, old.geometry_valid, old.footprint)
    model = Trajectory(sampler, 1., .7, GeometrySettings(attitude_spacing_m=.1))
    grid = dict(theta_rad=[-.075, .075], radius_m=1., target_x_m=[.4, .6], dx_m=.001, dq_m=.001)
    plan = shared_seam_plan(model, c, grid, .1)
    gap = next(w for w in plan if abs(w['q_center_m']) < 1e-12)
    assert 'nominal_probe_x_m' in gap
    training = [dict(source_window=dict(bands=[0, 1], shape=[53, 513], x_first_m=.3, q_first_m=-.051))]
    scored = [score_window(w, model, c, 9, training, grid, .001, rows, camera, truth, mesh)
              for w in plan if 'nominal_probe_x_m' in w]
    assert len(scored) == 3
    gap_score = next(item for item in scored if item[1]['window'] == gap['id'])
    assert gap_score[1]['optimized']['missing_samples'] == 9
    assert sum(gap_score[3]['optimized', s] for s in ('within_match_window', 'between_match_windows')) == 9
    strata = dict(optimized={})
    for name in ('within_match_window', 'between_match_windows'):
        values = np.concatenate([sample[5][sample[8] == name] for item in scored
                                 for sample in item[2] if sample[0] == 'optimized'])
        assert len(values)
        strata['optimized'][name] = dict(status='measured',
            missing_samples=sum(item[3].get(('optimized', name), 0) for item in scored),
            **residual_summary(values, .001))
    assert stratum_gate(strata, 1.)['status'] == 'fail'
    # A nominally disjoint envelope is insufficient if declared corrections
    # can reach the target. Only the expanded public bound can prove exclusion.
    grid['target_x_m'] = [.68, .70]
    uncertain = shared_seam_plan(model, c, grid, .1)
    assert next(w for w in uncertain if abs(w['q_center_m']) < 1e-12)['status'] == 'unmeasurable'
    grid['target_x_m'] = [10., 11.]
    outside = shared_seam_plan(model, c, grid, .1)
    assert all(w['status'] == 'excluded' for w in outside)


def test_partial_band_proof_uses_all_reachable_recorded_rows_not_a_single_endpoint():
    from ssb_tools.evaluate_global_geometry import missing_angle_envelope
    model, *_ = independent_fixture()
    sampler = model.sampler; usable = sampler.output_offsets
    reach = .04
    envelope, rule = missing_angle_envelope(sampler, 0, -.205, 1., reach, usable)
    assert rule == 'recorded_prefix_with_correction_bound'
    a, b = sampler.bounds[0]
    phases = sampler.phases[0]
    axes = sampler.projection['x_axis_m'][a:b]
    eligible = phases <= -.205+reach+sampler.footprint/2
    assert envelope[1] == pytest.approx(axes[eligible].max()+usable[-1]+reach)
    assert envelope[1] > axes[0]+usable[-1]+reach
    whole, rule = missing_angle_envelope(sampler, 0, 0., 1., reach, usable)
    assert rule == 'whole_band_internal_gap'
    assert whole == [sampler.band_x[0][0]-reach, sampler.band_x[0][1]+reach]


def test_drift_removes_translation_but_retains_scale_and_circumferential_shear():
    xs=np.linspace(3.,6.,33);qs=np.linspace(-1.,1.,49)
    xx,qq=np.meshgrid(xs,qs)
    points=np.stack((1.01*xx+.1,qq+.02+.002*(xx-3)),axis=-1)
    report=endpoint_drift(points,np.ones(xx.shape,bool),xs,qs)
    assert report['axial_scale_error_percent']==pytest.approx(1.)
    np.testing.assert_allclose(report['endpoint_delta_mean_m'],[.03,.006],atol=1e-12)
    valid=np.ones(xx.shape,bool);valid[:,0]=False
    assert endpoint_drift(points,valid,xs,qs)['status']=='unmeasurable'


def test_perimeter_enumerates_every_edge_pixel_without_duplicate_corners():
    grid=dict(shape=[7,5],target_x_m=[0.,2.],theta_rad=[-1.,1.],radius_m=1.,dx_m=.4,dq_m=2/7)
    boundary=boundary_coordinates(grid)
    values=np.concatenate([np.column_stack(v) for v in boundary.values()])
    assert len(values)==2*7+2*5-4 and len(np.unique(values,axis=0))==len(values)


def test_perimeter_requires_valid_original_pixels_and_does_not_fill_saturation():
    model,c,*_=independent_fixture()
    grid=dict(shape=[11,9],target_x_m=[.4,.6],theta_rad=[-.15,.15],radius_m=1.,
              dx_m=.2/9,dq_m=.3/11)
    baseline=boundary_support(model,np.zeros(model.size),grid)
    assert sum(v['missing_pixels'] for v in baseline.values())==0
    phase=model.sampler.projection['theta_rad']-2*np.pi*model.sampler.projection['segment']
    model.sampler.native.raw_rows[phase>=0]=255
    report=boundary_support(model,np.zeros(model.size),grid)
    assert report['bottom']['missing_pixels']==9
    assert report['top']['missing_pixels']==0
    assert report['left']['missing_pixels']>0


def test_evaluator_cannot_write_private_results_into_public_reconstruction(tmp_path):
    with pytest.raises(ValueError,match='evaluation/'):
        run(tmp_path/'nonexistent_session',tmp_path/'d1',tmp_path/'d3',tmp_path/'public')


@pytest.mark.parametrize('changed', ['evaluation/manifest.json','config/backend.json','evaluation/truth.json'])
def test_truth_reference_requires_the_original_archived_identity(tmp_path,changed):
    from types import SimpleNamespace
    from ssb_tools.session import sha256_file
    names=('config/observable_config.json','metadata/manifest.json','raw/index.json',
           'evaluation/truth.json','evaluation/manifest.json','evaluation/config_source.yaml',
           'config/backend.json','config/provenance.json')
    files={}
    for name in names:
        p=tmp_path/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('{}')
        files[name]=sha256_file(p)
    session=SimpleNamespace(root=tmp_path,summary=dict(status='complete',files=files))
    upstream=dict(source_observation_hashes={k:files[k] for k in names[:3]})
    verify_session(session,upstream)
    (tmp_path/changed).write_text('{"replaced":true}')
    with pytest.raises(ValueError,match='archived identity'):
        verify_session(session,upstream)


def test_fixed_seam_plan_interleaves_gaps_without_using_matching_planner(monkeypatch):
    import ssb_tools.match_bands as matching
    def forbidden(*args, **kwargs):
        raise AssertionError('evaluation must not reuse D2 window selection')
    monkeypatch.setattr(matching, 'plan_windows', forbidden)
    model, *_ = independent_fixture()
    grid = dict(theta_rad=[-.15, .15], radius_m=1., target_x_m=[.4, .6], dx_m=.001)
    plan = seam_plan(model.sampler, grid, .2)
    planned = [w for w in plan if w['status'] == 'planned']
    # Hand-computed quarter/three-quarter spacing from lower q = -0.15.
    np.testing.assert_allclose(sorted(w['q_center_m'] for w in planned), [-.1, 0., .1], atol=1e-15)
    assert {w['phase'] for w in planned} == {.25, .75}
    for w in planned:
        assert .4 < w['x_m'][0] < w['x_m'][1] < .6
    grid['target_x_m'] = [10., 11.]
    excluded = seam_plan(model.sampler, grid, .2)
    assert excluded and all(w['status'] == 'excluded' for w in excluded)


@pytest.mark.parametrize('translation', [-.01, .01])
def test_common_overlap_keeps_q_locations_and_retains_original_boundary_probes(translation, monkeypatch):
    import ssb_tools.evaluate_global_geometry as module
    def forbidden(*args, **kwargs):
        raise AssertionError('shared support must not read truth or pixel intensity')
    model, *_ = independent_fixture()
    model.sampler.native.gather = forbidden
    monkeypatch.setattr(module, 'mesh_points', forbidden)
    c = np.zeros(model.size); c[:model.sizes[0]] = translation/model.scale
    grid = dict(theta_rad=[-.15, .15], radius_m=1., target_x_m=[.2, .8], dx_m=.001)
    original = seam_plan(model.sampler, grid)
    shared = shared_seam_plan(model, c, grid)
    assert [(w['id'], w['q_center_m'], w['status']) for w in shared] == [
        (w['id'], w['q_center_m'], w['status']) for w in original]
    # Independent constant-translation geometry gives exact shared bounds.
    for old, new in zip(original, shared):
        if old['status'] != 'planned': continue
        assert new['nominal_probe_x_m'] == old['x_m']
        assert new['shared_support'] == 'measurable'
        expected_left = old['x_m'][0]+max(translation, 0.)
        expected_right = old['x_m'][1]+min(translation, 0.)
        np.testing.assert_allclose(new['x_m'], [expected_left, expected_right], atol=4e-7, rtol=0)


def test_empty_shared_interval_cannot_remove_a_planned_location():
    model, *_ = independent_fixture()
    # Both fields individually stay within the declared 30 mm correction bound,
    # but their truly narrow fields of view no longer overlap anywhere.
    model.sampler.offsets[:] *= .106/.256
    knot = model.knots[0]
    greville = np.array([knot[i+1:i+4].mean() for i in range(model.sizes[0])])
    c = np.zeros(model.size); c[:model.sizes[0]] = .2*(greville-.5)/model.scale
    grid = dict(theta_rad=[-.15, .15], radius_m=1., target_x_m=[.4, .6], dx_m=.001)
    original = seam_plan(model.sampler, grid)
    shared = shared_seam_plan(model, c, grid)
    assert len(shared) == len(original)
    target = next(w for w in shared if abs(w['q_center_m']) < 1e-12)
    assert target['status'] == 'planned' and target['shared_support'] == 'unmeasurable'
    assert target['x_m'] == target['nominal_probe_x_m']


@pytest.mark.parametrize('translation,target', [(.02, [.345, .346]), (-.02, [.654, .655])])
def test_observed_common_interval_outside_output_is_distinct_from_missing_observations(translation, target):
    model, *_ = independent_fixture()
    def forbidden(*args):
        raise AssertionError('outside-target proof must not inspect intensities')
    model.sampler.native.gather = forbidden
    c = np.zeros(model.size); c[:model.sizes[0]] = translation/model.scale
    grid = dict(theta_rad=[-.15, .15], radius_m=1., target_x_m=target, dx_m=.0001)
    windows = shared_seam_plan(model, c, grid)
    window = next(w for w in windows if abs(w['q_center_m']) < 1e-12)
    assert window['status'] == 'outside_target' and 'nominal_probe_x_m' in window
    lo, hi = window['outside_target_shared_x_m']
    assert lo >= target[1] or hi <= target[0]
    assert window['outside_target_proof_points'] == 9


def test_outside_target_exclusion_requires_actual_angular_observations(monkeypatch):
    import ssb_tools.evaluate_global_geometry as module
    model, *_ = independent_fixture()
    c = np.zeros(model.size); c[:model.sizes[0]] = 20.
    grid = dict(theta_rad=[-.15, .15], radius_m=1., target_x_m=[.345, .346], dx_m=.0001)
    monkeypatch.setattr(module, 'geometry_supported', lambda *args: False)
    plan = shared_seam_plan(model, c, grid)
    window = next(w for w in plan if abs(w['q_center_m']) < 1e-12)
    assert window['status'] == 'planned' and window['shared_support'] == 'unmeasurable'


def test_shared_plan_does_not_adapt_to_saturation_or_move_q_away_from_missing_exposures():
    model, *_ = independent_fixture()
    p = model.sampler.projection
    phase = p['theta_rad']-2*np.pi*p['segment']
    keep = ~np.isclose(phase, .004, atol=1e-12, rtol=0)
    old = model.sampler
    native = MemoryRows(old.native.raw_rows[keep], dict(offset=np.zeros(513),
                        gain=np.ones(513), valid=np.ones(513, bool)))
    sampler = BandSampler(p[keep], native, old.offsets, old.output_offsets,
                          old.geometry_valid, old.footprint)
    model = Trajectory(sampler, 1., .7, GeometrySettings(attitude_spacing_m=.1))
    c = np.zeros(model.size); c[model.starts[1]:model.starts[2]] = -4.
    grid = dict(theta_rad=[-.15, .15], radius_m=1., target_x_m=[.35, .75], dx_m=.001)
    plan = shared_seam_plan(model, c, grid)
    native.raw_rows[:] = 255
    assert shared_seam_plan(model, c, grid) == plan
    window = next(w for w in plan if abs(w['q_center_m']) < 1e-12)
    assert window['status'] == 'planned'  # No angular-gap exclusion or q relocation.
    # Restore valid intensity: the missing angular observation still fails.
    native.raw_rows[:] = 150
    _, valid, _ = sources_at(model, c, 0, np.linspace(*window['x_m'], 9), np.zeros(9))
    assert not valid.any()


def test_exact_row_centre_does_not_require_a_zero_weight_invalid_neighbour():
    model, *_ = independent_fixture()
    model.sampler.native.raw_rows[[49, 51]] = 255
    angles = np.array([0., 4e-16, 1e-9])
    _, valid, _, sources = model.sampler.sample(0, angles, np.array([.5]), sources=True)
    assert valid[:, 0].tolist() == [True, True, False]
    assert np.all(sources['lower'][:2] == 50) and np.all(sources['upper'][:2] == 50)


def test_seam_membership_checks_axial_and_angular_training_bounds():
    grid = dict(dx_m=.01, dq_m=.01)
    training = [dict(source_window=dict(bands=[0, 1], shape=[3, 3],
                                       x_first_m=1., q_first_m=2.))]
    labels = match_membership(np.array([1.01, 1.04, 1.01]),
                              np.array([2.01, 2.01, 2.04]), [0, 1], training, grid)
    assert labels.tolist() == ['within_match_window', 'between_match_windows', 'between_match_windows']
    assert (match_membership(np.array([1.01]), np.array([2.01]), [1, 2], training, grid)
            == 'between_match_windows').all()


@pytest.mark.parametrize('gap_status,missing,p95,expected', [
    ('measured', 0, .9, 'pass'), ('measured', 0, 1.1, 'fail'),
    ('measured', 1, .9, 'fail'), ('unmeasurable', 0, 0., 'unmeasurable')])
def test_gate_cannot_hide_bad_or_missing_gap_samples(gap_status, missing, p95, expected):
    strata = dict(optimized=dict(
        within_match_window=dict(status='measured', missing_samples=0, norm_px=dict(p95=.2)),
        between_match_windows=dict(status=gap_status, missing_samples=missing, norm_px=dict(p95=p95))))
    assert stratum_gate(strata, 1.)['status'] == expected


def test_worker_count_cannot_change_any_evaluation_value():
    import json
    model, c, rows, camera, truth, mesh = independent_fixture(.02)
    grid = dict(theta_rad=[-.15, .15], radius_m=1., target_x_m=[.2, .8], dx_m=.001)
    serial = shared_seam_plan(model, c, grid, .1)
    assert sum(w['status'] == 'planned' for w in serial) >= 2
    assert json.dumps(shared_seam_plan(model, c, grid, .1, workers=2)) == json.dumps(serial)
    x, q = np.linspace(.3, .7, 1200), np.linspace(-.1, .1, 1200)
    expected = map_points(model, c, x, q, rows, camera, truth, mesh)
    assert (expected[2] >= 0).sum() > 1000
    for value, reference in zip(map_points(model, c, x, q, rows, camera, truth, mesh, workers=3), expected):
        assert value.dtype == reference.dtype
        np.testing.assert_array_equal(value, reference)
