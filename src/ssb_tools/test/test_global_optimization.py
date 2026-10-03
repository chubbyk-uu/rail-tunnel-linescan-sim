import json
import math

import numpy as np
import pytest
from scipy.optimize import root

from ssb_tools.global_geometry import GeometrySettings, Trajectory, cylinder_points
from ssb_tools.initial_unroll import BandSampler, PROJECTION
from ssb_tools.match_bands import MATCH, native_sources
from ssb_tools.optimize_bands import fit, public_robot, window_weights, verified_matches, run, damped_solve
from ssb_tools.provenance import stage_record
from ssb_tools.session import sha256_file


def independent_hits(axis, theta, tangent, corrections, radius, height):
    """Matrix rotations and a general quadratic, independently of the production algebra."""
    result = []
    for x, angle, u, correction in zip(axis, theta, tangent, corrections):
        dx, dq, roll, pitch = correction[:4]
        dy, dz = correction[4:] if len(correction) == 6 else (0., correction[4]) if len(correction) == 5 else (0., 0.)
        a, b = roll, pitch
        rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
        ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
        rotation = ry @ rx
        origin = np.array([x+dx, dy, dz-height])+rotation @ np.array([0., 0., height])
        angle += dq/radius
        direction = rotation @ np.array([u, math.sin(angle), math.cos(angle)])
        coefficients = [direction[1:] @ direction[1:], 2*(origin[1:] @ direction[1:]),
                        origin[1:] @ origin[1:]-radius**2]
        distance = max(np.roots(coefficients))
        hit = origin+distance*direction
        result.append([hit[0], radius*math.atan2(hit[1], hit[2])])
    return np.asarray(result)


def synthetic_matches(translations=False):
    phase = np.linspace(-1., 1., 401)
    offsets = np.linspace(-.22, .22, 512)
    projection = np.zeros(3*len(phase), PROJECTION)
    for band in range(3):
        ids = slice(band*len(phase), (band+1)*len(phase))
        projection['sequence'][ids] = np.arange(ids.start, ids.stop)
        projection['segment'][ids] = band
        projection['lattice_row'][ids] = np.arange(len(phase))+band*10000
        projection['theta_rad'][ids] = phase+2*math.pi*band
        projection['x_axis_m'][ids] = .4+.25*band+.04*phase
    sampler = BandSampler(projection, np.zeros((len(projection), len(offsets)), np.float32),
                          offsets, offsets, np.ones(len(offsets), bool), .0051)
    model = Trajectory(sampler, 1., .7, GeometrySettings(attitude_spacing_m=.1,
        fit_translation=translations is True,fit_heave=translations=='heave'))

    def truth_at(axis):
        # Nonzero coupled position, phase, roll and pitch; no production function creates truth.
        s = np.asarray(axis)-.65
        values = np.column_stack((.0003*s, -.0002*s,
                                .0006+.0005*s+.0004*s*s,
                                -.0005+.001*s-.0006*s*s))
        if translations is True:
            values = np.column_stack((values, .0004+.0006*s, .001+.0007*s-.0003*s*s))
        elif translations=='heave':
            values = np.column_stack((values,.001+.0007*s-.0003*s*s))
        return values

    def measured_point(band, x, q):
        lo, hi, weight, supported = sampler.row_sources(band, np.array([q]))
        assert supported.all()
        out = np.zeros(2)
        for ids, w in ((lo, 1-weight), (hi, weight)):
            p = projection[ids]
            # Piecewise-linear sensor geometry is exact for these synthetic offsets.
            ray = independent_hits(p['x_axis_m'], p['theta_rad']-2*math.pi*p['segment'],
                                   (x-p['x_axis_m']), truth_at(p['x_axis_m']), 1., .7)
            out += w[0]*ray[0]
        return out

    rng = np.random.default_rng(40)
    records = []
    for band in range(2):
        for window, centre in enumerate((-.75, -.25, .25, .75)):
            for point in range(40):
                qa = centre+rng.uniform(-.08, .08)
                xa = .4+.25*band+.04*qa+.125+rng.uniform(-.04, .04)
                hit = measured_point(band, xa, qa)
                solution = root(lambda xy: measured_point(band+1, *xy)-hit, [xa, qa], tol=1e-10)
                assert solution.success and np.linalg.norm(solution.fun) < 1e-10
                row = np.zeros(1, MATCH)
                row['window'] = band*4+window; row['band_a'] = band; row['band_b'] = band+1
                row['x_a_m'], row['q_a_m'] = xa, qa
                row['x_b_m'], row['q_b_m'] = solution.x
                row['inlier'] = 1; row['holdout'] = int(point % 5 == 0)
                row['ncc'] = .95
                for side, k, x, q in (('a', band, xa, qa), ('b', band+1, *solution.x)):
                    for key, value in native_sources(sampler, k, np.array([x]), np.array([q]), 1.).items():
                        row[side+'_'+key] = value
                records.append(row)
    grid = dict(dx_m=.0002, dq_m=.0002)
    return model, np.concatenate(records), grid


def test_exact_support_rotation_and_cylinder_hit_against_independent_matrix_reference():
    rng = np.random.default_rng(17)
    axis = rng.uniform(2, 5, 24); theta = rng.uniform(-2.05, 2.05, 24)
    tangent = rng.uniform(-.15, .15, 24)
    correction = rng.uniform(-1, 1, (24, 6))*[.002, .001, .004, .003, .003, .002]
    expected = independent_hits(axis, theta, tangent, correction, 2.75, 1.715)
    np.testing.assert_allclose(cylinder_points(axis, theta, tangent, correction, 2.75, 1.715),
                               expected, atol=2e-14, rtol=0)


@pytest.mark.parametrize('translations', [False, True, 'heave'])
def test_sparse_native_jacobian_includes_angular_weights_and_upright_motion(translations):
    model, table, _ = synthetic_matches(translations)
    rays = model.native_side(table[:12], 'a')
    c = np.linspace(-.2, .3, model.size)
    analytic = rays.jacobian(c)
    columns = [0, model.starts[1]+1, model.starts[2]+3, model.starts[3]+5]
    columns += [model.starts[field]+3 for field in range(4,len(model.fields))]
    for column in columns:
        plus, minus = c.copy(), c.copy()
        plus[column] += 1e-4; minus[column] -= 1e-4
        reference = (rays.hits(plus)-rays.hits(minus))/(2e-4)
        for k in range(2):
            np.testing.assert_allclose(analytic[k][:, column].toarray().ravel(), reference[:, k],
                                       atol=2e-10, rtol=1e-5)


@pytest.mark.parametrize('translations', [False, True, 'heave'])
def test_global_fit_recovers_independent_coupled_scene_and_does_not_train_on_holdout(translations):
    model, table, grid = synthetic_matches(translations)
    c, scores, _, _, _, _, evidence = fit(model, table, grid)
    assert scores['heldout_before']['norm_px']['p95'] > .5
    assert scores['heldout_after']['norm_px']['p95'] < .03
    assert np.isfinite(c).all()
    assert evidence['data_rank'] < model.size  # Priors must not be presented as image observability.
    changed = table.copy(); selected = changed['holdout'] > 0
    changed['x_b_m'][selected] += .003
    for name, value in native_sources(model.sampler, 1,
            changed['x_b_m'][selected & (changed['band_b'] == 1)],
            changed['q_b_m'][selected & (changed['band_b'] == 1)], 1.).items():
        changed['b_'+name][selected & (changed['band_b'] == 1)] = value
    for name, value in native_sources(model.sampler, 2,
            changed['x_b_m'][selected & (changed['band_b'] == 2)],
            changed['q_b_m'][selected & (changed['band_b'] == 2)], 1.).items():
        changed['b_'+name][selected & (changed['band_b'] == 2)] = value
    other, changed_scores, _, _, _, _, _ = fit(model, changed, grid)
    np.testing.assert_array_equal(c, other)
    assert changed_scores['heldout_after']['norm_px']['p95'] > 10


def test_noise_weights_do_not_use_ncc_or_holdout_and_cap_correlated_window_information():
    model, table, grid = synthetic_matches()
    train, weights, descriptions = window_weights(table, grid, model.settings)
    changed = table.copy(); changed['ncc'] = np.linspace(-1, 1, len(table))
    changed['x_b_m'][changed['holdout'] > 0] += .2
    np.testing.assert_array_equal(window_weights(changed, grid, model.settings)[1], weights)
    assert not weights[~train].any()
    for item in descriptions:
        s = (table['window'] == item['window']) & train
        np.testing.assert_allclose(np.sum((weights[s]*item['sigma_px'])**2, axis=0),
                                   [16., 16.], atol=1e-12)


@pytest.mark.parametrize('kind', ['nan', 'source', 'column', 'band'])
def test_invalid_native_sources_are_rejected(kind):
    model, table, _ = synthetic_matches()
    table = table[:3].copy()
    if kind == 'nan': table['a_angular_weight'][0] = np.nan
    if kind == 'source': table['a_lower_sequence'][0] = 999999
    if kind == 'column': table['a_lower_column'][0] = -1
    if kind == 'band': table['band_a'][0] = 1
    with pytest.raises(ValueError): model.native_side(table, 'a')


def test_nominal_config_requires_public_identity_and_rejects_redirected_truth(tmp_path):
    config = tmp_path/'config'; config.mkdir()
    path = config/'observable_config.json'
    path.write_text(json.dumps(dict(schema='ssb.observable_config.v1',
        calibration=dict(radius_m=2.75), robot=dict(scan_axis_height_m=1.715))))
    assert public_robot(path, sha256_file(path), 2.75)[0] == 1.715
    with pytest.raises(ValueError, match='identity'): public_robot(path, '0'*64, 2.75)
    private = tmp_path/'evaluation'; private.mkdir()
    target = private/'truth.json'; path.rename(target); path.symlink_to(target)
    with pytest.raises(ValueError, match='escapes'): public_robot(path, sha256_file(target), 2.75)


def test_empty_or_disconnected_training_cannot_pass():
    model, table, grid = synthetic_matches()
    with pytest.raises(ValueError): fit(model, table[:0], grid)
    table['inlier'][table['band_a'] == 1] = 0
    with pytest.raises(ValueError): fit(model, table, grid)


@pytest.mark.parametrize('kind', ['corrupt', 'symlink', 'different_d1'])
def test_d2_hash_chain_and_d1_identity_are_checked(tmp_path, kind):
    from test_match_bands import bands_fixture
    from ssb_tools.match_bands import run as match, verified_bands
    from ssb_tools.band_matching import MatchSettings
    d1, d2 = tmp_path/'d1', tmp_path/'d2'
    bands_fixture(d1)
    match(d1, d2, spacing_m=.04, height=128, max_width=256, settings=MatchSettings(max_shift_mm=4.))
    sampler, upstream, _ = verified_bands(d1)
    if kind == 'corrupt':
        with (d2/'matches.npy').open('r+b') as file: file.write(b'bad')
        message = 'hash mismatch'
    elif kind == 'symlink':
        private = tmp_path/'evaluation'; private.mkdir()
        (d2/'matches.npy').rename(private/'matches.npy')
        (d2/'matches.npy').symlink_to(private/'matches.npy'); message = 'escapes'
    else:
        m = np.load(d1/'mapping.npz'); arrays = dict(m); arrays['native_x_offset_m'] += .0001
        np.savez(d1/'mapping.npz', **arrays); message = 'different D1 geometry'
    with pytest.raises(ValueError, match=message): verified_matches(d2, d1, upstream, sampler)


@pytest.mark.parametrize('settings', [GeometrySettings(attitude_spacing_m=.1),
    GeometrySettings(attitude_spacing_m=.1, observed_knots=True),
    GeometrySettings(attitude_spacing_m=.1, fit_translation=True),
    GeometrySettings(attitude_spacing_m=.1, fit_heave=True)])
def test_public_only_end_to_end_optimizer_preserves_upstream_and_writes_hash_chain(tmp_path, settings):
    from test_match_bands import bands_fixture
    from ssb_tools.match_bands import run as match
    from ssb_tools.band_matching import MatchSettings
    d1, d2, output = tmp_path/'d1', tmp_path/'d2', tmp_path/'d3'
    bands_fixture(d1)
    config = tmp_path/'config'; config.mkdir(); observable = config/'observable_config.json'
    observable.write_text(json.dumps(dict(schema='ssb.observable_config.v1',
        calibration=dict(radius_m=1.), robot=dict(scan_axis_height_m=.7))))
    files = sorted(p for p in d1.iterdir() if p.name != 'provenance.json')
    (d1/'provenance.json').write_text(json.dumps(stage_record('initial_unroll', [observable], files, {})))
    match(d1, d2, spacing_m=.04, height=128, max_width=256, settings=MatchSettings(max_shift_mm=4.))
    before = {str(p): sha256_file(p) for folder in (d1, d2) for p in folder.iterdir()}
    report = run(d1, d2, observable, output, settings)
    assert report['status'] == 'complete' and report['bands'] == 2
    assert report['image_consistency']['heldout_after']['norm_px']['p95'] < .5
    assert not (tmp_path/'evaluation').exists()
    provenance = json.loads((output/'provenance.json').read_text())
    assert provenance['stage'] == 'global_optimization'
    assert all(sha256_file(path) == digest for path, digest in provenance['outputs'].items())
    assert all(sha256_file(path) == digest for path, digest in before.items())
    with pytest.raises(ValueError, match='separate'): run(d1, d2, observable, d1/'wrong')
    from ssb_tools.global_resample import run as review
    result = review(d1, output, tmp_path/'review')
    assert len(result['crops']) >= 1 and (tmp_path/'review/review.html').is_file()
    assert (tmp_path/'review/nominal.png').is_file() and (tmp_path/'review/optimized.png').is_file()
    with (output/'trajectory.json').open('a') as file: file.write(' ')
    with pytest.raises(ValueError, match='D3 product hash'):
        review(d1, output, tmp_path/'rejected')


def test_damped_normal_solver_honors_bounds_and_recovers_known_quadratic():
    from scipy.sparse import csr_matrix
    matrix = csr_matrix([[3., 1.], [1., 4.], [1., -1.]])
    expected = np.array([.7, -.2]); target = matrix @ expected
    solution, evidence = damped_solve(lambda x: matrix @ x-target, lambda x: matrix,
                                     np.zeros(2), np.ones(2))
    np.testing.assert_allclose(solution, expected, atol=1e-7)
    assert evidence['cost'] < 1e-12
    bounded, _ = damped_solve(lambda x: matrix @ x-target, lambda x: matrix,
                              np.zeros(2), np.array([.3, 1.]))
    assert abs(bounded[0]-.3) < 1e-8 and abs(bounded[1]) <= 1
    # Independent KKT solution: x0=.3 is active; optimize the remaining coordinate.
    np.testing.assert_allclose(bounded,[.3,-1/15],atol=1e-6)


def test_pointwise_sampling_matches_band_sampler_and_preserves_invalid_footprints():
    from ssb_tools.global_resample import native_points
    model, _, _ = synthetic_matches()
    sampler = model.sampler
    image = np.arange(len(sampler.projection)*len(sampler.offsets), dtype=np.float32).reshape(sampler.native.shape)/10000
    sampler.native.image[:] = image
    sampler.geometry_valid[251] = False
    qs = np.array([-.8001, -.3207, .2456, .7012]); xs = np.linspace(.24, .59, 23)
    expected, mask, score = sampler.sample(0, qs, xs)
    xx, qq = np.meshgrid(xs, qs)
    actual, valid, actual_score, _ = native_points(sampler, 0, xx, qq, 1.)
    np.testing.assert_array_equal(valid, mask)
    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-5, equal_nan=True)
    np.testing.assert_allclose(actual_score, score, rtol=1e-6, atol=1e-5)
    _, valid, _, _ = native_points(sampler, 0, np.array([.45, -.5]), np.array([1.1, .2]), 1.)
    assert not valid.any()


def test_corrected_inverse_and_sampling_use_the_original_native_pixels_once():
    from ssb_tools.global_resample import inverse_points, sample_corrected
    model, _, _ = synthetic_matches()
    sampler = model.sampler
    image = np.broadcast_to(np.arange(512, dtype=np.float32)[None, :], sampler.native.shape).copy()
    sampler.native.image[:] = image
    coefficients = np.zeros(model.size)
    for k in range(4):
        coefficients[model.starts[k]:model.starts[k+1]] = [.5, -.3, .8, -.6][k]
    x = np.array([.4, .45, .51]); q = np.array([-.5, -.1, .4])
    target, supported = model.forward(0, x, q, coefficients)
    nx, nq, valid = inverse_points(model, coefficients, 0, target[:, 0], target[:, 1])
    assert valid.all() and supported.all()
    np.testing.assert_allclose(nx, x, atol=1e-8, rtol=0)
    np.testing.assert_allclose(nq, q, atol=1e-8, rtol=0)
    qs, xs = np.array([-.4, .21]), np.array([.39, .47])
    zero, valid, _ = sample_corrected(model, np.zeros(model.size), 0, qs, xs)
    expected, mask, _ = sampler.sample(0, qs, xs)
    np.testing.assert_array_equal(valid, mask)
    np.testing.assert_allclose(zero, expected, atol=1e-4, rtol=0)


@pytest.mark.parametrize('location', ['wide_gap', 'missing_row', 'band_edge'])
def test_finite_nearest_row_footprint_has_invertible_geometry_without_inventing_rows(location):
    from ssb_tools.global_resample import inverse_points, native_points
    p = np.zeros(3, PROJECTION)
    p['sequence'] = [0, 1, 2]
    p['lattice_row'] = [0, 2, 3] if location == 'missing_row' else [0, 1, 2]
    p['theta_rad'] = [-.011, 0., .011]
    p['x_axis_m'] = [.4, .401, .402]
    offsets = np.linspace(-.1, .1, 33)
    sampler = BandSampler(p, np.full((3, 33), 100., np.float32), offsets,
                          offsets, np.ones(33, bool), .01)
    model = Trajectory(sampler, 1., .7)
    c = np.zeros(model.size)
    for k, correction in enumerate([.3, -.4, .6, -.5]):
        c[model.starts[k]:model.starts[k+1]] = correction
    # Independent matrix reference uses the measured pose of the nearest row,
    # and a direction inside its footprint, not a fabricated exposure centre.
    query = -.013 if location == 'band_edge' else -.002
    row = 0 if location == 'band_edge' else 1
    expected = independent_hits(np.array([p['x_axis_m'][row]]), np.array([query]),
        np.array([.405-p['x_axis_m'][row]]),
        np.array([[.0003, -.0004, .0006, -.0005]]), 1., .7)
    actual, supported = model.forward(0, np.array([.405]), np.array([query]), c)
    assert supported.all()
    np.testing.assert_allclose(actual, expected, atol=2e-14, rtol=0)
    x, q, ok = inverse_points(model, c, 0, expected[:, 0], expected[:, 1])
    assert ok.all()
    np.testing.assert_allclose([x[0], q[0]], [.405, query], atol=1e-8, rtol=0)
    _, valid, _, sources = native_points(sampler, 0, x, q, 1.)
    assert valid.all()
    assert sources['lower_sequence'][0] == sources['upper_sequence'][0] == row
    # The centre of a real gap remains uncovered; inversion cannot fill it.
    _, _, gap_ok = inverse_points(model, np.zeros(model.size), 0,
                                   np.array([.405]), np.array([-.0055]))
    assert not gap_ok.any()


@pytest.mark.parametrize('translations', [False, True, 'heave'])
def test_obviously_wrong_inlier_matches_are_not_absorbed_as_motion(translations):
    model, table, grid = synthetic_matches(translations)
    corrupted = table.copy()
    train = np.flatnonzero((corrupted['inlier'] > 0) & (corrupted['holdout'] == 0))
    outliers = train[::17]
    # A consistent native-source identity does not make a correspondence physically right.
    corrupted['x_b_m'][outliers] += .003
    for band in (1, 2):
        selected = outliers[corrupted['band_b'][outliers] == band]
        sources = native_sources(model.sampler, band, corrupted['x_b_m'][selected],
                                 corrupted['q_b_m'][selected], model.radius)
        for key, value in sources.items(): corrupted['b_'+key][selected] = value
    _, scores, _, _, _, after, _ = fit(model, corrupted, grid)
    assert scores['heldout_after']['norm_px']['p95'] < .15
    assert np.median(np.linalg.norm(after[outliers], axis=1)/grid['dx_m']) > 10


def test_observed_knots_save_resources_without_claiming_bottom_gap_measurements():
    from ssb_tools.global_geometry import observed_spline_knots, spline_knots, curvature_stencil
    from types import SimpleNamespace
    phase=np.linspace(0,.4,101)
    p=np.zeros(34*len(phase),PROJECTION)
    bounds=[]
    for band in range(34):
        a,b=band*len(phase),(band+1)*len(phase)
        p['x_axis_m'][a:b]=band*.6+phase
        bounds.append((a,b))
    sampler=SimpleNamespace(projection=p,bounds=bounds)
    upper=float(p['x_axis_m'].max())
    knots=observed_spline_knots(sampler,0.,upper,.02)
    uniform=spline_knots(0.,upper,.02)
    assert len(knots)<.75*len(uniform)
    assert 2*(len(knots)-4)+2*(len(spline_knots(0.,upper,.6))-4)<2048
    for band in range(33):
        assert not np.any((knots>band*.6+.4+1e-10)&(knots<(band+1)*.6-1e-10))
    # A linear physical trajectory has zero curvature even beside long gaps.
    greville=np.array([knots[i+1:i+4].mean() for i in range(len(knots)-4)])
    for i in range(len(greville)-2):
        np.testing.assert_allclose(curvature_stencil(knots,i,.02) @ (3*greville[i:i+3]+2),0.,atol=1e-11)


def test_bound_active_set_can_release_a_coordinate_back_into_the_interior():
    from scipy.sparse import eye
    result,evidence=damped_solve(lambda x:x-np.array([.2,-.1]),lambda x:eye(2,format='csr'),
                                np.array([1.,-1.]),np.ones(2))
    np.testing.assert_allclose(result,[.2,-.1],atol=1e-6)
    assert evidence['active_bounds']==0
