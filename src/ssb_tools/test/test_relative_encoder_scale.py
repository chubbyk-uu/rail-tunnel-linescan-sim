"""Relative wheel scale stays separate from bounded local motion and pixel geometry."""
from dataclasses import replace

import numpy as np
import pytest
from scipy.sparse import diags, vstack

from ssb_tools.global_geometry import GeometrySettings, Trajectory
from ssb_tools.optimize_bands import FixedJacobian, regularizer, window_weights
from ssb_tools.reconstruction_support import relative_scale_reach_m
from test_global_optimization import independent_hits, synthetic_matches


def test_long_coupled_scale_is_estimated_from_correspondences_without_large_local_dx():
    from scipy.optimize import root
    from ssb_tools.initial_unroll import BandSampler, PROJECTION
    from ssb_tools.match_bands import MATCH, native_sources
    from ssb_tools.global_geometry import reconstruction_settings
    from ssb_tools.optimize_bands import fit
    phase = np.linspace(-1., 1., 41); bands = 34; offsets = np.linspace(-.42, .42, 512)
    p = np.zeros(bands*len(phase), PROJECTION)
    for band in range(bands):
        a, b = band*len(phase), (band+1)*len(phase)
        p['sequence'][a:b] = np.arange(a, b); p['segment'][a:b] = band
        p['lattice_row'][a:b] = np.arange(len(phase))+band*1000
        p['theta_rad'][a:b] = phase+2*np.pi*band
        p['x_axis_m'][a:b] = band*.6+.08*phase
    sampler = BandSampler(p, np.zeros((len(p), len(offsets)), np.float32), offsets, offsets,
                          np.ones(len(offsets), bool), .051)
    reference = (p['x_axis_m'].min()+p['x_axis_m'].max())/2
    fraction = .0125
    def observed(band, x, q):
        lo, hi, w, supported = sampler.row_sources(band, np.array([q])); assert supported.all()
        result = np.zeros(2)
        for ids, weight in ((lo, 1-w), (hi, w)):
            axis = p['x_axis_m'][ids]
            correction = np.column_stack((fraction*(axis-reference)+1e-5*(axis-reference)**2,
                np.full(1, .0007), .001+1e-4*np.sin(axis/2), -.0008+1e-4*np.cos(axis/3),
                np.full(1, .02), np.full(1, -.02)))
            hit = independent_hits(axis, p['theta_rad'][ids]-2*np.pi*band, x-axis,
                                   correction, 1., .7)
            result += weight[0]*hit[0]
        return result
    rng = np.random.default_rng(674); records = []
    for band in range(bands-1):
        for window, centre in enumerate((-.65, 0., .65)):
            for point in range(20):
                qa = centre+rng.uniform(-.04, .04)
                xa = band*.6+.08*qa+.3+rng.uniform(-.03, .03)
                hit = observed(band, xa, qa)
                sol = root(lambda xy: observed(band+1, *xy)-hit, [xa, qa], tol=1e-9)
                assert np.linalg.norm(sol.fun) < 1e-9
                row = np.zeros(1, MATCH); row['window'] = band*3+window
                row['band_a'] = band; row['band_b'] = band+1; row['inlier'] = 1
                row['holdout'] = point % 5 == 0; row['ncc'] = .9
                for side, k, x, q in (('a', band, xa, qa), ('b', band+1, *sol.x)):
                    row['x_'+side+'_m'] = x; row['q_'+side+'_m'] = q
                    for name, value in native_sources(sampler, k, np.array([x]), np.array([q]), 1.).items():
                        row[side+'_'+name] = value
                records.append(row)
    model = Trajectory(sampler, 1., .7, reconstruction_settings(.1, False, True, True))
    c, scores, *_ = fit(model, np.concatenate(records), dict(dx_m=.0002, dq_m=.0002))
    assert model.domain[1]-model.domain[0] > 19
    assert scores['heldout_after']['norm_px']['p95'] < .2
    assert abs(c[-1]*model.scale-fraction) < .0002
    assert abs(c[:model.sizes[0]]).max() < 30
    assert abs(c[-1]*model.scale)*(model.domain[1]-model.domain[0])/2 > .1


def model_and_matches():
    old, table, grid = synthetic_matches('fixed')
    return Trajectory(old.sampler, old.radius, old.height,
                      replace(old.settings, relative_encoder_scale=True)), table, grid


@pytest.mark.parametrize('fraction', [-.0125, .0125])
def test_coupled_scale_rays_and_sparse_jacobian_against_independent_reference(fraction):
    model, table, grid = model_and_matches()
    c = np.linspace(-.3, .4, model.size)
    c[model.scale_index] = fraction/model.scale
    rays = model.native_side(table[:12], 'a')
    local = np.column_stack([b @ c[model.starts[i]:model.starts[i+1]]
                             for i, b in enumerate(rays.bases)])*model.scale
    local[:, 0] += fraction*(rays.axis-model.scale_reference_m)
    expected = independent_hits(rays.axis, rays.theta, rays.tangent, local, model.radius, model.height)
    np.testing.assert_allclose(rays.hits(c), (expected.reshape(-1, 4, 2)*rays.weights[..., None]).sum(1),
                               rtol=0, atol=2e-14)
    for direction, jac in enumerate(rays.jacobian(c)):
        step = np.zeros(model.size); step[model.scale_index] = 1e-3
        numerical = (rays.hits(c+step)-rays.hits(c-step))/(2e-3)
        np.testing.assert_allclose(jac[:, model.scale_index].toarray().ravel(), numerical[:, direction],
                                   rtol=0, atol=2e-13)
    train, weights, _ = window_weights(table, grid, model.settings)
    a, b = model.native_side(table[train], 'a'), model.native_side(table[train], 'b')
    prior = regularizer(model); pitch = np.array([grid['dx_m'], grid['dq_m']])
    ja, jb = a.jacobian(c), b.jacobian(c)
    rows = vstack([diags(weights[train, d]/pitch[d]) @ (jb[d]-ja[d]) for d in range(2)], format='csr')
    order = np.column_stack((np.arange(len(table[train])), np.arange(len(table[train]))+len(table[train]))).ravel()
    expected = vstack((rows[order], prior), format='csr')
    actual = FixedJacobian(a, b, prior)(c, weights[train], pitch)
    assert abs(actual-expected).max() < 1e-9


def test_scale_adds_one_coefficient_and_removes_duplicate_local_linear_mode():
    model, _, _ = model_and_matches()
    old = Trajectory(model.sampler, model.radius, model.height, replace(model.settings, relative_encoder_scale=False))
    assert model.size == old.size+1
    assert model.sizes == old.sizes
    g = np.array([model.knots[0][i+1:i+4].mean() for i in range(model.sizes[0])])
    local = np.zeros(model.size); local[:model.sizes[0]] = g-g.mean()
    global_only = np.zeros(model.size); global_only[-1] = 1.
    assert np.linalg.norm(regularizer(model) @ local) > 1000
    assert np.linalg.norm(regularizer(model) @ global_only) == pytest.approx(.1)
    assert model.coefficient_bounds()[-1] == 30.


def test_twenty_metre_drift_exceeds_local_bounds_but_cache_and_serialization_keep_it():
    base, _, _ = synthetic_matches('fixed')
    base.sampler.projection['x_axis_m'] *= 30
    model = Trajectory(base.sampler, base.radius, base.height,
                      replace(base.settings, relative_encoder_scale=True, observed_knots=True))
    c = np.zeros(model.size); c[-1] = 12.5
    ids = np.array([0, len(model.sampler.projection)-1])
    expected = .0125*(model.sampler.projection['x_axis_m'][ids]-model.scale_reference_m)
    assert abs(expected).min() > .09
    np.testing.assert_array_equal(model.ray_parameters(ids, c)[:, 0], expected)
    model.prepare_ray_cache(c)
    np.testing.assert_array_equal(model.ray_parameters(ids, c)[:, 0], expected)
    record = model.serialize(c)
    assert record['schema'] == 'ssb.global_trajectory.v3'
    assert len(record['coefficients']) == model.spline_size
    assert record['relative_encoder_scale']['correction_fraction'] == .0125
    assert model.source_reach_m() > model.scale_margin_m() > .2


@pytest.mark.parametrize('settings', [GeometrySettings(relative_encoder_scale=1),
    GeometrySettings(relative_scale_bound_fraction=.031), GeometrySettings(relative_scale_prior_fraction=.04)])
def test_invalid_relative_scale_declaration_is_rejected(settings):
    with pytest.raises(ValueError, match='relative encoder scale'):
        settings.validate()


@pytest.mark.parametrize('domain,bound', [([0, 20], .03), ([8, 11], .03), ([0, 20], 0.)])
def test_public_support_margin_scales_with_progress_interval(domain, bound):
    assert relative_scale_reach_m(domain, bound) == bound*(domain[1]-domain[0])/2
