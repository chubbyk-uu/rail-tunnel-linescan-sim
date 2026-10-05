"""Fused native geometry against independent rotations and sparse algebra."""
from dataclasses import replace

import numpy as np
import pytest

from ssb_tools.fast_geometry import ray_values
from ssb_tools.global_geometry import Trajectory, cylinder_derivatives
from ssb_tools.optimize_bands import FixedJacobian, regularizer
from test_global_optimization import independent_hits, synthetic_matches


@pytest.mark.parametrize('fields', [4, 5, 6, 7])
def test_native_rays_and_analytic_derivatives_match_independent_coupled_geometry(fields):
    rng = np.random.default_rng(230)
    axis = rng.uniform(-2.5, 52.5, 41)
    phase = rng.uniform(-2.2, 2.2, 41)
    tangent = rng.uniform(-.2, .2, 41)
    correction = rng.uniform(-1, 1, (41, fields))*.005
    expected = independent_hits(axis, phase, tangent, correction, 2.75, 1.715)
    np.testing.assert_allclose(ray_values(axis, phase, tangent, correction, 2.75, 1.715),
                               expected, rtol=0, atol=3e-14)
    analytic = ray_values(axis, phase, tangent, correction, 2.75, 1.715, True)
    np.testing.assert_allclose(analytic, cylinder_derivatives(axis, phase, tangent, correction, 2.75, 1.715),
                               rtol=2e-13, atol=2e-14)
    for field in range(fields):
        plus, minus = correction.copy(), correction.copy()
        plus[:, field] += 1e-6; minus[:, field] -= 1e-6
        derivative = (independent_hits(axis, phase, tangent, plus, 2.75, 1.715)-
                      independent_hits(axis, phase, tangent, minus, 2.75, 1.715))/(2e-6)
        np.testing.assert_allclose(analytic[:, field], derivative, rtol=2e-6, atol=8e-9)


@pytest.mark.parametrize('relative', [False, True])
@pytest.mark.parametrize('translation', [False, 'heave', 'fixed', 'yaw'])
def test_fused_sparse_jacobian_matches_separate_sparse_chain_and_finite_differences(relative, translation):
    original, table, grid = synthetic_matches('fixed', yaw=-.0015) if translation == 'yaw' else synthetic_matches(translation)
    model = Trajectory(original.sampler, original.radius, original.height,
                        replace(original.settings, geometry_backend='cpu', relative_encoder_scale=relative))
    a, b = (model.native_side(table, side) for side in ('a', 'b'))
    prior = regularizer(model)
    plan = FixedJacobian(a, b, prior)
    coefficients = np.sin(np.arange(model.size))*.7
    current = np.random.default_rng(53).uniform(.1, 2., (len(table), 2))
    pitch = np.array([grid['dx_m'], grid['dq_m']])
    actual = plan(coefficients, current, pitch)
    np.testing.assert_allclose(plan.difference(coefficients), b.hits(coefficients)-a.hits(coefficients),
                               rtol=0, atol=3e-15)
    ja, jb = a.jacobian(coefficients), b.jacobian(coefficients)
    expected = np.vstack([((jb[d]-ja[d]).toarray()*current[:, d, None]/pitch[d]) for d in range(2)])
    interleaved = np.empty_like(expected)
    interleaved[0::2], interleaved[1::2] = expected[:len(table)], expected[len(table):]
    np.testing.assert_allclose(actual.toarray(), np.vstack((interleaved, prior.toarray())), rtol=3e-12, atol=2e-11)
    reference = Trajectory(model.sampler, model.radius, model.height,
                           replace(model.settings, geometry_backend='numpy'))
    aa, bb = (reference.native_side(table, side) for side in ('a', 'b'))
    separate = FixedJacobian(aa, bb, prior)(coefficients, current, pitch)
    np.testing.assert_allclose(actual.toarray(), separate.toarray(), rtol=3e-12, atol=2e-11)
    for column in [model.starts[2]+2, model.starts[3]+2, *([model.scale_index] if relative else [])]:
        plus, minus = coefficients.copy(), coefficients.copy()
        plus[column] += 1e-4; minus[column] -= 1e-4
        derivative = (((bb.hits(plus)-aa.hits(plus))-(bb.hits(minus)-aa.hits(minus)))*current/pitch/(2e-4)).ravel()
        np.testing.assert_allclose(actual[:2*len(table), column].toarray().ravel(), derivative,
                                   rtol=2e-5, atol=3e-6)


@pytest.mark.parametrize('bad', ['nan', 'radius', 'height', 'shape', 'miss'])
def test_invalid_native_ray_geometry_is_rejected(bad):
    axis, phase, tangent = np.zeros(3), np.ones(3), np.zeros(3)
    correction = np.zeros((3, 6)); radius, height = 2.75, 1.715
    if bad == 'nan': correction[0, 2] = np.nan
    elif bad == 'radius': radius = -1.
    elif bad == 'height': height = np.inf
    elif bad == 'shape': correction = correction[:, :3]
    else: correction[:, 5] = 20.
    with pytest.raises(ValueError): ray_values(axis, phase, tangent, correction, radius, height)


def test_fused_kernel_failure_and_bad_row_ranges_never_return_partial_jacobian():
    model, table, grid = synthetic_matches('fixed')
    plan = FixedJacobian(model.native_side(table, 'a'), model.native_side(table, 'b'), regularizer(model))
    current = np.ones((len(table), 2)); current[0, 0] = np.nan
    with pytest.raises(ValueError): plan(np.zeros(model.size), current, np.array(list(grid.values())))
    local = [model.parameters(np.zeros(model.size), side.bases, side.axis) for side in plan.sides]
    with pytest.raises(ValueError):
        plan.numeric.fill(-1, 2, local, current, np.array(list(grid.values())), np.empty(2*plan.nnz))


def test_exact_coefficient_cache_does_not_reuse_mutated_or_different_trials():
    model, table, _ = synthetic_matches('fixed')
    plan = FixedJacobian(model.native_side(table, 'a'), model.native_side(table, 'b'), regularizer(model))
    coefficients = np.zeros(model.size)
    before = plan.difference(coefficients).copy()
    coefficients[model.starts[3]+2] = 1.
    changed = plan.difference(coefficients).copy()
    assert np.max(abs(before-changed)) > 1e-6
    coefficients[:] = 0.
    np.testing.assert_array_equal(plan.difference(coefficients), before)
