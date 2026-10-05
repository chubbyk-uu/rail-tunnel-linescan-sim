import numpy as np
import pytest
from dataclasses import replace

from ssb_tools.global_geometry import cylinder_points, cylinder_derivatives, Trajectory
from ssb_tools.optimize_bands import fit
from ssb_tools.global_geometry import GeometrySettings
from test_global_optimization import independent_hits, synthetic_matches


def test_nonzero_yaw_ray_and_jacobian_against_independent_rotations():
    rng = np.random.default_rng(5020)
    axis = rng.uniform(0, 20, 25)
    theta = rng.uniform(-2, 2, 25)
    tangent = rng.uniform(-.15, .15, 25)
    local = rng.uniform(-1, 1, (25, 7))*[.002, .003, .004, .003, .02, .02, .002]
    actual = cylinder_points(axis, theta, tangent, local, 2.75, 1.7)
    np.testing.assert_allclose(actual, independent_hits(axis, theta, tangent, local, 2.75, 1.7), atol=1e-13)
    derivative = cylinder_derivatives(axis, theta, tangent, local, 2.75, 1.7)
    for field in range(7):
        plus, minus = local.copy(), local.copy()
        plus[:, field] += 1e-7
        minus[:, field] -= 1e-7
        numerical = (independent_hits(axis, theta, tangent, plus, 2.75, 1.7)-
                     independent_hits(axis, theta, tangent, minus, 2.75, 1.7))/(2e-7)
        np.testing.assert_allclose(derivative[:, field], numerical, atol=3e-8, rtol=1e-5)


def test_zero_yaw_preserves_old_ray_results_exactly():
    rng = np.random.default_rng(5021)
    local = rng.uniform(-1, 1, (20, 6))*[.002, .003, .004, .003, .02, .02]
    args = (np.arange(20)*.5, np.linspace(-2, 2, 20), np.linspace(-.1, .1, 20))
    np.testing.assert_array_equal(cylinder_points(*args, local, 2.75, 1.7),
                                  cylinder_points(*args, np.c_[local, np.zeros(20)], 2.75, 1.7))


def test_constant_yaw_adds_one_parameter_and_is_image_fitted():
    model, table, grid = synthetic_matches('fixed', yaw=-.0015)
    assert model.fields[-1] == 'axis_yaw_rad' and model.sizes[-1] == 1
    assert model.degrees[-1] == 0
    assert model.size == model.spline_size
    basis = model.bases(np.linspace(*model.domain, 8))[-1]
    np.testing.assert_array_equal(basis.toarray(), np.ones((8, 1)))
    coefficients, scores, *rest = fit(model, table, grid)
    assert scores['heldout_after']['norm_px']['p95'] < .1
    fitted = coefficients[model.starts[-2]]*model.scale
    # Nominal priors and weak modes bias the estimate: this is an image-fitted
    # ray correction, not an independently measured mechanical mounting yaw.
    assert -.003 < fitted < -.00075
    record = model.serialize(coefficients)
    assert record['schema'] == 'ssb.global_trajectory.v4' and record['field_degrees'][-1] == 0
    restored = Trajectory(model.sampler, model.radius, model.height, model.settings, record['knots'])
    assert restored.sizes == model.sizes
    model.prepare_ray_cache(coefficients)
    local = model.ray_parameters(np.array([0, 100, 300]), coefficients)
    np.testing.assert_allclose(local[:, -1], fitted, atol=1e-15)


def test_axis_yaw_cannot_be_silently_changed_into_a_time_varying_field():
    original, _, _ = synthetic_matches('fixed')
    settings = replace(original.settings, fit_axis_yaw=True)
    knots = original.knots+[np.array([original.domain[0], sum(original.domain)/2, original.domain[1]])]
    with pytest.raises(ValueError, match='one constant'):
        Trajectory(original.sampler, original.radius, original.height, settings, knots)


@pytest.mark.parametrize('settings', [GeometrySettings(fit_axis_yaw=True),
    GeometrySettings(fit_translation=True, fit_axis_yaw=1),
    GeometrySettings(fit_translation=True, fit_axis_yaw=True, axis_yaw_bound_mrad=11.)])
def test_yaw_requires_a_declared_bounded_coupled_model(settings):
    with pytest.raises(ValueError):
        settings.validate()
