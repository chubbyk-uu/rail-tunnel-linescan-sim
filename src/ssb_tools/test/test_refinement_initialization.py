"""Knot insertion preserves coupled geometry and uses only prior training fits."""
from dataclasses import replace

import numpy as np
import pytest
from scipy.interpolate import BSpline

from ssb_tools.global_geometry import Trajectory, refined_attitude_coefficients
from ssb_tools.optimize_bands import fit
from test_global_optimization import independent_hits, synthetic_matches


def refined_pair(translations=False, relative=False):
    original, table, grid = synthetic_matches(translations)
    source = Trajectory(original.sampler, original.radius, original.height,
                        replace(original.settings, relative_encoder_scale=relative))
    knots = [k.copy() for k in source.knots]
    inserted = np.linspace(*source.domain, 9)[1:-1]
    for field in (2, 3):
        knots[field] = np.sort(np.r_[knots[field], inserted, inserted[2]])
    target = Trajectory(source.sampler, source.radius, source.height, source.settings, knots)
    return source, target, table, grid


@pytest.mark.parametrize('translations', [False, True, 'heave', 'fixed'])
@pytest.mark.parametrize('relative', [False, True])
def test_refined_curves_and_native_hits_match_independent_coupled_geometry(translations, relative):
    source, target, table, _ = refined_pair(translations, relative)
    coefficients = np.sin(np.arange(source.size)+.4)*1.7
    original = coefficients.copy()
    seed = refined_attitude_coefficients(source, target, coefficients)
    axis = np.linspace(*source.domain, 79)
    local = np.column_stack([BSpline(knots, coefficients[source.starts[k]:source.starts[k+1]], 3)(axis)
                             for k, knots in enumerate(source.knots)])*source.scale
    if relative:
        local[:, 0] += source.scale*coefficients[source.scale_index]*(axis-source.scale_reference_m)
        assert seed[target.scale_index] == coefficients[source.scale_index]
    expected = independent_hits(axis, np.linspace(-1., 1., len(axis)),
                                np.linspace(-.17, .2, len(axis)), local, source.radius, source.height)
    actual = target.points(axis, np.linspace(-1., 1., len(axis)),
                           np.linspace(-.17, .2, len(axis)), seed)
    np.testing.assert_allclose(actual, expected, atol=2e-14, rtol=0)
    for side in ('a', 'b'):
        np.testing.assert_allclose(target.native_side(table, side).hits(seed),
                                   source.native_side(table, side).hits(coefficients), atol=2e-14, rtol=0)
    np.testing.assert_array_equal(coefficients, original)
    for field in range(len(source.fields)):
        if field not in (2, 3):
            np.testing.assert_array_equal(seed[target.starts[field]:target.starts[field+1]],
                                          coefficients[source.starts[field]:source.starts[field+1]])
    for derivative in (1, 2):
        for field in (2, 3):
            old = BSpline(source.knots[field], coefficients[source.starts[field]:source.starts[field+1]], 3)
            new = BSpline(target.knots[field], seed[target.starts[field]:target.starts[field+1]], 3)
            np.testing.assert_allclose(new(axis, nu=derivative), old(axis, nu=derivative), rtol=2e-12, atol=2e-11)


@pytest.mark.parametrize('bad', ['shape', 'nan', 'bounds', 'removed_knot', 'dx_knots', 'model'])
def test_invalid_refinement_inputs_are_rejected(bad):
    source, target, _, _ = refined_pair()
    coefficients = np.zeros(source.size)
    if bad == 'shape': coefficients = coefficients[:-1]
    elif bad == 'nan': coefficients[0] = np.nan
    elif bad == 'bounds': coefficients[0] = 31.
    elif bad == 'removed_knot': target.knots[2] = target.knots[2][1:]
    elif bad == 'dx_knots': target.knots[0] = np.sort(np.r_[target.knots[0], sum(source.domain)/2])
    elif bad == 'model': target.height += .001
    with pytest.raises(ValueError):
        refined_attitude_coefficients(source, target, coefficients)


def test_refined_fit_starts_from_training_curve_and_never_fits_holdout(monkeypatch):
    import ssb_tools.optimize_bands as optimize
    source, target, table, grid = refined_pair('fixed')
    coefficients, *_ = fit(source, table, grid)
    initial = refined_attitude_coefficients(source, target, coefficients)
    saved = initial.copy()
    solve = optimize.damped_solve
    starts = []
    def record(fun, jac, current, *args, **kwargs):
        starts.append(current.copy())
        return solve(fun, jac, current, *args, **kwargs)
    monkeypatch.setattr(optimize, 'damped_solve', record)
    fitted, scores, *_ = fit(target, table, grid, initial=initial)
    np.testing.assert_array_equal(starts[0], initial)
    np.testing.assert_array_equal(initial, saved)
    assert scores['training_before']['norm_px']['p95'] > 10  # still the zero-correction baseline
    assert scores['heldout_after']['norm_px']['p95'] < .05
    changed = table.copy()
    selected = changed['holdout'].astype(bool)
    # Corrupt only the held-out x column; keep its native-source geometry consistent.
    from ssb_tools.match_bands import native_sources
    changed['x_b_m'][selected] += .003
    for band in (1, 2):
        ids = selected & (changed['band_b'] == band)
        for name, values in native_sources(source.sampler, band, changed['x_b_m'][ids],
                                            changed['q_b_m'][ids], source.radius).items():
            changed['b_'+name][ids] = values
    repeated, bad_scores, *_ = fit(target, changed, grid, initial=initial)
    np.testing.assert_array_equal(repeated, fitted)
    assert bad_scores['heldout_after']['norm_px']['p95'] > 10


@pytest.mark.parametrize('bad', ['shape', 'nan', 'bounds'])
def test_fit_rejects_invalid_initial_coefficients(bad):
    model, table, grid = synthetic_matches()
    initial = np.zeros(model.size)
    if bad == 'shape': initial = initial[:-1]
    elif bad == 'nan': initial[0] = np.nan
    else: initial[0] = 31.
    with pytest.raises(ValueError, match='initial'):
        fit(model, table, grid, initial=initial)
