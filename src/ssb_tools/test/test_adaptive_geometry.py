"""Training-only local refinement: support, holdout isolation and resource caps."""
from types import SimpleNamespace

import numpy as np
import pytest

from ssb_tools.global_geometry import GeometrySettings, spline_knots
from ssb_tools.initial_unroll import PROJECTION
from ssb_tools.match_bands import MATCH
from ssb_tools.optimize_bands import refine_attitude_knots


def fixture(size=20):
    axes = np.arange(.005, .08, .01)
    projection = np.zeros(2*len(axes), PROJECTION)
    projection['sequence'] = np.arange(len(projection))
    projection['x_axis_m'] = np.repeat(axes, 2)+np.tile([0., .001], len(axes))
    table = np.zeros(len(axes), MATCH)
    table['window'] = np.arange(len(axes))
    table['inlier'] = 1
    table['a_lower_sequence'] = 2*np.arange(len(axes))
    table['b_lower_sequence'] = table['a_lower_sequence']+1
    knots = [spline_knots(0., .08, s) for s in (.6, .6, .04, .04)]
    model = SimpleNamespace(knots=knots, size=size, sampler=SimpleNamespace(projection=projection))
    delta = np.column_stack((np.r_[np.ones(4), np.full(4, 2.)], np.zeros(8)))*.0002
    grid = dict(dx_m=.0002, dq_m=.0002)
    return model, table, delta, grid


def test_supported_coherent_training_error_refines_only_attitude_and_preserves_inputs():
    model, table, delta, grid = fixture()
    originals = [k.copy() for k in model.knots]
    knots, report = refine_attitude_knots(model, table, delta, grid, .007)
    np.testing.assert_allclose(report['inserted_progress_m'], [.02, .06])
    assert report['after_coefficients'] == model.size+4
    assert report['training_only'] and not report['budget_deferred_progress_m']
    for i in range(4):
        np.testing.assert_array_equal(model.knots[i], originals[i])
    for i in (0, 1):
        np.testing.assert_array_equal(knots[i], originals[i])
    np.testing.assert_array_equal(knots[2], knots[3])


def test_holdout_and_rejected_rows_cannot_trigger_or_support_refinement():
    model, table, delta, grid = fixture()
    delta[:] = 0.
    table['holdout'][::2] = 1
    table['inlier'][1::2] = 0
    delta[:] = np.nan  # No training data: neither selection nor source lookup may touch these rows.
    table['a_lower_sequence'] = -1
    _, report = refine_attitude_knots(model, table, delta, grid, .007)
    assert not report['triggered_windows'] and not report['inserted_progress_m']


def test_many_points_in_one_window_do_not_supply_independent_child_support():
    model, table, delta, grid = fixture()
    table['window'][:4] = 0
    table['window'][4:] = 1
    table = np.repeat(table, 40)
    delta = np.repeat(delta, 40, axis=0)
    _, report = refine_attitude_knots(model, table, delta, grid, .007)
    assert not report['inserted_progress_m']


def test_insufficient_observed_spacing_prevents_finer_knots():
    model, table, delta, grid = fixture()
    _, report = refine_attitude_knots(model, table, delta, grid, .021)
    assert not report['inserted_progress_m']


@pytest.mark.parametrize('size,expected', [(2044, [.02, .06]), (2046, [.06]), (2048, [])])
def test_budget_allocates_paired_knots_by_training_severity_and_reports_deferral(size, expected):
    model, table, delta, grid = fixture(size)
    _, report = refine_attitude_knots(model, table, delta, grid, .007)
    np.testing.assert_allclose(report['inserted_progress_m'], expected)
    assert report['after_coefficients'] <= 2048
    assert len(report['budget_deferred_progress_m']) == 2-len(expected)
    # Record ordering and feature density must not change which physical knots receive budget.
    order = np.arange(len(table))[::-1]
    _, shuffled = refine_attitude_knots(model, table[order], delta[order], grid, .007)
    assert shuffled['inserted_progress_m'] == report['inserted_progress_m']


@pytest.mark.parametrize('spacing', [0., -1., np.nan, np.inf])
def test_invalid_support_spacing_is_rejected(spacing):
    model, table, delta, grid = fixture()
    with pytest.raises(ValueError, match='spacing'):
        refine_attitude_knots(model, table, delta, grid, spacing)


def test_missing_training_source_is_rejected():
    model, table, delta, grid = fixture()
    table['a_lower_sequence'][0] = 999
    with pytest.raises(ValueError, match='exposure'):
        refine_attitude_knots(model, table, delta, grid, .007)


@pytest.mark.parametrize('settings', [GeometrySettings(adaptive_attitude=1),
    GeometrySettings(adaptive_attitude=True),
    GeometrySettings(adaptive_attitude=True, observed_knots=True, fit_heave=True),
    GeometrySettings(adaptive_attitude=True, observed_knots=True, fit_translation=True)])
def test_refinement_requires_boolean_switch_and_four_field_observed_model(settings):
    with pytest.raises(ValueError, match='adaptive'):
        settings.validate()
