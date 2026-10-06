"""Robust reweighting must report whether it settled instead of hiding the iteration cap."""
from dataclasses import replace

import numpy as np
import pytest

from ssb_tools.global_geometry import Trajectory
from ssb_tools.match_bands import native_sources
from ssb_tools.optimize_bands import IRLS_WEIGHT_TOLERANCE, fit, irls_summary
from test_global_optimization import synthetic_matches


def corrupted(max_irls):
    model, table, grid = synthetic_matches()
    model = Trajectory(model.sampler, model.radius, model.height, replace(model.settings, max_irls=max_irls))
    table = table.copy()
    train = np.flatnonzero(table['inlier'].astype(bool) & ~table['holdout'].astype(bool))
    outliers = train[::17]
    # A few accepted but wrong correspondences give the robust weights real work;
    # their native-source identity stays consistent, as in the D2 outlier test.
    table['x_b_m'][outliers] += .003
    for band in (1, 2):
        selected = outliers[table['band_b'][outliers] == band]
        sources = native_sources(model.sampler, band, table['x_b_m'][selected], table['q_b_m'][selected], model.radius)
        for key, value in sources.items():
            table['b_'+key][selected] = value
    return model, table, grid


def test_reaching_the_cap_while_weights_still_move_is_reported_as_not_converged():
    model, table, grid = corrupted(1)
    *_, history, _, _, _ = fit(model, table, grid)
    summary = irls_summary(history, model.settings.max_irls)
    assert summary['iterations'] == len(history) == 1
    assert history[0]['max_relative_weight_change'] > IRLS_WEIGHT_TOLERANCE
    assert summary['converged'] is False and summary['max_irls'] == 1


def test_settled_weights_are_reported_as_converged_before_the_cap():
    model, table, grid = corrupted(10)
    *_, history, _, _, _ = fit(model, table, grid)
    summary = irls_summary(history, model.settings.max_irls)
    assert summary['converged'] is True and summary['iterations'] < 10
    assert summary['final_max_relative_weight_change'] == history[-1]['max_relative_weight_change']
    assert all(item['max_relative_weight_change'] >= IRLS_WEIGHT_TOLERANCE for item in history[:-1])


def test_a_longer_budget_does_not_change_the_iterations_already_taken():
    """Reporting only: the first k solves are identical whatever the cap."""
    short, table, grid = corrupted(2)
    longer, *_ = corrupted(10)
    first = fit(short, table, grid)
    second = fit(longer, table, grid)
    assert first[3] == second[3][:2]


@pytest.mark.parametrize('history', [[], [dict(cost=1.)]])
def test_history_without_recorded_weight_changes_is_rejected(history):
    with pytest.raises(ValueError):
        irls_summary(history, 5)
