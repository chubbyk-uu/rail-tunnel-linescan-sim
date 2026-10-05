"""Public fit-invariant reuse preserves noise and both optimization passes."""
from dataclasses import replace

import numpy as np
import pytest

from ssb_tools import optimize_bands as optimize
from ssb_tools.global_geometry import refined_attitude_coefficients
from test_global_optimization import synthetic_matches
from test_refinement_initialization import refined_pair


def test_stable_groups_preserve_old_full_scan_weights_with_shuffled_rows(monkeypatch):
    model, table, grid = synthetic_matches()
    np.random.default_rng(5).shuffle(table)
    grouped = optimize.window_weights(table, grid, model.settings)
    monkeypatch.setattr(optimize, 'window_groups', lambda t: [
        (int(w), np.flatnonzero(t['window'] == w)) for w in np.unique(t['window'])])
    old = optimize.window_weights(table, grid, model.settings)
    for actual, expected in zip(grouped[:2], old[:2]):
        np.testing.assert_array_equal(actual, expected)
    assert grouped[2] == old[2]


def test_refinement_reuses_noise_but_resets_robust_weights_and_matches_fresh_fit(monkeypatch):
    source, target, table, grid = refined_pair('fixed', True)
    original = optimize.window_weights
    calls = []
    def record(*args):
        calls.append(1)
        return original(*args)
    monkeypatch.setattr(optimize, 'window_weights', record)
    prepared = optimize.PreparedFit.prepare(source, table, grid)
    coarse = optimize.fit(source, table, grid, prepared=prepared)
    initial = refined_attitude_coefficients(source, target, coarse[0])
    refined = optimize.fit(target, table, grid, initial=initial, prepared=prepared)
    assert len(calls) == 1
    fresh = optimize.fit(target, table, grid, initial=initial)
    np.testing.assert_array_equal(refined[0], fresh[0])
    np.testing.assert_array_equal(refined[5], fresh[5])
    assert refined[1:4] == fresh[1:4]


@pytest.mark.parametrize('changed', ['copy', 'content', 'grid', 'settings', 'bands'])
def test_cached_observations_cannot_be_reused_with_changed_inputs(changed):
    model, table, grid = synthetic_matches()
    prepared = optimize.PreparedFit.prepare(model, table, grid)
    if changed == 'copy': table = table.copy()
    elif changed == 'content': table['x_a_m'][0] += .0001
    elif changed == 'grid': grid = dict(grid, dx_m=.0003)
    elif changed == 'settings': model.settings = replace(model.settings, noise_floor_px=.06)
    else: model.sampler.segments = model.sampler.segments[:-1]
    with pytest.raises(ValueError, match='observations changed'):
        optimize.fit(model, table, grid, prepared=prepared)
