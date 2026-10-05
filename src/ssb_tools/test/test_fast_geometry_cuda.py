"""Explicit CUDA fit backend: geometry, determinism and exceptional cleanup."""
from dataclasses import replace

import numpy as np
import pytest

from ssb_tools.fast_geometry import numeric_scope
from ssb_tools.global_geometry import Trajectory
from ssb_tools.optimize_bands import FixedJacobian, regularizer, fit
from test_global_optimization import synthetic_matches


@pytest.mark.parametrize('relative', [False, True])
@pytest.mark.parametrize('translation', [False, 'heave', 'fixed', 'yaw'])
def test_cuda_fused_geometry_matches_independent_sparse_chain_and_cpu(relative, translation):
    original, table, grid = synthetic_matches('fixed', yaw=-.0015) if translation == 'yaw' else synthetic_matches(translation)
    cpu = Trajectory(original.sampler, original.radius, original.height,
                     replace(original.settings, geometry_backend='cpu', relative_encoder_scale=relative))
    gpu = Trajectory(cpu.sampler, cpu.radius, cpu.height, replace(cpu.settings, geometry_backend='cuda'))
    c = np.sin(np.arange(cpu.size)+.3)*.7
    current = np.random.default_rng(65).uniform(.1, 2., (len(table), 2))
    pitch = np.array([grid['dx_m'], grid['dq_m']])
    a, b = (cpu.native_side(table, side) for side in ('a', 'b'))
    reference = FixedJacobian(a, b, regularizer(cpu))
    with numeric_scope():
        plan = FixedJacobian(gpu.native_side(table, 'a'), gpu.native_side(table, 'b'), regularizer(gpu))
        numeric = plan.numeric
        expected = reference(c, current, pitch).toarray()
        actual = plan(c, current, pitch).toarray()
        np.testing.assert_allclose(actual, expected, rtol=3e-12, atol=3e-11)
        np.testing.assert_array_equal(plan(c, current, pitch).toarray(), actual)
        np.testing.assert_allclose(plan.difference(c), b.hits(c)-a.hits(c), rtol=0, atol=3e-15)
        info = numeric.describe()
        assert info['allocated_peak_bytes'] < info['allocation_budget_bytes']
    assert numeric.handle is None
    with pytest.raises(RuntimeError, match='closed'):
        numeric.difference(plan.parameters(c))


def test_cuda_failure_cleans_context_without_silent_cpu_fallback():
    original, table, grid = synthetic_matches('fixed')
    model = Trajectory(original.sampler, original.radius, original.height,
                       replace(original.settings, geometry_backend='cuda'))
    numeric = None
    with pytest.raises(RuntimeError, match='CUDA public geometry'):
        with numeric_scope():
            plan = FixedJacobian(model.native_side(table, 'a'), model.native_side(table, 'b'), regularizer(model))
            numeric = plan.numeric
            current = np.ones((len(table), 2)); current[0, 0] = np.nan
            plan(np.zeros(model.size), current, np.array(list(grid.values())))
    assert numeric is not None and numeric.handle is None


def test_cuda_fit_preserves_coupled_scene_and_does_not_optimize_holdout():
    original, table, grid = synthetic_matches('fixed')
    model = Trajectory(original.sampler, original.radius, original.height,
                       replace(original.settings, geometry_backend='cuda'))
    c, scores, *_ = fit(model, table, grid)
    assert scores['heldout_after']['norm_px']['p95'] < .05
    changed = table.copy(); selected = changed['holdout'].astype(bool)
    changed['x_b_m'][selected] += .003
    from ssb_tools.match_bands import native_sources
    for band in (1, 2):
        ids = selected & (changed['band_b'] == band)
        for name, values in native_sources(model.sampler, band, changed['x_b_m'][ids], changed['q_b_m'][ids], model.radius).items():
            changed['b_'+name][ids] = values
    repeated, bad_scores, *_ = fit(model, changed, grid)
    np.testing.assert_array_equal(repeated, c)
    assert bad_scores['heldout_after']['norm_px']['p95'] > 10


def test_actual_fit_exception_releases_its_cuda_context(monkeypatch):
    from ssb_tools import optimize_bands as optimizer, fast_geometry
    original, table, grid = synthetic_matches('fixed')
    model = Trajectory(original.sampler, original.radius, original.height,
                       replace(original.settings, geometry_backend='cuda'))
    created = []
    initialize = fast_geometry.CudaJacobian.__init__
    def record(self, assembly):
        initialize(self, assembly)
        created.append(self)
    def failure(*args, **kwargs):
        raise RuntimeError('injected fit failure')
    monkeypatch.setattr(fast_geometry.CudaJacobian, '__init__', record)
    monkeypatch.setattr(optimizer, 'damped_solve', failure)
    with pytest.raises(RuntimeError, match='injected fit failure'):
        fit(model, table, grid)
    assert len(created) == 1 and created[0].handle is None
