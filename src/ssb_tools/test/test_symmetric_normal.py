"""Symmetric normal solves retain independent solutions and box KKT conditions."""
import itertools

import numpy as np
import pytest
from scipy.sparse import csc_matrix

from ssb_tools.optimize_bands import symmetric_normal_solve, bounded_normal_step


@pytest.mark.parametrize('size', [1, 7, 43])
@pytest.mark.parametrize('scaled', [False, True])
def test_symmetric_solve_matches_independent_known_solution(size, scaled):
    rng = np.random.default_rng(312)
    a = rng.normal(size=(size, size))
    normal = a.T@a+np.eye(size)*.2
    scale = np.geomspace(1e-4, 1e4, size) if scaled else np.ones(size)
    normal *= scale[:, None]*scale[None, :]
    expected = rng.normal(size=size)/scale
    solution = symmetric_normal_solve(csc_matrix(normal), normal@expected)
    np.testing.assert_allclose(solution, expected, atol=1e-9, rtol=2e-11)


def independent_box_minimum(matrix, gradient, lower, upper):
    """Enumerate faces in a tiny strictly convex problem; no production active set."""
    best = None
    for membership in itertools.product((-1, 0, 1), repeat=len(gradient)):
        fixed = np.flatnonzero(np.asarray(membership) != 0)
        free = np.flatnonzero(np.asarray(membership) == 0)
        x = np.zeros(len(gradient))
        x[fixed] = np.where(np.asarray(membership)[fixed] < 0, lower[fixed], upper[fixed])
        if len(free):
            x[free] = np.linalg.solve(matrix[np.ix_(free, free)],
                                     -gradient[free]-matrix[np.ix_(free, fixed)]@x[fixed])
        if np.any(x < lower-1e-10) or np.any(x > upper+1e-10):
            continue
        cost = .5*x@matrix@x+gradient@x
        if best is None or cost < best[0]: best = cost, x
    assert best is not None
    return best[1]


@pytest.mark.parametrize('seed', [14, 24, 35, 47])
def test_bounded_step_matches_independent_face_enumeration(seed):
    rng = np.random.default_rng(seed)
    a = rng.normal(size=(7, 4)); normal = a.T@a+np.eye(4)*.01
    gradient = rng.normal(size=4)*10
    bounds = np.array([.4, 1., .2, .7]); current = rng.uniform(-1, 1, 4)*bounds
    damping = .002
    matrix = normal+np.diag(damping*np.maximum(normal.diagonal(), 1e-8))
    expected = independent_box_minimum(matrix, gradient, -bounds-current, bounds-current)
    actual = bounded_normal_step(csc_matrix(normal), gradient, current, bounds, damping)
    np.testing.assert_allclose(actual, expected, atol=1e-9, rtol=1e-10)
    derivative = matrix@actual+gradient
    active = (np.isclose(actual, -bounds-current, atol=1e-9) & (derivative >= 0)) | (
        np.isclose(actual, bounds-current, atol=1e-9) & (derivative <= 0))
    assert abs(derivative[~active]).max(initial=0) < 1e-8


def test_inaccurate_finite_factorization_is_rejected(monkeypatch):
    import ssb_tools.optimize_bands as module
    class WrongFactor:
        def solve(self, rhs): return np.ones_like(rhs)
    monkeypatch.setattr(module, 'splu', lambda *a, **k: WrongFactor())
    with pytest.raises(ValueError, match='backward-error'):
        symmetric_normal_solve(csc_matrix(np.eye(3)), np.zeros(3))
