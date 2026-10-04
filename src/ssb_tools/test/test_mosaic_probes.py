from types import SimpleNamespace

import numpy as np
import pytest

from ssb_tools.surface_relief import SurfaceRelief
from ssb_tools.validate_global_mosaic import relief_reference_positions


@pytest.mark.parametrize('direction', ['x', 'q'])
def test_patch_maximum_outside_roi_cannot_hide_positive_depth_inside_roi(direction):
    data = np.array([[.03, 0., .01], [.03, 0., .01]], np.float32)
    grid = [-1., -.1, 1., .2]
    xs, qs = np.linspace(0., 1., 11), np.linspace(-.1, .1, 3)
    if direction == 'q':
        data = data.T; grid = [-.1, -1., .2, 1.]
        xs, qs = qs, xs
    model = SimpleNamespace(radius=1., relief=SurfaceRelief([dict(grid=grid, depth=data)]))
    probes = relief_reference_positions(model, qs, xs)
    assert probes
    for row, column in probes:
        assert model.relief.depth(xs[column], qs[row]) > 0


def test_positive_vertex_outside_roi_can_still_illuminate_a_cell_centre():
    model = SimpleNamespace(radius=1., relief=SurfaceRelief([
        dict(grid=[-.5, -.5, 1., 1.], depth=np.array([[.01, 0], [0, 0]], np.float32))]))
    probes = relief_reference_positions(model, np.array([0.]), np.array([0.]))
    assert probes == {(0, 0)}


def test_patch_without_a_positive_output_cell_does_not_invent_a_probe():
    model = SimpleNamespace(radius=1., relief=SurfaceRelief([
        dict(grid=[2., 2., 1., 1.], depth=np.full((2, 2), .02, np.float32))]))
    assert not relief_reference_positions(model, np.linspace(0, 1, 11), np.linspace(0, 1, 11))


def test_probe_budget_and_selection_are_deterministic():
    patches = [dict(grid=[float(i), 0., .5, 1.], depth=np.full((2, 2), .01, np.float32)) for i in range(30)]
    model = SimpleNamespace(radius=1., relief=SurfaceRelief(patches))
    xs, qs = np.linspace(0, 30, 301), np.linspace(0, 1, 11)
    first = relief_reference_positions(model, qs, xs)
    assert len(first) == 16 and first == relief_reference_positions(model, qs, xs)
