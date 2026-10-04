"""Mesh compaction must preserve the optical surface and its texture mapping."""
import math

import numpy as np
import pytest

from ssb_tools.stage_b_scene import Mesh


@pytest.mark.parametrize('reverse', [False, True])
def test_triangle_uses_three_vertices_without_changing_winding_or_uv(tmp_path, reverse):
    points = [(0., 0., 3.), (1., 0., 3.), (1., 1., 3.)]
    patch = points + [points[-1]]
    if reverse:
        patch.reverse()
    counters = dict(vertices=0, triangles=0)
    path = tmp_path/'triangle.obj'
    mesh = Mesh(path, counters, dict(max_scene_vertices=3, max_mesh_triangles=1), (0., 2., 0.))
    mesh.quad(patch)
    mesh.close()
    lines = path.read_text().splitlines()
    vertices = np.array([[float(x) for x in line.split()[1:]] for line in lines if line.startswith('v ')])
    normals = np.array([[float(x) for x in line.split()[1:]] for line in lines if line.startswith('vn ')])
    uv = np.array([[float(x) for x in line.split()[1:]] for line in lines if line.startswith('vt ')])
    expected = points[::-1] if reverse else points
    np.testing.assert_allclose(vertices, expected, atol=5e-10)
    normal = np.cross(vertices[1]-vertices[0], vertices[2]-vertices[0])
    normal /= np.linalg.norm(normal)
    np.testing.assert_allclose(normals, np.tile(normal, (3, 1)), atol=5e-10)
    expected_uv = [[x/2, (math.atan2(y, z)+math.pi)/(2*math.pi)] for x, y, z in expected]
    np.testing.assert_allclose(uv, expected_uv, atol=5e-10)
    assert [line for line in lines if line.startswith('f ')] == ['f 1/1/1 2/2/2 3/3/3']
    assert counters == dict(vertices=3, triangles=1)


def test_real_triangle_budget_is_still_enforced(tmp_path):
    counters = dict(vertices=0, triangles=0)
    mesh = Mesh(tmp_path/'quad.obj', counters, dict(max_scene_vertices=8, max_mesh_triangles=1))
    with pytest.raises(ValueError, match='resource budget'):
        mesh.quad([(0., 0., 0.), (1., 0., 0.), (1., 1., 0.), (0., 1., 0.)])
    mesh.close()
    assert counters == dict(vertices=0, triangles=0)
