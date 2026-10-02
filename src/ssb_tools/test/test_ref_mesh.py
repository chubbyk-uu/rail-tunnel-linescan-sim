"""Truth-side optical mesh reference against independent facet-plane intersections."""
import math
import numpy as np
import pytest
from ssb_tools.ref_mesh import OpticalMesh
from ssb_tools.evaluate_band_matches import signed_summary

AXIS_Z = 2.015


def faceted_lining(facets=720, radius=2.75, x=(-1., 3.), recess=None):
    """Closed faceted cylinder; optional recessed strip [x0, x1] x [a0, a1] at extra depth."""
    angles = np.linspace(-math.pi, math.pi, facets+1)
    vertices, faces, material = [], [], []
    for a, b in zip(angles[:-1], angles[1:]):
        base = len(vertices)
        for xx, ang in ((x[0], a), (x[0], b), (x[1], b), (x[1], a)):
            vertices.append([xx, radius*math.sin(ang), AXIS_Z+radius*math.cos(ang)])
        faces += [[base, base+1, base+2], [base, base+2, base+3]]; material += [0, 0]
    if recess:
        (x0, x1), (a0, a1), depth = recess
        base = len(vertices); r = radius+depth
        for xx, ang in ((x0, a0), (x0, a1), (x1, a1), (x1, a0)):
            vertices.append([xx, r*math.sin(ang), AXIS_Z+r*math.cos(ang)])
        faces += [[base, base+1, base+2], [base, base+2, base+3]]; material += [2, 2]
    return np.array(vertices), np.array(faces), np.array(material)


def plane_hit(origin, direction, p0, p1, p2):
    normal = np.cross(p1-p0, p2-p0)
    t = (normal@(p0-origin))/(normal@direction)
    return origin+t*direction


def test_faceted_hits_match_independent_plane_intersections():
    vertices, faces, material = faceted_lining()
    mesh = OpticalMesh(vertices, faces, material, AXIS_Z)
    rng = np.random.default_rng(4)
    origins = np.column_stack([rng.uniform(0, 2, 200), rng.uniform(-.02, .02, 200), AXIS_Z+rng.uniform(-.02, .02, 200)])
    phi = rng.uniform(-2.1, 2.1, 200)
    directions = np.column_stack([rng.uniform(-.15, .15, 200), np.sin(phi), np.cos(phi)])
    points, ids, materials = mesh.intersect(origins, directions)
    for origin, direction, point, tri in zip(origins, directions, points, ids):
        expected = plane_hit(origin, direction, *vertices[faces[tri]])
        np.testing.assert_allclose(point, expected, atol=1e-12)
        # Independent facet choice: the hit lies in the facet's angular span.
        theta = math.atan2(point[1], point[2]-AXIS_Z)
        corners = np.arctan2(vertices[faces[tri], 1], vertices[faces[tri], 2]-AXIS_Z)
        assert corners.min()-1e-12 <= theta <= corners.max()+1e-12
    assert np.all(materials == 0)
    # Chord sag: facet hits are inside the circumscribed cylinder, by at most R(1-cos(step/2)).
    r = np.hypot(points[:, 1], points[:, 2]-AXIS_Z)
    assert np.all(r <= 2.75+1e-12) and np.all(r >= 2.75*math.cos(math.pi/720)-1e-12)


def strip(radius, x=(.9, 1.1), angles=(-.05, .05)):
    """Two triangles of a flat strip whose edges lie on a cylinder of `radius`."""
    (x0, x1), (a0, a1) = x, angles
    corners = [(x0, a0), (x0, a1), (x1, a1), (x1, a0)]
    vertices = np.array([[xx, radius*math.sin(a), AXIS_Z+radius*math.cos(a)] for xx, a in corners])
    return vertices, np.array([[0, 1, 2], [0, 2, 3]])


def test_nearest_surface_wins_and_recess_reports_its_material():
    origin = np.array([[1., 0., AXIS_Z]]); up = np.array([[0., 0., 1.]])
    recess_v, recess_f = strip(2.78)
    alone = OpticalMesh(recess_v, recess_f, [2, 2], AXIS_Z)
    point, _, mat = alone.intersect(origin, up)
    assert mat[0] == 2 and abs(point[0, 2]-AXIS_Z-2.78*math.cos(.05)) < 1e-12  # flat strip below its edges
    lining_v, lining_f = strip(2.75, x=(0., 2.), angles=(-.1, .1))
    both = OpticalMesh(np.vstack([recess_v, lining_v]), np.vstack([recess_f, lining_f+4]), [2, 2, 0, 0], AXIS_Z)
    point, _, mat = both.intersect(origin, up)
    assert mat[0] == 0 and abs(point[0, 2]-AXIS_Z-2.75*math.cos(.1)) < 1e-12


def test_a_ray_without_a_triangle_is_rejected():
    vertices, faces, material = faceted_lining(x=(0., 1.))
    mesh = OpticalMesh(vertices, faces, material, AXIS_Z)
    with pytest.raises(ValueError, match='misses'):
        mesh.intersect(np.array([[5., 0., AXIS_Z]]), np.array([[0., 0., 1.]]))


def test_signed_summary_separates_common_bias_from_spread():
    table = np.zeros(400, dtype=[('window', '<i4'), ('band_a', '<i2'), ('a_lower_column', '<f8')])
    table['window'] = np.repeat(np.arange(4), 100); table['band_a'] = np.repeat([0, 0, 1, 1], 100)
    table['a_lower_column'] = np.linspace(3000, 4000, 400)
    rng = np.random.default_rng(1); pitch = 2e-4
    delta = np.column_stack([.26+rng.normal(0, .1, 400), rng.normal(0, .05, 400)])*pitch
    summary = signed_summary(delta, table, pitch)
    assert abs(summary['mean_px']['dx']-.26) < .02 and abs(summary['mean_px']['dq']) < .01
    assert .2 < summary['window_mean_px']['dx']['min'] <= summary['window_mean_px']['dx']['max'] < .32
    assert summary['window_mean_standard_error_px'] < .02
    assert set(summary['adjacent_pair_mean_px']) == {'0', '1'}
    assert sum(c['matches'] for c in summary['by_a_native_column']) == 400
