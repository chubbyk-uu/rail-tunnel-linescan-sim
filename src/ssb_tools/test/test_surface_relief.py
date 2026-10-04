import numpy as np
import pytest

from ssb_tools.surface_relief import ReliefSettings, SurfaceRelief
from ssb_tools.global_geometry import cylinder_points
from ssb_tools.surface_relief import stereo_flow


def test_radial_hit_retains_the_nominal_circumferential_coordinate_scale():
    theta = np.array([-.3, .2]); tangent = np.array([-.1, .12]); axis = np.array([3., 3.6])
    depth = np.array([.003, .007]); radius = 2.75
    points = cylinder_points(axis, theta, tangent, np.zeros((2, 4)), radius, 1.67, depth)
    # Independent central-camera analytic ray intersection.
    np.testing.assert_allclose(points[:, 0], axis+(radius+depth)*tangent, atol=1e-12)
    np.testing.assert_allclose(points[:, 1], radius*theta, atol=1e-12)


def test_shared_surface_interpolates_and_is_zero_outside_observed_patches(tmp_path):
    data = np.array([[0., .004, .008], [.002, .006, .010]], np.float32)
    relief = SurfaceRelief([dict(grid=[3., -1., .001, .002], depth=data)])
    path = tmp_path/'surface.npz'; relief.save(path)
    restored = SurfaceRelief.load(path)
    np.testing.assert_allclose(restored.depth(np.array([3.0005, 2., 3.002]),
        np.array([-.999, -.999, -.998])), [.003, 0., .01], atol=1e-9)


@pytest.mark.parametrize('change', [dict(angular_stride=0), dict(tile_rows=512),
    dict(max_depth_m=.04), dict(archive_budget_bytes=128 << 20)])
def test_relief_resource_limits_are_explicit(change):
    with pytest.raises(ValueError):
        ReliefSettings(**change).validate()


@pytest.mark.parametrize('value', [-.001, np.nan, .031])
def test_loader_rejects_unphysical_or_nonfinite_depth(tmp_path, value):
    path = tmp_path/'bad.npz'
    np.savez(path, grids=np.array([[0., 0., .001, .001]]),
             depth_0=np.full((2, 2), value, np.float32))
    with pytest.raises(ValueError):
        SurfaceRelief.load(path)


def test_stereo_observation_recovers_signed_subpixel_motion_with_different_brightness():
    import cv2
    rng = np.random.default_rng(921)
    a = cv2.GaussianBlur(rng.uniform(50, 180, (96, 384)).astype(np.float32), (0, 0), 1.)
    yy, xx = np.indices(a.shape, dtype=np.float32)
    # Independent resampling: the same feature appears 4.25 pixels to the
    # right in B; gain/offset do not change its geometric correspondence.
    b = cv2.remap(a, xx-4.25, yy, cv2.INTER_LINEAR)*1.08+3
    flow = stereo_flow(a, b)[16:-16, 32:-32]
    assert np.median(flow[:, :, 0]) == pytest.approx(4.25, abs=.25)
    assert abs(np.median(flow[:, :, 1])) < .15
