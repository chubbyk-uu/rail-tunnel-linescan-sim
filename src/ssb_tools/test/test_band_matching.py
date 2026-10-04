"""Known image deformations and independent ray-cylinder body tilt tests."""
import cv2
import numpy as np
import pytest
from scipy.ndimage import map_coordinates
from ssb_tools.band_matching import match_window, MatchSettings


def texture(seed=9, shape=(768, 1024)):
    rng = np.random.default_rng(seed)
    fine = cv2.GaussianBlur(rng.normal(size=shape).astype(np.float32), (0, 0), 1.)
    broad = cv2.GaussianBlur(rng.normal(size=shape).astype(np.float32), (0, 0), 5.)
    return (135+fine*28+broad*55).astype(np.float32)


@pytest.mark.parametrize('affine', [np.array([[1., 0., 11.3], [0., 1., -7.6]]),
    np.array([[1.004, .009, -16.2], [-.006, .997, 13.4]])])
def test_subpixel_matches_and_local_affine_are_measured_not_supplied(affine):
    a = texture(shape=(512, 768)); mask = np.ones(a.shape, np.uint8)
    b = cv2.warpAffine(a, affine, (a.shape[1], a.shape[0]), flags=cv2.INTER_LINEAR)*1.08+3
    mb = cv2.warpAffine(mask, affine, (a.shape[1], a.shape[0]), flags=cv2.INTER_NEAREST).astype(bool)
    report, matches = match_window(a, b, mask, mb, .0002)
    assert report['status'] == 'accepted', report
    selected = matches['inlier']
    expected = matches['points_a'][selected]@affine[:, :2].T+affine[:, 2]
    error = np.linalg.norm(expected-matches['points_b'][selected], axis=1)
    assert np.percentile(error, 95) < .35
    # Nontranslation test must retain deformation, rather than fitting it away.
    if affine[0, 1]: assert abs(np.asarray(report['affine_a_to_b_px'])[0, 1]) > .005


@pytest.mark.parametrize('kind', ['constant', 'stripes', 'unrelated', 'masked'])
def test_weak_repeated_unrelated_or_missing_support_does_not_produce_constraints(kind):
    a = texture(shape=(512, 768)); b = a.copy(); ma = np.ones(a.shape, bool); mb = ma.copy()
    if kind == 'constant': a.fill(120); b.fill(130)
    if kind == 'stripes':
        a[:] = 125+12*np.sin(np.arange(a.shape[1])[None, :]*2*np.pi/32); b[:] = np.roll(a, 8, axis=1)
    if kind == 'unrelated': b = texture(seed=101, shape=a.shape)
    if kind == 'masked': mb[:, :a.shape[1]//3] = False
    report, matches = match_window(a, b, ma, mb, .0002)
    assert report['status'] == 'unmeasurable', report
    assert len(matches['points_a']) == 0


def ray_map(x, q, axis_x, roll, pitch, radius=2.75, mast=1.715):
    # Independently assemble body Ry(pitch) Rx(roll) and solve cylinder roots.
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    rotation = np.array([[cp, sp*sr, sp*cr], [0., cr, -sr], [-sp, cp*sr, cp*cr]])
    theta = q/radius
    axis = axis_x+.6*theta/(2*np.pi)
    rays = np.stack([(x-axis)/radius, np.sin(theta), np.cos(theta)], axis=-1)@rotation.T
    origin = rotation@np.array([0., 0., mast])-np.array([0., 0., mast])
    dy, dz = rays[..., 1], rays[..., 2]
    quadratic = dy*dy+dz*dz
    linear = 2*(origin[1]*dy+origin[2]*dz)
    constant = origin[1]**2+origin[2]**2-radius**2
    distance = (-linear+np.sqrt(linear**2-4*quadratic*constant))/(2*quadratic)
    hit_x = axis+origin[0]+distance*rays[..., 0]
    hit_q = radius*np.arctan2(origin[1]+distance*dy, origin[2]+distance*dz)
    return hit_x, hit_q


@pytest.mark.parametrize('roll_pitch', [(.0012, -.001, -.0008, .0013),
                                       (.002, .003, -.001, -.002)])
def test_small_body_roll_and_pitch_with_mast_motion_and_helical_line_geometry(roll_pitch):
    pitch_m = .0002; height, width = 512, 768; centre_q = 2.75*.85
    xs = 1.+(np.arange(width)-(width-1)/2)*pitch_m
    qs = centre_q+(np.arange(height)-(height-1)/2)*pitch_m
    x, q = np.meshgrid(xs, qs)
    source = texture(seed=17, shape=(1200, 1600))
    def image(axis, roll, pitch):
        hx, hq = ray_map(x, q, axis, roll, pitch)
        return map_coordinates(source, [(hq-centre_q)/pitch_m+600, (hx-1.)/pitch_m+800],
                               order=1, mode='constant', cval=np.nan).astype(np.float32)
    ra, pa, rb, pb = roll_pitch
    a, b = image(.6, ra, pa), image(1.2, rb, pb)
    report, matches = match_window(a, b, np.isfinite(a), np.isfinite(b), pitch_m)
    assert report['status'] == 'accepted', report
    good = matches['inlier']
    def world(points, axis, roll, pitch):
        return ray_map(xs[0]+points[:, 0]*pitch_m, qs[0]+points[:, 1]*pitch_m, axis, roll, pitch)
    ax, aq = world(matches['points_a'][good], .6, ra, pa)
    bx, bq = world(matches['points_b'][good], 1.2, rb, pb)
    # Truth is used only here to score image-derived correspondences.
    error = np.hypot(ax-bx, aq-bq)/pitch_m
    assert np.percentile(error, 95) < .5, np.percentile(error, [50, 95, 100])


def test_invalid_window_settings_are_rejected():
    a = texture(shape=(64, 64)); mask = np.ones(a.shape, bool)
    with pytest.raises(ValueError): match_window(a, a, mask, mask, 0.)
    with pytest.raises(ValueError): match_window(a, a, mask, mask, .0002, MatchSettings(coarse_factor=0))
    with pytest.raises(ValueError): match_window(a, a, mask[:, :1], mask, .0002)
    with pytest.raises(ValueError): MatchSettings(max_shift_mm=float('nan'))


@pytest.mark.parametrize('recessed', [False, True])
def test_short_windows_keep_large_axial_search_with_explicit_smaller_angular_prior(recessed):
    a = texture(seed=31, shape=(256, 1024))
    if recessed:
        a[:, 470:550] = 75+(a[:, 470:550]-135)*.5
    affine = np.array([[1., 0., 103.3], [0., 1., -7.6]])
    b = cv2.warpAffine(a, affine, (1024, 256), flags=cv2.INTER_LINEAR)*1.08+3
    valid = np.ones(a.shape, bool)
    mb = cv2.warpAffine(valid.astype(np.uint8), affine, (1024, 256), flags=cv2.INTER_NEAREST).astype(bool)
    unsupported, _ = match_window(a, b, valid, mb, .0002)
    assert unsupported['status'] == 'unmeasurable'
    assert unsupported['reason'] == 'search range exceeds window support'
    report, matches = match_window(a, b, valid, mb, .0002, MatchSettings(max_q_shift_mm=10.))
    assert report['status'] == 'accepted', report
    assert report['search_limit_xq_px'] == [200., 50.]
    good = matches['inlier']
    expected = matches['points_a'][good]@affine[:, :2].T+affine[:, 2]
    assert np.percentile(np.linalg.norm(expected-matches['points_b'][good], axis=1),95) < .35


def test_separate_angular_prior_is_enforced_even_when_axial_search_allows_the_shift():
    a = texture(seed=17, shape=(512, 1024))
    affine = np.array([[1., 0., 11.3], [0., 1., 45.]])
    b = cv2.warpAffine(a, affine, (1024, 512), flags=cv2.INTER_LINEAR)
    valid = np.ones(a.shape, bool)
    mb = cv2.warpAffine(valid.astype(np.uint8), affine, (1024, 512), flags=cv2.INTER_NEAREST).astype(bool)
    report, matches = match_window(a, b, valid, mb, .0002, MatchSettings(max_q_shift_mm=8.))
    assert report['status'] == 'unmeasurable' and not len(matches['points_a'])


@pytest.mark.parametrize('limit', [0., -1., np.nan, np.inf])
def test_invalid_separate_angular_prior_is_rejected(limit):
    with pytest.raises(ValueError):
        MatchSettings(max_q_shift_mm=limit)


def test_holdout_residual_failure_is_distinguished_from_insufficient_points():
    a = texture(shape=(512, 768))
    affine = np.array([[1., 0., 11.3], [0., 1., -7.6]])
    b = cv2.warpAffine(a, affine, (a.shape[1], a.shape[0]), flags=cv2.INTER_LINEAR)*1.08+3
    valid = np.ones(a.shape, bool)
    valid_b = cv2.warpAffine(valid.astype(np.uint8), affine, (a.shape[1], a.shape[0]),
                            flags=cv2.INTER_NEAREST).astype(bool)
    report, matches = match_window(a, b, valid, valid_b, .0002,
                                  MatchSettings(max_holdout_p95_px=1e-5))
    assert report['status'] == 'unmeasurable'
    assert report['reason'] == 'independent holdout or inlier support failed'
    assert len(matches['points_a']) == 0
    assert report['heldout'] >= 5 and report['inliers'] >= 24
    assert report['support_checks']['holdout_count']
    assert report['support_checks']['inlier_count']
    assert not report['support_checks']['holdout_p95']
    assert len(report['spatial_covariance_eigenvalues']) == 2


@pytest.mark.parametrize('shift', [(.25, 0.), (.5, 0.), (0., .5), (12.25, -3.5), (50.5, 7.25)])
def test_mean_translation_is_unbiased_for_known_subpixel_shifts(shift):
    # Band-limited spline shift of the same content: any residual mean offset is
    # the matcher's own bias (pixel locking), not image formation or geometry.
    from scipy.ndimage import shift as spline_shift
    sx, sy = shift; pad = 64
    big = texture(seed=21, shape=(512+2*pad, 1024+2*pad)).astype(np.float64)
    a = big[pad:-pad, pad:-pad].astype(np.float32)
    b = spline_shift(big, (sy, sx), order=5, mode='nearest')[pad:-pad, pad:-pad].astype(np.float32)
    mask = np.ones(a.shape, bool)
    report, matches = match_window(a, b, mask, mask, .0002)
    assert report['status'] == 'accepted', report
    inlier = matches['inlier']
    bias = (matches['points_b'][inlier]-matches['points_a'][inlier]).mean(axis=0)-np.array([sx, sy])
    # Phase-dependent locking stays near 0.03 px; the D2 shading bias was 0.26 px.
    assert np.all(abs(bias) < .05), bias
