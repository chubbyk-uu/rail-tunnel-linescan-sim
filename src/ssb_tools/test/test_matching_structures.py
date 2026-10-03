import cv2
import numpy as np
import pytest

from ssb_tools.band_matching import MatchSettings, match_window
from ssb_tools.matching_structures import (long_dark_mask, column_nanmedian, unsafe_structure_footprint,
    safe_structure_points, masked_coarse_shift, LK_STRUCTURE_RADIUS)
from test_band_matching import texture


@pytest.mark.parametrize('centre', [160, 512, 900])
def test_recessed_band_has_independent_disparity_but_background_matches_remain_correct(centre):
    a = texture(seed=31, shape=(512, 1024))
    affine = np.array([[1., 0., 11.3], [0., 1., -7.6]])
    b = cv2.warpAffine(a, affine, (1024, 512), flags=cv2.INTER_LINEAR)*1.08+3
    valid = np.ones(a.shape, np.uint8)
    valid_b = cv2.warpAffine(valid, affine, (1024, 512), flags=cv2.INTER_NEAREST).astype(bool)
    # A depth-discontinuous feature has its own 6 px parallax. Background is a
    # known affine independently supplied by this fixture, never by the matcher.
    left, right = centre-40, centre+40
    a[:, left:right] = 75+(a[:, left:right]-135)*.5
    extra = affine.copy(); extra[0, 2] += 6
    recessed = cv2.warpAffine(a, extra, (1024, 512), flags=cv2.INTER_LINEAR)*1.08+3
    shifted_left, shifted_right = int(round(left+17.3)), int(round(right+17.3))
    b[:, shifted_left:shifted_right] = recessed[:, shifted_left:shifted_right]
    original_a, original_b = a.copy(), b.copy()
    report, matches = match_window(a, b, valid, valid_b, .0002)
    assert report['status'] == 'accepted', report
    assert all(r['bands_px'] for r in report['structure_masks'])
    inlier = matches['inlier']
    expected = matches['points_a'][inlier]@affine[:, :2].T+affine[:, 2]
    assert np.percentile(np.linalg.norm(expected-matches['points_b'][inlier], axis=1),95) < .35
    # Every correspondence's full pyramid footprint, not just its centre, must
    # remain outside the independently known recessed stripe on both images.
    for points, lo, hi in ((matches['points_a'],left,right),
                          (matches['points_b'],shifted_left,shifted_right)):
        assert np.all((points[:,0] < lo-LK_STRUCTURE_RADIUS) |
                      (points[:,0] >= hi+LK_STRUCTURE_RADIUS))
    np.testing.assert_array_equal(a, original_a); np.testing.assert_array_equal(b, original_b)


def test_masking_does_not_create_a_match_in_a_uniform_background():
    a = np.full((512,1024),135.,np.float32); b=a.copy()
    a[:,450:530]=70; b[:,460:540]=70
    valid=np.ones(a.shape,bool)
    report,matches=match_window(a,b,valid,valid,.0002)
    assert report['status']=='unmeasurable' and len(matches['points_a'])==0


@pytest.mark.parametrize('kind', ['pores','thin_crack','invalid_stripe'])
def test_pores_fine_cracks_and_missing_data_are_not_long_recessed_bands(kind):
    image=texture(shape=(512,1024)); valid=np.ones(image.shape,bool)
    if kind=='pores':image[200:230,400:430]=60
    if kind=='thin_crack':image[:,510:513]=60
    if kind=='invalid_stripe':image[:,470:550]=np.nan;valid[:,470:550]=False
    mask,report=long_dark_mask(image,valid,.0002)
    assert not mask.any() and not report['bands_px']


def test_footprint_checks_subpixel_positions_nonfinite_points_and_outside_image():
    mask=np.zeros((256,1024),bool);mask[:,500:580]=True
    unsafe=unsafe_structure_footprint(mask)
    points=np.array([[200.,100.],[500-LK_STRUCTURE_RADIUS-.5,100.],
                     [500.,100.],[-1.,100.],[np.nan,100.]])
    np.testing.assert_array_equal(safe_structure_points(unsafe,points),[True,False,False,False,False])


def test_explicit_ablation_keeps_legacy_matches_when_no_structure_is_detected():
    a=texture(shape=(512,768));valid=np.ones(a.shape,bool)
    enabled,em=match_window(a,a,valid,valid,.0002)
    disabled,dm=match_window(a,a,valid,valid,.0002,MatchSettings(exclude_long_structures=False))
    assert enabled['status']==disabled['status']=='accepted'
    assert not any(r['bands_px'] for r in enabled['structure_masks'])
    for key in em:np.testing.assert_array_equal(em[key],dm[key])


def test_masked_correlation_refuses_no_common_supported_background():
    a=texture(shape=(256,512));va=np.zeros(a.shape,bool);vb=va.copy()
    va[:,:60]=True;vb[:,-60:]=True
    shift,report=masked_coarse_shift(a,a,va,vb,10,MatchSettings(max_shift_mm=2.))
    assert shift is None and 'support' in report['reason']


def test_invalid_detector_settings_are_refused():
    with pytest.raises(ValueError):MatchSettings(exclude_long_structures=1)
    with pytest.raises(ValueError):long_dark_mask(np.zeros((4,4)),np.ones((4,4)),0)


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
def test_column_median_is_bit_identical_to_nanmedian(dtype):
    rng = np.random.default_rng(5)
    for rows in (511, 512):
        values = rng.normal(120, 9, (rows, 300)).astype(dtype)
        values[:, 40] = values[0, 40]                       # ties
        values[rng.random(values.shape) < .02] = np.nan     # partially missing columns
        values[:, :25] = rng.normal(120, 9, (rows, 25)).astype(dtype)  # complete columns
        profile = column_nanmedian(values)
        expected = np.nanmedian(values, axis=0)
        assert profile.dtype == expected.dtype
        np.testing.assert_array_equal(profile, expected)
