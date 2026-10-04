import numpy as np
import pytest

from ssb_tools.global_geometry import GeometrySettings, Trajectory
from ssb_tools.initial_unroll import BandSampler, PROJECTION
from ssb_tools.global_resample import sample_corrected
from ssb_tools.surface_relief import SurfaceRelief


def fixture():
    phases = np.linspace(-.5, .5, 101)
    p = np.zeros(202, PROJECTION)
    for band in (0, 1):
        sl = slice(101*band, 101*(band+1))
        p['sequence'][sl] = np.arange(sl.start, sl.stop)
        p['segment'][sl] = band
        p['lattice_row'][sl] = np.arange(101)+band*1000
        p['theta_rad'][sl] = phases+band*2*np.pi
        p['x_axis_m'][sl] = .4+.2*band+.04*phases
    offsets = np.linspace(-.4, .4, 513)
    raw = np.random.default_rng(32).uniform(0, 250, (202, 513)).astype(np.float32)
    sampler = BandSampler(p, raw, offsets, offsets, np.ones(513, bool), .01001)
    model = Trajectory(sampler, 1., .7, GeometrySettings(attitude_spacing_m=.1))
    coefficients = np.random.default_rng(18).uniform(-.7, .7, model.size)
    return model, coefficients


@pytest.mark.parametrize('relief', [False, True])
def test_cached_inverse_pixels_masks_and_scores_are_bit_identical_with_coupled_pose(relief):
    model, c = fixture()
    if relief:
        model.relief = SurfaceRelief([dict(grid=[.3, -.4, .3, .4],
            depth=np.array([[.003, .001, .005], [.004, .008, .002], [.009, .006, .003]], np.float32))])
    qs, xs = np.linspace(-.3, .3, 73), np.linspace(.4, .6, 91)
    before = sample_corrected(model, c, 0, qs, xs)
    allocated = model.prepare_ray_cache(c)
    assert 0 < allocated < 256 << 20
    after = sample_corrected(model, c, 0, qs, xs)
    for a, b in zip(before, after):
        np.testing.assert_array_equal(a, b)


@pytest.mark.parametrize('change', ['coefficients', 'progress', 'knots'])
def test_changed_inputs_cannot_use_stale_ray_parameters(change):
    model, c = fixture()
    model.prepare_ray_cache(c)
    ids = np.arange(101)
    if change == 'coefficients':
        c[0] += .3
    elif change == 'progress':
        model.sampler.projection['x_axis_m'][0] += .00001
    else:
        model.knots[0][-4:] += .00001
    cached = model.ray_parameters(ids, c)
    model._ray_cache = None
    direct = model.ray_parameters(ids, c)
    np.testing.assert_array_equal(cached, direct)


def test_cache_is_bounded_and_saved_inputs_are_read_only():
    model, c = fixture()
    assert model.prepare_ray_cache(c, 1) == 0
    assert model._ray_cache is None
    model.prepare_ray_cache(c)
    assert all(not array.flags.writeable for array in model._ray_cache[:3])
    with pytest.raises(ValueError):
        model.prepare_ray_cache(c, 512 << 20)


def test_cache_does_not_hide_invalid_mutated_knots():
    model, c = fixture()
    model.prepare_ray_cache(c)
    model.knots[0][4] += .00001  # deliberately makes a clamped upper end unsorted
    with pytest.raises(ValueError, match='non-decreasing'):
        model.ray_parameters(np.arange(101), c)
