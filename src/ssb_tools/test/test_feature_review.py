import numpy as np
import pytest

from ssb_tools.feature_review import line_candidates, raw_overview, search_spacing, overview_stride
from ssb_tools.match_bands import WINDOW_BUDGET
from ssb_tools.initial_unroll import BandSampler, PROJECTION
from ssb_tools.native_rows import MemoryRows


@pytest.mark.parametrize('nx,bands', [(15000, 8), (100000, 35), (250000, 85)])
def test_review_budget_scales_without_changing_pixel_or_matching_resolution(nx, bands):
    import math
    grid = dict(theta_rad=[-2*np.pi/3, 2*np.pi/3], radius_m=2.75, dq_m=.0002)
    spacing = search_spacing(grid, bands-1)
    count = math.ceil((np.diff(grid['theta_rad'])[0]*grid['radius_m']-511*grid['dq_m'])/spacing)
    assert spacing >= .09 and count*(bands-1) <= WINDOW_BUDGET
    stride = overview_stride([57596, nx])
    assert math.ceil(57596/stride)*math.ceil(nx/stride) <= 1 << 20
    assert math.ceil(57596/stride)*4096 <= 8 << 20
    if nx == 15000:
        assert spacing == .09 and stride == 32  # Existing 3 m review is unchanged.


def helix_fixture():
    """A stationary axial mark observed by a sensor advancing 1 mm per row."""
    rows, width = 81, 201
    projection = np.zeros(rows, PROJECTION)
    projection['sequence'] = np.arange(rows)
    projection['lattice_row'] = np.arange(rows)
    projection['theta_rad'] = np.linspace(-.4, .4, rows)
    projection['x_axis_m'] = np.arange(rows)*.001
    offsets = (np.arange(width)-100)*.001
    raw = np.full((rows, width), 180, np.uint8)
    # Fixed wall x=.05 m: native column moves left as the robot moves forward.
    raw[np.arange(rows), 150-np.arange(rows)] = 30
    flat = dict(offset=np.zeros(width), gain=np.ones(width), valid=np.ones(width, bool))
    return BandSampler(projection, MemoryRows(raw, flat), offsets, offsets,
                       np.ones(width, bool), .01001)


def test_raw_helix_preserves_actual_pixels_and_nominal_unroll_removes_slant():
    sampler = helix_fixture()
    angles = sampler.phases[0]
    xs = sampler.offsets
    # If raw display accidentally calls corrected gather/rows, this must fail.
    def forbidden(*args):
        raise AssertionError('raw preview must not apply flat correction')
    sampler.native.gather = forbidden
    raw, valid, records = raw_overview(sampler, angles, xs, xs)
    np.testing.assert_array_equal(raw, sampler.native.raw_rows)
    assert valid.all() and records[0]['observed_travel_m'] == pytest.approx(.08)
    np.testing.assert_array_equal(np.argmin(raw, axis=1), 150-np.arange(81))

    sampler = helix_fixture()
    nominal, supported, _ = sampler.sample(0, angles, np.linspace(.04, .06, 21))
    assert supported.all()
    np.testing.assert_array_equal(np.argmin(nominal, axis=1), np.full(81, 10))


def test_raw_preview_keeps_unsupported_angles_masked_and_enforces_budget():
    sampler = helix_fixture()
    raw, valid, _ = raw_overview(sampler, np.array([-.8, 0., .8]), sampler.offsets, sampler.offsets)
    assert not valid[[0, 2]].any() and valid[1].all()
    assert not raw[[0, 2]].any()
    with pytest.raises(ValueError, match='megapixel'):
        raw_overview(sampler, np.zeros(2048), np.zeros(2048), sampler.offsets)
    with pytest.raises(ValueError, match='increasing'):
        raw_overview(sampler, np.array([0.]), sampler.offsets, sampler.offsets[::-1])
    with pytest.raises(ValueError, match='8 MiB'):
        raw_overview(sampler, np.zeros(50000), np.array([0.]), sampler.offsets)


@pytest.mark.parametrize('thickness,kind', [(2, 'thin'), (7, 'thin'), (20, 'wide')])
def test_image_locator_finds_long_structures_but_not_round_pores(thickness, kind):
    image = np.full((512, 1024), 180., np.float32)
    image[250:250+thickness, 180:840] = 65.
    rng = np.random.default_rng(3)
    for x, y in rng.integers([20, 20], [1000, 190], size=(60, 2)):
        image[y:y+3, x:x+3] = 50.
    candidates = line_candidates(image, np.ones(image.shape, bool))
    assert len(candidates) == 1 and candidates[0]['kind'] == kind
    assert candidates[0]['bbox_px'][2] >= 650
    assert 250 <= candidates[0]['centre_px'][1] <= 250+thickness


def test_image_locator_does_not_select_invalid_boundary_or_blank_image():
    image = np.full((512, 1024), 180., np.float32)
    valid = np.ones(image.shape, bool)
    valid[240:260] = False
    image[240:260] = np.nan
    assert line_candidates(image, valid) == []
    assert line_candidates(image, np.zeros(image.shape, bool)) == []
    with pytest.raises(ValueError, match='bounded'):
        line_candidates(np.zeros((1025, 1025)), np.ones((1025, 1025), bool))
    image[100, 100] = np.nan
    with pytest.raises(ValueError, match='finite'):
        line_candidates(image, valid)


def test_structure_ending_exactly_at_seam_search_boundary_is_not_indexed_outside():
    image = np.full((512, 1024), 180., np.float32)
    image[250:252, :256] = 60.
    # The feature ends just before the admissible centre region. Clamping the
    # centre to column 256 would index one column beyond this component.
    assert line_candidates(image, np.ones(image.shape, bool)) == []


@pytest.mark.parametrize('fail', [False, True])
def test_feature_search_closes_cuda_on_success_and_exception(monkeypatch, fail):
    import ssb_tools.feature_review as module
    closed = []
    class Raster:
        def __init__(self, sampler):
            self.sampler = sampler
        def close(self):
            closed.append(self)
    monkeypatch.setattr(module, 'CudaRaster', Raster)
    sampler = object()
    def search():
        with module.feature_raster(sampler) as raster:
            assert raster.sampler is sampler
            if fail:
                raise RuntimeError('injected feature search failure')
    if fail:
        with pytest.raises(RuntimeError, match='injected'):
            search()
    else:
        search()
    assert len(closed) == 1
