from copy import deepcopy
import numpy as np
import pytest
from ssb_tools.initial_unroll import grid_axes
from ssb_tools.review_grid import display_grid, overview_stride
from ssb_tools import feature_review, global_resample


def source_grid():
    return dict(target_x_m=[0.,20.],theta_rad=[-2*np.pi/3,2*np.pi/3],radius_m=2.75,
                shape=[57596,100000],dx_m=.0002,dq_m=.0002)


def test_both_review_producers_share_the_bounded_stride_at_twenty_metres():
    assert feature_review.overview_stride is global_resample.overview_stride
    stride=overview_stride(source_grid()['shape'])
    assert np.ceil(57596/stride)*np.ceil(100000/stride)<=1<<20
    assert stride>56  # Old global review exceeded the common megapixel budget.


def test_display_subset_preserves_exact_source_grid_centres_and_full_target():
    source=source_grid();before=deepcopy(source)
    subset=display_grid(source,[8,11])
    assert source==before and subset['shape']==[57596,15000]
    np.testing.assert_array_equal(grid_axes(subset)[0],grid_axes(source)[0])
    np.testing.assert_allclose(grid_axes(subset)[1],grid_axes(source)[1][40000:55000],atol=2e-15)
    assert overview_stride(subset['shape'])==32


@pytest.mark.parametrize('bounds',[[-1,1],[19,21],[11,8],[8,8],[np.nan,9],[8.00001,11],[]])
def test_invalid_display_range_never_mutates_original_grid(bounds):
    grid=source_grid();before=deepcopy(grid)
    with pytest.raises(ValueError):display_grid(grid,bounds)
    assert grid==before


def test_default_display_copy_cannot_mutate_upstream_acceptance_metadata():
    original=source_grid();copy=display_grid(original);copy['target_x_m'][0]=9
    assert original['target_x_m']==[0,20]
