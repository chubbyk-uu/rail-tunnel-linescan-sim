import numpy as np
import pytest

from ssb_tools.evaluate_defect_preservation import path_samples, intensity_width, saved_samples


def test_path_samples_use_arclength_and_preserve_long_curve_and_duplicates():
    p, normal, length = path_samples([[0,0],[0,0],[3,0],[3,4]], [0,.5,1])
    assert length == 7
    np.testing.assert_allclose(p, [[0,0],[3,.5],[3,4]])
    np.testing.assert_allclose(np.linalg.norm(normal,axis=1), 1)


def test_apparent_width_of_known_dip_and_low_contrast_are_distinguished():
    x = np.linspace(-.002,.002,81)
    sigma = .0002/(2*np.sqrt(2*np.log(2)))
    profile = 150-40*np.exp(-x*x/(2*sigma*sigma))
    result = intensity_width(x,profile)
    assert result['status']=='measured'
    assert result['apparent_fwhm_mm'] == pytest.approx(.2,abs=.003)
    result = intensity_width(x,150-2*np.exp(-x*x/(2*sigma*sigma)))
    assert result['status']=='unmeasurable'


def test_saved_samples_respect_quantization_grid_and_invalid_neighbours():
    im = (64*np.array([[10,20,30],[30,40,50],[50,60,70]])).astype(np.uint16)
    grid = dict(target_x_m=[0,3],theta_rad=[0,3],radius_m=1,dx_m=1,dq_m=1)
    result = saved_samples(im,grid,np.array([[.5,.5],[1.,1.],[-.1,1.]]))
    np.testing.assert_allclose(result[:2],[10,25])
    assert np.isnan(result[2])
    im[0,1]=65535
    assert np.isnan(saved_samples(im,grid,np.array([[1.,1.]]))).all()


@pytest.mark.parametrize('case',['nan','flat','edge'])
def test_missing_or_unmeasurable_profiles_never_become_width_passes(case):
    x = np.linspace(-.002,.002,81)
    p = np.full(81,150.)
    if case=='nan':
        p[5]=np.nan
    elif case=='edge':
        p[:45]=100
    assert intensity_width(x,p)['status']=='unmeasurable'


@pytest.mark.parametrize('path', [[], [[0,0],[0,0]], [[0,0],[np.nan,1]]])
def test_degenerate_path_cannot_create_continuity_evidence(path):
    with pytest.raises(ValueError):
        path_samples(path,[.5])
