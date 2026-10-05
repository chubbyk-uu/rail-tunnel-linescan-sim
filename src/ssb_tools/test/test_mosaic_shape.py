import numpy as np
import pytest

from ssb_tools.evaluate_mosaic_shape import summarize


def grid():
    xs, qs = np.linspace(0, 20, 33), np.linspace(-5, 5, 49)
    xx, qq = np.meshgrid(xs, qs)
    return np.stack((xx, qq), -1), np.zeros(xx.shape, int), xs, qs


def test_common_translation_is_reported_without_creating_drift():
    p, bands, xs, qs = grid()
    p += [.13, .02]
    result = summarize(p, bands, xs, qs, 2.75)
    np.testing.assert_allclose(result['common_offset_m'], [.13, .02], atol=1e-14)
    np.testing.assert_allclose(result['endpoint_mean_m'], 0, atol=1e-14)
    assert result['coarse_local_scale']['p95_abs_deviation'] < 1e-13


def test_slow_twist_is_visible_even_with_identically_aligned_band_sources():
    p, bands, xs, qs = grid()
    p[..., 1] += xs[None, :]*.001
    result = summarize(p, bands, xs, qs, 2.75)
    np.testing.assert_allclose(result['endpoint_mean_m'], [0, .02], atol=1e-14)
    assert result['coarse_local_scale']['maximum'] > 1.0004
    assert result['endpoint_q_harmonic_m']['constant'] == pytest.approx(.02)


def test_harmonic_diagnostic_does_not_modify_input_or_remove_error():
    p, bands, xs, qs = grid()
    p[..., 1] += xs[None, :]/20*(.012+.008*np.cos(qs[:, None]/2.75))
    before = p.copy()
    result = summarize(p, bands, xs, qs, 2.75)
    assert result['endpoint_q_harmonic_m']['constant'] == pytest.approx(.012)
    assert result['endpoint_q_harmonic_m']['cosine'] == pytest.approx(.008)
    assert result['translation_only_residual_p95_m'][1] > .005
    np.testing.assert_array_equal(p, before)


@pytest.mark.parametrize('bad', ['nan', 'missing', 'reversed', 'empty'])
def test_shape_audit_cannot_pass_by_discarding_unmeasurable_samples(bad):
    p, bands, xs, qs = grid()
    if bad == 'nan':
        p[0, 0, 0] = np.nan
    elif bad == 'missing':
        bands[0, 0] = -1
    elif bad == 'reversed':
        xs = xs[::-1]
    else:
        xs, p, bands = xs[:0], p[:, :0], bands[:, :0]
    with pytest.raises(ValueError):
        summarize(p, bands, xs, qs, 2.75)
