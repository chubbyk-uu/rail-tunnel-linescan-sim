"""Synthetic public-image radiometry; geometry and sharp crack edges stay independent."""
from dataclasses import replace
import numpy as np
import pytest
from test_initial_unroll import analytic_sampler
from ssb_tools.global_geometry import Trajectory
from ssb_tools.global_resample import corrected_tile
from ssb_tools.seam_fusion import FusionSettings, CpuFusionRaster, solve_gains, estimate, calibration_values
from ssb_tools.wall_coverage import target_grid


def identity(model, gains=None, width=.002):
    from dataclasses import asdict
    return dict(schema='ssb.seam_fusion.v1', settings=asdict(FusionSettings(feather_width_m=width)),
                gains=[1.]*len(model.sampler.segments) if gains is None else gains)


@pytest.mark.parametrize('field,value', [('feather_width_m', 0), ('feather_width_m', .005),
    ('gain_limit', 1.2), ('minimum_dn', np.nan), ('axial_samples', 3), ('angular_samples', True)])
def test_settings_refuse_unbounded_radiometry_and_budgets(field, value):
    with pytest.raises(ValueError): replace(FusionSettings(), **{field: value}).validate()


def test_gain_solution_recovers_known_multiplicative_difference_and_keeps_gauge():
    gains, components = solve_gains(4, [(0, 1, np.log(1.06)), (1, 2, np.log(.98))], 1.08)
    np.testing.assert_allclose(gains[0]/gains[1], 1.06, rtol=.001)
    np.testing.assert_allclose(gains[1]/gains[2], .98, rtol=.001)
    assert abs(np.log(gains[:3]).mean()) < 1e-15 and gains[3] == 1.
    assert not components[-1]['measured']
    bounded, _ = solve_gains(4, [(0, 1, 1.), (1, 2, 1.), (2, 3, 1.)], 1.08)
    assert np.max(bounded) <= 1.08+1e-15 and np.min(bounded) >= 1/1.08-1e-15
    assert abs(np.log(bounded).mean()) < 1e-15


def test_public_overlap_fit_excludes_heldout_rows_and_reports_independent_improvement():
    sampler = analytic_sampler(); sampler.image[11:] *= 1.06
    model = Trajectory(sampler, 1., .7); c = np.zeros(model.size)
    grid = target_grid([.53, .58], 1., [-.08, .08], .004)
    record = estimate(model, c, grid)
    assert record['heldout']['after_median_dn'] < .02*record['heldout']['before_median_dn']
    assert abs(record['gains'][0]/record['gains'][1]-1.06) < .001
    assert record['pairs'][0]['training_samples'] and record['pairs'][0]['heldout_samples']


def test_empty_or_dark_overlap_cannot_claim_measured_calibration():
    sampler = analytic_sampler(); sampler.image[:] = 0
    model = Trajectory(sampler, 1., .7)
    with pytest.raises(ValueError, match='unmeasurable'): estimate(model, np.zeros(model.size), target_grid([.53, .58], 1., [-.08,.08], .004))


def test_narrow_fusion_never_changes_coverage_or_single_band_pixels():
    model = Trajectory(analytic_sampler(remove_centre=True), 1., .7); c = np.zeros(model.size)
    xs = np.linspace(.38, .73, 900); angles = np.array([-.08, 0., .08])
    original, number = corrected_tile(model, c, angles, xs)
    fused, count, source, other, weight = CpuFusionRaster(model, c, identity(model)).components(angles, xs)
    np.testing.assert_array_equal(count, number)
    np.testing.assert_array_equal(np.isnan(fused), np.isnan(original))
    np.testing.assert_array_equal(fused[weight == 1], original[weight == 1])
    assert np.any(weight < 1) and np.all((weight >= .5) & (weight <= 1))
    assert np.all(weight[count < 2] == 1)
    assert np.max(np.count_nonzero(weight < 1, axis=1)) <= 8 # width ~2mm at 0.39mm pitch


def test_invalid_calibration_cannot_apply_large_gain_or_wrong_source_count():
    model = Trajectory(analytic_sampler(), 1., .7)
    for gains in ([1.], [1., np.nan], [1.2, 1.]):
        with pytest.raises(ValueError, match='gains'): calibration_values(identity(model, gains), model)


def test_identical_sharp_crack_does_not_blur_or_brighten_with_identity_fusion(monkeypatch):
    import ssb_tools.seam_fusion as sf
    model = Trajectory(analytic_sampler(), 1., .7); c = np.zeros(model.size)
    # Independent aligned discontinuity with a 0.4mm dark crack through the feather.
    xs = np.linspace(.49, .59, 1001); angles = np.array([0.])
    crack = np.where(abs(xs-.538) < .0002, 12., 120.).astype(np.float32)
    def sample(_model, _coefficients, band, qs, xx):
        value = np.interp(xx, xs, crack).astype(np.float32)[None,:]
        score = ((xx-.418) if band == 0 else (.658-xx))*100
        return value, np.ones_like(value,bool), score.astype(np.float32)[None,:]
    monkeypatch.setattr(sf, 'sample_corrected', sample)
    result, _, _, _, weight = CpuFusionRaster(model,c,identity(model)).components(angles,xs)
    np.testing.assert_array_equal(result[0], crack)
    assert np.any(weight < 1)


def test_refuses_fusion_across_completely_missing_scan_circle():
    from ssb_tools.initial_unroll import BandSampler
    old = analytic_sampler(); p = old.projection.copy(); p['segment'][11:] += 1
    sampler = BandSampler(p, old.native, old.offsets, old.output_offsets, old.geometry_valid, old.footprint)
    model = Trajectory(sampler,1.,.7)
    with pytest.raises(ValueError, match='missing circle'): CpuFusionRaster(model,np.zeros(model.size),identity(model))


def test_weak_brightness_difference_is_measured_but_never_turns_into_false_gain():
    sampler=analytic_sampler();sampler.image[11:] *= 1.004
    model=Trajectory(sampler,1.,.7);record=estimate(model,np.zeros(model.size),target_grid([.53,.58],1.,[-.08,.08],.004))
    assert record['pairs'][0]['status']=='measured'
    assert not record['pairs'][0]['gain_enabled']
    assert record['pairs'][0]['applied_edge_log_ratio']==0.
    np.testing.assert_array_equal(record['gains'],[1.,1.])
    assert record['heldout']['before_median_dn']==record['heldout']['after_median_dn']
