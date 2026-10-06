"""Every D3 entry point must turn the same flags into the same declared settings."""
from dataclasses import asdict
import inspect
import sys

import pytest

from ssb_tools.global_geometry import reconstruction_settings

# Frozen d3 block of the 20 m milestone protocol (local_data/evaluation/
# milestone20_holdout_seed20270119_20261005/protocol.json, code 03cde91).
MILESTONE_D3 = {
    'attitude_spacing_m': 0.02, 'position_spacing_m': 0.6, 'position_prior_mm': 10.0,
    'attitude_prior_mrad': 5.0, 'position_curvature_mm': 2.0, 'attitude_curvature_mrad': 0.6,
    'noise_floor_px': 0.05, 'effective_points_per_window': 16.0, 'max_irls': 5, 'fit_translation': True,
    'fit_heave': False, 'translation_prior_mm': 10.0, 'translation_curvature_mm': 0.5,
    'translation_bound_mm': 30.0, 'observed_knots': True, 'adaptive_attitude': True,
    'coarse_translation': True, 'relative_encoder_scale': True, 'relative_scale_prior_fraction': 0.01,
    'relative_scale_bound_fraction': 0.03, 'fit_axis_yaw': True, 'axis_yaw_prior_mrad': 2.0,
    'axis_yaw_bound_mrad': 3.0, 'geometry_backend': 'cuda'}
MILESTONE_FLAGS = ['--adaptive-attitude', '--slow-translation', '--relative-encoder-scale']
FLAG_SETS = [[], ['--slow-translation'], ['--slow-translation', '--no-fit-axis-yaw'],
             ['--slow-translation', '--fit-axis-yaw'], MILESTONE_FLAGS,
             ['--geometry-backend', 'cpu', '--relative-encoder-scale']]


class Stop(Exception):
    pass


def captured(monkeypatch, module, function, index, argv):
    seen = []

    def fake(*args, **kwargs):
        seen.append(args if index is None else args[index])
        raise Stop
    monkeypatch.setattr(module, function, fake)
    monkeypatch.setattr(sys, 'argv', argv)
    with pytest.raises(Stop):
        module.main()
    assert len(seen) == 1
    return seen[0]


def public_reconstruction(monkeypatch, tmp_path, flags):
    import ssb_tools.public_reconstruction as module
    return captured(monkeypatch, module, 'run', 6, ['x', '--unroll', 'u', '--observable', 'o', '--root', 'r', *flags])


def optimize_bands(monkeypatch, tmp_path, flags):
    import ssb_tools.optimize_bands as module
    return captured(monkeypatch, module, 'run', 4,
                    ['x', '--unroll', 'u', '--matches', 'm', '--observable', 'o', '--output', 'out', *flags])


def reconstruction_budget(monkeypatch, tmp_path, flags):
    import ssb_tools.reconstruction_budget as module
    (tmp_path/'capture.yaml').write_text('{}\n'); (tmp_path/'calibration.json').write_text('{}\n')
    return captured(monkeypatch, module, 'plan', 4, ['x', '--demo', str(tmp_path), '--output', str(tmp_path/'budget.json'),
                                                     '--start', '0', '--length', '3', *flags])


def holdout_declaration(monkeypatch, tmp_path, flags):
    """The CLI forwards the shared switches; declare() itself is covered in test_holdout_protocol."""
    import ssb_tools.holdout_protocol as module
    arguments = captured(monkeypatch, module, 'declare', None, ['x', 'declare', '--workspace', 'w', '--demo', 'd',
                                                               '--output', 'o', '--start', '0', '--length', '3', *flags])
    adaptive, slow, relative, backend, yaw = arguments[6], *arguments[-4:]
    return reconstruction_settings(.02, adaptive, slow, relative, backend, yaw)


ENTRY_POINTS = [public_reconstruction, optimize_bands, reconstruction_budget, holdout_declaration]


@pytest.mark.parametrize('flags', FLAG_SETS)
def test_all_entry_points_declare_identical_settings(monkeypatch, tmp_path, flags):
    settings = [asdict(entry(monkeypatch, tmp_path, flags)) for entry in ENTRY_POINTS]
    assert all(s == settings[0] for s in settings[1:])


def test_milestone_flags_reproduce_the_frozen_protocol_settings(monkeypatch, tmp_path):
    for entry in ENTRY_POINTS:
        assert asdict(entry(monkeypatch, tmp_path, MILESTONE_FLAGS)) == MILESTONE_D3
    assert asdict(reconstruction_settings(.02, True, True, True, 'cuda')) == MILESTONE_D3


def test_axis_yaw_is_an_explicit_switch_that_still_requires_translation(monkeypatch, tmp_path):
    separate = optimize_bands(monkeypatch, tmp_path, ['--slow-translation', '--no-fit-axis-yaw'])
    assert separate.fit_axis_yaw is False and separate.coarse_translation is True
    separate.validate()
    with pytest.raises(ValueError):
        optimize_bands(monkeypatch, tmp_path, ['--fit-axis-yaw']).validate()


def test_standalone_matching_and_fit_defaults_follow_the_pipeline(monkeypatch, tmp_path):
    import ssb_tools.match_bands as matching
    import ssb_tools.public_reconstruction as pipeline
    for function in (matching.run, pipeline.run):
        assert inspect.signature(function).parameters['spacing_m'].default == .2
    standalone = asdict(optimize_bands(monkeypatch, tmp_path, []))
    assert standalone == asdict(public_reconstruction(monkeypatch, tmp_path, []))
    assert standalone['attitude_spacing_m'] == .02 and standalone['observed_knots'] is True
