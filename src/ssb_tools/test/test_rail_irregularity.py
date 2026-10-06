import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from ssb_tools.rail_irregularity import (cross_shape, decode_heightmap, profile, rails, segments,
                                         settings, shape, twist, SDF_SIZE_FACTOR, SEGMENT_MAX_M, SEGMENT_SAMPLES,
                                         SEGMENT_OVERLAP_M)
from ssb_tools.stage_b_scene import load_spec

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def inputs():
    return (yaml.safe_load((ROOT/'src/ssb_core/config/stage_b.yaml').read_text()),
            load_spec(ROOT/'src/ssb_tools/config/stage_b_scene.yaml'))


def irregular(config, chord=.002, seed=20261001, length=None, cross=0.):
    c = copy.deepcopy(config)
    if length:
        c['tunnel']['x_max_m'] = c['tunnel']['x_min_m']+length
    c['truth']['track_irregularity'] = dict(model='beijing_subway_vertical_v1', chord10_max_m=chord, seed=seed,
                                            cross_level_tier_m=cross, band_m=[.5, 10.], common_mode=cross == 0)
    return c


@pytest.mark.parametrize('chord', [.002, .005])
@pytest.mark.parametrize('length', [23., 53.])
def test_profile_is_scaled_to_the_10m_chord_tier_and_reproducible(inputs, chord, length):
    c = irregular(inputs[0], chord, length=length)
    x, z, record = profile(c)
    assert x[0] == c['tunnel']['x_min_m'] and x[-1] == pytest.approx(c['tunnel']['x_max_m'])
    # Independent chord evaluation by direct sampling, not the generator's helper.
    centres = np.arange(x[0]+5, x[-1]-5, .005)
    offsets = np.interp(centres, x, z)-.5*(np.interp(centres-5, x, z)+np.interp(centres+5, x, z))
    assert np.abs(offsets).max() == pytest.approx(chord, rel=2e-3)
    assert record['metrics']['chord10_max_m'] == pytest.approx(chord)
    x2, z2, _ = profile(c)
    assert np.array_equal(z, z2)
    assert not np.array_equal(z, profile(irregular(inputs[0], chord, seed=7, length=length))[1])


def test_profile_band_and_beijing_shape(inputs):
    x, z, _ = profile(irregular(inputs[0], .005, length=200.))
    # The profile is a crop of a longer periodic record: window it against leakage.
    spectrum = np.abs(np.fft.rfft((z-z.mean())*np.hanning(len(z))))**2
    n = np.fft.rfftfreq(len(z), x[1]-x[0])
    assert spectrum[(n > 2.5)].sum() < 1e-5*spectrum.sum()        # no wavelengths below 0.5 m
    assert spectrum[(n > 0)&(n < .08)].sum() < 1e-3*spectrum.sum()  # nothing beyond ~10 m
    # Corner near 2.9 m: long waves fall as wavelength^2, short waves as wavelength^4.
    ratio = lambda a, b: shape(a)/shape(b)
    assert ratio(40., 20.) == pytest.approx(4., rel=.05)    # well above the corner
    assert ratio(.4, .2) == pytest.approx(16., rel=.02)     # well below the corner


@pytest.mark.parametrize('cross', [.002, .004])
@pytest.mark.parametrize('length', [23., 53.])
def test_cross_level_and_5m_twist_are_bounded_by_the_tier(inputs, cross, length):
    x, left, right, record = rails(irregular(inputs[0], .002, length=length, cross=cross))
    d = left-right
    level, twisted = np.abs(d).max(), np.abs(twist(x, d, 5.)).max()
    assert max(level, twisted) == pytest.approx(cross) and min(level, twisted) <= cross
    # Independent 5 m twist by direct sampling.
    centres = np.arange(x[0], x[-1]-5, .005)
    assert np.abs(np.interp(centres+5, x, d)-np.interp(centres, x, d)).max() <= cross*(1+1e-6)
    assert record['metrics']['twist_5m_max_m'] == pytest.approx(twisted)
    # Adding the differential leaves the common profile (and so the vertical tier) unchanged.
    x0, z0, _ = profile(irregular(inputs[0], .002, length=length))
    assert np.allclose((left+right)/2, z0, rtol=0, atol=1e-15)
    # Cross-level only: both rails move in opposition.
    x1, l1, r1, _ = rails(irregular(inputs[0], 0., length=length, cross=cross))
    assert np.allclose(l1, -r1) and np.allclose(l1-r1, d)


def test_cross_level_shape_asymptotes():
    ratio = lambda a, b: cross_shape(a)/cross_shape(b)
    assert ratio(.4, .2) == pytest.approx(16., rel=.02)     # short waves fall as wavelength^4
    assert ratio(2e4, 1e4) == pytest.approx(.25, rel=.01)  # very long waves (beyond 2pi/Wr) roll off


def test_flat_and_invalid_settings(inputs):
    assert settings(inputs[0]) is None
    c = irregular(inputs[0], 0.)
    assert settings(c) is None and profile(c) is None
    for key, value in [('chord10_max_m', .02), ('band_m', [10, .5]), ('common_mode', False), ('model', 'other'),
                       ('cross_level_tier_m', -.001)]:
        bad = irregular(inputs[0]); bad['truth']['track_irregularity'][key] = value
        with pytest.raises(ValueError): settings(bad)
    with pytest.raises(ValueError): settings(_with(irregular(inputs[0], cross=.002), common_mode=True))
    assert settings(irregular(inputs[0], 0., cross=.002))['cross_model'].startswith('german')


@pytest.mark.parametrize('length', [23., 53.])
def test_segments_overlap_and_stay_bounded(length):
    parts = segments(-1.5, -1.5+length)
    assert parts[0][0] == -1.5 and parts[-1][1] == pytest.approx(-1.5+length)
    step = parts[0][1]-parts[0][0]
    for (a0, a1), (b0, b1) in zip(parts, parts[1:]):
        assert a1-b0 == pytest.approx(SEGMENT_OVERLAP_M, abs=step/(SEGMENT_SAMPLES-1))   # whole shared samples
        assert b1-b0 == pytest.approx(step)
    assert max(b-a for a, b in parts) <= SEGMENT_MAX_M+1e-9
    assert len(parts) == (9 if length == 23 else 21)


def _with(config, **values):
    config['truth']['track_irregularity'].update(values)
    return config


def test_track_heightmaps_match_profile_and_lower_guide_faces(tmp_path, inputs):
    from ssb_tools.stage_b_track import make_track, replace_track
    config, spec = inputs
    c = irregular(config, .005, cross=.004)
    with pytest.raises(ValueError): make_track(tmp_path, c, spec)
    world = ET.Element('world'); ET.SubElement(world, 'model', name='track')
    replace_track(world, tmp_path, c, spec)
    surfaces = [m for m in world.findall('model') if m.get('name').startswith('rail_surface_')]
    assert len(surfaces) == 2*len(segments(c['tunnel']['x_min_m'], c['tunnel']['x_max_m'])) and len(world.findall("model[@name='track']")) == 1
    x, left, right, _ = rails(c)
    z = np.minimum(left, right)
    record = yaml.safe_load((tmp_path/'track/rail_irregularity.json').read_text())
    entries = {e['file']: e for e in record['heightmaps']}
    for entry in record['heightmaps']:
        xs, zs = decode_heightmap(tmp_path/'track'/entry['file'], entry)
        rail = left if entry['rail'] == 'left' else right
        assert np.abs(zs-np.interp(xs, x, rail)).max() <= entry['quantization_m']+1e-12
    # gz-physics ignores <pos>: placement is the model pose; size spans the segment.
    for model in surfaces:
        px, py, pz = map(float, model.findtext('pose').split()[:3])
        sx, sy, sz = map(float, model.findtext('link/collision/geometry/heightmap/size').split())
        entry = entries[Path(model.findtext('link/collision/geometry/heightmap/uri')).name]
        assert model.get('name') == f"rail_surface_{entry['rail']}_{entry['file'][-6:-4]}"
        assert np.sign(py) == (1 if entry['rail'] == 'left' else -1)
        # DART spreads N samples over size*(N-1)/N: the SDF size is stretched to compensate.
        assert px == pytest.approx(sum(entry['x_m'])/2) and sx/SDF_SIZE_FACTOR == pytest.approx(entry['x_m'][1]-entry['x_m'][0])
        assert pz == pytest.approx(entry['z_m'][0]) and sy/SDF_SIZE_FACTOR == pytest.approx(spec['track']['head_width_m'])
        assert model.findtext('link/collision/geometry/heightmap/pos') == '0 0 0'
    # The physical check decodes both rails from the world alone; swapped rails are detected.
    from ssb_tools.physical_world import check
    track_checks = ('rail_models_complete', 'rail_placement', 'rail_coverage', 'rail_profile_matches_configuration')
    sdf = ET.Element('sdf'); sdf.append(world)
    ET.ElementTree(sdf).write(tmp_path/'world.sdf')
    report = lambda config: check(config, spec, tmp_path/'world.sdf')['checks']
    assert all(report(c)[name]['passed'] for name in track_checks)
    assert not report(_with(copy.deepcopy(c), seed=7))['rail_profile_matches_configuration']['passed']
    for model in surfaces:
        model.set('name', model.get('name').replace('left', 'tmp').replace('right', 'left').replace('tmp', 'right'))
    ET.ElementTree(sdf).write(tmp_path/'world.sdf')
    assert not report(c)['rail_placement']['passed'] and not report(c)['rail_profile_matches_configuration']['passed']
    sdf.remove(world)
    rail_link = world.find("model[@name='track']/link[@name='rails']")
    for side in ('left', 'right'):
        col = rail_link.find(f"collision[@name='{side}_head']")
        top = float(col.findtext('pose').split()[2])+float(col.findtext('geometry/box/size').split()[2])/2
        assert top == pytest.approx(z.min()-.003) and top < z.min()
        assert float(col.findtext('pose').split()[2])-float(col.findtext('geometry/box/size').split()[2])/2 == pytest.approx(-.038)
    # Regenerating flat removes the rail surfaces and restores the flat guide box.
    replace_track(world, tmp_path, config, spec)
    assert not [m for m in world.findall('model') if m.get('name').startswith('rail_surface_')]
    assert not (tmp_path/'track/rail_irregularity.json').exists()


def test_sprung_wheels_keep_mass_names_and_require_calibration(tmp_path, inputs):
    from ssb_tools.stage_b_robot import make_robot
    config, spec = inputs
    config['contact'] = {'enabled': True}
    for section in ('truth', 'calibration'):
        config[section].update(odo_left_diameter_m=.08, odo_right_diameter_m=.08)
    config['truth']['wheel_compliance'] = dict(static_deflection_m=.0002, damping_ratio=.2)
    with pytest.raises(ValueError): make_robot(tmp_path, config, spec)
    config['truth']['wheel_compliance'].update(stiffness_n_m=2.9e6, damping_n_s_m=3300.)
    car = make_robot(tmp_path, config, spec)
    assert sum(float(v.text) for v in car.findall('link/inertial/mass')) == pytest.approx(spec['robot']['total_mass_kg'])
    for joint, wheel in [('odometer', 'odometer_wheel'), ('wheel_joint_1', 'wheel_1'),
                         ('wheel_joint_2', 'wheel_2'), ('wheel_joint_3', 'wheel_3')]:
        spring = car.find(f"joint[@name='{joint}_suspension']")
        assert spring.get('type') == 'prismatic' and spring.findtext('parent') == 'base'
        assert float(spring.findtext('axis/dynamics/spring_stiffness')) == 2.9e6
        assert float(spring.findtext('axis/dynamics/damping')) == 3300.
        revolute = car.find(f"joint[@name='{joint}']")
        assert revolute.findtext('parent') == wheel+'_axle' and revolute.findtext('child') == wheel
        assert car.find(f"link[@name='{wheel}_axle']/pose").text == car.find(f"link[@name='{wheel}']/pose").text


def test_contact_validator_reference_geometry():
    from ssb_tools.validate_contact import disc_centre, speed_factor
    profile_knots = [[0., 0.], [1., 1.], [15., 1.], [16., 0.], [17., 0.]]
    f = speed_factor(profile_knots, np.array([-1, 0, .5, 1, 8, 15.5, 16, 20]))
    assert f.tolist() == pytest.approx([0, 0, .5, 1, 1, .5, 0, 0])
    x = np.linspace(-1, 1, 20001)
    assert disc_centre(x, np.zeros_like(x), np.array([0.]), .1)[0] == pytest.approx(0, abs=1e-12)
    # A 1 mm cosine dip of 40 mm wavelength curves tighter than the 0.1 m wheel: the disc
    # bridges the dip (stays above its bottom) and rests on a crest exactly.
    z = -.001*np.cos(2*np.pi*x/.04)
    assert -.001 < disc_centre(x, z, np.array([0.]), .1)[0] < 0
    assert disc_centre(x, z, np.array([.02]), .1)[0] == pytest.approx(.001, abs=1e-6)
