import json
import math
import cv2
import numpy as np
import pytest
from ssb_tools.band_matching import MatchSettings
from ssb_tools.initial_unroll import PROJECTION
from ssb_tools.match_bands import run, verified_bands, plan_windows
from ssb_tools.provenance import stage_record
from ssb_tools.session import sha256_file
from test_band_matching import texture


def bands_fixture(root):
    root.mkdir()
    phase = np.arange(-.18, .18001, .0004)
    width = 512; offsets = np.linspace(-.1, .1, width)
    projection = np.zeros(2*len(phase), PROJECTION)
    source = texture(seed=40, shape=(1400, 1400))
    image = np.empty((len(projection), width), np.float32)
    for band in range(2):
        ids = slice(band*len(phase), (band+1)*len(phase))
        projection['sequence'][ids] = np.arange(band*len(phase), (band+1)*len(phase))
        projection['segment'][ids] = band
        projection['lattice_row'][ids] = np.arange(len(phase))+band*10000
        projection['theta_rad'][ids] = phase+2*math.pi*band
        axis = .5+.07*band+.05*phase; projection['x_axis_m'][ids] = axis
        # Independent scene mapping, with a slight differential line tilt.
        x = axis[:, None]+offsets[None, :]+band*(.0015+.004*phase[:, None])
        q = np.broadcast_to(phase[:, None], x.shape)
        image[ids] = cv2.remap(source, ((x-.5)/.0004+600).astype(np.float32),
            (q/.0004+650).astype(np.float32), cv2.INTER_LINEAR)
    np.save(root/'projection.npy', projection); np.save(root/'sensor_flat.npy', image)
    np.savez(root/'mapping.npz', native_x_offset_m=offsets, corrected_x_offset_m=offsets,
        geometry_valid=np.ones(width, bool), angular_footprint_rad=.00044)
    grid = dict(radius_m=1., target_x_m=[.5, .6], theta_rad=[-.1, .1],
                dx_m=.0004, dq_m=.0004, requested_pitch_m=.0004, shape=[500, 250])
    (root/'report.json').write_text(json.dumps(dict(schema='ssb.initial_unroll.v1', grid=grid,
                                                  optical_signature='image-measured')))
    (root/'bands.json').write_text(json.dumps(dict(grid=grid)))
    files = sorted(root.iterdir())
    (root/'provenance.json').write_text(json.dumps(stage_record('initial_unroll', [], files, {})))
    return projection, offsets


def test_verified_public_band_pipeline_and_native_correspondence_sources(tmp_path):
    root = tmp_path/'d1'; projection, offsets = bands_fixture(root)
    digest = sha256_file(root/'sensor_flat.npy'); output = tmp_path/'d2'
    report = run(root, output, spacing_m=.04, height=128, max_width=256,
                 settings=MatchSettings(max_shift_mm=4.))
    assert report['status'] == 'complete', report
    assert report['windows']['accepted'] >= 3
    table = np.load(output/'matches.npy'); assert table['inlier'].sum() > 50
    assert sha256_file(root/'sensor_flat.npy') == digest
    for side in ('a', 'b'):
        lo, hi = table[side+'_lower_sequence'], table[side+'_upper_sequence']
        weight = table[side+'_angular_weight']
        x0 = projection['x_axis_m'][lo]+np.interp(table[side+'_lower_column'], np.arange(len(offsets)), offsets)
        x1 = projection['x_axis_m'][hi]+np.interp(table[side+'_upper_column'], np.arange(len(offsets)), offsets)
        np.testing.assert_allclose(x0*(1-weight)+x1*weight, table['x_'+side+'_m'], atol=1e-12, rtol=0)
        theta = projection['theta_rad']-2*math.pi*projection['segment']
        np.testing.assert_allclose(theta[lo]*(1-weight)+theta[hi]*weight, table['q_'+side+'_m'], atol=1e-12, rtol=0)
    provenance = json.loads((output/'provenance.json').read_text())
    assert all(str(root) in name for name in provenance['inputs'])
    assert not (root/'evaluation').exists() and (output/'review.html').is_file()
    with pytest.raises(FileExistsError): run(root, output)
    with pytest.raises(ValueError, match='separate'): run(root, root/'inside')


@pytest.mark.parametrize('kind', ['corrupt', 'symlink'])
def test_upstream_corruption_or_private_redirection_is_rejected(tmp_path, kind):
    root = tmp_path/'d1'; bands_fixture(root)
    if kind == 'corrupt':
        with (root/'projection.npy').open('r+b') as f: f.write(b'broken')
        message = 'hash mismatch'
    else:
        private = tmp_path/'evaluation'; private.mkdir()
        (root/'sensor_flat.npy').rename(private/'pixels.npy')
        (root/'sensor_flat.npy').symlink_to(private/'pixels.npy'); message = 'escapes'
    with pytest.raises(ValueError, match=message): run(root, tmp_path/'out')
    assert not (tmp_path/'out').exists()


def test_window_resource_budget_is_checked_before_large_plan_allocation(tmp_path):
    root = tmp_path/'d1'; bands_fixture(root)
    sampler, report, _ = verified_bands(root)
    with pytest.raises(ValueError, match='4096'):
        plan_windows(sampler, report['grid'], .000001, 128, 256, MatchSettings(max_shift_mm=4.))
