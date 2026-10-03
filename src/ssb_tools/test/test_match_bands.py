import json
import math
import cv2
import numpy as np
import pytest
from ssb_tools.band_matching import MatchSettings
from ssb_tools.initial_unroll import BandSampler, PROJECTION
from ssb_tools.match_bands import run, verified_bands, plan_windows, pack_matches
from ssb_tools.global_geometry import Trajectory
from ssb_tools.provenance import stage_record
from ssb_tools.session import sha256_file
from test_band_matching import texture


@pytest.mark.parametrize('edge', ['lower', 'upper', 'interior'])
def test_packed_match_uses_actual_row_centre_when_sampling_edge_footprint(edge):
    radius = 2.75
    phase = np.linspace(-.1, .1, 401)
    offsets = np.linspace(-.22, .22, 512)
    projection = np.zeros(2*len(phase), PROJECTION)
    for band in range(2):
        ids = slice(band*len(phase), (band+1)*len(phase))
        projection['sequence'][ids] = 100+3*np.arange(ids.start, ids.stop)
        projection['segment'][ids] = band
        projection['lattice_row'][ids] = np.arange(len(phase))+band*10000
        projection['theta_rad'][ids] = phase+2*math.pi*band
        projection['x_axis_m'][ids] = 18.6+.1*band+.05*phase
    sampler = BandSampler(projection, np.zeros((len(projection), 512), np.float32),
                          offsets, offsets, np.ones(512, bool), .00051)
    requested_angle = dict(lower=-.1002, upper=.1002, interior=.0123)[edge]
    requested_q = radius*requested_angle
    grid = dict(radius_m=radius, dx_m=.0002, dq_m=.0002)
    window = dict(id=0, bands=[0, 1], x_first_m=18.75, q_first_m=requested_q)
    matches = dict(points_a=np.zeros((1, 2)), points_b=np.zeros((1, 2)),
        ncc=np.ones(1), fb_error=np.zeros(1), residual=np.zeros(1),
        inlier=np.ones(1, np.uint8), holdout=np.ones(1, np.uint8))
    table = pack_matches(sampler, grid, window, matches)
    model = Trajectory(sampler, radius, 1.7)
    for side in ('a', 'b'):
        expected_q = requested_q if edge == 'interior' else radius*phase[0 if edge == 'lower' else -1]
        np.testing.assert_allclose(table['q_'+side+'_m'], expected_q, atol=1e-14, rtol=0)
        rays = model.native_side(table, side)
        np.testing.assert_allclose(rays.hits(np.zeros(model.size)),
            np.column_stack((table['x_'+side+'_m'], table['q_'+side+'_m'])), atol=1e-14, rtol=0)
    if edge != 'interior':
        # Supported footprint remains valid, but cannot invent an interpolated row.
        assert table['a_lower_sequence'][0] == table['a_upper_sequence'][0]
        assert table['a_angular_weight'][0] == 0


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


def raw_bands_fixture(root, session):
    """The same scene as bands_fixture, stored as D1 v2: uint8 raw blocks + flat params."""
    projection, offsets = bands_fixture(root)
    image = np.load(root/'sensor_flat.npy'); (root/'sensor_flat.npy').unlink()
    raw = np.clip(np.rint(image), 0, 254).astype(np.uint8)
    (session/'raw').mkdir(parents=True)
    blocks = []
    for first in range(0, len(raw), 700):
        path = session/'raw'/f'block_{first:06}.u8'; raw[first:first+700].tofile(path)
        blocks.append(dict(file=path.name, first_sequence=first, rows=len(raw[first:first+700]),
                           sha256=sha256_file(path)))
    width = raw.shape[1]
    with np.load(root/'mapping.npz') as m: arrays = dict(m)
    np.savez(root/'mapping.npz', **arrays, flat_offset=np.zeros(width, np.float32),
             flat_gain=np.ones(width, np.float32), flat_valid=np.ones(width, bool))
    (root/'native_source.json').write_text(json.dumps(dict(schema='ssb.native_source.v1',
        session_root=str(session), width=width, blocks=blocks)))
    report = json.loads((root/'report.json').read_text()); report['schema'] = 'ssb.initial_unroll.v2'
    (root/'report.json').write_text(json.dumps(report))
    files = sorted(p for p in root.iterdir() if p.name != 'provenance.json')
    (root/'provenance.json').write_text(json.dumps(stage_record('initial_unroll', [], files, {})))
    return raw


def test_v2_raw_rows_reproduce_v1_cache_matches(tmp_path):
    v1 = tmp_path/'v1'; bands_fixture(v1)
    image = np.load(v1/'sensor_flat.npy'); np.save(v1/'sensor_flat.npy', np.rint(image).clip(0, 254).astype(np.float32))
    files = sorted(p for p in v1.iterdir() if p.name != 'provenance.json')
    (v1/'provenance.json').write_text(json.dumps(stage_record('initial_unroll', [], files, {})))
    v2 = tmp_path/'v2'; raw_bands_fixture(v2, tmp_path/'session')
    kwargs = dict(spacing_m=.04, height=128, max_width=256, settings=MatchSettings(max_shift_mm=4.))
    run(v1, tmp_path/'m1', **kwargs); report = run(v2, tmp_path/'m2', **kwargs)
    assert report['status'] == 'complete'
    for name in ('matches.npy', 'windows.json'):
        assert sha256_file(tmp_path/'m1'/name) == sha256_file(tmp_path/'m2'/name), name
    # Relocated raw directory: still hash-checked against the D1 record.
    (tmp_path/'session/raw').rename(tmp_path/'moved')
    run(v2, tmp_path/'m3', raw_root=tmp_path/'moved', **kwargs)
    assert sha256_file(tmp_path/'m3/matches.npy') == sha256_file(tmp_path/'m1/matches.npy')
    with (tmp_path/'moved/block_000700.u8').open('r+b') as f: f.write(b'\x01\x02')
    with pytest.raises(ValueError, match='raw block hash'): run(v2, tmp_path/'m4', raw_root=tmp_path/'moved', **kwargs)
    assert not (tmp_path/'m4').exists()


def test_worker_processes_reproduce_serial_products_exactly(tmp_path):
    root = tmp_path/'d1'; raw_bands_fixture(root, tmp_path/'session')
    (tmp_path/'session/raw').rename(tmp_path/'moved')
    kwargs = dict(spacing_m=.04, height=128, max_width=256, settings=MatchSettings(max_shift_mm=4.),
                  raw_root=tmp_path/'moved')
    serial = run(root, tmp_path/'serial', **kwargs)
    parallel = run(root, tmp_path/'parallel', workers=2, **kwargs)
    assert parallel['performance']['workers'] == 2 and serial['windows'] == parallel['windows']
    for name in ('matches.npy', 'windows.json'):
        assert sha256_file(tmp_path/'serial'/name) == sha256_file(tmp_path/'parallel'/name), name
    for workers in (0, 1.5, 10**6):
        with pytest.raises(ValueError, match='worker count'): run(root, tmp_path/'bad', workers=workers, **kwargs)
    assert not (tmp_path/'bad').exists()


def test_v2_raw_block_cannot_escape_the_raw_directory(tmp_path):
    root = tmp_path/'d1'; raw_bands_fixture(root, tmp_path/'session')
    source = json.loads((root/'native_source.json').read_text())
    (tmp_path/'outside.u8').write_bytes((tmp_path/'session/raw'/source['blocks'][0]['file']).read_bytes())
    source['blocks'][0]['file'] = '../../outside.u8'
    (root/'native_source.json').write_text(json.dumps(source))
    files = sorted(p for p in root.iterdir() if p.name != 'provenance.json')
    (root/'provenance.json').write_text(json.dumps(stage_record('initial_unroll', [], files, {})))
    with pytest.raises(ValueError, match='escapes'):
        run(root, tmp_path/'out', spacing_m=.04, height=128, max_width=256, settings=MatchSettings(max_shift_mm=4.))
    assert not (tmp_path/'out').exists()


def test_worker_processes_inherit_the_public_input_audit(tmp_path, monkeypatch):
    from ssb_tools import public_audit
    root = tmp_path/'d1'; raw_bands_fixture(root, tmp_path/'session')
    public = tmp_path/'public'; (public/'raw').mkdir(parents=True)
    # Only workers install the hook here (it cannot be removed from pytest itself):
    # their raw reads come from the unstaged capture directory and must fail.
    monkeypatch.setenv(public_audit.ENVIRONMENT, json.dumps(dict(public=str(public), raw=str(public/'raw'))))
    with pytest.raises(RuntimeError, match='outside public read allowlist'):
        run(root, tmp_path/'out', spacing_m=.04, height=128, max_width=256,
            settings=MatchSettings(max_shift_mm=4.), workers=2)
