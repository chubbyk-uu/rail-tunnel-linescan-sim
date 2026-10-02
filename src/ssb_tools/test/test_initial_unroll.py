import json
import math
from pathlib import Path
import numpy as np
import pytest
from ssb_tools.initial_unroll import BandSampler, PROJECTION, rasterize, reconstruct, trace_pixel, preview_sample
from ssb_tools.public_capture import PublicCapture
from ssb_tools.session import sha256_file
from ssb_tools.wall_coverage import target_grid


def analytic_sampler(remove_centre=False):
    phase = np.arange(-5, 6)*.02
    ids = np.arange(len(phase))
    if remove_centre: phase = np.delete(phase, 5); ids = np.delete(ids, 5)
    u = np.linspace(-1, 1, 32)
    offsets = .1*u+.02*u**3+.007
    projection = np.zeros(len(phase)*2, PROJECTION)
    for band in range(2):
        a = band*len(phase); b = a+len(phase)
        projection['segment'][a:b] = band+3
        projection['sequence'][a:b] = np.arange(a, b)
        projection['lattice_row'][a:b] = ids+band*100
        projection['x_axis_m'][a:b] = .5+.06*band+.2*phase
        projection['theta_rad'][a:b] = phase+2*math.pi*(band+3)
    # Independently chosen world-coordinate ramp: resampling must retain it.
    image = (55+90*(projection['x_axis_m'][:, None]+offsets)+
             11*np.tile(phase, 2)[:, None]).astype(np.float32)
    sampler = BandSampler(projection, image, offsets, np.linspace(offsets[0], offsets[-1], 32),
                          np.ones(32, bool), .022)
    return sampler


def test_measured_nonlinear_mapping_and_helical_motion_preserve_world_ramp():
    sampler = analytic_sampler()
    angles = np.array([-.07, -.01, .03, .075]); xs = np.linspace(.53, .56, 17)
    expected = 55+90*xs[None, :]+11*angles[:, None]
    for band in (0, 1):
        values, valid, _, sources = sampler.sample(band, angles, xs, sources=True)
        assert valid.all()
        np.testing.assert_allclose(values, expected, atol=1e-5)
        # Recover every displayed value from native row/column provenance.
        recovered = []
        for row, (lo, hi, w) in enumerate(zip(sources['lower'], sources['upper'], sources['angular_weight'])):
            pair = []
            for index, name in [(lo, 'lower_column'), (hi, 'upper_column')]:
                columns = sources[name][row]; left = np.floor(columns).astype(int); alpha = columns-left
                pair.append(sampler.image[index, left]*(1-alpha)+sampler.image[index, left+1]*alpha)
            recovered.append(pair[0]*(1-w)+pair[1]*w)
        np.testing.assert_allclose(values, recovered, atol=1e-5)


def test_missing_line_and_saturated_column_are_not_interpolated_away():
    sampler = analytic_sampler(remove_centre=True)
    values, valid, _ = sampler.sample(0, np.array([0.]), np.array([.55]))
    assert not valid.any() and np.isnan(values).all()
    sampler = analytic_sampler()
    sampler.image[:, 16] = np.nan
    angle = -.02; x = .5+.2*angle+sampler.offsets[16]
    values, valid, _ = sampler.sample(0, np.array([angle]), np.array([x]))
    assert not valid.any() and np.isnan(values).all()
    values, valid, _ = sampler.sample(0, np.array([.2]), np.array([.55]))
    assert not valid.any()


def test_angular_tile_sizes_produce_identical_mosaic_counts_and_source_ids(tmp_path):
    sampler = analytic_sampler()
    grid = target_grid([.53, .56], 1., [-.08, .08], .004)
    products = []
    for tile in (1, 7, 32):
        root = tmp_path/str(tile); root.mkdir()
        report, arrays = rasterize(sampler, grid, root, tile)
        assert report['missing_pixels'] == 0 and report['overlapping_pixels'] == report['total_pixels']
        products.append(tuple(np.asarray(a).copy() for a in arrays))
    for product in products[1:]:
        for expected, actual in zip(products[0], product): np.testing.assert_array_equal(expected, actual)


def public_fixture(root):
    (root/'config').mkdir(parents=True); (root/'metadata').mkdir(); (root/'raw').mkdir()
    width = 32; step = 2*math.pi/4096
    phase = np.arange(math.ceil(-.048/step), math.floor(.048/step)+1)*step
    times = np.r_[phase+.05, phase+.05+2*math.pi]
    segments = np.repeat([0, 1], len(phase))
    config = dict(schema='ssb.observable_config.v1', camera=dict(width=width, optical_signature='measured', nominal_distance_m=1.),
        calibration=dict(radius_m=1., wheel_diameter_m=.2, head_mount_x_m=.003), contact=dict(enabled=False),
        scan_encoder=dict(ppr=1024, edges_per_cycle=4), odometer=dict(ppr=1000, edges_per_cycle=4, gear_ratio=1.),
        motion=dict(start_x_m=.5), gate=dict(start_rad=-.05, end_rad=.05),
        inspection=dict(target_x_m=[.5, .61], theta_rad=[-.05, .05], grid_pitch_m=.004))
    rows = np.zeros(len(times), dtype=[('sequence', '<i8'), ('row', '<i8'), ('segment', '<i8'), ('t_center', '<f8')])
    rows['sequence'] = np.arange(len(times)); rows['segment'] = segments; rows['t_center'] = times
    rows['row'] = np.r_[np.arange(len(phase)), np.arange(len(phase))+4096]
    edge = np.dtype([('t', '<f8'), ('count', '<i8'), ('dir', '<i8')])
    scan_counts = np.arange(-40, 4200)
    scan = np.zeros(len(scan_counts), edge); scan['count'] = scan_counts; scan['dir'] = 1
    scan['t'] = scan_counts*step+.05
    odo_counts = np.arange(1, 430)
    odo = np.zeros(len(odo_counts), edge); odo['count'] = odo_counts; odo['dir'] = 1
    odo['t'] = odo_counts*math.pi*.2/4000/.01
    gates = np.array([(0., 0, 0, 1), (2*math.pi, 1, 0, 1)],
        dtype=[('t', '<f8'), ('revolution', '<i8'), ('kind', '<i4'), ('dir', '<i4')])
    manifest = {}
    for name, table in dict(rows=rows, scan_edges=scan, odometer_edges=odo, gate_events=gates).items():
        path = root/'metadata'/(name+'.bin'); table.tofile(path)
        manifest[name] = dict(file=path.name, sha256=sha256_file(path), count=len(table),
                              dtype=table.dtype.descr, record_size=table.dtype.itemsize)
    (root/'metadata/manifest.json').write_text(json.dumps(manifest))
    q = (np.arange(width)-(width-1)/2)/(width/2)
    valid = np.ones(width, bool); valid[[0, -1]] = False
    calibration = dict(schema='ssb.measured_optical_calibration.v1', optical_signature='measured',
        flat=dict(offset=[5.]*width, gain=[1.2]*width, valid=[True]*width),
        geometry=dict(coefficients=[0., .064, 0., 0.], output_fov_m=.128,
                      lookup=list(range(width)), valid=valid.tolist()))
    x = .503+.01*times
    values = 55+90*(x[:, None]+q*.064)+11*np.tile(phase, 2)[:, None]
    raw = np.rint(values/1.2+5).astype(np.uint8)
    path = root/'raw/block.u8'; raw.tofile(path)
    index = dict(width=width, blocks=[dict(file='block.u8', rows=len(raw), first_sequence=0, sha256=sha256_file(path))])
    (root/'raw/index.json').write_text(json.dumps(index))
    (root/'config/observable_config.json').write_text(json.dumps(config))
    summary = dict(status='complete', motion=dict(complete=True), rows=len(raw), files={
        name: sha256_file(root/name) for name in ('config/observable_config.json','metadata/manifest.json','raw/index.json')})
    (root/'session.json').write_text(json.dumps(summary))
    cal = root/'calibration.json'; cal.write_text(json.dumps(calibration))
    return cal, index


def test_end_to_end_public_only_preserves_raw_and_separate_masks(tmp_path):
    root = tmp_path/'public'; cal, index = public_fixture(root)
    out = tmp_path/'unroll'
    report = reconstruct(root, cal, out, pitch=.004, angular_tile_rows=3)
    assert not (root/'evaluation').exists() and report['bands'] == 2
    assert sha256_file(root/'raw/block.u8') == index['blocks'][0]['sha256']
    native = np.load(out/'sensor_flat.npy'); bits = np.load(out/'sensor_valid_bits.npy')
    assert np.array_equal(np.isfinite(native), np.unpackbits(bits, axis=1, count=32, bitorder='little').astype(bool))
    image = np.load(out/'mosaic.npy'); counts = np.load(out/'coverage.npy'); source = np.load(out/'source_band.npy')
    assert np.array_equal(np.isfinite(image), counts > 0)
    assert np.array_equal(source >= 0, counts > 0)
    assert report['coverage']['overlapping_pixels'] > 0
    assert (out/'review.html').is_file() and (out/'provenance.json').is_file()
    q, x = np.argwhere(counts > 0)[len(np.argwhere(counts > 0))//2]
    traced = trace_pixel(out, int(q), int(x))
    assert traced['valid'] and traced['error_dn'] < 1e-5
    assert sum(v['weight'] for v in traced['contributions']) == pytest.approx(1.)
    assert sum(v['weight']*v['flat_dn'] for v in traced['contributions']) == pytest.approx(traced['stored_dn'], abs=1e-5)
    with pytest.raises(FileExistsError): reconstruct(root, cal, out, pitch=.004)
    with pytest.raises(ValueError, match='ROI'): reconstruct(root, cal, tmp_path/'outside', [.4, .6])


@pytest.mark.parametrize('kind', ['metadata', 'raw', 'symlink'])
def test_private_path_redirection_is_rejected_before_output_creation(tmp_path, kind):
    root = tmp_path/'public'; cal, _ = public_fixture(root)
    outside = tmp_path/'forbidden.bin'; outside.write_bytes(b'private')
    if kind == 'metadata':
        name = 'metadata/manifest.json'; value = json.loads((root/name).read_text())
        value['scan_edges']['file'] = '../../forbidden.bin'
    else:
        name = 'raw/index.json'; value = json.loads((root/name).read_text())
        if kind == 'symlink':
            (root/'raw/redirect').symlink_to(outside); value['blocks'][0]['file'] = 'redirect'
        else: value['blocks'][0]['file'] = '../../forbidden.bin'
    (root/name).write_text(json.dumps(value))
    summary = json.loads((root/'session.json').read_text()); summary['files'][name] = sha256_file(root/name)
    (root/'session.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError, match='escapes'): reconstruct(root, cal, tmp_path/'rejected')
    assert not (tmp_path/'rejected').exists()


def test_roi_reads_only_intersecting_raw_blocks_and_rejects_used_block_corruption(tmp_path):
    import hashlib
    root = tmp_path/'public'; cal, index = public_fixture(root)
    path = root/'raw/block.u8'; pixels = np.fromfile(path, np.uint8).reshape(-1, 32)
    half = len(pixels)//2; pixels[:half].tofile(path)
    index['blocks'] = [dict(file='block.u8', rows=half, first_sequence=0, sha256=sha256_file(path)),
        dict(file='not_exported.u8', rows=half, first_sequence=half,
             sha256=hashlib.sha256(pixels[half:].tobytes()).hexdigest())]
    (root/'raw/index.json').write_text(json.dumps(index))
    summary = json.loads((root/'session.json').read_text())
    summary['files']['raw/index.json'] = sha256_file(root/'raw/index.json')
    (root/'session.json').write_text(json.dumps(summary))
    report = reconstruct(root, cal, tmp_path/'roi', [.5, .501], pitch=.004)
    assert report['diagnostic_roi'] and len(report['sensor']['raw_blocks']) == 1
    assert not (root/'raw/not_exported.u8').exists()
    with path.open('r+b') as f: f.write(b'\xff')
    with pytest.raises(ValueError, match='raw block hash'):
        reconstruct(root, cal, tmp_path/'corrupt', [.5, .501], pitch=.004)
    assert not (tmp_path/'corrupt/provenance.json').exists()


def test_directory_symlink_cannot_redirect_public_config_to_evaluation(tmp_path):
    root = tmp_path/'public'; cal, _ = public_fixture(root)
    private = root/'evaluation'; private.mkdir()
    (root/'config/observable_config.json').rename(private/'observable_config.json')
    (root/'config').rmdir(); (root/'config').symlink_to(private, target_is_directory=True)
    with pytest.raises(ValueError, match='escapes'): PublicCapture(root, cal)


def test_bounded_preview_preserves_sparse_samples_and_uses_owned_memory(tmp_path):
    array = np.lib.format.open_memmap(tmp_path/'large.npy', 'w+', np.float32, shape=(77, 51))
    values = np.arange(77*51, dtype=np.float32).reshape(77, 51); array[:] = values
    for stride in (1, 3, 17):
        preview = preview_sample(array, stride)
        assert type(preview) is np.ndarray and not np.shares_memory(preview, array)
        np.testing.assert_array_equal(preview, values[::stride, ::stride])
