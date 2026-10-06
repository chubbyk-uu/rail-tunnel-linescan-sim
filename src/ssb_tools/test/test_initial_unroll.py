import json
import math
import numpy as np
import pytest
from ssb_tools.initial_unroll import (BandSampler, PROJECTION, rasterize, reconstruct, trace_pixel, preview_sample,
                                      cpu_tile, MOSAIC_INVALID)
from ssb_tools.native_rows import MemoryRows, NativeRows
from ssb_tools.optical_calibration import flat_correct
from ssb_tools.wall_coverage import RUN_DTYPE
from ssb_tools.public_capture import PublicCapture
from ssb_tools.session import sha256_file
from ssb_tools.wall_coverage import target_grid


FLAT = dict(offset=[5.]*32, gain=[1.2]*32, valid=[True]*32)


def analytic_sampler(remove_centre=False, raw=False):
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
    if raw:  # production path: uint8 rows, flat-corrected on demand
        image = MemoryRows(np.clip(np.rint(image/1.2+5), 0, 254).astype(np.uint8), FLAT)
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


def read_products(root):
    return [np.load(root/'mosaic_u16.npy'), np.load(root/'mosaic_count.npy'),
            np.fromfile(root/'coverage_runs.bin', RUN_DTYPE)]


@pytest.mark.parametrize('raw', [False, True])
def test_two_dimensional_tile_shapes_produce_identical_mosaic_and_coverage(tmp_path, raw):
    sampler = analytic_sampler(raw=raw)
    grid = target_grid([.53, .56], 1., [-.08, .08], .004)
    products = []
    for rows, columns in ((1, 1), (7, 3), (32, 8192)):
        root = tmp_path/f'{rows}_{columns}'; root.mkdir()
        report, cover = rasterize(sampler, grid, root, rows, mosaic=(0, grid['shape'][1]), tile_columns=columns)
        assert report['missing_pixels'] == 0 and report['overlapping_pixels'] == report['total_pixels']
        products.append(read_products(root)+[cover])
    for product in products[1:]:
        for expected, actual in zip(products[0], product): np.testing.assert_array_equal(expected, actual)


def test_band_prefilter_matches_visiting_every_band():
    sampler = analytic_sampler()
    angles = np.linspace(-.08, .08, 9)
    for xs in (np.linspace(.3, .4, 7), np.linspace(.53, .56, 11), np.linspace(.75, .9, 5)):
        from ssb_tools.initial_unroll import candidate_bands
        everything = cpu_tile(sampler, angles, xs, range(len(sampler.segments)))
        filtered = cpu_tile(sampler, angles, xs, candidate_bands(sampler, xs))
        for expected, actual in zip(everything, filtered): np.testing.assert_array_equal(expected, actual)


def test_native_rows_equal_the_former_float_cache_and_verify_blocks(tmp_path):
    rng = np.random.default_rng(5)
    raw = rng.integers(0, 256, size=(40, 32), dtype=np.uint8); raw[3, 7] = 255
    flat = dict(offset=rng.uniform(0, 8, 32).tolist(), gain=rng.uniform(.8, 1.6, 32).tolist(),
                valid=(rng.uniform(size=32) > .1).tolist())
    cache, valid = flat_correct(raw, flat); cache[~valid] = np.nan  # the former sensor_flat.npy content
    (tmp_path/'raw').mkdir()
    blocks = []
    for first in (0, 16, 30):
        rows = {0: 16, 16: 14, 30: 10}[first]
        path = tmp_path/'raw'/f'b{first}.u8'; raw[first:first+rows].tofile(path)
        blocks.append(dict(file=path.name, first_sequence=first, rows=rows, sha256=sha256_file(path)))
    sequences = np.arange(4, 37)  # an ROI spanning three blocks
    native = NativeRows(tmp_path/'raw', blocks, sequences, 32, flat)
    expected = cache[sequences]
    np.testing.assert_array_equal(native.rows(np.arange(len(sequences))), expected)
    ids = np.array([5, 0, 30, 5]); cols = rng.integers(0, 32, size=(4, 6))
    a, b = native.gather(ids, cols, cols[::-1])
    np.testing.assert_array_equal(a, expected[ids[:, None], cols])
    np.testing.assert_array_equal(b, expected[ids[:, None], cols[::-1]])
    np.testing.assert_array_equal(MemoryRows(raw, flat).rows(np.arange(40)), cache)
    with (tmp_path/'raw/b16.u8').open('r+b') as f: f.write(b'\x00')
    with pytest.raises(ValueError, match='hash mismatch'):
        NativeRows(tmp_path/'raw', blocks, sequences, 32, flat).rows([20])
    escaped = [dict(blocks[0], file='../outside.u8')]
    (tmp_path/'outside.u8').write_bytes(raw[:16].tobytes())
    with pytest.raises(ValueError, match='escapes'):
        NativeRows(tmp_path/'raw', escaped, np.arange(4), 32, flat).rows([0])
    with pytest.raises(ValueError, match='not covered'):
        NativeRows(tmp_path/'raw', blocks[:1], sequences, 32, flat)


def public_fixture(root):
    (root/'config').mkdir(parents=True); (root/'metadata').mkdir(); (root/'raw').mkdir()
    width = 32; step = 2*math.pi/4096
    phase = np.arange(math.ceil(-.048/step), math.floor(.048/step)+1)*step
    times = np.r_[phase+.05, phase+.05+2*math.pi]
    segments = np.repeat([0, 1], len(phase))
    config = dict(robot=dict(scan_axis_height_m=1.), schema='ssb.observable_config.v1', camera=dict(width=width, optical_signature='measured', nominal_distance_m=1.),
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
    # No native float cache: rows come from the hash-recorded raw blocks on demand.
    assert not (out/'sensor_flat.npy').exists() and not (out/'mosaic_u16.npy').exists()
    source = json.loads((out/'native_source.json').read_text())
    assert [b['file'] for b in source['blocks']] == ['block.u8'] and source['width'] == 32
    runs = np.fromfile(out/'coverage_runs.bin', RUN_DTYPE)
    assert np.sum(runs['x_end']-runs['x_begin']) == report['coverage']['total_pixels']
    assert report['coverage']['overlapping_pixels'] > 0 and report['mosaic'] == dict(mode='none')
    assert (out/'review.html').is_file() and (out/'provenance.json').is_file() and (out/'overlap.png').is_file()
    full = tmp_path/'full'
    reconstruct(root, cal, full, pitch=.004, angular_tile_rows=3, mosaic='full', tile_columns=5)
    code = np.load(full/'mosaic_u16.npy'); count = np.load(full/'mosaic_count.npy')
    np.testing.assert_array_equal(np.fromfile(full/'coverage_runs.bin', RUN_DTYPE), runs)
    assert np.array_equal(code != MOSAIC_INVALID, count > 0)
    q, x = np.argwhere(count > 0)[len(np.argwhere(count > 0))//2]
    traced = trace_pixel(full, int(q), int(x))
    assert traced['valid'] and traced['error_dn'] <= .5/64+1e-9
    assert sum(v['weight'] for v in traced['contributions']) == pytest.approx(1.)
    assert sum(v['weight']*v['flat_dn'] for v in traced['contributions']) == pytest.approx(traced['recomputed_dn'], abs=1e-5)
    assert trace_pixel(out, int(q), int(x))['recomputed_dn'] == traced['recomputed_dn']
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


def test_sensor_reads_only_supporting_bands_and_rejects_used_block_corruption(tmp_path):
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
    from ssb_tools.initial_unroll import prepare_sensor
    # A partial export is insufficient when the second scan is within the
    # declared correction envelope, even without nominal ROI pixels.
    with pytest.raises(ValueError, match='missing public input'):
        reconstruct(root, cal, tmp_path/'insufficient', [.5, .501], pitch=.004)
    capture = PublicCapture(root, cal)
    # Isolate source preparation with a distant public nominal scan.
    capture.x_axis[half:] += 1.
    output = tmp_path/'roi'; output.mkdir()
    sampler, _, report = prepare_sensor(capture, output, [.5, .501])
    sampler.native.close()
    assert len(report['raw_blocks']) == 1
    assert not (root/'raw/not_exported.u8').exists()
    with path.open('r+b') as f: f.write(b'\xff')
    corrupt = tmp_path/'corrupt'; corrupt.mkdir()
    with pytest.raises(ValueError, match='raw block hash'):
        prepare_sensor(capture, corrupt, [.5, .501])
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


@pytest.mark.parametrize('missing', [False, True])
def test_roi_retains_whole_recorded_scans_without_filling_real_gaps(missing):
    from ssb_tools.initial_unroll import retained_band_rows
    from ssb_tools.reconstruction_support import correction_reach_m
    # Independent spatial helix; outer scans contain no nominal ROI pixels,
    # but can reach the ROI under the declared motion correction.
    phase = np.linspace(-125., 125., 601)*math.pi/180
    rows = np.zeros(7*len(phase), PROJECTION)
    rows['sequence'] = np.arange(len(rows))
    rows['segment'] = np.repeat(np.arange(1, 8), len(phase))
    axis = np.repeat(-1.+np.arange(7)*.6, len(phase))+np.tile(phase, 7)*.6/(2*math.pi)
    if missing:
        keep = rows['sequence'] != 3*601+300
        rows, axis = rows[keep], axis[keep]
    selected = retained_band_rows(rows, axis, np.array([-.42, .42]), [1., 1.3],
                                  correction_reach_m(2.75, 1.715))
    expected = np.flatnonzero((rows['segment'] >= 3) & (rows['segment'] <= 6))
    np.testing.assert_array_equal(selected, expected)
    if missing:
        assert (rows['sequence'][selected] != 3*601+300).all()
    # Each retained scan preserves its actual public endpoints exactly.
    for segment in (3, 4, 5, 6):
        actual = rows['sequence'][selected][rows['segment'][selected] == segment]
        np.testing.assert_array_equal(actual, rows['sequence'][rows['segment'] == segment])


def test_sensor_preparation_retains_source_rows_outside_diagnostic_roi(tmp_path):
    from ssb_tools.initial_unroll import prepare_sensor
    root = tmp_path/'capture'; cal, _ = public_fixture(root)
    capture = PublicCapture(root, cal); output = tmp_path/'d1'; output.mkdir()
    sampler, _, report = prepare_sensor(capture, output, [.605, .609])
    try:
        np.testing.assert_array_equal(sampler.projection['sequence'], capture.rows['sequence'])
        assert report['selected_rows'] == len(capture.rows)
        assert report['correction_reach_m'] == pytest.approx(.11)
    finally:
        sampler.native.close()


@pytest.mark.parametrize('radius,height', [(0., 1.), (1., 0.), (float('nan'), 1.), (1., float('inf'))])
def test_public_support_bound_rejects_invalid_nominal_geometry(radius, height):
    from ssb_tools.reconstruction_support import correction_reach_m
    with pytest.raises(ValueError, match='positive nominal'):
        correction_reach_m(radius, height)
