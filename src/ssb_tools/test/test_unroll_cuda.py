"""CUDA is tested against analytic data and the independent NumPy sampler."""
import json
import numpy as np
import pytest
from test_initial_unroll import analytic_sampler, public_fixture, read_products
from ssb_tools.initial_unroll import rasterize, reconstruct, trace_pixel
from ssb_tools.wall_coverage import RUN_DTYPE
from ssb_tools.unroll_cuda import CudaRaster
from ssb_tools.wall_coverage import target_grid


@pytest.mark.parametrize('missing', [False, True])
@pytest.mark.parametrize('tile', [(1, 1), (7, 5), (32, 8192)])
def test_cuda_matches_reference_with_missing_lines_and_native_saturation(tmp_path, missing, tile):
    sampler = analytic_sampler(remove_centre=missing, raw=True)
    sampler.native.raw_rows[:, 16] = 255          # saturated native column
    sampler.native.raw_rows[3, 9] = 255           # one saturated sample
    sampler.native.valid[22] = False              # flat-invalid column
    sampler.geometry_valid[[0, 7, -1]] = False
    grid = target_grid([.35, .77], 1., [-.14, .14], .004)
    mosaic = (0, grid['shape'][1])
    root = tmp_path/'cpu'; root.mkdir()
    _, cpu_cover = rasterize(sampler, grid, root, tile[0], mosaic=mosaic, tile_columns=tile[1])
    reference = read_products(root)
    root = tmp_path/'cuda'; root.mkdir()
    accelerator = CudaRaster(sampler)
    try:
        _, cuda_cover = rasterize(sampler, grid, root, tile[0], accelerator, mosaic, tile[1])
        info = accelerator.describe()
        assert info['allocated_peak_bytes'] < info['allocation_budget_bytes']
        assert info['device'] and info['abi'] == 2
    finally:
        accelerator.close()
    actual = read_products(root)
    # Mosaic codes: identical mask; values within one 1/64 DN rounding step.
    np.testing.assert_array_equal(actual[0] == 65535, reference[0] == 65535)
    assert np.max(np.abs(actual[0].astype(int)-reference[0].astype(int))) <= 1
    for expected, result in zip(reference[1:], actual[1:]): np.testing.assert_array_equal(expected, result)
    np.testing.assert_array_equal(cpu_cover, cuda_cover)
    assert np.any(reference[0] == 65535) and np.any(reference[1] > 1)


def test_cuda_public_only_end_to_end_retains_provenance_and_trace(tmp_path):
    root = tmp_path/'public'; calibration, _ = public_fixture(root)
    output = tmp_path/'cuda'
    report = reconstruct(root, calibration, output, pitch=.004, angular_tile_rows=7, backend='cuda')
    assert report['backend']['name'] == 'cuda'
    assert not (root/'evaluation').exists()
    provenance = json.loads((output/'provenance.json').read_text())
    binary = provenance['parameters']['backend']
    assert len(binary['library_sha256']) == 64
    assert binary['library'] == report['backend']['library']
    assert all(str(root) in path for path in provenance['inputs'])
    assert provenance['performance']['wall_s'] >= report['performance']['wall_s']
    runs = np.fromfile(output/'coverage_runs.bin', RUN_DTYPE)
    covered = runs[runs['count'] > 0]
    for run in covered[::max(1, len(covered)//7)]:
        trace = trace_pixel(output, int(run['q_bin']), int(run['x_begin']))
        assert trace['valid'] and trace['observations'] == run['count']


def test_cuda_errors_do_not_fall_back_to_cpu(monkeypatch):
    import ssb_tools.unroll_cuda as adapter
    sampler = analytic_sampler(raw=True)
    def missing_library(*args): raise OSError('intentional missing CUDA library')
    monkeypatch.setattr(adapter.ct, 'CDLL', missing_library)
    with pytest.raises(OSError, match='intentional'): CudaRaster(sampler)


def test_cuda_rejects_nonmonotonic_geometry_and_excessive_tile():
    with pytest.raises(ValueError, match='uint8'): CudaRaster(analytic_sampler())  # float cache: CPU only
    sampler = analytic_sampler(raw=True)
    accelerator = CudaRaster(sampler)
    try:
        with pytest.raises(RuntimeError, match='tile dimensions'):
            accelerator.tile(np.array([0.]), np.zeros((1 << 20)+1))
        with pytest.raises(ValueError, match='finite'):
            accelerator.tile(np.array([np.nan]), np.array([.5]))
    finally:
        accelerator.close()
    sampler.offsets[3] = sampler.offsets[1]
    with pytest.raises(ValueError, match='mapping'): CudaRaster(sampler)


def test_feature_search_failure_releases_real_cuda_context():
    from ssb_tools.feature_review import feature_raster
    sampler = analytic_sampler(raw=True)
    with pytest.raises(RuntimeError, match='tile dimensions'):
        with feature_raster(sampler) as raster:
            assert raster.handle
            raster.tile(np.array([0.]), np.zeros((1 << 20)+1))
    assert raster.handle is None
    # A fresh context can run after the exception; no silent CPU fallback.
    with feature_raster(sampler) as next_raster:
        next_raster.tile(np.array([0.]), np.array([.5]))
    assert next_raster.handle is None


@pytest.mark.parametrize('missing', [False, True])
@pytest.mark.parametrize('fields', [4, 5, 6])
def test_global_cuda_matches_cpu_coupled_rays_and_invalid_native_samples(tmp_path, missing, fields):
    from ssb_tools.global_geometry import Trajectory, GeometrySettings
    from ssb_tools.global_cuda import GlobalCudaRaster
    from ssb_tools.global_resample import corrected_tile
    sampler = analytic_sampler(remove_centre=missing, raw=True)
    sampler.native.raw_rows[3, 9] = 255
    sampler.native.raw_rows[:, 16] = 255
    sampler.geometry_valid[7] = False
    sampler.native.valid[22] = False
    model = Trajectory(sampler, 1., .7, GeometrySettings(fit_heave=fields == 5, fit_translation=fields == 6))
    coefficients = np.empty(model.size)
    for k in range(fields):
        coefficients[model.starts[k]:model.starts[k+1]] = (.7-.25*k)+.12*np.sin(np.arange(model.sizes[k]))
    qs = np.linspace(-.14, .14, 31)
    xs = np.linspace(.35, .77, 49)
    expected, counts = corrected_tile(model, coefficients, qs, xs)
    raster = GlobalCudaRaster(model, coefficients)
    try:
        actual, number, _ = raster.tile(qs, xs)
        np.testing.assert_array_equal(number, counts)
        np.testing.assert_array_equal(np.isfinite(actual), np.isfinite(expected))
        np.testing.assert_allclose(actual, expected, rtol=0, atol=3e-5, equal_nan=True)
        assert raster.describe()['allocated_peak_bytes'] < 256 << 20
        # A zero correction must reproduce the independently tested D1 sampler.
    finally:
        raster.close()
    assert np.count_nonzero(counts) and np.any(counts == 0) and np.any(counts > 1)


def test_global_cuda_zero_correction_matches_d1_and_preserves_shifted_band_candidates(tmp_path):
    from ssb_tools.global_geometry import Trajectory
    from ssb_tools.global_cuda import GlobalCudaRaster
    from ssb_tools.global_mosaic import CpuGlobalRaster
    sampler = analytic_sampler(raw=True)
    model = Trajectory(sampler, 1., .7)
    qs, xs = np.linspace(-.09, .09, 13), np.linspace(.4, .7, 17)
    nominal, corrected = CudaRaster(sampler), GlobalCudaRaster(model, np.zeros(model.size))
    try:
        expected, n, _ = nominal.tile(qs, xs)
        actual, c, _ = corrected.tile(qs, xs)
        np.testing.assert_array_equal(c, n)
        np.testing.assert_allclose(actual, expected, rtol=0, atol=2e-5, equal_nan=True)
    finally:
        nominal.close(); corrected.close()
    # Native observations shifted beyond nominal support must still be visited.
    c = np.zeros(model.size); c[model.starts[0]:model.starts[1]] = 20.
    corrected = GlobalCudaRaster(model, c)
    try:
        x = np.array([sampler.band_x[-1][1]+.015])
        q = np.array([sampler.phases[-1][-1]])
        actual, counts, _ = corrected.tile(q, x)
        expected, n, _ = CpuGlobalRaster(model, c).tile(q, x)
        np.testing.assert_array_equal(counts, n)
        assert n[0, 0] == 1
        np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=0)
    finally:
        corrected.close()


@pytest.mark.parametrize('coupled', [False, True])
@pytest.mark.parametrize('kind', ['constant', 'groove', 'partial'])
def test_global_cuda_applies_shared_relief_and_matches_cpu_across_tiles(coupled, kind):
    from test_evaluate_global_geometry import independent_fixture
    from ssb_tools.global_cuda import GlobalCudaRaster
    from ssb_tools.global_resample import corrected_tile
    from ssb_tools.surface_relief import SurfaceRelief
    model, *_ = independent_fixture()
    offsets = model.sampler.offsets
    model.sampler.native.raw_rows[:] = np.rint(100+200*offsets).astype(np.uint8)
    c = np.zeros(model.size)
    if coupled:
        for field, value in enumerate([.7, -.4, .6, -.5]):
            c[model.starts[field]:model.starts[field+1]] = value
    qs, xs = np.linspace(-.1, .1, 21), np.linspace(.45, .55, 31)
    plain, _ = corrected_tile(model, c, qs, xs)
    data = np.full((3, 3), .02, np.float32)
    if kind == 'groove': data[:, [0, 2]] = 0.
    patch = dict(grid=[.47 if kind == 'partial' else .4, -.06, .035, .06], depth=data)
    model.relief = SurfaceRelief([patch])
    expected, counts = corrected_tile(model, c, qs, xs)
    assert np.nanmax(abs(expected-plain)) > .1
    raster = GlobalCudaRaster(model, c)
    try:
        actual, number, _ = raster.tile(qs, xs)
        np.testing.assert_array_equal(number, counts)
        np.testing.assert_allclose(actual, expected, atol=3e-5, rtol=0, equal_nan=True)
        # Reuse the context with different tile dimensions and no depth support.
        for sl in (slice(0, 5), slice(5, 17), slice(17, 21)):
            actual, number, _ = raster.tile(qs[sl], xs[7:19])
            np.testing.assert_array_equal(number, counts[sl, 7:19])
            np.testing.assert_allclose(actual, expected[sl, 7:19], atol=3e-5, rtol=0, equal_nan=True)
        assert raster.describe()['surface_relief'] is True
        assert raster.describe()['allocated_peak_bytes'] < 256 << 20
        model.relief = None
        actual, number, _ = raster.tile(qs, xs)
        reference, counts = corrected_tile(model, c, qs, xs)
        np.testing.assert_array_equal(number, counts)
        np.testing.assert_allclose(actual, reference, atol=3e-5, rtol=0, equal_nan=True)
    finally:
        raster.close()


def test_global_cuda_rejects_old_depthless_abi_and_releases_context(monkeypatch):
    import ctypes
    from ssb_tools.global_cuda import GlobalCudaRaster
    from test_evaluate_global_geometry import independent_fixture
    model, *_ = independent_fixture()
    from ament_index_python.packages import get_package_prefix
    from pathlib import Path
    library = ctypes.CDLL(str(Path(get_package_prefix('ssb_core'))/'lib/libssb_unroll_cuda.so'))
    monkeypatch.setattr(library, 'ssb_unroll_global_abi', lambda: 1)
    monkeypatch.setattr(ctypes, 'CDLL', lambda *a: library)
    closed = []
    destroy = library.ssb_unroll_destroy
    destroy.argtypes = [ctypes.c_void_p]; destroy.restype = None
    def record_destroy(handle):
        closed.append(handle); destroy(handle)
    # The adapter assigns ctypes signatures to the wrapper just as to a symbol.
    monkeypatch.setattr(library, 'ssb_unroll_destroy', record_destroy)
    with pytest.raises(RuntimeError, match='global ABI'):
        GlobalCudaRaster(model, np.zeros(model.size))
    assert len(closed) == 1


@pytest.mark.parametrize('coupled', [False, True])
def test_cuda_inverse_preserves_finite_nearest_row_footprints_and_real_gaps(coupled):
    from ssb_tools.global_geometry import Trajectory
    from ssb_tools.global_cuda import GlobalCudaRaster
    from ssb_tools.global_resample import corrected_tile
    from ssb_tools.initial_unroll import BandSampler, PROJECTION
    from ssb_tools.native_rows import MemoryRows
    p = np.zeros(3, PROJECTION)
    p['sequence'] = p['lattice_row'] = np.arange(3)
    p['theta_rad'] = [-.011, 0., .011]
    p['x_axis_m'] = [.4, .401, .402]
    offsets = np.linspace(-.1, .1, 33)
    pixels = np.broadcast_to(np.arange(33, dtype=np.uint8)+80, (3, 33)).copy()
    flat = dict(offset=np.zeros(33), gain=np.ones(33), valid=np.ones(33, bool))
    sampler = BandSampler(p, MemoryRows(pixels, flat), offsets, offsets, np.ones(33, bool), .01)
    model = Trajectory(sampler, 1., .7)
    c = np.zeros(model.size)
    if coupled:
        for k, correction in enumerate([.3, -.4, .6, -.5]):
            c[model.starts[k]:model.starts[k+1]] = correction
    # Nearest-row interiors, outer footprint, and an actual unsupported gap.
    qs, xs = np.array([-.013, -.0055, -.002, .002]), np.array([.395, .405, .415])
    if coupled:
        from test_global_optimization import independent_hits
        qs[1] = independent_hits(np.array([.4]), np.array([-.0055]), np.array([.005]),
                                 np.array([[.0003, -.0004, .0006, -.0005]]), 1., .7)[0, 1]
    expected, counts = corrected_tile(model, c, qs, xs)
    assert counts[0].all() and counts[2:].all() and not counts[1].any()
    raster = GlobalCudaRaster(model, c)
    try:
        actual, number, _ = raster.tile(qs, xs)
        np.testing.assert_array_equal(number, counts)
        np.testing.assert_allclose(actual, expected, atol=3e-5, rtol=0, equal_nan=True)
    finally:
        raster.close()


def test_cuda_row_centre_ignores_only_zero_weight_neighbours():
    from test_evaluate_global_geometry import independent_fixture
    from ssb_tools.global_cuda import GlobalCudaRaster
    model, *_ = independent_fixture()
    model.sampler.native.raw_rows[[49, 51]] = 255
    c = np.zeros(model.size); c[:model.sizes[0]] = 10.
    raster = GlobalCudaRaster(model, c)
    try:
        _, count, _ = raster.tile(np.array([0., 4e-16, 1e-9]), np.array([.5]))
        # Band 0 is invalid only for the genuinely nonzero neighbour weight;
        # band 1 remains valid throughout.
        assert count[:, 0].tolist() == [2, 2, 1]
    finally:
        raster.close()


def test_global_cuda_row_vectors_match_independent_matrix_ray_equations():
    from test_global_optimization import independent_hits
    from ssb_tools.global_geometry import Trajectory, GeometrySettings
    from ssb_tools.global_cuda import row_rays
    sampler = analytic_sampler(raw=True)
    model = Trajectory(sampler, 1., .7, GeometrySettings(fit_translation=True))
    constants = np.array([.0007, -.0004, .0013, -.0009, .0008, -.0011])
    c = np.concatenate([np.full(n, value/model.scale) for n, value in zip(model.sizes, constants)])
    rays = row_rays(model, c)
    tangent = np.linspace(-.1, .1, len(rays))
    expected = independent_hits(rays['axis'], rays['phase'], tangent,
                                np.tile(constants, (len(rays), 1)), 1., .7)
    origin = np.column_stack([rays[k] for k in ('ox', 'oy', 'oz')])
    direction = np.column_stack((rays['tx']*tangent+rays['rx'], rays['ry'], rays['tz']*tangent+rays['rz']))
    aa = np.sum(direction[:, 1:]**2, axis=1)
    bb = 2*np.sum(origin[:, 1:]*direction[:, 1:], axis=1)
    cc = np.sum(origin[:, 1:]**2, axis=1)-1
    hit = origin+((-bb+np.sqrt(bb*bb-4*aa*cc))/(2*aa))[:, None]*direction
    actual = np.column_stack((hit[:, 0], np.arctan2(hit[:, 1], hit[:, 2])))
    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-14)


@pytest.mark.parametrize('saturated', [False, True])
@pytest.mark.parametrize('relief', [False, True])
def test_full_global_mosaic_public_only_quantized_output_and_exact_coverage(tmp_path, saturated, relief):
    from ssb_tools.global_geometry import Trajectory, GeometrySettings
    from ssb_tools.global_mosaic import run
    from ssb_tools.match_bands import verified_bands
    from ssb_tools.provenance import stage_record
    from ssb_tools.session import sha256_file
    root = tmp_path/'public'; cal, index = public_fixture(root)
    if saturated:
        data = np.fromfile(root/'raw/block.u8', np.uint8).reshape(-1, 32)
        data[:, 16] = 255; data.tofile(root/'raw/block.u8')
        index['blocks'][0]['sha256'] = sha256_file(root/'raw/block.u8')
        (root/'raw/index.json').write_text(json.dumps(index))
        summary = json.loads((root/'session.json').read_text())
        summary['files']['raw/index.json'] = sha256_file(root/'raw/index.json')
        (root/'session.json').write_text(json.dumps(summary))
    # This fixture keeps target centres inside captured centres. Separate tests
    # cover valid nearest-row footprints beyond centres and reject real gaps.
    config = json.loads((root/'config/observable_config.json').read_text())
    config['inspection']['theta_rad'] = [-.04, .04]
    (root/'config/observable_config.json').write_text(json.dumps(config))
    summary = json.loads((root/'session.json').read_text())
    summary['files']['config/observable_config.json'] = sha256_file(root/'config/observable_config.json')
    (root/'session.json').write_text(json.dumps(summary))
    d1 = tmp_path/'d1'; reconstruct(root, cal, d1, pitch=.004)
    sampler, upstream, inputs = verified_bands(d1)
    settings = GeometrySettings()
    model = Trajectory(sampler, 1., .7, settings)
    fit = tmp_path/'fit'; fit.mkdir()
    (fit/'trajectory.json').write_text(json.dumps(model.serialize(np.zeros(model.size))))
    (fit/'windows.json').write_text('[]')
    fit_report = dict(schema='ssb.global_optimization.v1',
        settings=vars(settings), grid=upstream['grid'], optical_signature=upstream['optical_signature'],
        source_observation_hashes=upstream['source_observation_hashes'])
    if relief:
        from dataclasses import asdict
        from ssb_tools.surface_relief import ReliefSettings, SurfaceRelief
        SurfaceRelief([dict(grid=[.2, -.2, .8, .4], depth=np.full((2, 2), .02, np.float32))]).save(fit/'surface_relief.npz')
        fit_report['surface_relief'] = dict(schema='ssb.stereo_radial_relief.v1', settings=asdict(ReliefSettings()))
    (fit/'report.json').write_text(json.dumps(fit_report))
    (fit/'provenance.json').write_text(json.dumps(stage_record('global_optimization',
        [d1/name for name in ('projection.npy', 'mapping.npz', 'bands.json')], sorted(fit.iterdir()), {})))
    sampler.native.close()
    baseline = {p: sha256_file(p) for folder in (root, d1, fit) for p in folder.rglob('*') if p.is_file()}
    output = tmp_path/'mosaic'
    report = run(d1, fit, output, angular_tile_rows=3, tile_columns=5)
    products = [read_products(output/name) for name in ('nominal', 'optimized')]
    if not relief:
        for a, b in zip(*products): np.testing.assert_array_equal(a, b)
    else:
        cpu = tmp_path/'cpu_mosaic'
        run(d1, fit, cpu, backend='cpu', angular_tile_rows=2, tile_columns=7)
        reference = read_products(cpu/'optimized')
        assert np.max(abs(products[1][0].astype(int)-reference[0].astype(int))) <= 1
        for actual, expected in zip(products[1][1:], reference[1:]): np.testing.assert_array_equal(actual, expected)
        assert np.any(products[0][0] != products[1][0])
        assert report['products']['optimized']['backend']['surface_relief']
    code, count, runs = products[1]
    assert np.array_equal(code == 65535, count == 0)
    stats = report['products']['optimized']['coverage']
    assert stats['missing_pixels'] == int(np.count_nonzero(count == 0))
    assert np.sum(runs['x_end']-runs['x_begin']) == count.size
    assert report['coverage_gate']['status'] == ('fail' if saturated else 'pass')
    assert report['performance']['peak_rss_bytes'] > 0
    provenance = json.loads((output/'provenance.json').read_text())
    assert provenance['stage'] == 'global_mosaic'
    assert all(sha256_file(path) == digest for path, digest in provenance['outputs'].items())
    assert all(sha256_file(path) == digest for path, digest in baseline.items())
    assert not (root/'evaluation').exists() and (output/'review.html').is_file()
    from ssb_tools.validate_global_mosaic import validate
    checked = validate(d1, fit, output, tmp_path/'validation')
    assert checked['full_pixel_consistency'] and checked['total_pixels'] == count.size
    assert checked['coverage_gate'] == report['coverage_gate']['status']
    assert checked['status'] == ('fail' if saturated else 'pass')
    assert checked['cpu_reference']['maximum_code_error'] <= 1
    assert (checked['cpu_reference']['relief_probes'] > 0) is relief
    with (output/'optimized/mosaic_u16.npy').open('r+b') as file:
        file.seek(-2, 2); file.write(b'xx')
    with pytest.raises(ValueError, match='product hash mismatch'):
        validate(d1, fit, output, tmp_path/'rejected_validation')
    with pytest.raises(ValueError, match='separate'):
        run(d1, fit, d1/'wrong')


def test_global_cuda_rejects_nonfinite_or_unbounded_parameters_and_excessive_tiles():
    from ssb_tools.global_geometry import Trajectory
    from ssb_tools.global_cuda import GlobalCudaRaster
    sampler = analytic_sampler(raw=True); model = Trajectory(sampler, 1., .7)
    c = np.zeros(model.size); c[model.starts[2]] = 11.
    with pytest.raises(ValueError, match='physical bounds'):
        GlobalCudaRaster(model, c)
    c[:] = np.nan
    with pytest.raises(ValueError, match='finite'):
        GlobalCudaRaster(model, c)
    raster = GlobalCudaRaster(model, np.zeros(model.size))
    try:
        with pytest.raises(RuntimeError, match='tile dimensions'):
            raster.tile(np.array([0.]), np.zeros((1 << 20)+1))
        with pytest.raises(ValueError, match='finite ordered'):
            raster.tile(np.array([np.nan]), np.array([.5]))
    finally:
        raster.close()


def test_global_cuda_matches_cpu_at_declared_motion_bounds_and_large_native_window(tmp_path):
    from ssb_tools.global_geometry import Trajectory, GeometrySettings
    from ssb_tools.global_cuda import GlobalCudaRaster
    from ssb_tools.global_resample import corrected_tile
    from ssb_tools.initial_unroll import BandSampler, PROJECTION
    from ssb_tools.native_rows import MemoryRows
    phase = np.linspace(-2.2, 2.2, 44001)
    p = np.zeros(len(phase), PROJECTION)
    p['sequence'] = p['lattice_row'] = np.arange(len(p))
    p['theta_rad'] = phase; p['x_axis_m'] = 12.+.1*phase
    width = 32; offsets = np.linspace(-.4, .4, width)
    raw = np.random.default_rng(52).integers(0, 255, (len(p), width), dtype=np.uint8)
    flat = dict(offset=[3.]*width, gain=[1.1]*width, valid=[True]*width)
    sampler = BandSampler(p, MemoryRows(raw, flat), offsets, offsets, np.ones(width, bool), .0001001)
    model = Trajectory(sampler, 2.75, 1.715, GeometrySettings(fit_translation=True))
    c = np.concatenate([np.full(n, v) for n, v in zip(model.sizes, [29., -29., 9.9, -9.9, 4.9, -4.9])])
    q = 2.75*np.array([-2.19, -.3, 0., .7, 2.19]); x = np.linspace(11.75, 12.45, 19)
    expected, count = corrected_tile(model, c, q, x)
    engine = GlobalCudaRaster(model, c)
    try:
        # Individual row tiles exercise compact loading, not whole-band upload.
        for i in range(len(q)):
            actual, n, _ = engine.tile(q[i:i+1]/2.75, x)
            np.testing.assert_array_equal(n, count[i:i+1])
            np.testing.assert_allclose(actual, expected[i:i+1], atol=3e-5, rtol=0, equal_nan=True)
    finally:
        engine.close()
