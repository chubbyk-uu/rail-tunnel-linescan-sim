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
