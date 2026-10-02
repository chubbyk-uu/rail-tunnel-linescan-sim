"""CUDA is tested against analytic data and the independent NumPy sampler."""
import json
import numpy as np
import pytest
from test_initial_unroll import analytic_sampler, public_fixture
from ssb_tools.initial_unroll import rasterize, reconstruct, trace_pixel
from ssb_tools.unroll_cuda import CudaRaster
from ssb_tools.wall_coverage import target_grid


@pytest.mark.parametrize('missing', [False, True])
@pytest.mark.parametrize('tile', [1, 7, 32])
def test_cuda_matches_reference_with_missing_lines_and_native_saturation(tmp_path, missing, tile):
    sampler = analytic_sampler(remove_centre=missing)
    sampler.image[:, 16] = np.nan
    sampler.geometry_valid[[0, 7, -1]] = False
    grid = target_grid([.35, .77], 1., [-.14, .14], .004)
    root = tmp_path/'cpu'; root.mkdir()
    _, reference = rasterize(sampler, grid, root, tile)
    root = tmp_path/'cuda'; root.mkdir()
    accelerator = CudaRaster(sampler)
    try:
        _, actual = rasterize(sampler, grid, root, tile, accelerator)
        info = accelerator.describe()
        assert info['allocated_peak_bytes'] < info['allocation_budget_bytes']
        assert info['device'] and info['abi'] == 1
    finally:
        accelerator.close()
    np.testing.assert_array_equal(np.isfinite(actual[0]), np.isfinite(reference[0]))
    np.testing.assert_allclose(actual[0], reference[0], atol=1e-5, rtol=0, equal_nan=True)
    for expected, result in zip(reference[1:], actual[1:]): np.testing.assert_array_equal(expected, result)


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
    counts = np.load(output/'coverage.npy')
    for q, x in np.argwhere(counts > 0)[::max(1, np.count_nonzero(counts)//7)]:
        trace = trace_pixel(output, int(q), int(x))
        assert trace['valid'] and trace['error_dn'] <= 1e-5


def test_cuda_errors_do_not_fall_back_to_cpu(monkeypatch):
    import ssb_tools.unroll_cuda as adapter
    sampler = analytic_sampler()
    def missing_library(*args): raise OSError('intentional missing CUDA library')
    monkeypatch.setattr(adapter.ct, 'CDLL', missing_library)
    with pytest.raises(OSError, match='intentional'): CudaRaster(sampler)


def test_cuda_rejects_nonmonotonic_geometry_and_excessive_tile():
    sampler = analytic_sampler()
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
