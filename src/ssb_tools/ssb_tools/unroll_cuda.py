"""Bounded CUDA tile adapter; inputs are measured mappings and native pixels."""
import ctypes as ct
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_prefix
from .session import sha256_file

ROW = np.dtype([('lower', '<i4'), ('upper', '<i4'), ('weight', '<f8'),
                ('supported', '<i4'), ('reserved', '<i4')])


def pointer(array):
    if not array.flags.c_contiguous: raise ValueError('CUDA input must be contiguous')
    return ct.c_void_p(array.ctypes.data)


class CudaRaster:
    def __init__(self, sampler):
        self.sampler = sampler
        self.path = Path(get_package_prefix('ssb_core'))/'lib/libssb_unroll_cuda.so'
        self.lib = ct.CDLL(str(self.path))
        signatures = {
            'abi': ([], ct.c_int), 'error': ([], ct.c_char_p),
            'create': ([ct.c_int, ct.c_void_p, ct.c_void_p, ct.c_void_p], ct.c_void_p),
            'destroy': ([ct.c_void_p], None), 'device': ([ct.c_void_p], ct.c_char_p),
            'peak_bytes': ([ct.c_void_p], ct.c_size_t),
            'begin': ([ct.c_void_p, ct.c_int, ct.c_int, ct.c_void_p], ct.c_int),
            'band': ([ct.c_void_p, ct.c_int, ct.c_void_p, ct.c_void_p,
                      ct.c_void_p, ct.c_int, ct.c_int, ct.c_int], ct.c_int),
            'finish': ([ct.c_void_p, ct.c_void_p, ct.c_void_p, ct.c_void_p], ct.c_int),
        }
        for name, (args, result) in signatures.items():
            function = getattr(self.lib, 'ssb_unroll_'+name)
            function.argtypes, function.restype = args, result
        if self.lib.ssb_unroll_abi() != 1: raise RuntimeError('unsupported CUDA unroll ABI')
        native = np.ascontiguousarray(sampler.offsets, np.float64)
        output = np.ascontiguousarray(sampler.output_offsets, np.float64)
        valid = np.ascontiguousarray(sampler.geometry_valid, np.uint8)
        if (native.ndim != 1 or len(native) < 2 or native.shape != output.shape or
            native.shape != valid.shape or not np.isfinite(native).all() or
            not np.isfinite(output).all() or not np.all(np.diff(native) > 0) or
            not np.all(np.diff(output) > 0) or not valid.any()):
            raise ValueError('invalid measured CUDA mapping')
        if sampler.pixels.dtype != np.float32 or sampler.pixels.shape[1] != len(native):
            raise ValueError('CUDA requires native float32 samples')
        self.handle = self.lib.ssb_unroll_create(len(native), pointer(native), pointer(output), pointer(valid))
        if not self.handle: raise RuntimeError(self.lib.ssb_unroll_error().decode())

    def check(self, result):
        if result: raise RuntimeError('CUDA unroll: '+self.lib.ssb_unroll_error().decode())

    def close(self):
        if self.handle:
            self.lib.ssb_unroll_destroy(self.handle); self.handle = None

    def describe(self):
        return dict(name='cuda', device=self.lib.ssb_unroll_device(self.handle).decode(),
                    library=str(self.path), abi=1,
                    library_sha256=sha256_file(self.path),
                    allocated_peak_bytes=self.lib.ssb_unroll_peak_bytes(self.handle),
                    allocation_budget_bytes=256 << 20,
                    geometry_precision='float64', radiometry_precision='float32; fmad disabled')

    def tile(self, angles, xs):
        sampler = self.sampler
        angles = np.asarray(angles, np.float64)
        xs = np.ascontiguousarray(xs, np.float64)
        if angles.ndim != 1 or xs.ndim != 1 or not np.isfinite(angles).all() or not np.isfinite(xs).all():
            raise ValueError('finite one-dimensional CUDA grid required')
        self.check(self.lib.ssb_unroll_begin(self.handle, len(angles), len(xs), pointer(xs)))
        usable = sampler.output_offsets[sampler.geometry_valid]
        for band in range(len(sampler.segments)):
            lo, hi, weight, supported = sampler.row_sources(band, angles)
            if not np.any(supported): continue
            axes = sampler.projection['x_axis_m'][np.r_[lo[supported], hi[supported]]]
            left = int(np.searchsorted(xs, axes.min()+usable[0], side='left'))
            right = int(np.searchsorted(xs, axes.max()+usable[-1], side='right'))
            if right <= left: continue
            # Compact contiguous acquisition range: no full-image GPU upload.
            first = int(min(lo.min(), hi.min())); last = int(max(lo.max(), hi.max()))+1
            if (last-first)*sampler.pixels.shape[1]*4 > 128 << 20:
                raise ValueError('CUDA native input exceeds 128 MiB')
            samples = np.ascontiguousarray(sampler.pixels[first:last])
            axis = np.ascontiguousarray(sampler.projection['x_axis_m'][first:last])
            rows = np.zeros(len(angles), ROW)
            rows['lower'] = lo-first; rows['upper'] = hi-first
            rows['weight'] = weight; rows['supported'] = supported
            self.check(self.lib.ssb_unroll_band(self.handle, last-first, pointer(samples),
                       pointer(axis), pointer(rows), left, right, band))
        shape = (len(angles), len(xs))
        image = np.empty(shape, np.float32); count = np.empty(shape, np.uint16)
        source = np.empty(shape, np.int16)
        self.check(self.lib.ssb_unroll_finish(self.handle, pointer(image), pointer(count), pointer(source)))
        return image, count, source
