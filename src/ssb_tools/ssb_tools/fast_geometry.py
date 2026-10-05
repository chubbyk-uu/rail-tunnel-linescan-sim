"""Installed fused ray numerics; public arrays only, no renderer or scene."""
import ctypes as ct
from contextlib import contextmanager, ExitStack
from contextvars import ContextVar
from functools import lru_cache
from pathlib import Path
import weakref

import numpy as np
from ament_index_python.packages import get_package_prefix

from .session import sha256_file

_scope = ContextVar('public_geometry_resources', default=None)


@contextmanager
def numeric_scope():
    """Explicit CUDA cleanup on successful fits and all exceptional exits."""
    with ExitStack() as resources:
        token = _scope.set(resources)
        try:
            yield
        finally:
            _scope.reset(token)


class Block(ct.Structure):
    _fields_ = [('sign', ct.c_double), ('side', ct.c_int), ('field', ct.c_int),
                ('groups', ct.c_int), ('masks', ct.c_uint32*4),
                ('basis', ct.c_void_p), ('inverse', ct.c_void_p), ('parameter_basis', ct.c_void_p)]


def pointer(array):
    return ct.c_void_p(array.ctypes.data)


def pointers(arrays):
    return (ct.c_void_p*len(arrays))(*(a.ctypes.data for a in arrays))


@lru_cache(maxsize=1)
def backend():
    path = Path(get_package_prefix('ssb_core'))/'lib/libssb_ray_numeric.so'
    lib = ct.CDLL(str(path))
    lib.ssb_ray_abi.restype = ct.c_int
    if lib.ssb_ray_abi() != 2:
        raise RuntimeError('unsupported public ray CPU ABI')
    lib.ssb_ray_hits.argtypes = [ct.c_int64, ct.c_int, ct.c_double, ct.c_double,
                                *([ct.c_void_p]*4), ct.c_int, ct.c_void_p]
    lib.ssb_ray_hits.restype = ct.c_int
    lib.ssb_ray_jacobian.argtypes = [ct.c_int64, ct.c_int64, ct.c_int, ct.c_double, ct.c_double,
                                    *([ct.c_void_p]*7), ct.c_int, *([ct.c_void_p]*5)]
    lib.ssb_ray_jacobian.restype = ct.c_int
    lib.ssb_ray_difference.argtypes = [ct.c_int64, ct.c_int64, ct.c_int, ct.c_double, ct.c_double,
                                      *([ct.c_void_p]*6)]
    lib.ssb_ray_difference.restype = ct.c_int
    return lib, dict(name='cpu_fused_analytic_rays', abi=2, geometry_precision='float64',
                     library=str(path), library_sha256=sha256_file(path),
                     source='public native rays and image-fitted coefficients only')


def ray_values(axis, phase, tangent, correction, radius, height, derivatives=False):
    arrays = [np.ascontiguousarray(a, np.float64) for a in (axis, phase, tangent, correction)]
    count = len(arrays[0])
    if (any(a.shape != (count,) for a in arrays[:3]) or arrays[3].ndim != 2 or
            arrays[3].shape[0] != count or arrays[3].shape[1] not in (4, 5, 6)):
        raise ValueError('matching public ray arrays and four to six correction fields required')
    fields = arrays[3].shape[1]
    result = np.empty((count, fields, 2) if derivatives else (count, 2), np.float64)
    code = backend()[0].ssb_ray_hits(count, fields, radius, height,
                                    *map(pointer, arrays), int(derivatives), pointer(result))
    if code:
        raise ValueError(f'invalid public ray geometry ({code})')
    return result


class CpuJacobian:
    def __init__(self, assembly):
        self.assembly = weakref.proxy(assembly)
        self.arrays = [[np.ascontiguousarray(getattr(side, name), np.float64)
                        for side in assembly.sides] for name in ('axis', 'theta', 'tangent', 'weights')]
        self.sources = [pointers(arrays) for arrays in self.arrays]
        self.keep = []
        self.blocks = (Block*len(assembly.blocks))()
        for block, (sign, side, field, groups, values, inverse) in zip(self.blocks, assembly.blocks):
            values = np.ascontiguousarray(values, np.float64)
            inverse = np.ascontiguousarray(inverse, np.int64)
            if values.shape != (assembly.n, len(groups), 4) or inverse.shape != (assembly.n, 4*len(groups)):
                raise ValueError('invalid fused Jacobian group pattern')
            self.keep.extend((values, inverse))
            block.sign, block.side, block.field, block.groups = sign, side, field, len(groups)
            block.masks[:] = [sum(1 << k for k in g) for g in groups]+[0]*(4-len(groups))
            block.basis, block.inverse = values.ctypes.data, inverse.ctypes.data
        self.lib = backend()[0]

    def describe(self):
        return dict(backend()[1], scope='CPU fit residuals, analytic Jacobian and diagnostics')

    def difference(self, local):
        from .global_geometry import in_chunks
        a = self.assembly
        output = np.empty((a.n, 2), np.float64)
        corrections = pointers(local)
        def fill(first, last):
            code = self.lib.ssb_ray_difference(first, last, len(a.model.fields), a.model.radius, a.model.height,
                *self.sources, corrections, pointer(output))
            if code:
                raise ValueError(f'invalid fused public ray difference ({code})')
        in_chunks(fill, a.n, .25)
        return output

    def fill(self, first, last, local, current, pitch, output):
        a = self.assembly
        arrays = [*local, current, pitch, output]
        if (not 0 <= first <= last <= a.n or len(local) != 2 or
                any(value.shape != (4*a.n, len(a.model.fields)) for value in local) or
                current.shape != (a.n, 2) or pitch.shape != (2,) or output.shape != (2*a.nnz,) or
                any(value.dtype != np.float64 or not value.flags.c_contiguous for value in arrays)):
            raise ValueError('fused Jacobian row range outside observations')
        code = self.lib.ssb_ray_jacobian(first, last, len(a.model.fields), a.model.radius, a.model.height,
            *self.sources, pointers(local), pointer(a.starts), ct.cast(self.blocks, ct.c_void_p), len(self.blocks),
            pointer(a.scale_positions) if a.scale_positions is not None else None,
            pointer(a.scale_derivative), pointer(current), pointer(pitch), pointer(output))
        if code:
            raise ValueError(f'invalid fused public Jacobian ({code})')


@lru_cache(maxsize=1)
def cuda_backend():
    path = Path(get_package_prefix('ssb_core'))/'lib/libssb_ray_cuda.so'
    lib = ct.CDLL(str(path))
    lib.ssb_ray_cuda_abi.restype = ct.c_int
    if lib.ssb_ray_cuda_abi() != 2:
        raise RuntimeError('unsupported public ray CUDA ABI')
    lib.ssb_ray_cuda_error.restype = ct.c_char_p
    lib.ssb_ray_cuda_create.argtypes = [ct.c_int64, ct.c_int, ct.c_double, ct.c_double,
                                       *([ct.c_void_p]*6), ct.c_int, ct.c_void_p, ct.c_void_p,
                                       ct.c_void_p, ct.c_int, ct.c_double, ct.c_int, ct.c_double, ct.c_size_t]
    lib.ssb_ray_cuda_create.restype = ct.c_void_p
    lib.ssb_ray_cuda_destroy.argtypes = [ct.c_void_p]
    lib.ssb_ray_cuda_bytes.argtypes = [ct.c_void_p]
    lib.ssb_ray_cuda_bytes.restype = ct.c_size_t
    lib.ssb_ray_cuda_difference.argtypes = [ct.c_void_p, ct.c_void_p, ct.c_void_p]
    lib.ssb_ray_cuda_difference.restype = ct.c_int
    lib.ssb_ray_cuda_jacobian.argtypes = [*([ct.c_void_p]*5)]
    lib.ssb_ray_cuda_jacobian.restype = ct.c_int
    return lib, dict(name='cuda_fused_analytic_rays', abi=2, geometry_precision='float64; fmad disabled',
                     library=str(path), library_sha256=sha256_file(path), allocation_budget_bytes=4 << 30,
                     scope='CUDA training residual/Jacobian; CPU sparse normal, bounded solve and diagnostics',
                     source='public native rays and image-fitted coefficients only')


class CudaJacobian(CpuJacobian):
    """Fixed public inputs remain on device; no global reductions or float atomics."""
    def __init__(self, assembly):
        super().__init__(assembly)
        for block, (_, side, field, groups, _, _) in zip(self.blocks, assembly.blocks):
            basis = assembly.sides[side].bases[field].data.reshape(assembly.n, 4, 4)
            values = np.ascontiguousarray(basis[:, [g[0] for g in groups]], np.float64)
            self.keep.append(values)
            block.parameter_basis = values.ctypes.data
        self.lib, self.identity = cuda_backend()
        self.handle = self.lib.ssb_ray_cuda_create(assembly.n, len(assembly.model.fields),
            assembly.model.radius, assembly.model.height, *self.sources, pointer(assembly.starts),
            ct.cast(self.blocks, ct.c_void_p), len(self.blocks),
            pointer(assembly.scale_positions) if assembly.scale_positions is not None else None,
            pointer(assembly.scale_derivative), pointer(assembly.columns), assembly.model.size,
            assembly.model.scale, assembly.model.scale_index if assembly.model.scale_index is not None else -1,
            assembly.model.scale_reference_m, self.identity['allocation_budget_bytes'])
        if not self.handle:
            raise RuntimeError('CUDA public geometry: '+self.lib.ssb_ray_cuda_error().decode())
        self.peak_bytes = self.lib.ssb_ray_cuda_bytes(self.handle)
        self.uploaded_coefficients = None
        resources = _scope.get()
        if resources is not None:
            resources.callback(self.close)

    def close(self):
        if getattr(self, 'handle', None):
            self.lib.ssb_ray_cuda_destroy(self.handle)
            self.handle = None

    def __del__(self):
        self.close()

    def check(self, status):
        if status:
            self.uploaded_coefficients = None
            raise RuntimeError('CUDA public geometry: '+self.lib.ssb_ray_cuda_error().decode())

    def describe(self):
        return dict(self.identity, allocated_peak_bytes=self.peak_bytes,
                    spline_parameters='device resident fixed basis; upload coefficient vector only',
                    host_diagnostic_backend=backend()[1])

    def coefficients(self, coefficients):
        coefficients = np.ascontiguousarray(coefficients, np.float64)
        if (coefficients.shape != (self.assembly.model.size,) or not np.isfinite(coefficients).all()):
            raise ValueError('finite fitted CUDA coefficient vector required')
        if self.uploaded_coefficients is not None and np.array_equal(coefficients, self.uploaded_coefficients):
            return None
        self.uploaded_coefficients = coefficients.copy()
        return pointer(self.uploaded_coefficients)

    def difference(self, coefficients):
        if not self.handle:
            raise RuntimeError('CUDA public geometry context is closed')
        output = np.empty((self.assembly.n, 2), np.float64)
        self.check(self.lib.ssb_ray_cuda_difference(self.handle, self.coefficients(coefficients), pointer(output)))
        return output

    def fill(self, first, last, coefficients, current, pitch, output):
        a = self.assembly
        if (first != 0 or last != a.n or
                current.shape != (a.n, 2) or pitch.shape != (2,) or output.shape != (2*a.nnz,) or
                any(value.dtype != np.float64 or not value.flags.c_contiguous
                    for value in [current, pitch, output])):
            raise ValueError('CUDA Jacobian requires one complete contiguous public batch')
        if not self.handle:
            raise RuntimeError('CUDA public geometry context is closed')
        self.check(self.lib.ssb_ray_cuda_jacobian(self.handle, self.coefficients(coefficients),
                                                 pointer(current), pointer(pitch), pointer(output)))
