"""CUDA D3 inverse rays and native resampling; public fitted trajectory only."""
import ctypes as ct
import numpy as np
from .reconstruction_support import correction_reach_m
from .unroll_cuda import CudaRaster, pointer

RAY = np.dtype([(name, '<f8') for name in
                ('axis', 'phase', 'ox', 'oy', 'oz', 'tx', 'tz', 'rx', 'ry', 'rz')]+[('lattice', '<i8')])


def row_rays(model, coefficients):
    p = model.sampler.projection
    axis = p['x_axis_m']
    local = model.ray_parameters(np.arange(len(p)), coefficients)
    dx, dq, roll, pitch = local[:, :4].T
    ca, sa, cb, sb = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    rays = np.empty(len(p), RAY)
    rays['axis'] = axis
    rays['phase'] = p['theta_rad']-2*np.pi*p['segment']
    theta = rays['phase']+dq/model.radius
    ry = ca*np.sin(theta)-sa*np.cos(theta)
    rz = sa*np.sin(theta)+ca*np.cos(theta)
    rays['ox'] = axis+dx+model.height*sb*ca
    rays['oy'] = (local[:, 4] if local.shape[1] == 6 else 0.)-model.height*sa
    rays['oz'] = (local[:, -1] if local.shape[1] > 4 else 0.)+model.height*(cb*ca-1.)
    rays['tx'], rays['tz'] = cb, -sb
    rays['rx'], rays['ry'], rays['rz'] = sb*rz, ry, cb*rz
    rays['lattice'] = p['lattice_row']
    return rays


class GlobalCudaRaster(CudaRaster):
    def __init__(self, model, coefficients, fusion=None):
        self.model = model
        self.fusion = fusion
        self.gains = None
        coefficients = np.asarray(coefficients, float)
        if coefficients.shape != (model.size,) or not np.isfinite(coefficients).all():
            raise ValueError('finite fitted trajectory coefficients required')
        if np.any(abs(coefficients) > model.coefficient_bounds()+1e-7):
            raise ValueError('trajectory exceeds declared physical bounds')
        # Match the CPU's conservative band selection; this is not an image warp.
        self.margin = correction_reach_m(model.radius, model.height)
        self.band_x = model.band_extents(coefficients)
        self.rays = row_rays(model, coefficients)
        super().__init__(model.sampler)
        try:
            self.lib.ssb_unroll_global_abi.argtypes = []
            self.lib.ssb_unroll_global_abi.restype = ct.c_int
            if self.lib.ssb_unroll_global_abi() != 2:
                raise RuntimeError('unsupported CUDA global ABI')
            self.lib.ssb_unroll_global_depth.argtypes = [ct.c_void_p, ct.c_void_p]
            self.lib.ssb_unroll_global_depth.restype = ct.c_int
            self.lib.ssb_unroll_global_band.argtypes = [ct.c_void_p, ct.c_int, ct.c_void_p,
                ct.c_void_p, ct.c_void_p, ct.c_void_p, ct.c_int, ct.c_int, ct.c_int, ct.c_double, ct.c_double]
            self.lib.ssb_unroll_global_band.restype = ct.c_int
            if fusion is not None:
                from .seam_fusion import calibration_values
                self.gains, self.feather_columns = calibration_values(fusion, model)
                self.lib.ssb_unroll_fusion_abi.restype = ct.c_int
                if self.lib.ssb_unroll_fusion_abi() != 1:
                    raise RuntimeError('unsupported CUDA fusion ABI')
                for name in ('begin', 'gain'):
                    function = getattr(self.lib, 'ssb_unroll_fusion_'+name)
                    function.argtypes = [ct.c_void_p, ct.c_double]
                    function.restype = ct.c_int
        except BaseException:
            self.close()
            raise

    def candidate_bands(self, xs):
        return [i for i, (lo, hi) in enumerate(self.band_x)
                if hi+self.margin >= xs[0] and lo-self.margin <= xs[-1]]

    def describe(self):
        return dict(super().describe(), fusion_abi=1 if self.fusion is not None else None, global_abi=2, surface_relief=self.model.relief is not None,
                    geometry='fitted Ry(pitch) Rx(roll); double inverse, 10 iterations, 1e-8 m support threshold')

    def tile(self, angles, xs, bands=None):
        angles = np.asarray(angles, np.float64)
        xs = np.ascontiguousarray(xs, np.float64)
        if (angles.ndim != 1 or xs.ndim != 1 or not len(angles) or not len(xs) or
                not np.isfinite(angles).all() or not np.isfinite(xs).all() or np.any(np.diff(xs) < 0)):
            raise ValueError('finite ordered one-dimensional CUDA grid required')
        self.check(self.lib.ssb_unroll_begin(self.handle, len(angles), len(xs), pointer(xs)))
        if self.fusion is not None:
            self.check(self.lib.ssb_unroll_fusion_begin(self.handle, self.feather_columns))
        qs = np.ascontiguousarray(angles*self.model.radius)
        if self.model.relief is not None:
            depth = np.ascontiguousarray(self.model.relief.depth(xs[None, :], qs[:, None]), np.float64)
            if depth.shape != (len(qs), len(xs)):
                raise ValueError('shared radial depth must match the CUDA output tile')
            self.check(self.lib.ssb_unroll_global_depth(self.handle, pointer(depth)))
        for band in (self.candidate_bands(xs) if bands is None else bands):
            lo, hi = self.band_x[band]
            left = int(np.searchsorted(xs, lo-self.margin))
            right = int(np.searchsorted(xs, hi+self.margin, side='right'))
            if right <= left:
                continue
            a, b = self.sampler.bounds[band]
            phase = self.sampler.phases[band]
            angular_margin = self.margin/self.model.radius
            if angles.max() < phase[0]-angular_margin or angles.min() > phase[-1]+angular_margin:
                continue
            # Extra rows retain both bracketing samples at the loading boundary.
            first = max(0, int(np.searchsorted(phase, angles.min()-angular_margin))-2)+a
            last = min(b-a, int(np.searchsorted(phase, angles.max()+angular_margin, side='right'))+2)+a
            if (last-first)*self.sampler.native.shape[1] > 128 << 20:
                raise ValueError('CUDA native input exceeds 128 MiB')
            samples = np.ascontiguousarray(self.sampler.native.raw(np.arange(first, last)))
            axes = np.ascontiguousarray(self.sampler.projection['x_axis_m'][first:last])
            rays = np.ascontiguousarray(self.rays[first:last])
            if self.fusion is not None:
                self.check(self.lib.ssb_unroll_fusion_gain(self.handle, float(self.gains[band])))
            self.check(self.lib.ssb_unroll_global_band(self.handle, last-first, pointer(samples), pointer(axes),
                pointer(rays), pointer(qs), left, right, band, self.model.radius, self.sampler.footprint))
        shape = (len(angles), len(xs))
        image = np.empty(shape, np.float32)
        count = np.empty(shape, np.uint16)
        source = np.empty(shape, np.int16)
        self.check(self.lib.ssb_unroll_finish(self.handle, pointer(image), pointer(count), pointer(source)))
        return image, count, source
