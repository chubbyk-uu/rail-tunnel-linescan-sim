"""Image-derived radial relief after pose optimization; no scene or truth inputs.

Local stereo disparity is converted to radial depth using the estimated camera
baseline. A shared surface is sampled by every band; no per-band free image warp,
brightness modification, blending, or discarded seam evaluation point is used.
"""
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import shutil
import time

import cv2
import numpy as np
from scipy.ndimage import map_coordinates

from .global_resample import load_global, sample_corrected
from .match_bands import verified_bands
from .matching_structures import long_dark_mask
from .provenance import stage_record
from .stage_b_scene import peak_rss_bytes


@dataclass(frozen=True)
class ReliefSettings:
    angular_stride: int = 8
    tile_rows: int = 128
    max_depth_m: float = .03
    structure_padding_mm: float = 3.
    max_cycle_px: float = 1.
    archive_budget_bytes: int = 64 << 20

    def validate(self):
        if (type(self.angular_stride) is not int or not 1 <= self.angular_stride <= 16 or
                type(self.tile_rows) is not int or not 64 <= self.tile_rows <= 256 or
                not np.isfinite([self.max_depth_m, self.structure_padding_mm, self.max_cycle_px]).all() or
                not 0 < self.max_depth_m <= .03 or not 0 < self.structure_padding_mm <= 10 or
                not 0 < self.max_cycle_px <= 2 or
                type(self.archive_budget_bytes) is not int or not 0 < self.archive_budget_bytes <= 64 << 20):
            raise ValueError('invalid bounded radial-relief settings')


class SurfaceRelief:
    def __init__(self, patches, maximum=.03):
        self.patches = patches
        self.maximum = maximum

    def depth(self, x, q):
        x, q = np.broadcast_arrays(np.asarray(x), np.asarray(q))
        total = np.zeros(x.shape); weight = np.zeros(x.shape)
        if not x.size:
            return total
        bounds = (x.min(), x.max(), q.min(), q.max())
        for patch in self.patches:
            x0, q0, dx, dq = patch['grid']
            data = patch['depth']
            if (x0 > bounds[1]+1e-12 or x0+(data.shape[1]-1)*dx < bounds[0]-1e-12 or
                    q0 > bounds[3]+1e-12 or q0+(data.shape[0]-1)*dq < bounds[2]-1e-12):
                continue
            u, v = (x-x0)/dx, (q-q0)/dq
            inside = (u >= -1e-8) & (u <= data.shape[1]-1+1e-8) & (v >= -1e-8) & (v <= data.shape[0]-1+1e-8)
            if inside.any():
                values = map_coordinates(data, [np.clip(v[inside], 0, data.shape[0]-1),
                    np.clip(u[inside], 0, data.shape[1]-1)], order=1, prefilter=False)
                total[inside] += values; weight[inside] += 1
        return np.divide(total, weight, out=total, where=weight > 0)

    def save(self, path):
        arrays = {'grids': np.asarray([p['grid'] for p in self.patches], float).reshape(-1, 4)}
        arrays.update({f'depth_{i}': p['depth'] for i, p in enumerate(self.patches)})
        np.savez_compressed(path, **arrays)

    @classmethod
    def load(cls, path, maximum=.03, budget=64 << 20):
        with np.load(path, allow_pickle=False) as archive:
            grids = archive['grids']
            if grids.ndim != 2 or grids.shape[1] != 4 or not np.isfinite(grids).all() or np.any(grids[:, 2:] <= 0):
                raise ValueError('invalid relief patch coordinates')
            patches = []; used = grids.nbytes
            for i, grid in enumerate(grids):
                data = archive[f'depth_{i}']
                used += data.nbytes
                if (data.ndim != 2 or min(data.shape) < 2 or data.dtype != np.float32 or
                        used > budget or not np.isfinite(data).all() or np.any(data < 0) or np.any(data > maximum)):
                    raise ValueError('invalid or excessive relief patch')
                patches.append(dict(grid=grid, depth=data))
            if set(archive.files) != {'grids', *[f'depth_{i}' for i in range(len(patches))]}:
                raise ValueError('unexpected relief product fields')
        return cls(patches, maximum)


def stereo_flow(a, b):
    engine = cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM)
    engine.setFinestScale(0)
    engine.setPatchSize(8)
    engine.setPatchStride(2)
    return engine.calc(np.clip(a, 0, 255).astype(np.uint8), np.clip(b, 0, 255).astype(np.uint8), None)


def camera_axis(model, coefficients, band, qs):
    ids = model.sampler.row_sources(band, qs/model.radius)[0]
    axis = model.sampler.projection['x_axis_m'][ids]
    c = model.ray_parameters(ids, coefficients)
    return axis+c[:, 0]+model.height*np.sin(c[:, 3])*np.cos(c[:, 2])


def run(unroll, trajectory, output, raw_root=None, settings=ReliefSettings()):
    settings.validate(); started = time.monotonic()
    output = Path(output).resolve()
    if output.exists() or any(output.is_relative_to(Path(p).resolve()) for p in (unroll, trajectory)):
        raise ValueError('fresh relief output separate from its inputs required')
    sampler, upstream, inputs = verified_bands(unroll, raw_root)
    try:
        model, coefficients, original, trajectory_inputs = load_global(trajectory, sampler, upstream, unroll)
        if model.relief is not None:
            raise ValueError('relief must be estimated once from the original optimized trajectory')
        grid = upstream['grid']; dx = grid['dx_m']; dq = grid['dq_m']*settings.angular_stride
        qaxis = np.arange(grid['theta_rad'][0]*model.radius, grid['theta_rad'][1]*model.radius, dq)
        patches, diagnostic = [], []; used = 0
        cv2.setNumThreads(1)
        for band in range(len(sampler.segments)-1):
            if sampler.segments[band+1] != sampler.segments[band]+1:
                continue
            for first in range(0, len(qaxis)-1, settings.tile_rows-2):
                qs = qaxis[first:first+settings.tile_rows]
                if len(qs) < 8:
                    continue
                ranges = []
                for k in (band, band+1):
                    lo, hi, _, supported = sampler.row_sources(k, qs/model.radius)
                    axes = sampler.projection['x_axis_m'][np.r_[lo, hi]]
                    ranges.append((axes.max()+sampler.offsets[0]+.02, axes.min()+sampler.offsets[-1]-.02))
                left, right = max(r[0] for r in ranges), min(r[1] for r in ranges)
                if right-left < .05:
                    continue
                xs = np.arange(left, right, dx)
                if len(xs)*len(qs) > 1 << 20:
                    raise ValueError('relief stereo tile exceeds one megapixel')
                # Cheap public-image screening; only selected grooves need the
                # expensive rigid-body inverse and dense stereo calculations.
                preview, preview_valid, _ = sampler.sample(band, qs[::4]/model.radius, xs[::4])
                _, preview_detection = long_dark_mask(preview, preview_valid, 4*dx)
                if not preview_detection['bands_px']:
                    sampler.native.release(); continue
                # Stereo needs the structure and its local background, not the
                # full sensor overlap. Include the entire 30 mm depth search.
                stereo_margin = int(np.ceil(.03/dx))
                begin = max(0, min(a for a, b in preview_detection['bands_px'])*4-stereo_margin)
                end = min(len(xs), max(b for a, b in preview_detection['bands_px'])*4+stereo_margin)
                xs = xs[begin:end]
                a, ma, _ = sample_corrected(model, coefficients, band, qs, xs)
                b, mb, _ = sample_corrected(model, coefficients, band+1, qs, xs)
                mask, detection = long_dark_mask(a, ma, dx)
                padding = int(np.ceil(settings.structure_padding_mm/(1000*dx)))
                mask = cv2.dilate(mask.astype(np.uint8), np.ones((1, 2*padding+1), np.uint8)).astype(bool)
                if not mask.any():
                    sampler.native.release(); continue
                a = np.where(ma, a, 128); b = np.where(mb, b, 128)
                flow = stereo_flow(a, b)
                reverse = stereo_flow(b, a)
                yy, xx = np.indices(a.shape, dtype=np.float32)
                reverse_at_b = cv2.remap(reverse, xx+flow[:, :, 0], yy+flow[:, :, 1], cv2.INTER_LINEAR,
                                        borderMode=cv2.BORDER_CONSTANT, borderValue=1000)
                cycle = flow+reverse_at_b
                cycle_px = np.hypot(cycle[:, :, 0], settings.angular_stride*cycle[:, :, 1])
                observed_b = cv2.remap(mb.astype(np.float32), xx+flow[:, :, 0], yy+flow[:, :, 1],
                                      cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                supported = ma & (observed_b >= 1-1e-6) & (cycle_px <= settings.max_cycle_px)
                background = ma & mb & ~cv2.dilate(mask.astype(np.uint8), np.ones((1, 41), np.uint8)).astype(bool)
                if background.sum() < 128:
                    sampler.native.release(); continue
                bias = float(np.median(flow[:, :, 0][background]))
                shift = flow[:, :, 0]-bias
                axis_a = camera_axis(model, coefficients, band, qs)
                axis_b = camera_axis(model, coefficients, band+1, qs)
                baseline = axis_b-axis_a
                if np.any(baseline <= .1):
                    raise ValueError('radial stereo needs an observable positive estimated camera baseline')
                depth = np.clip(shift*dx*model.radius/baseline[:, None], 0, settings.max_depth_m)
                depth[~(mask & supported)] = 0
                # A's cylindrical coordinates become shared surface coordinates.
                xa = xs[None, :]+depth*(xs[None, :]-axis_a[:, None])/model.radius
                monotone = (np.diff(xa, axis=1) > 0).all(axis=1)
                depth[~monotone] = 0
                xa[~monotone] = xs
                data = np.array([np.interp(xs, row_x, row_d, left=0, right=0)
                                 for row_x, row_d in zip(xa, depth)], np.float32)
                active = np.flatnonzero(np.any(data > 0, axis=0))
                if not len(active):
                    sampler.native.release(); continue
                begin, end = max(0, int(active[0])-2), min(len(xs), int(active[-1])+3)
                data = data[:, begin:end].copy()
                used += data.nbytes+32
                if used > settings.archive_budget_bytes:
                    raise ValueError('radial relief exceeds its declared memory budget')
                patches.append(dict(grid=[float(xs[begin]), float(qs[0]), dx, dq], depth=data))
                diagnostic.append(dict(bands=[band, band+1], q_m=[float(qs[0]), float(qs[-1])],
                    detected_bands_px=detection['bands_px'], background_shift_px=bias,
                    depth_p95_mm=float(np.percentile(depth[mask], 95))*1000,
                    stereo_supported_fraction=float(supported[mask].mean()),
                    cycle_p95_px=float(np.percentile(cycle_px[mask], 95)),
                    nonmonotone_rows=int((~monotone).sum())))
                sampler.native.release()
        output.mkdir(parents=True)
        for name in ('trajectory.json', 'windows.json', 'match_residuals.npz'):
            shutil.copyfile(Path(trajectory)/name, output/name)
        relief = SurfaceRelief(patches, settings.max_depth_m)
        relief.save(output/'surface_relief.npz')
        report = dict(original)
        report['surface_relief'] = dict(schema='ssb.stereo_radial_relief.v1', settings=asdict(settings),
            patches=len(patches), uncompressed_bytes=used, diagnostics=diagnostic,
            pose_unchanged=True, input_scope='public images, estimated trajectory and nominal geometry only',
            limitations=['unobserved or unsupported relief remains nominal; no missing original pixel is filled',
                         'depth boundaries and weak texture require independent optical-mesh acceptance'])
        report['performance'] = dict(original['performance'], relief_wall_s=time.monotonic()-started,
                                    relief_peak_rss_bytes=peak_rss_bytes())
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        (output/'provenance.json').write_text(json.dumps(stage_record('global_optimization',
            inputs+trajectory_inputs, sorted(output.iterdir()), dict(relief=asdict(settings))), indent=2)+'\n')
        return report
    finally:
        if hasattr(sampler.native, 'close'):
            sampler.native.close()
