"""Stage D1: flat-correct native pixels and project measured rays onto a nominal cylinder.

Only PublicCapture inputs are used. Individual helical bands remain independently
sampleable; the first mosaic selects one band without blending or fitting motion.
Native rows are read from the capture's uint8 raw blocks and flat-corrected on demand
(no float cache). Coverage is stored exactly as run lengths; a full-resolution mosaic
is written only on request (`--mosaic roi|full`), as uint16 DN/64.
"""
import argparse
import json
import math
import mmap
from pathlib import Path
import time
import numpy as np
from PIL import Image
from .provenance import stage_record
from .public_capture import PublicCapture, confined_file
from .session import sha256_file
from .wall_coverage import target_grid, RUN_DTYPE
from .native_rows import NativeRows, FloatRows
from .stage_b_scene import peak_rss_bytes
from .reconstruction_support import correction_reach_m, relative_scale_reach_m

PROJECTION = np.dtype([('sequence', '<i8'), ('lattice_row', '<i8'), ('segment', '<i8'),
                       ('x_axis_m', '<f8'), ('theta_rad', '<f8')])
MOSAIC_SCALE = 64          # stored mosaic code = round(DN*64); 1/64 DN quantization
MOSAIC_INVALID = 65535     # no valid observation
TILE_PIXELS = 1 << 20      # bounded per-tile temporaries (CPU arrays / CUDA buffers)


def release_pages(array):
    # Drop this process's mapped pages; leave the shared page cache and batching
    # policy to the kernel. No per-row flush/fsync or persistent status files.
    mapping = getattr(array, '_mmap', None)
    if mapping is not None and hasattr(mapping, 'madvise'):
        mapping.madvise(mmap.MADV_DONTNEED)


def sensor_geometry(config, calibration):
    width = config['camera']['width']
    coefficients = np.asarray(calibration['geometry']['coefficients'], float)
    q = (np.arange(width)-(width-1)/2)/(width/2)
    scale = config['calibration']['radius_m']/config['camera']['nominal_distance_m']
    offsets = np.polynomial.polynomial.polyval(q, coefficients)*scale
    if coefficients.shape != (4,) or not np.isfinite(offsets).all() or not np.all(np.diff(offsets) > 0):
        raise ValueError('measured sensor mapping must be finite and strictly increasing')
    output_offsets = q*calibration['geometry']['output_fov_m']/2*scale
    return offsets, output_offsets, np.asarray(calibration['geometry']['valid'], bool)


class BandSampler:
    """Invert band geometry into native sensor pixels, then interpolate once.

    Source provenance is factorized: band + output grid + projection table +
    measured mapping uniquely recover the exposure rows, columns and weights.
    """
    def __init__(self, projection, image, offsets, output_offsets, geometry_valid, footprint):
        self.projection, self.image = projection, image
        self.offsets, self.output_offsets, self.geometry_valid = offsets, output_offsets, geometry_valid
        self.footprint = footprint
        self.segments = np.unique(projection['segment'])
        self.bounds = []
        self.phases = []
        self.columns = np.arange(len(offsets))
        # `image` is a float32 array (legacy v1 cache, analytic tests) or a native row
        # source that flat-corrects uint8 raw rows on demand; both expose gather().
        self.native = image if hasattr(image, 'gather') else FloatRows(np.asarray(image))
        if not np.any(geometry_valid): raise ValueError('no calibrated usable columns')
        usable = np.asarray(output_offsets)[np.asarray(geometry_valid, bool)]
        self.band_x = []
        for segment in self.segments:
            ids = np.flatnonzero(projection['segment'] == segment)
            if not np.array_equal(ids, np.arange(ids[0], ids[-1]+1)):
                raise ValueError('interleaved acquisition segments')
            if len(ids) < 2 or not np.all(np.diff(projection['theta_rad'][ids]) > 0):
                raise ValueError('band angles must increase and contain at least two rows')
            self.bounds.append((int(ids[0]), int(ids[-1]+1)))
            self.phases.append(projection['theta_rad'][ids]-2*math.pi*int(segment))
            # Superset of every per-tile x support of this band; used only to skip bands.
            axes = projection['x_axis_m'][ids]
            self.band_x.append((float(axes.min()+usable[0]), float(axes.max()+usable[-1])))

    def row_sources(self, band, angles):
        a, b = self.bounds[band]
        rows = self.projection[a:b]
        phase = self.phases[band]
        right = np.searchsorted(phase, angles)
        left = np.clip(right-1, 0, len(rows)-1); right = np.clip(right, 0, len(rows)-1)
        gap = phase[right]-phase[left]
        contiguous = ((rows['lattice_row'][right]-rows['lattice_row'][left]) == 1) & (gap <= self.footprint*(1+1e-8))
        between = (angles >= phase[left]) & (angles <= phase[right]) & (gap > 0)
        interpolate = contiguous & between
        weight = np.divide(angles-phase[left], gap, out=np.zeros_like(angles), where=gap > 0)
        nearest = np.where(abs(angles-phase[left]) <= abs(angles-phase[right]), left, right)
        # A centre ray has one source, not a zero-weight neighbour. Roundoff
        # from inverse atan2 must not switch its column bounds to another row.
        centre_tolerance = 8*np.finfo(float).eps*np.maximum(1., abs(angles))
        at_centre = abs(angles-phase[nearest]) <= centre_tolerance
        interpolate &= ~at_centre
        supported = np.minimum(abs(angles-phase[left]), abs(angles-phase[right])) <= self.footprint/2+1e-12
        left = np.where(interpolate, left, nearest)
        right = np.where(interpolate, right, nearest)
        weight = np.where(interpolate, np.clip(weight, 0, 1), 0.)
        return left+a, right+a, weight, supported

    def sample(self, band, angles, xs, sources=False):
        angles = np.asarray(angles, float); xs = np.asarray(xs, float)
        lo, hi, weight, supported = self.row_sources(band, angles)
        def along(ids):
            delta = xs[None, :]-self.projection['x_axis_m'][ids, None]
            u = np.interp(delta, self.offsets, self.columns)
            left = np.floor(u).astype(np.int32); right = np.minimum(left+1, len(self.offsets)-1)
            w = (u-left).astype(np.float32)
            v0, v1 = self.native.gather(ids, left, right)
            # Measured target domain: no extrapolation into uncalibrated edges.
            cu = np.interp(delta, self.output_offsets, self.columns)
            c0 = np.floor(cu).astype(np.int32); c1 = np.minimum(c0+1, len(self.offsets)-1)
            valid = ((delta >= self.offsets[0]) & (delta <= self.offsets[-1]) &
                     (delta >= self.output_offsets[0]) & (delta <= self.output_offsets[-1]) &
                     self.geometry_valid[c0] & self.geometry_valid[c1] &
                     np.isfinite(v0) & np.isfinite(v1))
            value = v0*(1-w)+v1*w
            return value, valid, u
        v0, valid0, u0 = along(lo); v1, valid1, u1 = along(hi)
        valid = supported[:, None] & valid0 & valid1
        value = v0*(1-weight[:, None])+v1*weight[:, None]
        value[~valid] = np.nan
        score = np.minimum(u0, len(self.offsets)-1-u0)  # most central native observation
        if sources:
            return value.astype(np.float32), valid, score, dict(lower=lo, upper=hi, angular_weight=weight,
                                                               lower_column=u0, upper_column=u1)
        return value.astype(np.float32), valid, score


def retained_band_rows(rows, x_axis, usable, target, reach):
    """Keep complete recorded scans that can reach the ROI within public bounds.

    Cropping individual rows creates artificial angular holes at ROI edges.
    This selects existing row identities only; actual gaps remain gaps.
    """
    eligible = ((x_axis+usable[-1]+reach >= target[0]) &
                (x_axis+usable[0]-reach <= target[1]))
    if not np.any(eligible):
        raise ValueError('no exposure rows intersect the requested ROI support envelope')
    segments = np.unique(rows['segment'][eligible])
    return np.flatnonzero(np.isin(rows['segment'], segments))


def prepare_sensor(capture, output, target, chunk_rows=256):
    """Retain complete ROI-supporting scans, verify raw blocks and record native sources.

    Nothing pixel-sized is written: the raw blocks stay the only native image store.
    Saturation/invalid counts are taken in one sequential pass that also hashes blocks.
    """
    offsets, corrected_offsets, geometry_valid = sensor_geometry(capture.config, capture.calibration)
    usable = corrected_offsets[geometry_valid]
    reach = correction_reach_m(capture.config['calibration']['radius_m'],
                               capture.config['robot']['scan_axis_height_m'])
    bound = capture.config.get('inspection', {}).get('relative_encoder_scale_bound_fraction', 0.)
    reach += relative_scale_reach_m([float(capture.x_axis.min()), float(capture.x_axis.max())], bound)
    selected = retained_band_rows(capture.rows, capture.x_axis, usable, target, reach)
    projection = np.zeros(len(selected), PROJECTION)
    for name, field in [('sequence', 'sequence'), ('lattice_row', 'row'), ('segment', 'segment')]:
        projection[name] = capture.rows[field][selected]
    projection['x_axis_m'] = capture.x_axis[selected]; projection['theta_rad'] = capture.theta[selected]
    flat = capture.calibration['flat']
    np.save(output/'projection.npy', projection)
    np.savez(output/'mapping.npz', native_x_offset_m=offsets, corrected_x_offset_m=corrected_offsets,
             geometry_valid=geometry_valid, angular_footprint_rad=capture.footprint,
             flat_offset=np.asarray(flat['offset'], np.float32), flat_gain=np.asarray(flat['gain'], np.float32),
             flat_valid=np.asarray(flat['valid'], bool))
    inputs = list(capture.inputs); raw_blocks = []; saturated = 0; invalid = 0
    flat_valid = np.asarray(flat['valid'], bool)
    for block in capture.raw_index['blocks']:
        first, count = block['first_sequence'], block['rows']
        a, b = np.searchsorted(selected, [first, first+count])
        if a == b: continue
        path = confined_file(capture.root/'raw', block['file'])
        if path.stat().st_size != count*capture.width: raise ValueError('raw block size mismatch')
        if sha256_file(path) != block['sha256']: raise ValueError('raw block hash mismatch: '+str(path))
        inputs.append(path); raw_blocks.append(block)
        raw = np.memmap(path, np.uint8, mode='r', shape=(count, capture.width))
        for lo in range(a, b, chunk_rows):
            samples = raw[selected[lo:min(b, lo+chunk_rows)]-first]
            saturated += int(np.count_nonzero(samples == 255))
            invalid += int(np.count_nonzero(~(flat_valid[None, :] & (samples != 255))))
        release_pages(raw)
    source = dict(schema='ssb.native_source.v1', session_root=str(capture.root),
                  raw_index_sha256=sha256_file(capture.root/'raw/index.json'), width=capture.width,
                  blocks=raw_blocks, flat='mapping.npz flat_offset/flat_gain/flat_valid',
                  correction='float32 (raw - offset) * gain; invalid where flat_valid is false or raw == 255')
    (output/'native_source.json').write_text(json.dumps(source, indent=2)+'\n')
    native = NativeRows(capture.root/'raw', raw_blocks, projection['sequence'], capture.width, flat,
                        verified={b['file'] for b in raw_blocks})
    return BandSampler(projection, native, offsets, corrected_offsets, geometry_valid, capture.footprint), inputs, dict(
        selected_rows=len(selected), source_retention='complete recorded bands intersecting bounded ROI',
        correction_reach_m=reach, raw_blocks=raw_blocks, saturated_samples=saturated, invalid_native_samples=invalid)


def cpu_tile(sampler, angles, xs, bands):
    """Reference tile: bands in index order; the most central valid native column wins."""
    shape = (len(angles), len(xs))
    values = np.full(shape, np.nan, np.float32); number = np.zeros(shape, np.uint16)
    chosen = np.full(shape, -1, np.int16); best = np.full(shape, -np.inf, np.float32)
    usable = sampler.output_offsets[sampler.geometry_valid]
    for band in bands:
        lo, hi, _, supported = sampler.row_sources(band, angles)
        if not np.any(supported): continue
        axes = sampler.projection['x_axis_m'][np.r_[lo[supported], hi[supported]]]
        left = np.searchsorted(xs, axes.min()+usable[0], side='left')
        right = np.searchsorted(xs, axes.max()+usable[-1], side='right')
        if right <= left: continue
        columns = slice(left, right)
        result, valid, score = sampler.sample(band, angles, xs[columns])
        number[:, columns] += valid
        better = valid & (score > best[:, columns])
        values[:, columns][better] = result[better]
        best[:, columns][better] = score[better]
        chosen[:, columns][better] = band
    return values, number, chosen


def candidate_bands(sampler, xs):
    """Bands whose conservative x support meets [xs[0], xs[-1]], in index order."""
    return [k for k, (lo, hi) in enumerate(sampler.band_x) if hi >= xs[0] and lo <= xs[-1]]


def render_tile(sampler, angles, xs, accelerator=None):
    bands = (accelerator.candidate_bands(xs) if accelerator is not None and
             hasattr(accelerator, 'candidate_bands') else candidate_bands(sampler, xs))
    if accelerator is not None: return accelerator.tile(angles, xs, bands)
    return cpu_tile(sampler, angles, xs, bands)


def grid_axes(grid):
    nq, nx = grid['shape']
    angles = grid['theta_rad'][0]+(np.arange(nq)+.5)*grid['dq_m']/grid['radius_m']
    xs = grid['target_x_m'][0]+(np.arange(nx)+.5)*grid['dx_m']
    return angles, xs


def coverage_runs(counts, q0):
    """Exact run-length encoding of a block of coverage rows (RUN_DTYPE)."""
    rows, nx = counts.shape
    change = np.ones(counts.shape, bool); change[:, 1:] = counts[:, 1:] != counts[:, :-1]
    starts = np.flatnonzero(change.ravel())
    ends = np.r_[starts[1:], rows*nx]
    q, x = np.divmod(starts, nx)
    runs = np.empty(len(starts), RUN_DTYPE)
    runs['q_bin'] = q+q0; runs['x_begin'] = x; runs['x_end'] = ends-q*nx; runs['count'] = counts.ravel()[starts]
    return runs


def rasterize(sampler, grid, output, angular_tile_rows=32, accelerator=None, mosaic=None,
              tile_columns=8192, band_pixels=1 << 24, preview_stride=None):
    """Two-dimensional tiles; only overlapping bands are visited, so work is linear in length.

    Writes exact coverage runs (coverage_runs.bin) and, when `mosaic` = (c0, c1) column
    range, a uint16 mosaic of those columns. Returns statistics and the min-pooled
    coverage preview (any uncovered pixel in a preview cell stays visible).
    """
    nq, nx = grid['shape']
    if nx > TILE_PIXELS: raise ValueError('grid width exceeds one million pixels')
    angles_all, xs_all = grid_axes(grid)
    row_band = max(1, min(nq, band_pixels//nx))
    tile_cols = max(1, min(nx, tile_columns))
    tile_rows = max(1, min(angular_tile_rows, row_band, TILE_PIXELS//tile_cols))
    stride = preview_stride or max(1, math.ceil(max(nx, nq)/1800))
    cover_min = np.full((math.ceil(nq/stride), math.ceil(nx/stride)), np.iinfo(np.uint16).max, np.uint16)
    stored = None
    if mosaic is not None:
        c0, c1 = mosaic
        stored = (np.lib.format.open_memmap(output/'mosaic_u16.npy', 'w+', np.uint16, shape=(nq, c1-c0)),
                  np.lib.format.open_memmap(output/'mosaic_count.npy', 'w+', np.uint8, shape=(nq, c1-c0)))
    histogram = {}; runs_written = 0; clipped = dict(below_zero=0, above_range=0)
    with (output/'coverage_runs.bin').open('wb') as runs_file:
        for q0 in range(0, nq, row_band):
            q1 = min(nq, q0+row_band)
            counts = np.zeros((q1-q0, nx), np.uint16)
            for a in range(q0, q1, tile_rows):
                b = min(q1, a+tile_rows)
                for c in range(0, nx, tile_cols):
                    d = min(nx, c+tile_cols)
                    values, number, _ = render_tile(sampler, angles_all[a:b], xs_all[c:d], accelerator)
                    counts[a-q0:b-q0, c:d] = number
                    if stored is not None:
                        lo, hi = max(c, mosaic[0]), min(d, mosaic[1])
                        if lo < hi:
                            part = values[:, lo-c:hi-c].astype(np.float64)*MOSAIC_SCALE
                            ok = number[:, lo-c:hi-c] > 0
                            clipped['below_zero'] += int(np.count_nonzero(ok & (part < 0)))
                            clipped['above_range'] += int(np.count_nonzero(ok & (part > MOSAIC_INVALID-1)))
                            code = np.where(ok, np.clip(np.rint(part), 0, MOSAIC_INVALID-1), MOSAIC_INVALID)
                            stored[0][a:b, lo-mosaic[0]:hi-mosaic[0]] = code.astype(np.uint16)
                            stored[1][a:b, lo-mosaic[0]:hi-mosaic[0]] = np.minimum(number[:, lo-c:hi-c], 255)
            runs = coverage_runs(counts, q0); runs.tofile(runs_file); runs_written += len(runs)
            for value, n in zip(*np.unique(counts, return_counts=True)):
                histogram[int(value)] = histogram.get(int(value), 0)+int(n)
            column_min = np.minimum.reduceat(counts, np.arange(0, nx, stride), axis=1)
            np.minimum.at(cover_min, np.arange(q0, q1)//stride, column_min)
            sampler.native.release()
            if stored is not None:
                for array in stored: release_pages(array)
    if stored is not None:
        for array in stored: array.flush(); release_pages(array)
    total = nq*nx
    if sum(histogram.values()) != total: raise ValueError('coverage accounting does not span the grid')
    statistics = dict(missing_pixels=histogram.get(0, 0),
                      overlapping_pixels=sum(n for k, n in histogram.items() if k > 1), total_pixels=total,
                      count_histogram={str(k): v for k, v in sorted(histogram.items())},
                      runs=dict(file='coverage_runs.bin', records=runs_written, dtype=RUN_DTYPE.descr,
                                record_bytes=RUN_DTYPE.itemsize),
                      tiles=dict(angular_rows=tile_rows, columns=tile_cols, row_band=row_band))
    if stored is not None:
        statistics['mosaic_clipped_samples'] = clipped
    return statistics, cover_min


def preview_values(sampler, grid, stride, accelerator=None):
    """Exact mosaic values at every stride-th grid cell centre, without a full mosaic."""
    angles, xs = grid_axes(grid)
    angles, xs = angles[::stride], xs[::stride]
    out = np.full((len(angles), len(xs)), np.nan, np.float32)
    # Few sampled rows per tile: each spans stride grid rows, and the native rows a band
    # uploads for a tile grow with the arc it covers (bounded at 128 MiB on the GPU).
    rows = max(1, min(len(angles), 32))
    for a in range(0, len(angles), rows):
        for c in range(0, len(xs), 8192):
            values, _, _ = render_tile(sampler, angles[a:a+rows], xs[c:c+8192], accelerator)
            out[a:a+rows, c:c+8192] = values
        sampler.native.release()  # sparse rows span the whole ROI; keep mapped pages bounded
    return out


def display(values):
    return np.clip(np.nan_to_num(values, nan=0.), 0, 255).astype(np.uint8)


def preview_sample(array, stride):
    """Copy a sparse preview in bounded row groups, dropping mapped pages.

    A single strided operation can fault most of a large file into this process
    (especially with huge pages), despite the small resulting image.
    """
    shape = tuple(math.ceil(n/stride) for n in array.shape)
    result = np.empty(shape, array.dtype)
    for first in range(0, shape[0], 16):
        last = min(shape[0], first+16)
        result[first:last] = array[first*stride:last*stride:stride, ::stride]
        release_pages(array)
    return result


def load_bands(output, raw_root=None):
    """Load self-contained D1 products; v2 reads native rows from the hash-recorded raw blocks.

    `raw_root` relocates the capture's raw directory (each block is still hash-checked).
    Legacy v1 products use their float32 cache. Never reads capture metadata or evaluation.
    """
    output = Path(output)
    projection = np.load(output/'projection.npy', mmap_mode='r')
    with np.load(output/'mapping.npz') as mapping:
        arrays = {k: mapping[k] for k in mapping.files}
    source_path = output/'native_source.json'
    if source_path.is_file():
        source = json.loads(source_path.read_text())
        if source.get('schema') != 'ssb.native_source.v1': raise ValueError('unsupported native source')
        raw_dir = Path(raw_root) if raw_root is not None else Path(source['session_root'])/'raw'
        native = NativeRows(raw_dir, source['blocks'], projection['sequence'], source['width'],
                            dict(offset=arrays['flat_offset'], gain=arrays['flat_gain'], valid=arrays['flat_valid']))
    else:
        native = FloatRows(np.load(output/'sensor_flat.npy', mmap_mode='r'))
    return BandSampler(projection, native, arrays['native_x_offset_m'], arrays['corrected_x_offset_m'],
                       arrays['geometry_valid'], float(arrays['angular_footprint_rad']))


def select_band(sampler, theta, x):
    """The mosaic's source band at one point: same order, score and float32 best as rasterize."""
    best = np.float32(-np.inf); chosen = -1; count = 0
    for band in candidate_bands(sampler, np.array([x])):
        _, valid, score = sampler.sample(band, [theta], [x])
        if not valid[0, 0]: continue
        count += 1
        if score[0, 0] > best: best = np.float32(score[0, 0]); chosen = band
    return chosen, count


def trace_pixel(output, q_index, x_index, raw_root=None):
    """Recover native exposure/column contributions to one unblended mosaic pixel."""
    output = Path(output); report = json.loads((output/'report.json').read_text()); grid = report['grid']
    nq, nx = grid['shape']
    if not (0 <= q_index < nq and 0 <= x_index < nx): raise ValueError('pixel outside output grid')
    sampler = load_bands(output, raw_root)
    theta = grid['theta_rad'][0]+(q_index+.5)*grid['dq_m']/grid['radius_m']
    x = grid['target_x_m'][0]+(x_index+.5)*grid['dx_m']
    band, observations = select_band(sampler, theta, x)
    if band < 0: return dict(valid=False, contributions=[], observations=0)
    values, valid, _, sources = sampler.sample(band, [theta], [x], sources=True)
    combined = {}
    angular = sources['angular_weight'][0]
    for name, row, wq in [('lower_column', sources['lower'][0], 1-angular),
                          ('upper_column', sources['upper'][0], angular)]:
        column = sources[name][0, 0]; left = int(math.floor(column)); alpha = column-left
        for u, wu in [(left, 1-alpha), (min(left+1, sampler.native.shape[1]-1), alpha)]:
            if wu*wq <= 0: continue
            key = int(row), u
            combined[key] = combined.get(key, 0.)+float(wu*wq)
    contributions = []
    rows = {row: sampler.native.rows([row])[0] for row, _ in combined}
    for (row, column), weight in combined.items():
        sequence = int(sampler.projection['sequence'][row])
        block = next(b for b in report['sensor']['raw_blocks']
            if b['first_sequence'] <= sequence < b['first_sequence']+b['rows'])
        contributions.append(dict(sequence=sequence, column=column, weight=weight,
            flat_dn=float(rows[row][column]), raw_block=block['file'],
            block_row=sequence-block['first_sequence'], raw_block_sha256=block['sha256']))
    result = dict(valid=True, band=band, observations=observations, x_m=x, theta_rad=theta,
                  contributions=contributions, recomputed_dn=float(values[0, 0]))
    mosaic = report.get('mosaic', {})
    if mosaic.get('mode', 'none') != 'none' and mosaic['columns'][0] <= x_index < mosaic['columns'][1]:
        code = int(np.load(output/'mosaic_u16.npy', mmap_mode='r')[q_index, x_index-mosaic['columns'][0]])
        result['stored_dn'] = code/MOSAIC_SCALE
        result['error_dn'] = abs(result['stored_dn']-result['recomputed_dn'])
    return result


def write_review(sampler, grid, output, cover_min, stride, accelerator=None):
    Image.fromarray(display(preview_values(sampler, grid, stride, accelerator))).save(output/'preview.png')
    # Min-pooled: a preview cell is red if any of its grid pixels is uncovered.
    count = cover_min
    rgb = np.zeros((*count.shape, 3), np.uint8)
    rgb[count == 0] = [190, 40, 40]; rgb[count == 1] = [60, 150, 80]; rgb[count > 1] = [45, 105, 210]
    Image.fromarray(rgb).save(output/'overlap.png')
    reviews = []
    # At most three true-grid overlap pairs; these are image-derived samples, not
    # crack ground-truth coordinates. No sharpen/contrast stretch is applied.
    candidates = []
    for band in range(len(sampler.segments)-1):
        lo0, hi0, _, ok0 = sampler.row_sources(band, np.array([0.]))
        lo1, hi1, _, ok1 = sampler.row_sources(band+1, np.array([0.]))
        if not (ok0[0] and ok1[0]): continue
        axes = [sampler.projection['x_axis_m'][int(lo0[0])], sampler.projection['x_axis_m'][int(lo1[0])]]
        usable = sampler.output_offsets[sampler.geometry_valid]
        left = max(grid['target_x_m'][0], max(axes)+usable[0])
        right = min(grid['target_x_m'][1], min(axes)+usable[-1])
        if right-left > .04: candidates.append((band, left, right))
    for idx in np.unique(np.linspace(0, len(candidates)-1, min(3, len(candidates)), dtype=int)):
        band, left, right = candidates[idx]
        centre = (left+right)/2; columns = min(1024, int((right-left)/grid['dx_m'])-4)
        xs = centre+(np.arange(columns)-(columns-1)/2)*grid['dx_m']
        angles = (np.arange(256)-127.5)*grid['dq_m']/grid['radius_m']
        names = []
        for k in (band, band+1):
            values, valid, _ = sampler.sample(k, angles, xs)
            name = f'overlap_{band}_{k}.png'; Image.fromarray(display(values)).save(output/name); names.append(name)
        reviews.append(dict(bands=[band, band+1], x_m=[float(xs[0]), float(xs[-1])], theta_center_rad=0.,
                            shape=[len(angles), len(xs)], files=names))
    html = ['<!doctype html><meta charset="utf-8"><title>Stage D1 — nominal unroll</title>',
        '<style>body{font:16px sans-serif;background:#222;color:#eee;margin:24px}img{max-width:100%;image-rendering:auto}figure{display:inline-block;vertical-align:top;margin:8px}a{color:#9cf}</style>',
        '<h1>Stage D1: nominal cylinder, no registration or blending</h1>',
        f'<p>Linear DN, fixed 0–255 display. {grid["requested_pitch_m"]*1000:g} mm requested grid; preview is reduced. Dark invalid pixels remain masked. Top-to-bottom: right lower wall → crown → left lower wall.</p>',
        '<figure><figcaption>Nominal mosaic</figcaption><a href="preview.png"><img src="preview.png" height="650"></a></figure>',
        '<figure><figcaption>Cell minimum — red: any invalid pixel; green: single; blue: all overlapping</figcaption><img src="overlap.png" height="650"></figure>',
        '<h2>Adjacent bands at native output scale</h2><p>Each link opens an unscaled PNG. Misalignment is preserved for later matching.</p>']
    for review in reviews:
        html.append(f'<p>Bands {review["bands"]}; x = {review["x_m"]} m</p>')
        for name in review['files']:
            html.append(f'<figure><figcaption>{name}</figcaption><a href="{name}"><img src="{name}"></a></figure>')
    (output/'review.html').write_text('\n'.join(html))
    return reviews


def reconstruct(root, calibration_path, output, target_x=None, pitch=.0002, angular_tile_rows=32, backend='cpu',
                mosaic='none', mosaic_x=None, tile_columns=8192):
    started = time.monotonic(); capture = PublicCapture(root, calibration_path)
    timings = dict(public_inputs_s=time.monotonic()-started)
    if backend not in ('cpu', 'cuda'): raise ValueError('unknown unroll backend')
    if mosaic not in ('none', 'roi', 'full'): raise ValueError('mosaic must be none, roi or full')
    if (mosaic == 'roi') != (mosaic_x is not None): raise ValueError('mosaic roi requires --mosaic-x, and only roi takes it')
    inspection = capture.config.get('inspection', {})
    full_target = inspection.get('target_x_m')
    if full_target is None: raise ValueError('capture has no predeclared wall target')
    target = list(map(float, target_x if target_x is not None else full_target))
    if not (len(target) == 2 and full_target[0] <= target[0] < target[1] <= full_target[1]):
        raise ValueError('diagnostic ROI must lie inside the declared wall target')
    if not isinstance(angular_tile_rows, int) or angular_tile_rows < 1: raise ValueError('positive tile rows required')
    if not isinstance(tile_columns, int) or tile_columns < 1: raise ValueError('positive tile columns required')
    grid = target_grid(target, capture.config['calibration']['radius_m'], inspection['theta_rad'], pitch)
    if grid['shape'][1] > TILE_PIXELS: raise ValueError('grid width exceeds one million pixels')
    columns = None
    if mosaic == 'full': columns = (0, grid['shape'][1])
    elif mosaic == 'roi':
        a, b = map(float, mosaic_x)
        if not (target[0] <= a < b <= target[1]): raise ValueError('mosaic roi must lie inside the ROI')
        c0 = max(0, math.ceil((a-target[0])/grid['dx_m']-.5)); c1 = min(grid['shape'][1], math.floor((b-target[0])/grid['dx_m']-.5)+1)
        if c1 <= c0: raise ValueError('mosaic roi contains no grid cell centre')
        columns = (c0, c1)
    output = Path(output).resolve()
    if output.is_relative_to(capture.root):
        if not output.is_relative_to(capture.root/'reconstruction'):
            raise ValueError('outputs inside a session must stay in reconstruction/')
    output.mkdir(parents=True, exist_ok=False)
    phase_start = time.monotonic()
    sampler, inputs, sensor_report = prepare_sensor(capture, output, target)
    timings['prepare_sensor_s'] = time.monotonic()-phase_start
    bands = []
    for band, (a, b) in enumerate(sampler.bounds):
        rows = sampler.projection[a:b]
        bands.append(dict(band=band, segment=int(sampler.segments[band]), source_rows=[a, b],
            exposure_sequence=[int(rows['sequence'][0]), int(rows['sequence'][-1])],
            theta_rad=[float(rows['theta_rad'][0]), float(rows['theta_rad'][-1])],
            axis_x_m=[float(rows['x_axis_m'].min()), float(rows['x_axis_m'].max())]))
    (output/'bands.json').write_text(json.dumps(dict(schema='ssb.nominal_bands.v1', bands=bands,
        grid=grid, source_convention='native raw rows plus measured flat/inverse mapping; sample with BandSampler'), indent=2)+'\n')
    accelerator = None
    backend_info = dict(name='cpu', geometry_precision='float64')
    nq, nx = grid['shape']; stride = max(1, math.ceil(max(nx, nq)/1800))
    try:
        if backend == 'cuda':
            from .unroll_cuda import CudaRaster
            accelerator = CudaRaster(sampler)
        phase_start = time.monotonic()
        statistics, cover_min = rasterize(sampler, grid, output, angular_tile_rows, accelerator, columns,
                                          tile_columns, preview_stride=stride)
        timings['rasterize_s'] = time.monotonic()-phase_start
        if statistics['missing_pixels'] == statistics['total_pixels']: raise ValueError('no valid projected pixels')
        phase_start = time.monotonic()
        reviews = write_review(sampler, grid, output, cover_min, stride, accelerator)
        timings['review_s'] = time.monotonic()-phase_start
        if accelerator is not None: backend_info = accelerator.describe()
    finally:
        if accelerator is not None: accelerator.close()
    (output/'calibration.json').write_text(json.dumps(capture.calibration, indent=2)+'\n')
    mosaic_info = dict(mode=mosaic)
    if columns is not None:
        mosaic_info.update(columns=list(columns), x_m=[target[0]+(columns[0]+.5)*grid['dx_m'], target[0]+(columns[1]-.5)*grid['dx_m']],
            files=['mosaic_u16.npy', 'mosaic_count.npy'], code=f'round(DN*{MOSAIC_SCALE}) clipped to [0,{MOSAIC_INVALID-1}]; {MOSAIC_INVALID}=invalid',
            count='independent valid bands, saturating at 255', clipped_samples=statistics.pop('mosaic_clipped_samples'))
    report = dict(schema='ssb.initial_unroll.v2', stage='D1', grid=grid,
        declared_target_x_m=full_target, diagnostic_roi=target != full_target,
        bands=len(bands), sensor=sensor_report, coverage=statistics, mosaic=mosaic_info, review_pairs=reviews,
        preview=dict(stride=stride, values='exact mosaic values at every stride-th cell centre',
                     coverage='minimum count over each stride x stride cell'),
        optical_signature=capture.calibration['optical_signature'],
        resampling='flat-correct native uint8 columns on demand; measured geometric inverse and angular interpolation in one projection',
        provenance_mapping='grid + projection.npy + mapping.npz + native_source.json recover raw block/row/column interpolation weights (trace_pixel)',
        limitations=['nominal centered cylinder; no IMU or actual body pose', 'no matching, optimization or seam blending',
                     'calibration assumes radial lens and square pixels; sensor noise parameters are not reconstruction inputs'],
        backend=backend_info,
        performance=dict(wall_s=time.monotonic()-started, stages=timings, peak_rss_bytes=peak_rss_bytes(),
                         rss_source='current executable /proc/self/status VmHWM'))
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    outputs = sorted(p for p in output.iterdir() if p.is_file())
    hash_start = time.monotonic()
    provenance = stage_record('initial_unroll', inputs, outputs,
                              dict(grid=grid, angular_tile_rows=angular_tile_rows, tile_columns=tile_columns,
                                   mosaic=mosaic_info, backend=backend_info))
    provenance['performance'] = dict(hash_s=time.monotonic()-hash_start, wall_s=time.monotonic()-started,
                                    peak_rss_bytes=peak_rss_bytes())
    (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True); parser.add_argument('--calibration', required=True)
    parser.add_argument('--output', required=True); parser.add_argument('--target-x', type=float, nargs=2)
    parser.add_argument('--pitch-mm', type=float, default=.2)
    parser.add_argument('--angular-tile-rows', type=int, default=32)
    parser.add_argument('--tile-columns', type=int, default=8192)
    parser.add_argument('--mosaic', choices=['none', 'roi', 'full'], default='none',
                        help='full-resolution diagnostic mosaic; not used by D2/D3')
    parser.add_argument('--mosaic-x', type=float, nargs=2, help='mosaic roi x range [m]')
    parser.add_argument('--backend', choices=['cpu', 'cuda'], default='cuda',
                        help='CUDA is explicit and mandatory by default; CPU remains the reference')
    args = parser.parse_args()
    report = reconstruct(args.session, args.calibration, args.output, args.target_x,
                         args.pitch_mm/1000, args.angular_tile_rows, args.backend,
                         args.mosaic, args.mosaic_x, args.tile_columns)
    print(json.dumps({k: report[k] for k in ('bands', 'coverage', 'mosaic', 'performance')}))


if __name__ == '__main__': main()
