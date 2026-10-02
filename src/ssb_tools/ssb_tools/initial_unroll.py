"""Stage D1: flat-correct native pixels and project measured rays onto a nominal cylinder.

Only PublicCapture inputs are used. Individual helical bands remain independently
sampleable; the first mosaic selects one band without blending or fitting motion.
"""
import argparse
import json
import math
import mmap
from pathlib import Path
import time
import numpy as np
from PIL import Image
from .optical_calibration import flat_correct
from .provenance import stage_record
from .public_capture import PublicCapture, confined_file
from .session import sha256_file
from .wall_coverage import target_grid
from .stage_b_scene import peak_rss_bytes

PROJECTION = np.dtype([('sequence', '<i8'), ('lattice_row', '<i8'), ('segment', '<i8'),
                       ('x_axis_m', '<f8'), ('theta_rad', '<f8')])


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
        # Keep the owning mmap for page release, but gather through an ndarray
        # view to avoid thousands of memmap subclass operations per tile.
        self.pixels = np.asarray(image)
        for segment in self.segments:
            ids = np.flatnonzero(projection['segment'] == segment)
            if not np.array_equal(ids, np.arange(ids[0], ids[-1]+1)):
                raise ValueError('interleaved acquisition segments')
            if len(ids) < 2 or not np.all(np.diff(projection['theta_rad'][ids]) > 0):
                raise ValueError('band angles must increase and contain at least two rows')
            self.bounds.append((int(ids[0]), int(ids[-1]+1)))
            self.phases.append(projection['theta_rad'][ids]-2*math.pi*int(segment))

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
            v0, v1 = self.pixels[ids[:, None], left], self.pixels[ids[:, None], right]
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


def prepare_sensor(capture, output, target, chunk_rows=256):
    offsets, corrected_offsets, geometry_valid = sensor_geometry(capture.config, capture.calibration)
    usable = corrected_offsets[geometry_valid]
    selected = np.flatnonzero((capture.x_axis+usable[-1] >= target[0]) &
                              (capture.x_axis+usable[0] <= target[1]))
    if not len(selected): raise ValueError('no exposure rows intersect the requested ROI')
    projection = np.zeros(len(selected), PROJECTION)
    for name, field in [('sequence', 'sequence'), ('lattice_row', 'row'), ('segment', 'segment')]:
        projection[name] = capture.rows[field][selected]
    projection['x_axis_m'] = capture.x_axis[selected]; projection['theta_rad'] = capture.theta[selected]
    np.save(output/'projection.npy', projection)
    np.savez(output/'mapping.npz', native_x_offset_m=offsets, corrected_x_offset_m=corrected_offsets,
             geometry_valid=geometry_valid, angular_footprint_rad=capture.footprint)
    image = np.lib.format.open_memmap(output/'sensor_flat.npy', 'w+', np.float32, shape=(len(selected), capture.width))
    masks = np.lib.format.open_memmap(output/'sensor_valid_bits.npy', 'w+', np.uint8,
                                     shape=(len(selected), (capture.width+7)//8))
    inputs = list(capture.inputs); raw_blocks = []; saturated = 0; invalid = 0
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
            hi = min(b, lo+chunk_rows)
            samples = raw[selected[lo:hi]-first]
            corrected, valid = flat_correct(samples, capture.calibration['flat'])
            corrected[~valid] = np.nan
            image[lo:hi] = corrected
            masks[lo:hi] = np.packbits(valid, axis=1, bitorder='little')
            saturated += int(np.count_nonzero(samples == 255)); invalid += int(np.count_nonzero(~valid))
        release_pages(raw); release_pages(image); release_pages(masks)
    image.flush(); masks.flush(); release_pages(image); release_pages(masks)
    del masks
    return BandSampler(projection, image, offsets, corrected_offsets, geometry_valid, capture.footprint), inputs, dict(
        selected_rows=len(selected), raw_blocks=raw_blocks, saturated_samples=saturated, invalid_native_samples=invalid)


def rasterize(sampler, grid, output, angular_tile_rows=32):
    nq, nx = grid['shape']
    image = np.lib.format.open_memmap(output/'mosaic.npy', 'w+', np.float32, shape=(nq, nx))
    counts = np.lib.format.open_memmap(output/'coverage.npy', 'w+', np.uint16, shape=(nq, nx))
    source = np.lib.format.open_memmap(output/'source_band.npy', 'w+', np.int16, shape=(nq, nx))
    if len(sampler.segments) > np.iinfo(np.int16).max: raise ValueError('too many bands')
    xs = grid['target_x_m'][0]+(np.arange(nx)+.5)*grid['dx_m']
    missing = overlap = 0
    for a in range(0, nq, angular_tile_rows):
        b = min(nq, a+angular_tile_rows)
        angles = grid['theta_rad'][0]+(np.arange(a, b)+.5)*grid['dq_m']/grid['radius_m']
        shape = (b-a, nx)
        values = np.full(shape, np.nan, np.float32); number = np.zeros(shape, np.uint16)
        chosen = np.full(shape, -1, np.int16); best = np.full(shape, -np.inf, np.float32)
        for band in range(len(sampler.segments)):
            lo, hi, _, supported = sampler.row_sources(band, angles)
            if not np.any(supported): continue
            axes = sampler.projection['x_axis_m'][np.r_[lo[supported], hi[supported]]]
            usable = sampler.output_offsets[sampler.geometry_valid]
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
        image[a:b] = values; counts[a:b] = number; source[a:b] = chosen
        missing += int(np.count_nonzero(number == 0)); overlap += int(np.count_nonzero(number > 1))
        # Keep file-backed RSS bounded as the multi-billion-pixel output grows.
        for array in (image, counts, source, sampler.image): release_pages(array)
    for array in (image, counts, source): array.flush(); release_pages(array)
    return dict(missing_pixels=missing, overlapping_pixels=overlap, total_pixels=nq*nx), (image, counts, source)


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


def load_bands(output):
    """Load only self-contained D1 products, with no capture or evaluation access."""
    output = Path(output)
    with np.load(output/'mapping.npz') as mapping:
        return BandSampler(np.load(output/'projection.npy', mmap_mode='r'),
            np.load(output/'sensor_flat.npy', mmap_mode='r'), mapping['native_x_offset_m'],
            mapping['corrected_x_offset_m'], mapping['geometry_valid'],
            float(mapping['angular_footprint_rad']))


def trace_pixel(output, q_index, x_index):
    """Recover native exposure/column contributions to one unblended mosaic pixel."""
    output = Path(output); report = json.loads((output/'report.json').read_text()); grid = report['grid']
    nq, nx = grid['shape']
    if not (0 <= q_index < nq and 0 <= x_index < nx): raise ValueError('pixel outside output grid')
    band = int(np.load(output/'source_band.npy', mmap_mode='r')[q_index, x_index])
    if band < 0: return dict(valid=False, contributions=[])
    sampler = load_bands(output)
    theta = grid['theta_rad'][0]+(q_index+.5)*grid['dq_m']/grid['radius_m']
    x = grid['target_x_m'][0]+(x_index+.5)*grid['dx_m']
    values, valid, _, sources = sampler.sample(band, [theta], [x], sources=True)
    if not valid[0, 0]: raise ValueError('source map points to an invalid sample')
    combined = {}
    angular = sources['angular_weight'][0]
    for name, row, wq in [('lower_column', sources['lower'][0], 1-angular),
                          ('upper_column', sources['upper'][0], angular)]:
        column = sources[name][0, 0]; left = int(math.floor(column)); alpha = column-left
        for u, wu in [(left, 1-alpha), (min(left+1, sampler.image.shape[1]-1), alpha)]:
            if wu*wq <= 0: continue
            key = int(row), u
            combined[key] = combined.get(key, 0.)+float(wu*wq)
    contributions = []
    for (row, column), weight in combined.items():
        sequence = int(sampler.projection['sequence'][row])
        block = next(b for b in report['sensor']['raw_blocks']
            if b['first_sequence'] <= sequence < b['first_sequence']+b['rows'])
        contributions.append(dict(sequence=sequence, column=column, weight=weight,
            flat_dn=float(sampler.image[row, column]), raw_block=block['file'],
            block_row=sequence-block['first_sequence'], raw_block_sha256=block['sha256']))
    stored = float(np.load(output/'mosaic.npy', mmap_mode='r')[q_index, x_index])
    return dict(valid=True, band=band, x_m=x, theta_rad=theta, contributions=contributions,
                stored_dn=stored, recomputed_dn=float(values[0, 0]),
                error_dn=abs(stored-float(values[0, 0])))


def write_review(sampler, grid, output, mosaic):
    image, counts, source = mosaic
    nq, nx = grid['shape']; stride = max(1, math.ceil(max(nx, nq)/1800))
    Image.fromarray(display(preview_sample(image, stride))).save(output/'preview.png')
    count = preview_sample(counts, stride)
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
        '<figure><figcaption>Red: invalid; green: single; blue: overlapping</figcaption><img src="overlap.png" height="650"></figure>',
        '<h2>Adjacent bands at native output scale</h2><p>Each link opens an unscaled PNG. Misalignment is preserved for later matching.</p>']
    for review in reviews:
        html.append(f'<p>Bands {review["bands"]}; x = {review["x_m"]} m</p>')
        for name in review['files']:
            html.append(f'<figure><figcaption>{name}</figcaption><a href="{name}"><img src="{name}"></a></figure>')
    (output/'review.html').write_text('\n'.join(html))
    return reviews


def reconstruct(root, calibration_path, output, target_x=None, pitch=.0002, angular_tile_rows=32):
    started = time.monotonic(); capture = PublicCapture(root, calibration_path)
    inspection = capture.config.get('inspection', {})
    full_target = inspection.get('target_x_m')
    if full_target is None: raise ValueError('capture has no predeclared wall target')
    target = list(map(float, target_x if target_x is not None else full_target))
    if not (len(target) == 2 and full_target[0] <= target[0] < target[1] <= full_target[1]):
        raise ValueError('diagnostic ROI must lie inside the declared wall target')
    if not isinstance(angular_tile_rows, int) or angular_tile_rows < 1: raise ValueError('positive tile rows required')
    grid = target_grid(target, capture.config['calibration']['radius_m'], inspection['theta_rad'], pitch)
    # One tile's temporary arrays scale with width; enforce before allocation.
    if grid['shape'][1] > 1024*1024:
        raise ValueError('grid width exceeds one million pixels')
    angular_tile_rows = min(angular_tile_rows, (1024*1024)//grid['shape'][1])
    output = Path(output).resolve()
    if output.is_relative_to(capture.root):
        if not output.is_relative_to(capture.root/'reconstruction'):
            raise ValueError('outputs inside a session must stay in reconstruction/')
    output.mkdir(parents=True, exist_ok=False)
    sampler, inputs, sensor_report = prepare_sensor(capture, output, target)
    bands = []
    for band, (a, b) in enumerate(sampler.bounds):
        rows = sampler.projection[a:b]
        bands.append(dict(band=band, segment=int(sampler.segments[band]), source_rows=[a, b],
            exposure_sequence=[int(rows['sequence'][0]), int(rows['sequence'][-1])],
            theta_rad=[float(rows['theta_rad'][0]), float(rows['theta_rad'][-1])],
            axis_x_m=[float(rows['x_axis_m'].min()), float(rows['x_axis_m'].max())]))
    (output/'bands.json').write_text(json.dumps(dict(schema='ssb.nominal_bands.v1', bands=bands,
        grid=grid, source_convention='native sensor image plus measured inverse mapping; sample with BandSampler'), indent=2)+'\n')
    statistics, mosaic = rasterize(sampler, grid, output, angular_tile_rows)
    if statistics['missing_pixels'] == statistics['total_pixels']: raise ValueError('no valid projected pixels')
    reviews = write_review(sampler, grid, output, mosaic)
    (output/'calibration.json').write_text(json.dumps(capture.calibration, indent=2)+'\n')
    report = dict(schema='ssb.initial_unroll.v1', stage='D1', grid=grid,
        declared_target_x_m=full_target, diagnostic_roi=target != full_target,
        bands=len(bands), sensor=sensor_report, coverage=statistics, review_pairs=reviews,
        optical_signature=capture.calibration['optical_signature'],
        resampling='flat-correct native columns; measured geometric inverse and angular interpolation in one projection',
        provenance_mapping='source_band pixel + grid + projection.npy + mapping.npz recover raw row/column interpolation weights',
        limitations=['nominal centered cylinder; no IMU or actual body pose', 'no matching, optimization or seam blending',
                     'calibration assumes radial lens and square pixels; sensor is currently noiseless'],
        performance=dict(wall_s=time.monotonic()-started, peak_rss_bytes=peak_rss_bytes(),
                         rss_source='current executable /proc/self/status VmHWM'))
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    outputs = sorted(p for p in output.iterdir() if p.is_file())
    provenance = stage_record('initial_unroll', inputs, outputs, dict(grid=grid, angular_tile_rows=angular_tile_rows))
    (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True); parser.add_argument('--calibration', required=True)
    parser.add_argument('--output', required=True); parser.add_argument('--target-x', type=float, nargs=2)
    parser.add_argument('--pitch-mm', type=float, default=.2)
    parser.add_argument('--angular-tile-rows', type=int, default=32)
    args = parser.parse_args()
    report = reconstruct(args.session, args.calibration, args.output, args.target_x,
                         args.pitch_mm/1000, args.angular_tile_rows)
    print(json.dumps({k: report[k] for k in ('bands', 'coverage', 'performance')}))


if __name__ == '__main__': main()
