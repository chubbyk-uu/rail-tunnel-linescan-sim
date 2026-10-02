"""Nominal wall coverage from public encoders, exposure records and measured calibration.

This is geometric coverage, not a claim about actual wall pose, sharpness or saturation.
No simulation trajectory or scene asset is a reconstruction input. Count runs encode a
0.2 mm centre-sampled grid without allocating the multi-billion-pixel full image.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

from .session import Session, read_json, sha256_file
from .provenance import stage_record
from .unroll import row_axis_coordinates

RUN_DTYPE = np.dtype([('q_bin', '<i4'), ('x_begin', '<i4'), ('x_end', '<i4'), ('count', '<i4')])


def calibrated_spans(config, calibration):
    """Usable corrected-column footprints; identical static mask rules to optical correction."""
    camera = config['camera']; width = camera['width']
    geometry, flat = calibration['geometry'], calibration['flat']
    lookup = np.asarray(geometry['lookup'], float)
    valid = np.asarray(geometry['valid'], bool)
    flat_valid = np.asarray(flat['valid'], bool)
    if lookup.shape != (width,) or valid.shape != (width,) or flat_valid.shape != (width,):
        raise ValueError('calibration/camera width mismatch')
    if not np.isfinite(lookup).all() or np.any(np.diff(lookup) < 0) or lookup.min() < 0 or lookup.max() > width-1:
        raise ValueError('invalid calibration lookup')
    left = np.floor(lookup).astype(int); right = np.minimum(left+1, width-1)
    valid = valid & flat_valid[left] & flat_valid[right]
    indices = np.flatnonzero(valid)
    if not len(indices): raise ValueError('no calibrated usable columns')
    fov = float(geometry['output_fov_m'])
    radius = float(config['calibration']['radius_m'])
    distance = float(camera['nominal_distance_m'])
    if not all(math.isfinite(x) and x > 0 for x in (fov, radius, distance)):
        raise ValueError('invalid calibrated field of view')
    step = fov/width*radius/distance
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1)+1)
    # A column represents a finite sensor footprint, not only its centre ray.
    return [(float((g[0]-width/2)*step), float((g[-1]+1-width/2)*step)) for g in groups]


def target_grid(target_x, radius, theta_range, pitch):
    x0, x1 = map(float, target_x); lo, hi = map(float, theta_range)
    if not all(math.isfinite(x) for x in (x0, x1, radius, lo, hi, pitch)) or x1 <= x0 or radius <= 0 or pitch <= 0:
        raise ValueError('invalid target or grid pitch')
    if not 0 < hi-lo <= 2*math.pi:
        raise ValueError('target angles must be an increasing arc of at most 360 degrees')
    nx = math.ceil((x1-x0)/pitch); nq = math.ceil(radius*(hi-lo)/pitch)
    if max(nx, nq) >= np.iinfo(np.int32).max:
        raise ValueError('coverage grid indices exceed int32')
    return dict(target_x_m=[x0, x1], theta_rad=[lo, hi], radius_m=radius,
                shape=[nq, nx], dx_m=(x1-x0)/nx, dq_m=radius*(hi-lo)/nq,
                requested_pitch_m=pitch, convention='cell centres; finite nominal column/row footprints')


def calibrated_row_footprint(config, calibration):
    """Conservative perpendicular pixel angle for the nominal centred radial lens.

    The image-measured cubic inverse is x(q_d)=c0+c1*q_d+c2*q_d²+c3*q_d³.
    Its odd radial part gives the perpendicular gain c1+c3*q_d², including
    the q_d=0 limit. Square sensor pixels span 2/width in normalized coordinates.
    This is a nominal radial-lens assumption, not a measured body attitude or 2-D PSF.
    """
    coefficients = np.asarray(calibration['geometry']['coefficients'], float)
    if coefficients.shape != (4,) or not np.isfinite(coefficients).all():
        raise ValueError('invalid measured radial lens coefficients')
    width = config['camera']['width']
    lookup = np.asarray(calibration['geometry']['lookup'], float)
    valid = np.asarray(calibration['geometry']['valid'], bool)
    q = (lookup[valid]-(width-1)/2)/(width/2)
    gain = np.min(coefficients[1]+coefficients[3]*q*q)
    if not math.isfinite(gain) or gain <= 0:
        raise ValueError('invalid measured perpendicular pixel gain')
    # x is measured at the nominal calibration distance, not necessarily the tunnel radius.
    return 2*math.atan(gain/(width*config['camera']['nominal_distance_m']))


def coverage_runs(x_axis, theta, config, spans, grid, q_tile_rows=512, angular_footprint_rad=None):
    """Sweep interval endpoints in each angular tile, including zero-coverage runs.

    Memory scales with captured rows plus a small angular tile, not wall pixel count.
    Pixel support is independent of encoder sampling distance; missing row gaps
    are never bridged by expanding neighbouring observations.
    """
    x_axis = np.asarray(x_axis, float); theta = np.asarray(theta, float)
    if x_axis.shape != theta.shape or x_axis.ndim != 1 or not np.isfinite(x_axis).all() or not np.isfinite(theta).all():
        raise ValueError('invalid row projection')
    if q_tile_rows <= 0: raise ValueError('angular tile size must be positive')
    nq, nx = grid['shape']; dx, dq = grid['dx_m'], grid['dq_m']; radius = grid['radius_m']
    lo, hi = grid['theta_rad']; x0 = grid['target_x_m'][0]
    gate = config['gate']; gate_span = (gate['end_rad']-gate['start_rad']) % (2*math.pi)
    if gate_span == 0: gate_span = 2*math.pi
    target_start = (lo-gate['start_rad']) % (2*math.pi)
    if target_start+hi-lo > gate_span+1e-10:
        raise ValueError('target extends outside the configured acquisition gate')
    # The rescaler lattice is only a fallback for direct synthetic lattice tests.
    encoder, rescaler = config['scan_encoder'], config['rescaler']
    row_step = 2*math.pi/(encoder['ppr']*encoder['edges_per_cycle']*rescaler['multiply']/rescaler['divide'])
    footprint = row_step if angular_footprint_rad is None else angular_footprint_rad
    if not math.isfinite(footprint) or footprint <= 0:
        raise ValueError('pixel angular footprint must be positive')
    phase = (theta-gate['start_rad']) % (2*math.pi)
    q = radius*(phase-target_start); half = radius*footprint/2
    first = np.maximum(0, np.ceil((q-half)/dq-.5-1e-9).astype(np.int64))
    last = np.minimum(nq, np.floor((q+half)/dq-.5+1e-9).astype(np.int64)+1)
    copies = int(max(0, np.max(last-first, initial=0)))
    if copies > 64: raise ValueError('grid is too fine relative to camera row spacing')
    # Sorted row references allow each tile to touch only its contributing observations.
    row_ids, q_ids = [], []
    for offset in range(copies):
        selected = np.flatnonzero(first+offset < last)
        row_ids.append(selected); q_ids.append(first[selected]+offset)
    rows = np.concatenate(row_ids) if row_ids else np.empty(0, np.int64)
    bins = np.concatenate(q_ids) if q_ids else np.empty(0, np.int64)
    order = np.argsort(bins, kind='stable'); bins = bins[order]; rows = rows[order]
    for begin in range(0, nq, q_tile_rows):
        end = min(nq, begin+q_tile_rows)
        i, j = np.searchsorted(bins, [begin, end])
        qs, centres = bins[i:j], x_axis[rows[i:j]]
        keys, changes = [], []
        for left, right in spans:
            a = np.clip(np.ceil((centres+left-x0)/dx-.5-1e-9).astype(np.int64), 0, nx)
            b = np.clip(np.floor((centres+right-x0)/dx-.5+1e-9).astype(np.int64)+1, 0, nx)
            valid = a < b
            keys.extend([qs[valid]*(nx+1)+a[valid], qs[valid]*(nx+1)+b[valid]])
            changes.extend([np.ones(valid.sum(), np.int64), -np.ones(valid.sum(), np.int64)])
        # Empty bins and uncovered leading/trailing intervals must also be emitted.
        boundaries = np.arange(begin, end, dtype=np.int64)*(nx+1)
        keys.extend([boundaries, boundaries+nx]); changes.extend([np.zeros(end-begin, np.int64)]*2)
        key = np.concatenate(keys); delta = np.concatenate(changes)
        order = np.argsort(key, kind='stable'); key = key[order]; delta = delta[order]
        unique = np.r_[0, np.flatnonzero(np.diff(key))+1]
        key = key[unique]; counts = np.cumsum(np.add.reduceat(delta, unique))
        if np.any(counts < 0) or counts[-1] != 0: raise ValueError('unbalanced coverage intervals')
        qid, xpos = np.divmod(key, nx+1)
        keep = (qid[:-1] == qid[1:]) & (xpos[1:] > xpos[:-1])
        if counts.max(initial=0) > np.iinfo(np.int32).max: raise ValueError('coverage count overflow')
        result = np.empty(keep.sum(), RUN_DTYPE)
        result['q_bin'] = qid[:-1][keep]; result['x_begin'] = xpos[:-1][keep]
        result['x_end'] = xpos[1:][keep]; result['count'] = counts[:-1][keep]
        yield result


def write_coverage(x_axis, theta, config, spans, grid, output, q_tile_rows=512, angular_footprint_rad=None):
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    histogram = {}; missing_bounds = None; run_count = 0
    nq, nx = grid['shape']; preview_w, preview_h = min(nx, 1600), min(nq, 900)
    preview_x = np.minimum(nx-1, ((np.arange(preview_w)+.5)*nx/preview_w).astype(int))
    preview_q = np.minimum(nq-1, ((np.arange(preview_h)+.5)*nq/preview_h).astype(int))
    preview = np.zeros((preview_h, preview_w, 3), np.uint8)
    colours = np.array([[190, 40, 45], [65, 160, 110], [55, 115, 220]], np.uint8)
    path = output/'counts.bin'
    with path.open('wb') as file:
        for tile in coverage_runs(x_axis, theta, config, spans, grid, q_tile_rows, angular_footprint_rad):
            tile.tofile(file); run_count += len(tile)
            lengths = tile['x_end'].astype(np.int64)-tile['x_begin']
            for count in np.unique(tile['count']):
                histogram[int(count)] = histogram.get(int(count), 0)+int(lengths[tile['count'] == count].sum())
            empty = tile[tile['count'] == 0]
            if len(empty):
                bounds = [int(empty['x_begin'].min()), int(empty['x_end'].max()),
                          int(empty['q_bin'].min()), int(empty['q_bin'].max()+1)]
                if missing_bounds is None: missing_bounds = bounds
                else: missing_bounds = [min(bounds[0], missing_bounds[0]), max(bounds[1], missing_bounds[1]),
                                       min(bounds[2], missing_bounds[2]), max(bounds[3], missing_bounds[3])]
            # Preview samples the exact runs. It does not replace full-resolution accounting.
            for v in np.flatnonzero((preview_q >= tile['q_bin'].min()) & (preview_q <= tile['q_bin'].max())):
                line = tile[tile['q_bin'] == preview_q[v]]
                idx = np.searchsorted(line['x_end'], preview_x, side='right')
                preview[v] = colours[np.minimum(line['count'][idx], 2)]
    pixels = nx*nq
    if sum(histogram.values()) != pixels: raise ValueError('coverage accounting does not span the target grid')
    missing = histogram.get(0, 0); multiple = sum(n for c, n in histogram.items() if c >= 2)
    report = dict(schema='ssb.wall_coverage.v1', scope='nominal calibrated geometric coverage from observable measurements',
                  nominal_complete=missing == 0, grid=grid, recorded_rows=len(x_axis),
                  missing_pixels=missing, missing_fraction=missing/pixels, overlap_pixels=multiple,
                  count_histogram=histogram, missing_bounds_bins=missing_bounds, calibrated_spans_m=spans,
                  counts=dict(file=path.name, sha256=sha256_file(path), records=run_count,
                              dtype=RUN_DTYPE.descr, record_bytes=RUN_DTYPE.itemsize),
                  angular_footprint_rad=angular_footprint_rad,
                  limitations=['No IMU: body attitude and height use nominal geometry.',
                               'No actual-pose or scene inputs. Nominal coverage does not certify actual wall coverage.',
                               'Static calibration masks included; row-dependent saturation, blur and occlusion not assessed.',
                               'Square sensor pixels and a centred radial lens; perpendicular gain inferred '
                               'from the image-measured cubic inverse, not an independent 2-D calibration.',
                               'Finite footprints on a centre-sampled grid; not a subpixel continuous-surface proof.'])
    Image.fromarray(preview).save(output/'preview.png')
    report['preview_legend'] = {'red': 'uncovered', 'green': 'one observation', 'blue': 'multiple observations'}
    return report


def inspect_session(root, calibration_path, output, target_x=None, pitch=None, q_tile_rows=512):
    session = Session(root); config = session.config(); inspection = config.get('inspection', {})
    if session.summary['status'] != 'complete' or not session.summary['motion']['complete']:
        raise ValueError('coverage acceptance requires a completed planned capture')
    manifest_path = session.root/'metadata/manifest.json'
    for name in ('config/observable_config.json', 'metadata/manifest.json'):
        expected = session.summary.get('files', {}).get(name)
        if expected and sha256_file(session.root/name) != expected:
            raise ValueError(f'public input changed since capture completion: {name}')
    manifest = read_json(manifest_path)
    names = ['rows', 'scan_edges', 'odometer_edges', 'gate_events']
    if config.get('contact', {}).get('enabled'): names.append('odometer_right_edges')
    # A forged metadata URI must not turn a public-table read into a private-file read.
    for name in names:
        path = session.root/'metadata'/manifest[name]['file']
        if not path.resolve().is_relative_to((session.root/'metadata').resolve()):
            raise ValueError(f'metadata dependency escapes the public directory: {name}')
    calibration = read_json(calibration_path)
    if config['camera']['optical_signature'] != calibration['optical_signature']:
        raise ValueError('session optical signature mismatch')
    target_x = target_x if target_x is not None else inspection.get('target_x_m')
    if target_x is None: raise ValueError('target wall interval must be specified before coverage acceptance')
    if inspection and not np.allclose(target_x, inspection['target_x_m'], rtol=0, atol=1e-12):
        raise ValueError('cannot change the recorded wall target for acceptance')
    pitch = pitch if pitch is not None else inspection.get('grid_pitch_m', .0002)
    if inspection and not math.isclose(pitch, inspection['grid_pitch_m'], rel_tol=0, abs_tol=1e-15):
        raise ValueError('cannot change the recorded acceptance grid pitch')
    rows = session.metadata('rows'); scan = session.metadata('scan_edges')
    odo = session.metadata('odometer_edges'); gates = session.metadata('gate_events')
    right = session.metadata('odometer_right_edges') if config.get('contact', {}).get('enabled') else None
    if len(rows) != session.summary['rows'] or not len(rows) or not np.array_equal(rows['sequence'], np.arange(len(rows))):
        raise ValueError('incomplete or repeated exposure sequences')
    if not np.all(np.diff(rows['t_center']) > 0): raise ValueError('exposure times must increase')
    x, theta = row_axis_coordinates(config, rows, scan, odo, gates, right)
    angles = inspection.get('theta_rad', [config['gate']['start_rad'], config['gate']['end_rad']])
    grid = target_grid(target_x, config['calibration']['radius_m'], angles, pitch)
    report = write_coverage(x, theta, config, calibrated_spans(config, calibration), grid, output,
                            q_tile_rows, calibrated_row_footprint(config, calibration))
    report['inputs'] = dict(observable_config_sha256=sha256_file(session.root/'config/observable_config.json'),
                           metadata_manifest_sha256=sha256_file(manifest_path), calibration_sha256=sha256_file(calibration_path))
    report['metadata_tables'] = {name: manifest[name]['sha256'] for name in names}
    output = Path(output)
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    inputs = [session.root/'config/observable_config.json', manifest_path, Path(calibration_path),
              session.root/'session.json']
    inputs += [session.root/'metadata'/manifest[name]['file'] for name in names]
    provenance = stage_record('wall_coverage', inputs,
                              [output/name for name in ('counts.bin', 'preview.png', 'report.json')],
                              dict(grid=grid, angular_tile_rows=q_tile_rows,
                                   angular_footprint_rad=report['angular_footprint_rad']))
    (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True); parser.add_argument('--calibration', required=True)
    parser.add_argument('--output', required=True); parser.add_argument('--target-x', nargs=2, type=float)
    parser.add_argument('--pitch-mm', type=float); parser.add_argument('--angular-tile-rows', type=int, default=512)
    args = parser.parse_args()
    result = inspect_session(args.session, args.calibration, args.output, args.target_x,
                             None if args.pitch_mm is None else args.pitch_mm/1000, args.angular_tile_rows)
    print(json.dumps({k: result[k] for k in ('nominal_complete', 'recorded_rows', 'missing_pixels', 'missing_fraction', 'overlap_pixels')}))
    return 0 if result['nominal_complete'] else 1


if __name__ == '__main__': raise SystemExit(main())
