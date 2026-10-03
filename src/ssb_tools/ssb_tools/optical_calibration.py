"""Image-estimated line-scan calibration; no rendering coefficients as correction inputs."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
from .session import Session, sha256_file


def flat_field(dark, bright):
    if any(a.dtype != np.uint8 or a.ndim != 2 or len(a) < 256 for a in (dark, bright)) or dark.shape[1] != bright.shape[1]:
        raise ValueError('need at least 256 Mono8 dark/bright rows with matching width')
    offset = dark.mean(axis=0)
    signal = bright.mean(axis=0) - offset
    valid = (signal >= 16) & ((bright == 255).mean(axis=0) < .001)
    if valid.mean() < .95:
        raise ValueError('too many weak/saturated flat-field columns')
    target = float(np.median(signal[valid]))
    gain = np.divide(target, signal, out=np.zeros_like(signal), where=valid)
    return dict(offset=offset.tolist(), gain=gain.tolist(), valid=valid.tolist(), target_dn=target)


def centers(image):
    profile = np.median(image, axis=0)
    lo, hi = np.percentile(profile, [1, 75])
    if not np.isfinite(profile).all() or hi-lo < 16:
        raise ValueError('invalid/low contrast stripe target')
    indices = np.flatnonzero(profile < lo+.4*(hi-lo))
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1)+1)
    result = []
    for g in groups:
        if len(g) < 2 or g[0] < 4 or g[-1] >= len(profile)-4:
            continue
        # Include partially covered edge pixels; threshold-only centroids can bias
        # a stripe by ~0.3 px as it crosses pixel boundaries.
        samples = np.arange(g[0]-2, g[-1]+3)
        background = np.median(np.r_[profile[g[0]-4:g[0]-2], profile[g[-1]+3:g[-1]+5]])
        weights = np.maximum(0, background-profile[samples])
        weights[weights < 2] = 0  # reject sub-DN flat-field quantisation fluctuations
        result.append(np.average(samples, weights=weights))
    return np.asarray(result)


def flat_correct(raw, flat):
    raw = np.asarray(raw)
    if raw.dtype != np.uint8 or raw.ndim != 2 or raw.shape[1] != len(flat['offset']):
        raise ValueError('Mono8/width mismatch')
    out = (raw.astype(np.float32)-np.asarray(flat['offset'], np.float32))*np.asarray(flat['gain'], np.float32)
    valid = np.asarray(flat['valid'], bool)[None, :] & (raw != 255)
    return out, valid


def fit_geometry(columns, positions, width, fov):
    positions = np.asarray(positions)
    if len(columns) != len(positions) or len(columns) < 8 or np.any(np.diff(positions) <= 0):
        raise ValueError('stripe count/order mismatch')
    q = (columns-(width-1)/2)/(width/2)
    # Cubic inverse approximation, estimated solely from observed stripe centres.
    coefficients = np.polynomial.polynomial.polyfit(q, positions, 3)
    residual = (np.polynomial.polynomial.polyval(q, coefficients)-positions)/(fov/width)
    sensor_q = (np.arange(width)-(width-1)/2)/(width/2)
    mapping = np.polynomial.polynomial.polyval(sensor_q, coefficients)
    if np.any(np.diff(mapping) <= 0) or np.max(np.abs(residual)) > .5:
        raise ValueError('non-monotonic or inaccurate target fit')
    output_x = sensor_q*fov/2
    lookup = np.interp(output_x, mapping, np.arange(width))
    valid = (output_x >= positions[0]) & (output_x <= positions[-1])
    return dict(coefficients=coefficients.tolist(), lookup=lookup.tolist(), valid=valid.tolist(),
                output_fov_m=fov, fit_max_error_px=float(np.max(np.abs(residual))))


def correct(raw, calibration, signature):
    if signature != calibration['optical_signature']:
        raise ValueError('optical conditions changed; recalibration required')
    data, valid = flat_correct(raw, calibration['flat'])
    geometry = calibration['geometry']
    lookup = np.asarray(geometry['lookup'])
    left = np.floor(lookup).astype(int)
    right = np.minimum(left+1, raw.shape[1]-1)
    alpha = (lookup-left).astype(np.float32)
    result = data[:, left]*(1-alpha)+data[:, right]*alpha
    mask = valid[:, left] & valid[:, right] & np.asarray(geometry['valid'])[None, :]
    result[~mask] = np.nan
    return result.astype(np.float32), mask


def fit(bench, output):
    bench = Path(bench).resolve(); meta = json.loads(bench.read_text()); root = bench.parent
    images, signatures, hashes = {}, set(), {}
    for name, target in meta['targets'].items():
        directory = root/target['capture']
        probe = json.loads((directory/'probe.json').read_text())
        image = directory/'image.pgm'
        digest = sha256_file(image)
        if digest != probe['image_sha256']:
            raise ValueError('calibration image hash mismatch')
        images[name] = np.array(Image.open(image))
        signatures.add(probe['optical_signature']); hashes[name] = digest
    if len(signatures) != 1:
        raise ValueError('calibration captures have different optical conditions')
    flat = flat_field(images['dark'], images['bright'])
    stripes, mask = flat_correct(images['bars'], flat)
    if not mask.all():
        raise ValueError('invalid target pixels')
    width = stripes.shape[1]; fov = meta['fov_m']
    geometry = fit_geometry(centers(stripes), meta['targets']['bars']['positions_m'], width, fov)
    holdout, mask = flat_correct(images['bars_holdout'], flat)
    columns = centers(holdout)
    positions = np.asarray(meta['targets']['bars_holdout']['positions_m'])
    if len(columns) != len(positions) or not mask.all():
        raise ValueError('holdout target mismatch')
    measured = np.polynomial.polynomial.polyval((columns-(width-1)/2)/(width/2), geometry['coefficients'])
    error = float(np.max(np.abs(measured-positions))/(fov/width))
    bright, bright_valid = flat_correct(images['bright_holdout'], flat)
    profile = np.mean(bright, axis=0)[bright_valid.all(axis=0)]
    uniformity = float(np.std(profile)/np.mean(profile))
    raw_profile = images['bright_holdout'].mean(axis=0)
    raw_uniformity = float(raw_profile.std()/raw_profile.mean())
    temporal_std = float(np.median(bright[:, bright_valid.all(axis=0)].std(axis=0, ddof=1)))
    dark_std = float(np.median(images['dark'].std(axis=0, ddof=1)))
    if error > .5 or uniformity > .01:
        raise ValueError(f'holdout failed: {error=}, {uniformity=}')
    result = dict(schema='ssb.measured_optical_calibration.v1', optical_signature=signatures.pop(),
                  flat=flat, geometry=geometry, image_sha256=hashes,
                  validation=dict(holdout_max_error_px=error, flat_cv=uniformity, raw_flat_cv=raw_uniformity,
                                  valid_output_fraction=float(np.mean(geometry['valid'])),
                                  measured_dark_temporal_std_dn=dark_std,
                                  measured_bright_temporal_std_dn=temporal_std,
                                  rows_per_target={k:len(v) for k,v in images.items()}),
                  limitations='Measured from independent targets; finite-row noise remains; fixed nominal distance; no edge extrapolation or true sensor parameters.')
    with Path(output).open('x') as f:
        json.dump(result, f, indent=2); f.write('\n')
    return result


def correct_session(session_path, calibration_path, output):
    session = Session(session_path); calibration_path = Path(calibration_path)
    calibration = json.loads(calibration_path.read_text())
    signature = session.config()['camera']['optical_signature']
    if signature != calibration['optical_signature']:
        raise ValueError('session optical signature mismatch')
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    index = session.raw_index(); blocks = []
    # Memory bounded independently of the full run length; float DN is never clipped to Mono8.
    for block in index['blocks']:
        source = session.root/'raw'/block['file']
        if sha256_file(source) != block['sha256']:
            raise ValueError('raw block hash mismatch')
        rows, width = block['rows'], index['width']
        if source.stat().st_size != rows*width:
            raise ValueError('raw block size mismatch')
        raw = np.memmap(source, np.uint8, mode='r', shape=(rows, width))
        stem = f"{block['first_sequence']:012d}"
        image_path, mask_path = output/(stem+'.npy'), output/(stem+'_valid.npy')
        image = np.lib.format.open_memmap(image_path, mode='w+', dtype=np.float32, shape=raw.shape)
        masks = np.lib.format.open_memmap(mask_path, mode='w+', dtype=bool, shape=raw.shape)
        for first in range(0, rows, 256):
            image[first:first+256], masks[first:first+256] = correct(raw[first:first+256], calibration, signature)
        image.flush(); masks.flush(); del image, masks, raw
        blocks.append(dict(first_sequence=block['first_sequence'], rows=rows, source_sha256=block['sha256'],
                           image=image_path.name, mask=mask_path.name, image_sha256=sha256_file(image_path),
                           mask_sha256=sha256_file(mask_path)))
    manifest = dict(schema='ssb.corrected_raw.v1', width=index['width'], optical_signature=signature,
                    calibration_sha256=sha256_file(calibration_path), units='linear DN float32; NaN means invalid',
                    output_fov_m=calibration['geometry']['output_fov_m'], blocks=blocks)
    (output/'index.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__); sub = parser.add_subparsers(dest='command', required=True)
    fit_parser = sub.add_parser('fit'); fit_parser.add_argument('--bench', required=True); fit_parser.add_argument('--output', required=True)
    apply_parser = sub.add_parser('apply'); apply_parser.add_argument('--session', required=True)
    apply_parser.add_argument('--calibration', required=True); apply_parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.command == 'fit':
        print(json.dumps(fit(args.bench, args.output)['validation']))
    else:
        print(json.dumps(dict(blocks=len(correct_session(args.session, args.calibration, args.output)['blocks']))))

if __name__ == '__main__':
    main()
