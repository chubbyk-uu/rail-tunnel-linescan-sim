"""D2: verified public D1 bands -> image correspondences, no pose truth."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
from PIL import Image, ImageDraw
from .band_matching import MatchSettings, match_window
from .initial_unroll import load_bands, release_pages, display
from .public_capture import confined_file
from .provenance import stage_record
from .session import read_json, sha256_file
from .stage_b_scene import peak_rss_bytes

MATCH = np.dtype([('window', '<i4'), ('band_a', '<i2'), ('band_b', '<i2'),
    ('x_a_m', '<f8'), ('q_a_m', '<f8'), ('x_b_m', '<f8'), ('q_b_m', '<f8'),
    ('ncc', '<f4'), ('fb_px', '<f4'), ('residual_px', '<f4'), ('inlier', 'u1'), ('holdout', 'u1')]+
    [(side+'_'+name, kind) for side in ('a', 'b') for name, kind in
        [('lower_sequence', '<i8'), ('upper_sequence', '<i8'), ('angular_weight', '<f8'),
         ('lower_column', '<f8'), ('upper_column', '<f8')]])


def verified_bands(root, raw_root=None):
    """Hash-verified D1 products. v2 samples the capture's public uint8 raw blocks (hashes
    recorded by D1; `raw_root` relocates them); legacy v1 uses its float32 cache."""
    root = Path(root).resolve()
    names = ['provenance.json', 'report.json', 'bands.json', 'projection.npy', 'mapping.npz']
    names.append('native_source.json' if (root/'native_source.json').exists() else 'sensor_flat.npy')
    paths = [confined_file(root, name) for name in names]
    provenance = read_json(paths[0])
    if provenance.get('stage') != 'initial_unroll': raise ValueError('verified D1 products required')
    outputs = {}
    for path, digest in provenance['outputs'].items():
        name = Path(path).name
        if name in outputs: raise ValueError('ambiguous upstream product identity')
        outputs[name] = digest
    for path in paths[1:]:
        if outputs.get(path.name) != sha256_file(path):
            raise ValueError('D1 product hash mismatch: '+path.name)
    report = read_json(root/'report.json')
    if report.get('schema') not in ('ssb.initial_unroll.v1', 'ssb.initial_unroll.v2'): raise ValueError('unsupported D1 schema')
    # Carry only public observation identities. Never dereference the archived
    # source paths; relocated D1 products remain sufficient for matching.
    allowed = ('config/observable_config.json', 'metadata/manifest.json', 'raw/index.json')
    identities = {}
    for name in allowed:
        hashes = [value for path, value in provenance['inputs'].items()
                  if Path(path).as_posix().endswith('/'+name)]
        if len(hashes) > 1: raise ValueError('ambiguous public observation identity')
        if hashes: identities[name] = hashes[0]
    report['source_observation_hashes'] = identities
    sampler = load_bands(root, raw_root)
    if tuple(sampler.native.shape) != (len(sampler.projection), len(sampler.offsets)):
        raise ValueError('inconsistent D1 native image dimensions')
    if hasattr(sampler.native, 'verify_all'): sampler.native.verify_all()
    if hasattr(sampler.native, 'blocks'):
        # Raw blocks are public observations; they enter provenance by identity.
        paths += [sampler.native.raw_dir/b['file'] for b in sampler.native.blocks]
    return sampler, report, paths


def plan_windows(sampler, grid, spacing_m, height, max_width, settings, halo_m=.25):
    if (not math.isfinite(spacing_m) or spacing_m <= 0 or height < 64 or max_width < 64 or
        height*max_width > 1024*1024 or not math.isfinite(halo_m) or not 0 <= halo_m <= .5):
        raise ValueError('invalid or excessive matching window budget')
    dq, dx, radius = grid['dq_m'], grid['dx_m'], grid['radius_m']
    half = (height-1)*dq/2
    lower, upper = np.asarray(grid['theta_rad'])*radius
    count = max(0, math.ceil((upper-lower-2*half)/spacing_m))
    if count*max(0, len(sampler.segments)-1) > 4096:
        raise ValueError('matching plan exceeds 4096-window resource budget')
    usable = sampler.output_offsets[sampler.geometry_valid]
    windows = []
    for band in range(len(sampler.segments)-1):
        if sampler.segments[band+1] != sampler.segments[band]+1: continue
        for centre in np.arange(lower+half+dq, upper-half-dq, spacing_m):
            item = dict(id=len(windows), bands=[band, band+1], q_center_m=float(centre))
            angles = (centre+(np.arange(height)-(height-1)/2)*dq)/radius
            ranges = []
            for k in item['bands']:
                lo, hi, _, supported = sampler.row_sources(k, angles)
                if not supported.all(): break
                axes = sampler.projection['x_axis_m'][np.r_[lo, hi]]
                ranges.append((float(axes.max()+usable[0]), float(axes.min()+usable[-1])))
            if len(ranges) != 2:
                item.update(status='unmeasurable', reason='window includes unsupported exposure rows')
            else:
                left = max(grid['target_x_m'][0]-halo_m, *(v[0] for v in ranges))
                right = min(grid['target_x_m'][1]+halo_m, *(v[1] for v in ranges))
                width = min(max_width, math.floor((right-left)/dx)-4)
                minimum = max(64, math.ceil(2*settings.max_shift_mm/(1000*min(dx, dq)))+20*settings.coarse_factor)
                if width < minimum:
                    item.update(status='unmeasurable', reason='overlap too narrow for configured search range')
                else:
                    item.update(status='planned', shape=[height, width],
                        x_first_m=float((left+right)/2-(width-1)*dx/2),
                        q_first_m=float(centre-half))
            windows.append(item)
    return windows


def sample_window(sampler, grid, window):
    height, width = window['shape']
    xs = window['x_first_m']+np.arange(width)*grid['dx_m']
    qs = window['q_first_m']+np.arange(height)*grid['dq_m']
    arrays = [sampler.sample(band, qs/grid['radius_m'], xs)[:2] for band in window['bands']]
    return arrays


def native_sources(sampler, band, x, q, radius):
    lo, hi, weight, supported = sampler.row_sources(band, q/radius)
    if not supported.all(): raise ValueError('match extends outside supported exposure rows')
    return dict(lower_sequence=sampler.projection['sequence'][lo],
        upper_sequence=sampler.projection['sequence'][hi], angular_weight=weight,
        lower_column=np.interp(x-sampler.projection['x_axis_m'][lo], sampler.offsets, sampler.columns),
        upper_column=np.interp(x-sampler.projection['x_axis_m'][hi], sampler.offsets, sampler.columns))


def pack_matches(sampler, grid, window, matches):
    table = np.zeros(len(matches['points_a']), MATCH)
    table['window'] = window['id']; table['band_a'], table['band_b'] = window['bands']
    for side, band in zip(('a', 'b'), window['bands']):
        points = matches['points_'+side]
        x = window['x_first_m']+points[:, 0]*grid['dx_m']
        q = window['q_first_m']+points[:, 1]*grid['dq_m']
        table['x_'+side+'_m'], table['q_'+side+'_m'] = x, q
        sources = native_sources(sampler, band, x, q, grid['radius_m'])
        for name, values in sources.items(): table[side+'_'+name] = values
    for name, key in [('ncc', 'ncc'), ('fb_px', 'fb_error'), ('residual_px', 'residual'),
                      ('holdout', 'holdout'), ('inlier', 'inlier')]: table[name] = matches[key]
    return table


def graph_components(count, windows):
    neighbours = [set() for _ in range(count)]
    for window in windows:
        if window['status'] != 'accepted': continue
        a, b = window['bands']; neighbours[a].add(b); neighbours[b].add(a)
    remaining = set(range(count)); components = []
    while remaining:
        pending = [min(remaining)]; component = []
        while pending:
            node = pending.pop()
            if node not in remaining: continue
            remaining.remove(node); component.append(node); pending.extend(neighbours[node])
        components.append(sorted(component))
    return components


def write_review(sampler, grid, windows, table, output):
    accepted = [w for w in windows if w['status'] == 'accepted']
    selected = np.unique(np.linspace(0, len(accepted)-1, min(3, len(accepted)), dtype=int))
    html = ['<!doctype html><meta charset="utf-8"><title>Stage D2 band matches</title>',
        '<style>body{font:16px sans-serif;background:#222;color:#eee;margin:24px}img{max-width:100%}a{color:#9cf}</style>',
        '<h1>D2: image matches with local affine consistency</h1>',
        '<p>Affine fits are relative image diagnostics, not measured body roll/pitch. No global optimization or seam blending has been applied. Physical DN uses fixed 0–255 display.</p>',
        '<p><a href="report.json">Summary</a> · <a href="windows.json">All accepted/rejected windows</a></p>']
    for index in selected:
        window = accepted[index]
        (a, ma), (b, mb) = sample_window(sampler, grid, window)
        height, width = a.shape
        raw_a, raw_b = Image.fromarray(display(a)).convert('RGB'), Image.fromarray(display(b)).convert('RGB')
        # Alignment is a review-only preview; native caches and mosaic stay unchanged.
        affine = np.asarray(window['affine_a_to_b_px'], np.float32)
        aligned = cv2.warpAffine(b, affine, (width, height), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=float('nan'))
        mask = cv2.warpAffine(mb.astype(np.float32), affine, (width, height),
                             flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP, borderValue=0)
        aligned[(mask < 1-1e-6) | ~ma] = np.nan
        canvas = Image.new('RGB', (width*2, height*2+48), '#222')
        canvas.paste(raw_a, (0, 24)); canvas.paste(raw_b, (width, 24))
        canvas.paste(raw_a, (0, height+48))
        canvas.paste(Image.fromarray(display(aligned)).convert('RGB'), (width, height+48))
        draw = ImageDraw.Draw(canvas)
        draw.text((5, 5), 'A original / B original', fill='white')
        draw.text((5, height+29), 'A original / B locally aligned (review only)', fill='white')
        points = table[(table['window'] == window['id']) & table['inlier'].astype(bool)]
        for point in points[::max(1, len(points)//24)]:
            pa = ((point['x_a_m']-window['x_first_m'])/grid['dx_m'],
                  (point['q_a_m']-window['q_first_m'])/grid['dq_m']+24)
            pb = ((point['x_b_m']-window['x_first_m'])/grid['dx_m']+width,
                  (point['q_b_m']-window['q_first_m'])/grid['dq_m']+24)
            color = '#ffb347' if point['holdout'] else '#6df39c'
            draw.line([pa, pb], fill=color, width=1)
            for x, y in (pa, pb): draw.ellipse((x-2, y-2, x+2, y+2), outline=color)
        name = f'window_{window["id"]:03}.png'; canvas.save(output/name)
        html.extend([f'<h2>Bands {window["bands"]}, q={window["q_center_m"]:.3f} m</h2>',
            f'<p>Held-out P95 {window["holdout_p95_px"]:.3f} px; nominal displacement P95 {window["nominal_displacement_p95_px"]:.2f} px. Orange points were excluded from affine fitting.</p>',
            f'<a href="{name}"><img src="{name}"></a>'])
        sampler.native.release()
    (output/'review.html').write_text('\n'.join(html))


def run(root, output, spacing_m=.4, height=512, max_width=1024, settings=MatchSettings(), halo_m=.25, raw_root=None):
    started = time.monotonic(); sampler, upstream, inputs = verified_bands(root, raw_root)
    input_s = time.monotonic()-started; grid = upstream['grid']
    output = Path(output).resolve()
    if output.is_relative_to(Path(root).resolve()): raise ValueError('D2 output must be separate from D1 inputs')
    windows = plan_windows(sampler, grid, spacing_m, height, max_width, settings, halo_m)
    output.mkdir(parents=True, exist_ok=False)
    previous_threads = cv2.getNumThreads(); cv2.setNumThreads(1)
    tables = []; sampling_s = matching_s = 0.
    try:
        for window in windows:
            if window['status'] != 'planned': continue
            t = time.monotonic(); (a, ma), (b, mb) = sample_window(sampler, grid, window)
            sampling_s += time.monotonic()-t; t = time.monotonic()
            diagnostic, matches = match_window(a, b, ma, mb, min(grid['dx_m'], grid['dq_m']), settings)
            matching_s += time.monotonic()-t; window.update(diagnostic)
            if window['status'] == 'accepted': tables.append(pack_matches(sampler, grid, window, matches))
            sampler.native.release()
    finally:
        cv2.setNumThreads(previous_threads)
    table = np.concatenate(tables) if tables else np.empty(0, MATCH)
    np.save(output/'matches.npy', table)
    (output/'windows.json').write_text(json.dumps(windows, indent=2)+'\n')
    accepted = [w for w in windows if w['status'] == 'accepted']
    holdout = table[table['holdout'].astype(bool)]
    components = graph_components(len(sampler.segments), windows)
    report = dict(schema='ssb.band_matches.v1', stage='D2',
        status=('unmeasurable' if not accepted else 'partial' if len(components) > 1 else 'complete'),
        upstream_grid=grid, settings=asdict(settings), optical_signature=upstream['optical_signature'],
        source_observation_hashes=upstream['source_observation_hashes'],
        matching_halo_m=halo_m,
        matching_x_domain_m=[grid['target_x_m'][0]-halo_m, grid['target_x_m'][1]+halo_m],
        windows=dict(planned=len(windows), accepted=len(accepted), unmeasurable=len(windows)-len(accepted),
                     eligible=sum('shape' in w for w in windows),
                     rejection_reasons=dict(Counter(w['reason'] for w in windows if w['status'] != 'accepted'))),
        matches=dict(total=len(table), inliers=int(table['inlier'].sum()), heldout=len(holdout),
            holdout_p95_px=float(np.percentile(holdout['residual_px'], 95)) if len(holdout) else None,
            nominal_displacement_p95_px=float(np.percentile(np.hypot(table['x_b_m']-table['x_a_m'],
                table['q_b_m']-table['q_a_m'])/grid['requested_pitch_m'], 95)) if len(table) else None),
        connected_components=components,
        coordinate_convention='A nominal (x,q) -> B nominal (x,q); q=radius*theta; native sources use measured D1 geometry',
        limitations=['local affine is a consistency test, not recovered body roll/pitch',
            'holdout residual measures internal image consistency, not absolute reconstruction accuracy',
            'scan-periodic common deformations and absolute scale remain unobservable without independent priors',
            'no global optimization, blending, IMU or noiseless-baseline robustness claim'],
        performance=dict(input_check_s=input_s, sampling_s=sampling_s, matching_s=matching_s))
    review_start = time.monotonic(); write_review(sampler, grid, windows, table, output)
    report['performance'].update(review_s=time.monotonic()-review_start,
        wall_s=time.monotonic()-started, peak_rss_bytes=peak_rss_bytes())
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    provenance = stage_record('band_matching', inputs, sorted(output.iterdir()),
        dict(spacing_m=spacing_m, height=height, max_width=max_width, settings=asdict(settings),
             halo_m=halo_m, opencv=cv2.__version__))
    provenance['performance'] = dict(wall_s=time.monotonic()-started, peak_rss_bytes=peak_rss_bytes())
    (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--unroll', required=True); parser.add_argument('--output', required=True)
    parser.add_argument('--spacing-m', type=float, default=.4)
    parser.add_argument('--height', type=int, default=512); parser.add_argument('--max-width', type=int, default=1024)
    parser.add_argument('--max-shift-mm', type=float, default=40.)
    parser.add_argument('--raw', help='relocated public raw directory of the source capture (hash-checked)')
    parser.add_argument('--halo-m', type=float, default=.25,
                        help='use already recorded native columns around ROI edges; no extra capture')
    args = parser.parse_args()
    report = run(args.unroll, args.output, args.spacing_m, args.height, args.max_width,
                 MatchSettings(max_shift_mm=args.max_shift_mm), args.halo_m, args.raw)
    print(json.dumps({k:report[k] for k in ('status', 'windows', 'matches', 'performance')}))
    if report['status'] == 'unmeasurable': raise SystemExit(2)


if __name__ == '__main__': main()
