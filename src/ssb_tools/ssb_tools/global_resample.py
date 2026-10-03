"""One native-image resampling through a verified D3 trajectory; no image warps/fusion."""
import argparse
import html
import json
import math
from pathlib import Path
import time

import numpy as np
from PIL import Image

from .global_geometry import GeometrySettings, Trajectory
from .initial_unroll import cpu_tile, display, grid_axes
from .match_bands import verified_bands
from .provenance import stage_record
from .public_capture import confined_file
from .session import read_json, sha256_file
from .stage_b_scene import peak_rss_bytes


def load_global(root, sampler, upstream, d1_root):
    root = Path(root).resolve()
    paths = [confined_file(root, name) for name in ('provenance.json', 'report.json', 'trajectory.json', 'windows.json')]
    provenance = read_json(paths[0])
    if provenance.get('stage') != 'global_optimization':
        raise ValueError('verified D3 trajectory required')
    for path in paths[1:]:
        hashes = [v for k, v in provenance['outputs'].items() if Path(k).name == path.name]
        if len(hashes) != 1 or hashes[0] != sha256_file(path):
            raise ValueError('D3 product hash mismatch: '+path.name)
    for name in ('projection.npy', 'mapping.npz', 'bands.json'):
        hashes = [v for k, v in provenance['inputs'].items() if Path(k).name == name]
        if len(hashes) != 1 or hashes[0] != sha256_file(Path(d1_root)/name):
            raise ValueError('trajectory belongs to different D1 geometry')
    report, record = read_json(paths[1]), read_json(paths[2])
    if (report.get('schema') != 'ssb.global_optimization.v1' or report.get('grid') != upstream['grid'] or
            report.get('optical_signature') != upstream['optical_signature'] or
            report.get('source_observation_hashes') != upstream['source_observation_hashes']):
        raise ValueError('D3 public observation identity mismatch')
    if (record.get('schema') != 'ssb.global_trajectory.v1' or record.get('degree') != 3 or
            record.get('radius_m') != upstream['grid']['radius_m'] or
            record.get('fields') != ['carriage_dx_m', 'scan_phase_dq_m', 'roll_rad', 'pitch_rad']):
        raise ValueError('unsupported global trajectory geometry')
    model = Trajectory(sampler, record['radius_m'], record['support_height_m'],
                       GeometrySettings(**report['settings']), record['knots'])
    coefficients = np.asarray(record['coefficients'], float)/model.scale
    if coefficients.shape != (model.size,) or not np.isfinite(coefficients).all() or model.sizes != record['sizes']:
        raise ValueError('invalid global trajectory coefficients')
    for field, knot in enumerate(model.knots):
        if (not np.isfinite(knot).all() or np.any(np.diff(knot) < 0) or
                knot[3] != model.domain[0] or knot[-4] != model.domain[1]):
            raise ValueError('global trajectory knot domain differs from source progress')
        bound = 30. if field < 2 else 10.
        if np.any(abs(coefficients[model.starts[field]:model.starts[field+1]]) > bound+1e-7):
            raise ValueError('global trajectory exceeds declared physical bounds')
    return model, coefficients, report, paths


def inverse_points(model, coefficients, band, x, q):
    x, q = np.broadcast_arrays(np.asarray(x, float), np.asarray(q, float))
    if x.size > 1 << 20 or not np.isfinite(x).all() or not np.isfinite(q).all():
        raise ValueError('nonfinite or excessive inverse mapping tile')
    target = np.stack((x, q), axis=-1)
    nominal = target.copy()
    for _ in range(10):
        projected, _ = model.forward(band, nominal[..., 0], nominal[..., 1], coefficients)
        error = projected-target
        nominal -= error
        if np.max(abs(error), initial=0.) < 1e-9:
            break
    projected, supported = model.forward(band, nominal[..., 0], nominal[..., 1], coefficients)
    valid = supported & (np.max(abs(projected-target), axis=-1) < 1e-8)
    return nominal[..., 0], nominal[..., 1], valid


def native_points(sampler, band, x, q, radius):
    """Pointwise equivalent of BandSampler.sample; at most 2048 native rows per gather."""
    x, q = np.broadcast_arrays(x, q)
    lo, hi, angular, supported = sampler.row_sources(band, q.ravel()/radius)
    values = np.full(x.size, np.nan, np.float32)
    valid = np.zeros(x.size, bool); score = np.full(x.size, -np.inf, np.float32)
    columns = []
    along_values = []; along_valid = []
    for ids in (lo, hi):
        delta = x.ravel()-sampler.projection['x_axis_m'][ids]
        u = np.interp(delta, sampler.offsets, sampler.columns)
        left = np.floor(u).astype(np.int32); right = np.minimum(left+1, len(sampler.offsets)-1)
        weight = (u-left).astype(np.float32)
        corrected = np.interp(delta, sampler.output_offsets, sampler.columns)
        c0 = np.floor(corrected).astype(np.int32); c1 = np.minimum(c0+1, len(sampler.offsets)-1)
        v0 = np.empty(x.size, np.float32); v1 = np.empty(x.size, np.float32)
        for start in range(0, x.size, 2048):
            sl = slice(start, start+2048)
            first, second = sampler.native.gather(ids[sl], left[sl, None], right[sl, None])
            v0[sl], v1[sl] = first.ravel(), second.ravel()
        ok = ((delta >= sampler.offsets[0]) & (delta <= sampler.offsets[-1]) &
              (delta >= sampler.output_offsets[0]) & (delta <= sampler.output_offsets[-1]) &
              sampler.geometry_valid[c0] & sampler.geometry_valid[c1] & np.isfinite(v0) & np.isfinite(v1))
        along_values.append(v0*(1-weight)+v1*weight); along_valid.append(ok); columns.append(u)
    valid[:] = supported & along_valid[0] & along_valid[1]
    values[:] = along_values[0]*(1-angular)+along_values[1]*angular
    values[~valid] = np.nan
    score[:] = np.minimum(columns[0], len(sampler.offsets)-1-columns[0])
    sources = dict(lower_sequence=sampler.projection['sequence'][lo], upper_sequence=sampler.projection['sequence'][hi],
                   angular_weight=angular, lower_column=columns[0], upper_column=columns[1])
    return values.reshape(x.shape), valid.reshape(x.shape), score.reshape(x.shape), sources


def sample_corrected(model, coefficients, band, qs, xs):
    xx, qq = np.meshgrid(xs, qs)
    x, q, supported = inverse_points(model, coefficients, band, xx, qq)
    values, valid, score, _ = native_points(model.sampler, band, x, q, model.radius)
    valid &= supported; values[~valid] = np.nan
    return values, valid, score


def corrected_tile(model, coefficients, qs, xs):
    values = np.full((len(qs), len(xs)), np.nan, np.float32)
    best = np.full(values.shape, -np.inf, np.float32); count = np.zeros(values.shape, np.uint16)
    # Conservative margin from the declared 30 mm / 10 mrad parameter bounds.
    margin = .03+.04*(model.radius+model.height)
    for band, (lo, hi) in enumerate(model.sampler.band_x):
        begin = np.searchsorted(xs, lo-margin); end = np.searchsorted(xs, hi+margin, side='right')
        if end <= begin:
            continue
        result, valid, score = sample_corrected(model, coefficients, band, qs, xs[begin:end])
        sl = np.s_[:, begin:end]
        better = valid & (score > best[sl])
        values[sl][better] = result[better]; best[sl][better] = score[better]
        count[sl] += valid
    return values, count


def run(unroll, trajectory, output, raw_root=None):
    started = time.monotonic()
    output = Path(output).resolve()
    if any(output.is_relative_to(Path(p).resolve()) for p in (unroll, trajectory)):
        raise ValueError('comparison output must be separate from inputs')
    sampler, upstream, inputs = verified_bands(unroll, raw_root)
    try:
        model, coefficients, optimized, trajectory_inputs = load_global(trajectory, sampler, upstream, unroll)
        windows = read_json(Path(trajectory)/'windows.json')
        output.mkdir(parents=True, exist_ok=False)
        grid = upstream['grid']; angles, xs = grid_axes(grid)
        stride = max(1, math.ceil(max(grid['shape'])/1800))
        qs = angles[::stride]*model.radius; xs = xs[::stride]
        before = np.full((len(qs), len(xs)), np.nan, np.float32); after = before.copy()
        for start in range(0, len(qs), 16):
            sl = slice(start, start+16)
            before[sl] = cpu_tile(sampler, qs[sl]/model.radius, xs, range(len(sampler.segments)))[0]
            after[sl] = corrected_tile(model, coefficients, qs[sl], xs)[0]
            sampler.native.release()
        for name, pixels in (('nominal', before), ('optimized', after)):
            Image.fromarray(display(pixels)).save(output/(name+'.png'))
        # Show the largest original mismatch, the worst remaining window, and the middle window.
        chosen = sorted(set([max(windows, key=lambda w: w['heldout_before']['norm_px']['p95'])['window'],
                             max(windows, key=lambda w: w['heldout_after']['norm_px']['p95'])['window'],
                             windows[len(windows)//2]['window']]))
        # Obtain window records from the explicit public trajectory archive, not archived absolute paths.
        crops = []
        for identifier in chosen:
            window = next(w for w in windows if w['window'] == identifier)
            if 'source_window' not in window:
                raise ValueError('D3 window extents missing; regenerate optimization products')
            extent = window['source_window']
            qs_crop = extent['q_first_m']+np.arange(extent['shape'][0])*grid['dq_m']
            xs_crop = extent['x_first_m']+np.arange(extent['shape'][1])*grid['dx_m']
            left, right = extent['bands']; half = len(xs_crop)//2
            images = []
            for corrected in (False, True):
                image = np.full((len(qs_crop), len(xs_crop)), np.nan, np.float32)
                for start in range(0, len(qs_crop), 16):
                    sl = slice(start, start+16)
                    for band, columns in ((left, slice(0, half)), (right, slice(half, None))):
                        if corrected:
                            image[sl, columns] = sample_corrected(model, coefficients, band, qs_crop[sl], xs_crop[columns])[0]
                        else:
                            image[sl, columns] = sampler.sample(band, qs_crop[sl]/model.radius, xs_crop[columns])[0]
                    sampler.native.release()
                images.append(image)
            for name, pixels in zip(('nominal', 'optimized'), images):
                Image.fromarray(display(pixels)).save(output/f'window_{identifier}_{name}.png')
            crops.append(dict(window=identifier, shape=list(images[0].shape), seam_column=half,
                              bands=extent['bands'], grid_pitch_m=[grid['dx_m'], grid['dq_m']],
                              heldout_p95_before_px=window['heldout_before']['norm_px']['p95'],
                              heldout_p95_after_px=window['heldout_after']['norm_px']['p95']))
        scores = optimized['image_consistency']; gate = optimized['image_consistency_gate']
        page = ['<!doctype html><meta charset="utf-8"><title>D3 拼接对比</title>',
            '<style>body{background:#202124;color:#eee;font:16px sans-serif;margin:24px}a{color:#9cf}.pair{display:flex;gap:16px;flex-wrap:wrap}.pair img{max-width:100%;height:auto}.overview img{max-height:800px;width:auto}figure{margin:8px;max-width:48%}figcaption{margin:8px 0}.notice{color:#ffcb80}</style>',
            '<h1>3 米：编码器名义拼接 / 全局轨迹优化</h1>',
            '<p>两侧均采后校正，按同一 0.2 mm 网格从原始列取样。固定 DN 0–255；无锐化、自动对比度或接缝融合。x 水平，q 周向竖直。</p>',
            f'<p class="notice">留出点 P95：{scores["heldout_before"]["norm_px"]["p95"]:.3f} → {scores["heldout_after"]["norm_px"]["p95"]:.3f} px；0.5 px 图像一致性检查：{html.escape(gate["status"])}。这不是独立网格接缝验收。</p>',
            '<p>图像可点击查看原尺寸。概览是每隔固定网格间距的采样；下方局部图为 100% 输出像素。黑色可包含无效样本，不补洞。</p>',
            '<div class="pair overview"><figure><figcaption>编码器名义位置</figcaption><a href="nominal.png"><img src="nominal.png"></a></figure><figure><figcaption>全局轨迹优化</figcaption><a href="optimized.png"><img src="optimized.png"></a></figure></div>']
        for crop in crops:
            identifier = crop['window']
            page.append(f'<h2>窗口 {identifier}，条带 {crop["bands"]}，中线为硬接缝</h2>')
            page.append(f'<p>留出点 P95：{crop["heldout_p95_before_px"]:.3f} → {crop["heldout_p95_after_px"]:.3f} px</p><div class="pair">')
            for name, label in (('nominal', '名义位置'), ('optimized', '全局优化')):
                file = f'window_{identifier}_{name}.png'
                page.append(f'<figure><figcaption>{label}</figcaption><a href="{file}"><img src="{file}"></a></figure>')
            page.append('</div>')
        page.append('<p><a href="report.json">资源、范围与限制</a> · <a href="provenance.json">输入输出哈希</a></p>')
        (output/'review.html').write_text('\n'.join(page))
        report = dict(schema='ssb.global_comparison.v1', grid=grid, preview_stride=stride,
            preview_invalid=dict(nominal=int(np.count_nonzero(~np.isfinite(before))),
                                 optimized=int(np.count_nonzero(~np.isfinite(after)))),
            crops=crops, selection='largest nominal displacement, largest remaining holdout residual, middle window',
            image_consistency_gate=gate, limitations=['diagnostic preview, not a full-resolution final mosaic',
                'no blending; invalid samples stay masked; no independent mesh seam acceptance here'],
            performance=dict(wall_s=time.monotonic()-started, peak_rss_bytes=peak_rss_bytes()))
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        (output/'provenance.json').write_text(json.dumps(stage_record('global_resampling_review',
            inputs+trajectory_inputs, sorted(output.iterdir()), dict(preview_stride=stride, blend=False)), indent=2)+'\n')
        return report
    finally:
        if hasattr(sampler.native, 'close'):
            sampler.native.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--unroll', required=True); parser.add_argument('--trajectory', required=True)
    parser.add_argument('--output', required=True); parser.add_argument('--raw')
    args = parser.parse_args()
    print(json.dumps(run(args.unroll, args.trajectory, args.output, args.raw)))


if __name__ == '__main__':
    main()
