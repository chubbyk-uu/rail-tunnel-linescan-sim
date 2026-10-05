"""Public-image-only review of raw helix, nominal unroll and optimized seams."""
import argparse
from contextlib import contextmanager
import html
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
from PIL import Image

from .band_matching import MatchSettings
from .global_resample import load_global, run as geometry_review, sample_corrected
from .initial_unroll import display, grid_axes
from .match_bands import WINDOW_BUDGET, plan_windows, verified_bands
from .optimize_bands import public_robot
from .provenance import stage_record
from .session import read_json
from .stage_b_scene import peak_rss_bytes
from .unroll_cuda import CudaRaster
from .review_grid import display_grid, overview_stride


def raw_overview(sampler, angles, xs, nominal_offsets):
    """Nearest original uint8 pixels, freezing each band's first available x.

    No flat/distortion correction and no compensation for within-band travel.
    The nominal column scale is only a display placement convention.
    """
    if not hasattr(sampler.native, 'raw'):
        raise ValueError('original uint8 acquisition rows required')
    if len(angles)*len(xs) > 1 << 20:
        raise ValueError('raw preview exceeds one megapixel')
    offsets = np.asarray(nominal_offsets, float)
    if offsets.shape != sampler.columns.shape or not np.all(np.diff(offsets) > 0):
        raise ValueError('strictly increasing nominal sensor offsets required')
    if len(angles)*len(offsets) > 8 << 20:
        raise ValueError('raw row read exceeds 8 MiB preview budget')
    image = np.zeros((len(angles), len(xs)), np.uint8)
    best = np.full(image.shape, -np.inf)
    valid = np.zeros(image.shape, bool)
    records = []
    for band, (first, end) in enumerate(sampler.bounds):
        reference = float(sampler.projection['x_axis_m'][first])
        lo, hi, weight, supported = sampler.row_sources(band, angles)
        rows = np.where(weight <= .5, lo, hi)
        delta = xs-reference
        inside = (delta >= offsets[0]) & (delta <= offsets[-1])
        columns = np.rint(np.interp(delta, offsets, sampler.columns)).astype(int)
        score = np.minimum(columns, len(offsets)-1-columns)
        use = supported[:, None] & inside[None, :] & (score[None, :] > best)
        if np.any(use):
            pixels = sampler.native.raw(rows)[:, columns]
            image[use] = pixels[use]
            best[use] = np.broadcast_to(score, image.shape)[use]
            valid |= use
        records.append(dict(band=band, first_sequence=int(sampler.projection['sequence'][first]),
                            reference_x_m=reference,
                            observed_travel_m=float(sampler.projection['x_axis_m'][end-1]-reference)))
        sampler.native.release()
    return image, valid, records


def line_candidates(image, valid):
    """Locate elongated dark structures, not defect identities or ground truth.

    DN is never altered in the published crops. Highpass and morphology only
    select locations; round pores and invalid borders are excluded.
    """
    if image.ndim != 2 or image.shape != valid.shape or image.size > 1 << 20:
        raise ValueError('bounded image and matching validity mask required')
    if not valid.any():
        return []
    if not np.isfinite(image[valid]).all():
        raise ValueError('valid image samples must be finite')
    values = np.where(valid, image, np.median(image[valid])).astype(np.float32)
    interior = cv2.erode(valid.astype(np.uint8), np.ones((17, 17), np.uint8)).astype(bool)
    # The wider background estimate fills broad grooves, rather than selecting
    # their two narrow edges and misclassifying those edges as separate cracks.
    dark = np.maximum(cv2.GaussianBlur(values, (0, 0), 5.)-values,
                      cv2.GaussianBlur(values, (0, 0), 25.)-values)
    mask = ((dark > 18.) & interior).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    candidates = []
    height, width = image.shape
    for identifier in range(1, count):
        x, y, w, h, area = map(int, stats[identifier])
        # Structures must extend axially so a hard A/B splice can cross them.
        if w < 120 or w < 2*h or area < 120 or x+w <= width//4 or x > 3*width//4:
            continue
        component = labels[y:y+h, x:x+w] == identifier
        thickness = float(np.median(component.sum(axis=0)[w//5:4*w//5]))
        # A slightly broadened crack is still a fine structure. Broad grooves
        # must span at least ten output pixels; do not label a 5 px crack a joint.
        kind = 'thin' if thickness < 10 else 'wide'
        centre_x = int(np.clip(x+w//2, width//4, 3*width//4))
        ys = np.flatnonzero(component[:, centre_x-x])
        if not len(ys):
            continue
        centre_y = int(y+np.median(ys))
        if not 24 <= centre_y < height-24:
            continue
        contrast = float(np.median(dark[y:y+h, x:x+w][component]))
        candidates.append(dict(kind=kind, centre_px=[centre_x, centre_y],
                               bbox_px=[x, y, w, h], thickness_px=thickness,
                               score=float(w*contrast)))
    return sorted(candidates, key=lambda item: -item['score'])


def search_spacing(grid, adjacent_pairs):
    """Coarsen image-only feature selection, never production D2 matching."""
    spacing = .09
    if adjacent_pairs:
        per_pair = WINDOW_BUDGET//adjacent_pairs
        if per_pair < 1:
            raise ValueError('too many bands for bounded feature selection')
        span = np.diff(grid['theta_rad'])[0]*grid['radius_m']-511*grid['dq_m']
        spacing = max(spacing, float(np.nextafter(span/per_pair, np.inf)))
    return spacing


def find_features(sampler, grid, raster):
    settings = MatchSettings(max_shift_mm=1.)
    spacing = search_spacing(grid, max(0, len(sampler.segments)-1))
    windows = plan_windows(sampler, grid, spacing, 512, 1024, settings)
    candidates = []
    tested = 0
    for window in windows:
        if window['status'] != 'planned':
            continue
        height, width = window['shape']
        qs = window['q_first_m']+np.arange(height)*grid['dq_m']
        xs = window['x_first_m']+np.arange(width)*grid['dx_m']
        pixels, count, _ = raster.tile(qs/grid['radius_m'], xs, [window['bands'][0]])
        tested += 1
        for item in line_candidates(pixels, count > 0)[:6]:
            cx, cy = item['centre_px']
            item.update(window=window, x_m=float(xs[cx]), q_m=float(qs[cy]))
            candidates.append(item)
        sampler.native.release()
    selected = []
    for kind in ('wide', 'thin'):
        for item in sorted((c for c in candidates if c['kind'] == kind), key=lambda c: -c['score']):
            if any(abs(item['x_m']-c['x_m']) < .05 and abs(item['q_m']-c['q_m']) < .08
                   for c in selected):
                continue
            selected.append(item)
            if sum(c['kind'] == kind for c in selected) >= 2:
                break
    if not selected:
        raise ValueError('no supported elongated image structures found')
    return selected, dict(tested_windows=tested, candidate_count=len(candidates),
                          spacing_m=spacing, search_shape=[512, 1024],
                          wide_structure_min_thickness_px=10, backend=raster.describe())


def raw_band_crop(sampler, band, angles, xs, nominal_offsets, anchor_angle):
    """Original pixels with ONE encoder placement per band for the entire crop.

    The anchor only centres the same feature in a local comparison. It cannot
    straighten the helix: every row uses the identical sensor-column lookup.
    No flat correction, calibrated distortion map or estimated trajectory.
    """
    lo, hi, weight, supported = sampler.row_sources(band, np.asarray(angles))
    rows = np.where(weight <= .5, lo, hi)
    a, b, w, _ = sampler.row_sources(band, np.array([anchor_angle]))
    anchor = a[0] if w[0] <= .5 else b[0]
    reference = float(sampler.projection['x_axis_m'][anchor])
    delta = np.asarray(xs)-reference
    columns = np.rint(np.interp(delta, nominal_offsets, sampler.columns)).astype(int)
    inside = (delta >= nominal_offsets[0]) & (delta <= nominal_offsets[-1])
    values = sampler.native.raw(rows)[:, columns].astype(np.float32)
    values[~(supported[:, None] & inside[None, :])] = np.nan
    return values, dict(reference_x_m=reference,
                        anchor_sequence=int(sampler.projection['sequence'][anchor]),
                        placement='one fixed encoder anchor per band; no within-band motion compensation')


def feature_crop(sampler, model, coefficients, grid, item, nominal_offsets):
    window = item['window']
    height, width = window['shape']
    # Stay within the searched native overlap. Centre the hard splice on the feature.
    cx, _ = item['centre_px']
    half = min(256, width//4)
    begin = int(np.clip(cx-half, 0, width-2*half))
    xs = window['x_first_m']+np.arange(begin, begin+2*half)*grid['dx_m']
    qs = window['q_first_m']+np.arange(height)*grid['dq_m']
    # The detector works in nominal coordinates. Locate that same observation
    # in the fitted coordinate system instead of cropping unrelated content at
    # the old coordinates. This only translates the display rectangle.
    band = window['bands'][0]
    nominal_centre = np.array([item['x_m'], item['q_m']])
    centre, supported = model.forward(band, nominal_centre[None, 0],
                                      nominal_centre[None, 1], coefficients)
    if getattr(model, 'relief', None) is not None:
        for _ in range(10):
            depth = model.relief.depth(centre[:, 0], centre[:, 1])
            next_centre, supported = model.forward(band, nominal_centre[None, 0],
                nominal_centre[None, 1], coefficients, radial_depth=depth)
            change = np.max(abs(next_centre-centre))
            centre = next_centre
            if change < 1e-9:
                break
    if not np.asarray(supported).all() or not np.isfinite(centre).all():
        raise ValueError('image-selected feature lacks fitted display support')
    shift = centre[0]-nominal_centre
    optimized_xs, optimized_qs = xs+shift[0], qs+shift[1]
    raw = np.full((height, len(xs)), np.nan, np.float32)
    anchors = []
    for band, columns in zip(window['bands'], (slice(0, half), slice(half, None))):
        values, anchor = raw_band_crop(sampler, band, qs/model.radius, xs[columns],
                                      nominal_offsets, qs[len(qs)//2]/model.radius)
        raw[:, columns] = values
        anchors.append(dict(band=band, **anchor))
        sampler.native.release()
    pairs = [raw]
    for corrected in (False, True):
        image = np.full((height, len(xs)), np.nan, np.float32)
        for start in range(0, height, 16):
            rows = slice(start, start+16)
            for band, columns in zip(window['bands'], (slice(0, half), slice(half, None))):
                if corrected:
                    result = sample_corrected(model, coefficients, band, optimized_qs[rows], optimized_xs[columns])[0]
                else:
                    result = sampler.sample(band, qs[rows]/model.radius, xs[columns])[0]
                image[rows, columns] = result
            sampler.native.release()
        pairs.append(image)
    return pairs, dict(shape=[height, len(xs)], seam_column=half,
                       bands=window['bands'], x_first_m=float(xs[0]), q_first_m=float(qs[0]),
                       optimized_x_first_m=float(optimized_xs[0]), optimized_q_first_m=float(optimized_qs[0]),
                       display_translation_m=shift.tolist(),
                       display_alignment='same public native observation, image-fitted trajectory; independent crop origins only',
                       grid_pitch_m=[grid['dx_m'], grid['dq_m']],
                       presentation_order=['raw', 'nominal', 'optimized'], raw_anchors=anchors,
                       invalid_pixels=[int(np.count_nonzero(~np.isfinite(p))) for p in pairs])


@contextmanager
def feature_raster(sampler):
    """Keep CUDA ownership scoped to search, including exceptional exits."""
    raster = CudaRaster(sampler)
    try:
        yield raster
    finally:
        raster.close()


def run(unroll, trajectory, observable, output, raw_root=None, target_x=None):
    started = time.monotonic()
    output = Path(output).resolve()
    if any(output.is_relative_to(Path(p).resolve()) for p in (unroll, trajectory)):
        raise ValueError('review output must be separate from inputs')
    sampler, upstream, inputs = verified_bands(unroll, raw_root)
    try:
        model, coefficients, optimized, trajectory_inputs = load_global(trajectory, sampler, upstream, unroll)
        height, observable = public_robot(observable, upstream['source_observation_hashes'].get(
            'config/observable_config.json'), model.radius)
        if height != model.height:
            raise ValueError('D3 support height differs from verified public geometry')
        camera = read_json(observable)['camera']
        width = int(camera['width'])
        pitch = float(camera['pixel_pitch_m'])*model.radius/float(camera['focal_length_m'])
        if width != len(sampler.columns) or not math.isfinite(pitch) or pitch <= 0:
            raise ValueError('nominal camera geometry differs from native rows')
        nominal_offsets = (np.arange(width)-(width-1)/2)*pitch
        output.mkdir(parents=True, exist_ok=False)
        grid = display_grid(upstream['grid'], target_x)
        angles, xs = grid_axes(grid)
        stride = overview_stride(grid['shape'])
        raw, valid, references = raw_overview(sampler, angles[::stride], xs[::stride], nominal_offsets)
        Image.fromarray(raw).save(output/'raw_helix.png')
        # Display a complete original band horizontally: exposure sequence along
        # x, original sensor columns along y. Only transpose and integer decimate.
        complete = [k for k, r in enumerate(references) if r['observed_travel_m'] > .35]
        if not complete:
            raise ValueError('no sufficiently complete raw band for the helix illustration')
        band = complete[len(complete)//2]
        first, end = sampler.bounds[band]
        raw_stride = max(1, math.ceil(max(end-first, width)/1800))
        row_ids = np.arange(first, end, raw_stride)
        raw_band = sampler.native.raw(row_ids)[:, ::raw_stride].T
        Image.fromarray(raw_band).save(output/'raw_band.png')
        with feature_raster(sampler) as raster:
            features, search = find_features(sampler, grid, raster)
        crops = []
        for identifier, item in enumerate(features):
            pairs, record = feature_crop(sampler, model, coefficients, grid, item, nominal_offsets)
            record.update(candidate=item, id=identifier)
            for name, pixels in zip(('raw', 'nominal', 'optimized'), pairs):
                Image.fromarray(display(pixels)).save(output/f'feature_{identifier}_{name}.png')
            crops.append(record)
        # Keep the previous error-based diagnostics and metrics beside feature-selected crops.
        geometry = geometry_review(unroll, trajectory, output/'geometry', raw_root, target_x)
        scores = optimized['image_consistency']
        gate = geometry['image_consistency_gate']
        gate_label = '通过' if gate['status'] == 'pass' else '未通过'
        page = ['<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>原始螺旋条带与接缝对比</title>',
            '<style>body{margin:24px;background:#202124;color:#eee;font:16px sans-serif;line-height:1.6}a{color:#9cf}.overview{display:flex;gap:16px;align-items:flex-start}.overview figure{margin:0;flex:1;min-width:0}.overview img{width:100%;height:auto}.pair{display:flex;gap:16px;flex-wrap:wrap}.pair figure{margin:0}.pair img{width:512px;max-width:100%;height:auto}.notice{color:#ffcb80}figcaption{margin:8px 0}img{image-rendering:auto}button{padding:8px;cursor:pointer}</style>',
            '<h1>三种状态：原始螺旋条带 → 名义展开 → 全局优化</h1>',
            '<p><a href="#raw-band">直接看原始倾斜条带</a> · <a href="#features">直接看板缝与裂缝接缝</a> · <a href="geometry/review.html">误差窗口</a></p>',
            '<p>原来的左图属于第二种：已经根据每行编码器位置补偿前进位移，所以板缝大部分不再倾斜。第一张保留圈内前进造成的真实斜向变化，没有人为旋转图像。</p>',
            '<p>原始图按每圈首个可用曝光的轴向位置摆放，同一圈各行固定在该位置；像素直接取原始 Mono8，未做畸变、平场或位移补偿。它只是条带摆放示意，不能拿来作为 D3 精度基线。后两张在同一网格上完成采后光学校正；D3 只与名义展开比较。</p>',
            f'<p class="notice">名义 → 优化的留出 P95：{scores["heldout_before"]["norm_px"]["p95"]:.3f} → {scores["heldout_after"]["norm_px"]["p95"]:.3f} px；图像一致性检查{gate_label}（P95 ≤ {gate["threshold_p95_px"]:g} px）。本页不读取独立真值评价，不能用图像残差替代真实网格接缝、漂移和覆盖验收。</p>',
            '<p>x 轴向水平，q 周向竖直。概览缩小显示，点击查看文件；下方局部图每个输出像素约 0.2 mm，默认按 100% 像素显示。固定 DN 0–255，无锐化、自动对比度或接缝融合。</p><div class="overview">']
        for file, label in (('raw_helix.png', '① 未补偿的原始条带摆放'),
                            ('geometry/optimized.png', '③ 图像匹配与全局轨迹优化')):
            page.append(f'<figure><figcaption>{label}</figcaption><a href="{file}"><img src="{file}"></a></figure>')
        page.append('</div><details><summary>中间状态：编码器名义展开（精度比较基线）</summary><img style="width:100%" src="geometry/nominal.png"></details><h2 id="raw-band">单圈原始图：圈内前进造成的倾斜</h2>')
        page.append(f'<p>条带 {band}，原始 {width} 列 × {end-first} 曝光行；这一采集段编码器前进 {references[band]["observed_travel_m"]:.3f} m。横轴为曝光顺序，纵轴为传感器列；只转置并每隔 {raw_stride} 个像素抽样，不做图像变形。竖直板缝在此视图中呈斜线。</p><a href="raw_band.png"><img style="width:100%;height:auto" src="raw_band.png"></a>')
        page.append('<h2 id="features">板缝 / 裂缝形态附近的硬接缝</h2><p>位置仅从公开图像的细长暗结构选取，未读取裂缝编号、场景坐标或真值。局部右图按拟合轨迹定位同一原始观测，仅平移裁切框，不修改图像、旋转或拉直左图。两侧局部坐标原点不同，不能用裁切后的中心位置评价几何精度。形态标签不是独立缺陷鉴定；这些图用于看连续性，不另报挑选区域的验收分数。接缝在每张图正中，左右分别取相邻圈。</p>')
        for crop in crops:
            identifier = crop['id']
            item = crop['candidate']
            label = '宽线结构：板缝候选' if item['kind'] == 'wide' else '细线结构：裂缝候选'
            page.append(f'<h3>{html.escape(label)} {identifier+1} · 条带 {crop["bands"]}</h3><div class="pair">')
            for name, title in (('raw', '未补偿螺旋的原始条带'), ('optimized', '全局优化')):
                file = f'feature_{identifier}_{name}.png'
                page.append(f'<figure><figcaption>{title}</figcaption><a href="{file}"><img src="{file}"></a></figure>')
            page.append(f'</div><details><summary>中间状态：名义展开</summary><img src="feature_{identifier}_nominal.png"></details>')
        page.append('<p><a href="geometry/review.html">保留的最大原始误差 / 最大剩余误差窗口与完整对比</a> · <a href="report.json">选择规则与资源报告</a> · <a href="provenance.json">输入输出哈希</a></p></html>')
        (output/'review.html').write_text('\n'.join(page), encoding='utf-8')
        report = dict(schema='ssb.feature_review.v2', grid=grid, preview_stride=stride,
            declared_target_x_m=upstream['grid']['target_x_m'], diagnostic_roi=grid != upstream['grid'],
            raw=dict(selection='nearest original row and column; fixed first available x per band',
                     optical_correction=False, within_band_travel_compensation=False,
                     nominal_column_pitch_m=pitch, invalid_pixels=int(np.count_nonzero(~valid)),
                     band_references=references,
                     single_band=dict(band=band, original_shape=[end-first, width],
                                      selection='middle sufficiently complete band',
                                      transpose=True, integer_stride=raw_stride)), search=search, crops=crops,
            image_consistency_gate=geometry['image_consistency_gate'],
            performance=dict(wall_s=time.monotonic()-started, peak_rss_bytes=peak_rss_bytes()))
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        files = sorted(p for p in output.rglob('*') if p.is_file())
        (output/'provenance.json').write_text(json.dumps(stage_record('feature_review',
            inputs+trajectory_inputs+[observable], files,
            dict(raw_correction=False, source_selection='public image only', fusion=False, target_x=target_x)), indent=2)+'\n')
        return report
    finally:
        if hasattr(sampler.native, 'close'):
            sampler.native.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--unroll', required=True)
    parser.add_argument('--trajectory', required=True)
    parser.add_argument('--observable', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--raw')
    parser.add_argument('--target-x',type=float,nargs=2,help='display subset only; retains the complete fitted trajectory and original acceptance target')
    args = parser.parse_args()
    print(json.dumps(run(args.unroll, args.trajectory, args.observable, args.output, args.raw, args.target_x)))


if __name__ == '__main__':
    main()
