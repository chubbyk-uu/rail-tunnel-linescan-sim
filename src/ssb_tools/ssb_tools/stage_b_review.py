"""Bounded visual review of archived captures; evaluation truth selects examples only.

No stitching or acceptance-by-appearance is performed here. Saved PNGs are display
copies; corrected float DN, masks, and original sensor data remain unchanged.
"""
import argparse
import html
import json
import resource
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

from .optical_calibration import correct
from .optical_identity import check_calibration
from .session import Session, read_json, sha256_file


def display_rgb(data, valid=None, srgb=False):
    values = np.nan_to_num(np.asarray(data, np.float32), nan=0.) / 255.
    values = np.clip(values, 0., 1.)
    if srgb:
        values = np.where(values <= .0031308, values*12.92,
                          1.055*np.power(values, 1/2.4)-.055)
    rgb = np.repeat(np.uint8(np.rint(values*255))[..., None], 3, axis=-1)
    if valid is not None:
        rgb[~valid] = [255, 0, 255]
    return rgb


def check_corrected(session, root, calibration):
    """Hash every block, sample arithmetic, and inspect all masks in bounded chunks."""
    index = read_json(root/'index.json')
    raw_index = session.raw_index()
    signature = session.config()['camera']['optical_signature']
    if index['optical_signature'] != signature or calibration['optical_signature'] != signature:
        raise ValueError('review calibration/signature mismatch')
    if len(index['blocks']) != len(raw_index['blocks']):
        raise ValueError('corrected block count mismatch')
    worst, valid_count, pixels, saturated = 0., 0, 0, 0
    low, high = float('inf'), float('-inf')
    for source, block in zip(raw_index['blocks'], index['blocks']):
        if any(source[k] != block[k] for k in ('rows', 'first_sequence')):
            raise ValueError('corrected block row identity mismatch')
        raw_path = session.root/'raw'/source['file']
        if sha256_file(raw_path) != source['sha256'] or block['source_sha256'] != source['sha256']:
            raise ValueError('raw source hash mismatch')
        for name in ('image', 'mask'):
            if sha256_file(root/block[name]) != block[name+'_sha256']:
                raise ValueError('corrected payload hash mismatch')
        shape = (source['rows'], raw_index['width'])
        if raw_path.stat().st_size != np.prod(shape):
            raise ValueError('raw payload size mismatch')
        raw = np.memmap(raw_path, np.uint8, mode='r', shape=shape)
        image = np.load(root/block['image'], mmap_mode='r')
        mask = np.load(root/block['mask'], mmap_mode='r')
        if image.shape != shape or mask.shape != shape or image.dtype != np.float32 or mask.dtype != bool:
            raise ValueError('corrected shape/type mismatch')
        for first in sorted({0, max(0, shape[0]//2-4), max(0, shape[0]-8)}):
            rows = slice(first, first+8)
            expected, expected_mask = correct(raw[rows], calibration, signature)
            if not np.array_equal(mask[rows], expected_mask):
                raise ValueError('corrected sampled mask differs from calibration')
            error = np.abs(image[rows][expected_mask]-expected[expected_mask])
            worst = max(worst, float(error.max(initial=0)))
        for first in range(0, shape[0], 256):
            values, valid = image[first:first+256], mask[first:first+256]
            if not np.array_equal(np.isfinite(values), valid) or not np.isnan(values[~valid]).all():
                raise ValueError('corrected finite/NaN mask mismatch')
            if valid.any():
                low = min(low, float(values[valid].min()))
                high = max(high, float(values[valid].max()))
            valid_count += int(valid.sum()); pixels += valid.size
            saturated += int((raw[first:first+256] == 255).sum())
        del raw, image, mask, values, valid
    if not pixels or not valid_count or worst > 2e-5:
        raise ValueError('empty or inconsistent corrected capture')
    return dict(blocks=len(index['blocks']), rows=raw_index['rows'],
                sampled_recompute_max_error_dn=worst, valid_fraction=valid_count/pixels,
                raw_saturated_fraction=saturated/pixels, corrected_range_dn=[low, high])


def locate_patch(rows, truth, x, q, radius, fov, width, side=512):
    """Select a same-gate native crop using evaluation geometry, without resampling."""
    angle = (truth['theta']+np.pi) % (2*np.pi)-np.pi
    columns = (width-1)/2+(x-truth['x'])*width/fov
    candidate = (columns >= side/2) & (columns < width-side/2)
    distance = np.where(candidate, np.abs(radius*angle-q), np.inf)
    row = int(np.argmin(distance))
    if not np.isfinite(distance[row]) or distance[row] > .002:
        raise ValueError('target not covered by this capture')
    segment = rows['segment'][row]
    indices = np.flatnonzero(rows['segment'] == segment)
    if len(indices) < side:
        raise ValueError('gate too short for review crop')
    first = min(max(row-side//2, int(indices[0])), int(indices[-1])-side+1)
    left = int(np.clip(round(columns[row])-side//2, 0, width-side))
    return first, left, row


def read_corrected_crop(root, index, first, count, left, width):
    image = np.empty((count, width), np.float32)
    valid = np.empty((count, width), bool)
    filled = np.zeros(count, bool)
    for b in index['blocks']:
        lo, hi = max(first, b['first_sequence']), min(first+count, b['first_sequence']+b['rows'])
        if lo >= hi:
            continue
        target = slice(lo-first, hi-first)
        source = slice(lo-b['first_sequence'], hi-b['first_sequence'])
        image[target] = np.load(root/b['image'], mmap_mode='r')[source, left:left+width]
        valid[target] = np.load(root/b['mask'], mmap_mode='r')[source, left:left+width]
        filled[target] = True
    if not filled.all():
        raise ValueError('corrected crop contains missing rows')
    return image, valid


def save_pair(output, name, raw, corrected, valid):
    paths = {}
    for mode in ('linear', 'srgb'):
        paths[mode] = []
        for label, values, mask in [('raw', raw, None), ('corrected', corrected, valid)]:
            path = Path('images')/f'{name}_{label}_{mode}.png'
            Image.fromarray(display_rgb(values, mask, mode == 'srgb')).save(output/path)
            paths[mode].append(str(path))
    return paths


def prepare(session_path, demo, output):
    session = Session(session_path); demo = Path(demo).resolve(); output = Path(output).resolve()
    if output.exists():
        raise ValueError('refuse to overwrite review output')
    output.mkdir(parents=True); (output/'images').mkdir(); (output/'evaluation').mkdir()
    calibration_path = demo/'calibration.json'; calibration = read_json(calibration_path)
    check_calibration(demo/'capture.yaml', calibration_path)
    corrected_root = session.root/'processed/optical'
    index = read_json(corrected_root/'index.json')
    if index['calibration_sha256'] != sha256_file(calibration_path):
        raise ValueError('review calibration file differs from saved correction')
    verification = check_corrected(session, corrected_root, calibration)
    rows, truth = session.metadata('rows'), session.evaluation('row_truth')
    if len(rows) != len(truth) or not np.array_equal(rows['sequence'], np.arange(len(rows))) or not np.array_equal(rows['sequence'], truth['sequence']):
        raise ValueError('review row identity mismatch')
    config = session.config(); radius = session.truth()['tunnel']['radius_m']
    width, fov = index['width'], index['output_fov_m']
    assets = read_json(session.root/'evaluation/optical_assets.json')
    selected = [('background', '背景细气孔', 4., 1., None),
                ('ring_joint', '填充环缝', 3.6, 2.5, None)]
    for key, title in [('crack_000', '长裂缝片段'), ('crack_018', '较细的短裂缝'),
                       ('crack_056', '少量分叉裂缝')]:
        instance = next(i for i in assets['defects']['instances'] if i['id'] == key)
        path = np.asarray(instance['main_path_xq_m'])
        points = path[(path[:, 0] > 3.) & (path[:, 0] < 5.8)]
        point = points[len(points)//2]
        if instance['has_minor_branches']:
            # Centre on a junction, rather than a main-path midpoint hiding the fork.
            point = np.asarray(instance['paths_xq_m'][-1][0])
        selected.append((key, title, *point, instance['body_width_mm']))
    examples = []
    for name, title, x, q, nominal_width in selected:
        first, left, centre = locate_patch(rows, truth, x, q, radius, fov, width)
        raw = session.raw_rows(np.arange(first, first+512))[:, left:left+512]
        corrected, valid = read_corrected_crop(corrected_root, index, first, 512, left, 512)
        paths = save_pair(output, name, raw, corrected, valid)
        examples.append(dict(name=name, title=title, source='archived_encoder_capture',
            target_xq_m=[float(x), float(q)], nominal_body_width_mm=nominal_width,
            first_sequence=first, left_column=left, centre_sequence=centre,
            rows=512, columns=512, images=paths))
    report = dict(schema='ssb.stage_b_visual_review.v1',
        scope='Evaluation-only native-pixel review; not stitching or a completed human acceptance.',
        session=str(session.root.resolve()), verification=verification,
        calibration_validation=calibration['validation'], examples=examples,
        frozen_inputs={str(p.resolve()):sha256_file(p) for p in
                       [demo/'capture.yaml', demo/'scene.json', demo/'world/world.sdf', calibration_path,
                        session.root/'raw/index.json', corrected_root/'index.json',
                        session.root/'evaluation/optical_assets.json',
                        session.root/'config/provenance.json']},
        optical_signature=config['camera']['optical_signature'],
        limitations=['Crop axes are sensor columns / acquisition rows, not an unrolled square-pixel map.',
                     'Nominal crack body widths are evaluation truth, not measured image widths.',
                     'Only a fragment of the long crack is covered; full-length continuity awaits 20 m acquisition.',
                     'Network and endpoint width examples need supplementary optical probes.',
                     'No MTF/noise/real crack recess or robot optical occluders.'],
        human_review='pending', peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024)
    (output/'evaluation/review.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    write_html(output, report)
    return report


def supplementary_probes(session_path, demo, output, report):
    """Render sparse assessment examples, clearly separate from encoder acceptance."""
    repo = Path(__file__).resolve().parents[3]
    output, demo = Path(output).resolve(), Path(demo).resolve()
    private = output/'evaluation/probes'; private.mkdir()
    config_path = demo/'capture.yaml'; c = yaml.safe_load(config_path.read_text())
    calibration = read_json(demo/'calibration.json')
    assets = read_json(Path(session_path)/'evaluation/optical_assets.json')
    radius = c['tunnel']['radius_m']; rate = c['motion']['line_rate_hz']
    lines_per_rev = c['scan_encoder']['ppr']*c['scan_encoder']['edges_per_cycle']*c['rescaler']['multiply']/c['rescaler']['divide']
    omega = 2*np.pi*rate/lines_per_rev
    speed = c['motion']['advance_per_rev_m']*rate/lines_per_rev

    def probe(name, config, point):
        target = private/name
        command = ['bash', str(repo/'tools/with_optix_runtime.sh'),
                   str(repo/'install/ssb_core/lib/ssb_core/ssb_probe'), '--config', str(config),
                   '--output', str(target), '--rows', '1024', '--x', str(point[0]),
                   '--theta', str(point[1]/radius), '--speed', str(speed),
                   '--omega', str(omega), '--rate', str(rate)]
        with (private/(name+'.log')).open('w') as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=90)
        meta = read_json(target/'probe.json')
        if not meta['self_check']['passed'] or sha256_file(target/'image.pgm') != meta['image_sha256']:
            raise ValueError('optical review probe failed self-check or hash verification')
        return np.array(Image.open(target/'image.pgm')), meta, target

    middle = np.s_[256:768, c['camera']['width']//2-256:c['camera']['width']//2+256]
    for name, title in [('crack_020', '补充：网状裂缝'), ('crack_047', '补充：较宽裂缝')]:
        instance = next(i for i in assets['defects']['instances'] if i['id'] == name)
        path = instance['main_path_xq_m']; point = path[len(path)//2]
        raw, meta, target = probe(name, config_path, point)
        fixed, valid = correct(raw, calibration, meta['optical_signature'])
        report['examples'].append(dict(name=name, title=title,
            source='supplementary_optical_probe_not_encoder_capture', target_xq_m=point,
            nominal_body_width_mm=instance['body_width_mm'], rows=512, columns=512,
            images=save_pair(output, name, raw[middle], fixed[middle], valid[middle]),
            probe_report=str(target/'probe.json'), probe_sha256=sha256_file(target/'probe.json')))
    point = [4.745, .494]
    base, _, base_root = probe('fill_002', config_path, point)
    scene_path = demo/c['render']['optical_scene']; scene = read_json(scene_path)

    def absolute(node):
        if isinstance(node, dict):
            if 'file' in node:
                node['file'] = str((scene_path.parent/node['file']).resolve())
            for value in node.values(): absolute(value)
        elif isinstance(node, list):
            for value in node: absolute(value)
    absolute(scene); scene['indirect_fill_relative'] = 0.
    weak_scene = private/'no_fill_scene.json'
    weak_scene.write_text(json.dumps(scene, indent=2)+'\n')
    c['render']['optical_scene'] = str(weak_scene)
    weak_config = private/'no_fill_capture.yaml'
    weak_config.write_text(yaml.safe_dump(c, sort_keys=False))
    dark, _, dark_root = probe('fill_000', weak_config, point)
    delta = base.astype(np.int16)-dark.astype(np.int16)
    report['weak_fill_comparison'] = dict(
        scope='Same ideal optical poses, raw DN only; no calibration reused across changed signatures.',
        min_delta_dn=int(delta.min()), max_delta_dn=int(delta.max()),
        mean_delta_dn=float(delta.mean()), changed_fraction=float((delta != 0).mean()),
        base_probe_sha256=sha256_file(base_root/'probe.json'),
        no_fill_probe_sha256=sha256_file(dark_root/'probe.json'))
    if delta.min() < 0 or delta.max() > 1:
        raise ValueError('weak fill exceeds the accepted 0–1 DN effect in this specimen')
    report['examples'].append(dict(name='weak_fill', title='补充：关闭／开启弱补光',
        source='controlled_raw_optical_probe_pair', target_xq_m=point, nominal_body_width_mm=None,
        rows=512, columns=512, images=save_pair(output, 'weak_fill', dark[middle], base[middle],
                                             np.ones((512, 512), bool)),
        image_labels=['弱补光 0.000（原始 DN）', '弱补光 0.002（原始 DN）']))
    report['limitations'][3] = 'Network/wider cracks are supplementary probes; exact 0.2/0.6 mm boundary specimens are not claimed.'
    report['peak_rss_bytes'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024
    (output/'evaluation/review.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    write_html(output, report)


def write_html(output, report):
    cards = []
    source_labels = dict(archived_encoder_capture='3 米实际编码器采集',
        supplementary_optical_probe_not_encoder_capture='指定位置补充抽查（非编码器采集）',
        controlled_raw_optical_probe_pair='相同位姿的弱补光对照（原始 DN）')
    for e in report['examples']:
        labels = e.get('image_labels', ('原始 Mono8', '平场＋几何校正'))
        pictures = ''.join(f'<figure><figcaption>{html.escape(label)}</figcaption><img data-linear="{paths[0]}" data-srgb="{paths[1]}" src="{paths[0]}" width="512" height="512"></figure>'
                           for label, paths in zip(labels, zip(e['images']['linear'], e['images']['srgb'])))
        width = '' if e['nominal_body_width_mm'] is None else f"；实例名义宽度 {e['nominal_body_width_mm']:.3f} mm（评估真值）"
        source = source_labels.get(e['source'], e['source'])
        cards.append(f'<section><h2>{html.escape(e["title"])}</h2><p>{html.escape(source)}{width}</p><div class="pair">{pictures}</div></section>')
    document = '''<!doctype html><html lang="zh"><meta charset="utf-8"><title>阶段 B 画面复核</title>
<style>body{background:#202124;color:#eee;font:16px sans-serif;margin:24px}section{margin:32px 0}figure{margin:0}figcaption{margin:8px 0}.pair{display:flex;gap:20px;overflow:auto}img{image-rendering:pixelated;max-width:none}select{font:inherit}a{color:#8ab4f8}</style>
<h1>阶段 B 画面复核</h1><p>所有裁剪保留原像素。横向是线阵列，纵向是采集行；不是拼接全图。紫色代表无效像素。</p>
<p>显示：<select id="mode"><option value="linear">线性 DN 直接显示（无拉伸）</option><option value="srgb">统一 sRGB 显示曲线（仅显示副本）</option></select>
放大：<select id="zoom"><option value="1">100%</option><option value="2">200%</option><option value="4">400%</option></select></p>
<p>原始与校正图使用同一显示曲线，无自动对比度或锐化。显示 PNG 的数值裁剪不会修改 float32 测量数据。</p>
<p>此页为评估材料，使用缺陷真值选取样例，不能作为盲重建输入。</p>'''
    review = report.get('human_review', 'pending')
    accepted = isinstance(review, dict) and review.get('status') == 'accepted_by_user'
    document += '<p>用户已确认当前画质作为后续 20 米采集的基线。</p>' if accepted else '<p>人工确认待完成。</p>'
    document += ''.join(cards)
    for frame in report.get('gui_frames', []):
        label = html.escape(frame['title']); path = html.escape(frame['image'], quote=True)
        document += f'<section><h2>{label}</h2><p>现行 Gazebo 世界的观察截图，非相机采集图。<a href="{path}">查看截图原尺寸</a></p><img class="gui-preview" src="{path}" style="width:min(100%,960px);height:auto"></section>'
    document += '<p><a href="evaluation/review.json">完整检查与输入哈希</a></p><script>const mode=document.getElementById("mode"),zoom=document.getElementById("zoom");mode.onchange=()=>document.querySelectorAll("img[data-linear]").forEach(i=>i.src=i.dataset[mode.value]);zoom.onchange=()=>document.querySelectorAll("img[data-linear]").forEach(i=>{i.width=512*Number(zoom.value);i.height=512*Number(zoom.value)});</script></html>'
    (output/'index.html').write_text(document)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', required=True); p.add_argument('--demo', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--supplementary-probes', action='store_true', help='also render sparse network/wider crack and weak-fill specimens')
    a = p.parse_args(); report = prepare(a.session, a.demo, a.output)
    if a.supplementary_probes:
        supplementary_probes(a.session, a.demo, a.output, report)
    print(json.dumps(dict(verification=report['verification'], examples=len(report['examples']),
                         peak_rss_bytes=report['peak_rss_bytes']), ensure_ascii=False))


if __name__ == '__main__':
    main()
