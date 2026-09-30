"""Translation-matchable repetition in a generated wall surface.

Independently composes the recipe albedo (area-averaged source at a coarse pitch,
bilinear seam alpha) over a wall region, then for random windows searches a
neighbourhood by normalised cross-correlation and reports the best match away
from the window's own position. A high off-peak score means a translated copy of
that window exists nearby, i.e. a candidate false match for strip registration.

Scores are computed on a high-passed image (default: subtract a 10 mm Gaussian): smooth
large-scale shading correlates with its own small shifts, which is similarity, not
repeated texture. --area tiles a whole wall rectangle into regions so a result can
cover e.g. the full 20 m x 240 deg instead of one sample region.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from .stage_b_surface import TEXEL


def compose(surface_path, x0, q0, size_m, pitch, use_macro=True):
    surface_path = Path(surface_path)
    s = json.loads(surface_path.read_text()); r = json.loads((surface_path.parent/s['recipe']['file']).read_text())
    n = round(size_m/pitch); g = r['guide_texel_m']; side = r['patch_pixels']
    sources = []
    for e in r['sources']:
        w, h = e.get('width', e.get('side')), e.get('height', e.get('side'))
        a = np.memmap(surface_path.parent/e['file'], TEXEL, mode='r', shape=(h, w))['albedo']
        native = e['source_width_m']/w
        sources.append((cv2.resize(np.asarray(a, np.float32)/65535, (round(w*native/pitch), round(h*native/pitch)),
                                   interpolation=cv2.INTER_AREA), native))
    alpha = np.fromfile(surface_path.parent/r['alpha']['file'], np.uint8).reshape(-1, side, side).astype(np.float32)/255
    xs = x0+(np.arange(n)+.5)*pitch; qs = q0+(np.arange(n)+.5)*pitch
    gx = (xs-r['origin_xq_m'][0])/g; gy = (qs-r['origin_xq_m'][1])/g
    out = np.zeros((n, n), np.float32); filled = np.zeros((n, n), np.float32)
    for i, p in enumerate(r['placements']):
        xi = np.flatnonzero((gx >= p['left']) & (gx < p['left']+side)); yi = np.flatnonzero((gy >= p['top']) & (gy < p['top']+side))
        if not len(xi) or not len(yi):
            continue
        px, py = np.meshgrid(gx[xi]-p['left'], gy[yi]-p['top'])
        a = cv2.remap(alpha[i], np.float32(px-.5), np.float32(py-.5), cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        m = np.asarray(p['source_matrix']); o = p['source_offset_m']; src, _ = sources[p['material']]
        u = (m[0, 0]*px*g+m[0, 1]*py*g+o[0])/pitch-.5; v = (m[1, 0]*px*g+m[1, 1]*py*g+o[1])/pitch-.5
        val = cv2.remap(src, np.float32(u), np.float32(v), cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        sl = np.ix_(yi, xi); out[sl] = out[sl]*(1-a)+val*a; filled[sl] = filled[sl]*(1-a)+a
    if filled.min() < .999:
        raise ValueError('region outside complete layout')
    macro = r.get('macro')
    if macro and use_macro:
        out *= sample_macro(surface_path.parent, macro, s['period_q_m'], xs, qs)
    return out


def sample_macro(root, macro, period, xs, qs):
    m = np.fromfile(Path(root)/macro['file'], '<u2').reshape(macro['height'], macro['width']).astype(np.float32)/macro['scale']
    u = (xs-macro['origin_xq_m'][0])/macro['pitch_m']-.5
    v = ((qs-macro['origin_xq_m'][1]) % period)/macro['pitch_m']-.5
    U, V = np.meshgrid(np.float32(u), np.float32(v))
    return cv2.remap(m, U, V, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


def off_peak(image, pitch, window_m=.1, search_m=.3, exclude_m=.015, count=300, seed=1):
    """Best NCC of each random window against its surroundings, excluding its own peak."""
    rng = np.random.default_rng(seed); w = round(window_m/pitch); s = round(search_m/pitch); ex = round(exclude_m/pitch)
    n = image.shape[0]; scores = []
    for _ in range(count):
        y, x = rng.integers(s, n-s-w, size=2)
        tpl = image[y:y+w, x:x+w]; area = image[y-s:y+s+w, x-s:x+s+w]
        if tpl.std() < 1e-6:
            continue
        c = cv2.matchTemplate(area, tpl, cv2.TM_CCOEFF_NORMED)
        c[s-ex:s+ex+1, s-ex:s+ex+1] = -1
        scores.append(float(c.max()))
    return np.asarray(scores)


def highpass(image, pitch, sigma_m):
    return image if sigma_m <= 0 else (image-cv2.GaussianBlur(image, (0, 0), sigma_m/pitch)).astype(np.float32)


def summary(scores):
    return dict(windows=len(scores), p50=float(np.median(scores)), p95=float(np.percentile(scores, 95)),
                max=float(scores.max()), fraction_above_0_8=float((scores > .8).mean()), fraction_above_0_9=float((scores > .9).mean()))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--surface', required=True); p.add_argument('--x', type=float, default=4.0); p.add_argument('--q', type=float, default=-1.5)
    p.add_argument('--size', type=float, default=3.0); p.add_argument('--pitch', type=float, default=.001)
    p.add_argument('--png'); p.add_argument('--output'); p.add_argument('--no-macro', action='store_true')
    p.add_argument('--highpass-m', type=float, default=.01, help='Gaussian sigma removed before matching; 0 disables')
    p.add_argument('--windows', type=int, default=300, help='windows per region'); p.add_argument('--search-m', type=float, default=.3)
    p.add_argument('--window-m', type=float, default=.1)
    p.add_argument('--area', type=float, nargs=4, metavar=('X0', 'X1', 'Q0', 'Q1'), help='tile this wall rectangle with --size regions')
    a = p.parse_args()
    regions = [(a.x, a.q)]
    if a.area:
        x0, x1, q0, q1 = a.area
        regions = [(x, q) for x in np.arange(x0, x1-a.size+1e-9, a.size) for q in np.arange(q0, q1-a.size+1e-9, a.size)]
    scope = dict(region_size_m=a.size, regions=len(regions), windows_per_region=a.windows, window_m=a.window_m, search_m=a.search_m,
                 highpass_sigma_m=a.highpass_m, pitch_m=a.pitch, macro=not a.no_macro,
                 limit='Translation-only NCC inside each region; repeats across region borders or farther than search_m are not measured.')
    per, pooled, img = [], [], None
    for i, (x, q) in enumerate(regions):
        img = compose(a.surface, x, q, a.size, a.pitch, not a.no_macro)
        sc = off_peak(highpass(img, a.pitch, a.highpass_m), a.pitch, a.window_m, a.search_m, count=a.windows, seed=1+i)
        per.append(dict(x_m=float(x), q_m=float(q), **summary(sc))); pooled.append(sc)
        print(json.dumps(per[-1]), flush=True)
    result = dict(surface=str(Path(a.surface).resolve()), scope=scope, pooled=summary(np.concatenate(pooled)), regions=per)
    if a.png and len(regions) == 1:
        v = np.clip(img/img.mean()*.45, 0, 1); cv2.imwrite(a.png, np.uint8(np.where(v <= .0031308, 12.92*v, 1.055*v**(1/2.4)-.055)*255+.5))
    if a.output:
        Path(a.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(scope=scope, pooled=result['pooled'])))


if __name__ == '__main__':
    main()
