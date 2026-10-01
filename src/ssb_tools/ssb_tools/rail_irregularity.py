"""Vertical rail-top irregularity (simulation truth) and its native heightmap collision.

Common (both-rail) component: Beijing Subway Line 10 measured vertical profile, fitted as
S(f) = A B^2 / (f^2 (f^2 + B^2)) with B^2 = 18.69 (Urban Rail Transit 2015, 1(3):159-163,
Table 2). Fig. 1 plots f in Hz of the inspection car (40-50 km/h), so the spatial shape is
converted at 12.5 m/s (corner wavelength about 2.9 m). The fitted absolute level is not
used: amplitude is scaled so that the maximum 10 m mid-chord offset equals the configured
tier (GB/T 50299-2018 acceptance 2 mm; DB11/T 718-2016 maintenance 4-7 mm).

Differential component d = left - right (cross-level, 水平): no Beijing cross-level fit exists,
so the German low-disturbance cross-level PSD shape is used,
S(W) ~ Wc^2 W^2 / ((W^2+Wr^2)(W^2+Wc^2)(W^2+Ws^2)), Wc=0.8246, Wr=0.0206, Ws=0.4380 rad/m.
DB11/T 718-2016 sets equal cross-level and 5 m-base twist (三角坑) limits at every maintenance
level, so the amplitude is the largest that keeps both within the tier; for this band the 5 m
twist binds. A rigid four-wheel chassis therefore sees about 1-1.5 mm twist per tier-2 mm across
its 0.7 m wheelbase, which can unload hard tread wheels: that is physical, not a defect.
Profiles and heightmaps are evaluation truth; reconstruction must never read them.
"""
import json
import math
from pathlib import Path

import numpy as np

MODEL = 'beijing_subway_vertical_v1'
B2_HZ2 = 18.69            # Table 2, temporal frequency squared
CROSS_MODEL = 'german_low_disturbance_cross_level_shape_v1'
CROSS_W_RAD_M = (.8246, .0206, .4380)   # Wc, Wr, Ws
TWIST_BASE_M = 5.         # DB11/T 718-2016 static twist base
INSPECTION_SPEED_M_S = 12.5
CHORD_M = 10.
GRID_M = .005             # fine synthesis grid
SEGMENT_MAX_M = 12.5      # one native heightmap per segment; tunnel length only adds segments
SEGMENT_OVERLAP_M = .5    # identical heights in the overlap, so the wheel sees no seam
SEGMENT_SAMPLES = 513     # 2^n+1 square image required by gz-physics/DART
# gz-physics/DART spreads the N samples over size*(N-1)/N, not size (measured: a 513-sample,
# 10 m ramp reports a +/-4.99025 m AABB and heights match x*513/512). SDF sizes are stretched by
# N/(N-1) so the physical sample spacing is the intended span/(N-1).
SDF_SIZE_FACTOR = SEGMENT_SAMPLES/(SEGMENT_SAMPLES-1)
BOX_MARGIN_M = .003       # flat guide-face box top below the lowest rail-top point
GUIDE_FACE_MIN_M = .012   # guide face height that must remain above the bearing bottom


def settings(config):
    """Validated truth settings, or None for the flat baseline.

    chord10_max_m scales the common (both-rail) vertical profile; cross_level_tier_m bounds both
    the cross-level d(x) = left - right and its 5 m twist (independent random stream).
    """
    entry = (config.get('truth') or {}).get('track_irregularity')
    if not entry:
        return None
    if entry.get('model', MODEL) != MODEL:
        raise ValueError('unknown track irregularity model')
    chord = float(entry.get('chord10_max_m', 0.))
    cross = float(entry.get('cross_level_tier_m', 0.))
    band = [float(v) for v in entry.get('band_m', [.5, 10.])]
    seed = int(entry['seed'])
    if not (math.isfinite(chord) and 0 <= chord <= .01):
        raise ValueError('chord10_max_m must be in [0, 0.01] m')
    if not (math.isfinite(cross) and 0 <= cross <= .01):
        raise ValueError('cross_level_tier_m must be in [0, 0.01] m')
    if not (len(band) == 2 and .1 <= band[0] < band[1] <= 50.):
        raise ValueError('band_m must be increasing wavelengths within [0.1, 50] m')
    if entry.get('common_mode', cross == 0) != (cross == 0):
        raise ValueError('common_mode must be true exactly when cross_level_tier_m is zero')
    if chord == 0 and cross == 0:
        return None
    out = dict(model=MODEL, chord10_max_m=chord, cross_level_tier_m=cross, band_m=band, seed=seed,
               common_mode=cross == 0)
    if cross > 0:
        out['cross_model'] = CROSS_MODEL
    return out


def shape(wavelength):
    """Relative spatial PSD at the given wavelengths (arbitrary absolute level)."""
    f = INSPECTION_SPEED_M_S/np.asarray(wavelength, float)
    return 1./(f*f*(f*f+B2_HZ2))


def cross_shape(wavelength):
    """Relative cross-level PSD at the given wavelengths (arbitrary absolute level)."""
    w = 2*np.pi/np.asarray(wavelength, float)
    c, r, s = CROSS_W_RAD_M
    return c*c*w*w/((w*w+r*r)*(w*w+c*c)*(w*w+s*s))


def chord_offsets(x, z, chord=CHORD_M):
    """Mid-chord offsets z(x) - (z(x-c/2)+z(x+c/2))/2 where the full chord is on the profile."""
    half = chord/2
    inside = (x >= x[0]+half) & (x <= x[-1]-half)
    centre = x[inside]
    return centre, z[inside]-.5*(np.interp(centre-half, x, z)+np.interp(centre+half, x, z))


def _synthesise(x0, x1, band, rng, psd=shape):
    """Unit-scale random profile with the given PSD shape over [x0, x1] on the fine grid."""
    lo, hi = band
    # Periodic synthesis over a longer record, then cropped: band edges do not wrap.
    count = int(2**math.ceil(math.log2((x1-x0+2*hi)/GRID_M)))
    n = np.fft.rfftfreq(count, GRID_M)
    keep = (n >= 1/hi) & (n <= 1/lo)
    spectrum = np.zeros(len(n), complex)
    spectrum[keep] = np.sqrt(psd(1/n[keep]))*np.exp(2j*np.pi*rng.random(int(keep.sum())))
    record = np.fft.irfft(spectrum, count)
    x = x0+np.arange(int(round((x1-x0)/GRID_M))+1)*GRID_M
    start = int(round(hi/GRID_M))
    z = record[start:start+len(x)]
    return x, z-z.mean()


def _metrics(x, z):
    slope = np.gradient(z, x)
    return dict(rms_m=float(z.std()), min_m=float(z.min()), max_m=float(z.max()),
                chord10_max_m=float(np.abs(chord_offsets(x, z)[1]).max()), max_slope=float(np.abs(slope).max()),
                max_curvature_per_m=float(np.abs(np.gradient(slope, x)).max()))


def twist(x, cross, base):
    """Cross-level change over a longitudinal base (三角坑 for base 5 m / 6.25 m)."""
    inside = x <= x[-1]-base
    return cross[inside]-np.interp(x[inside]+base, x, cross)


def rails(config):
    """(x, z_left, z_right, record) on a fine grid over the tunnel length, or None when flat."""
    s = settings(config)
    if s is None:
        return None
    t = config['tunnel']
    x0, x1 = float(t['x_min_m']), float(t['x_max_m'])
    if x1-x0 < CHORD_M+1:
        raise ValueError('track too short for a 10 m chord scale')
    # The common stream keeps the original seed alone, so common-only profiles are unchanged.
    x, z = _synthesise(x0, x1, s['band_m'], np.random.default_rng(s['seed']))
    _, offsets = chord_offsets(x, z)
    z = z*(s['chord10_max_m']/np.abs(offsets).max())
    d = np.zeros_like(z)
    if s['cross_level_tier_m'] > 0:
        _, d = _synthesise(x0, x1, s['band_m'], np.random.default_rng([s['seed'], 1]), cross_shape)
        d *= s['cross_level_tier_m']/max(np.abs(d).max(), np.abs(twist(x, d, TWIST_BASE_M)).max())
    left, right = z+d/2, z-d/2
    metrics = dict(_metrics(x, z), left=_metrics(x, left), right=_metrics(x, right),
                   cross_level_max_m=float(np.abs(d).max()),
                   twist_5m_max_m=float(np.abs(twist(x, d, 5.)).max()),
                   twist_wheelbase_0_7m_max_m=float(np.abs(twist(x, d, .7)).max()))
    return x, left, right, dict(settings=s, metrics=metrics, grid_m=GRID_M,
        source='common: Beijing Subway vertical PSD shape (Urban Rail Transit 2015 Table 2/Fig. 1); '
               'cross-level: German low-disturbance shape, bounded with its 5 m twist by the tier (DB11/T 718-2016)')


def profile(config):
    """Common-mode view (x, mean of both rails, record), or None when flat."""
    result = rails(config)
    if result is None:
        return None
    x, left, right, record = result
    return x, (left+right)/2, record


def segments(x0, x1):
    length = x1-x0
    count = max(1, math.ceil((length-SEGMENT_OVERLAP_M)/(SEGMENT_MAX_M-SEGMENT_OVERLAP_M)))
    size = (length+(count-1)*SEGMENT_OVERLAP_M)/count
    return [(x0+i*(size-SEGMENT_OVERLAP_M), x0+i*(size-SEGMENT_OVERLAP_M)+size) for i in range(count)]


def guide_box_top(z):
    """Top of the flat guide-face collision box: below every rail-top point."""
    top = float(z.min())-BOX_MARGIN_M
    # Guide bearings span 19 +/- 12 mm below the nominal top; keep a usable face.
    if top < -.031+GUIDE_FACE_MIN_M:
        raise ValueError('irregularity too deep for the rail guide faces')
    return top


def write_heightmaps(folder, config, rail_ys, width):
    """Write per-rail, per-segment 16-bit heightmaps; returns (models, record) or None if flat.

    rail_ys: [('left', y), ('right', y)].
    """
    from PIL import Image
    from .stage_b_scene import sub, digest
    from .stage_b_track import friction
    import xml.etree.ElementTree as ET
    result = rails(config)
    if result is None:
        return None
    x, left, right, record = result
    profiles = dict(left=left, right=right)
    folder = Path(folder)
    np.savez_compressed(folder/'rail_profile.npz', x=x, left=left, right=right)
    models, images = [], []
    for k, (a, b) in enumerate(segments(x[0], x[-1])):
        xs = np.linspace(a, b, SEGMENT_SAMPLES)
        for side, y in rail_ys:
            zs = np.interp(xs, x, profiles[side])
            low, high = float(zs.min()), float(zs.max())
            span = max(high-low, 1e-6)
            row = np.rint((zs-low)/span*65535).astype(np.uint16)
            path = folder/f'rail_top_{side}_{k:02d}.png'
            # Rows along y are identical (one rail head is level across); columns run along +x.
            Image.fromarray(np.tile(row, (SEGMENT_SAMPLES, 1)), mode='I;16').save(path)
            images.append(dict(file=path.name, rail=side, sha256=digest(path), x_m=[a, b], z_m=[low, low+span],
                               quantization_m=span/65535/2))
            # gz-physics ignores <heightmap><pos>: place the surface by model pose only.
            model = ET.Element('model', name=f'rail_surface_{side}_{k:02d}')
            sub(model, 'static', 'true')
            sub(model, 'pose', f'{(a+b)/2:.12g} {y:.12g} {low:.12g} 0 0 0')
            collision = sub(sub(model, 'link', name='top'), 'collision', name='rail_top')
            shape_node = sub(sub(collision, 'geometry'), 'heightmap')
            sub(shape_node, 'uri', str(path.resolve()))
            sub(shape_node, 'size', f'{(b-a)*SDF_SIZE_FACTOR:.12g} {width*SDF_SIZE_FACTOR:.12g} {span:.12g}')
            sub(shape_node, 'pos', '0 0 0')
            friction(collision, 1.0)
            models.append(model)
    record = dict(record, schema='ssb.rail_irregularity.v2', profile_file='rail_profile.npz',
                  profile_sha256=digest(folder/'rail_profile.npz'), heightmaps=images,
                  segment_samples=SEGMENT_SAMPLES, segment_overlap_m=SEGMENT_OVERLAP_M,
                  guide_box_top_m=guide_box_top(np.minimum(left, right)),
                  role='simulation truth: wheel collision surface; evaluation only')
    (folder/'rail_irregularity.json').write_text(json.dumps(record, indent=2)+'\n')
    return models, record


def decode_heightmap(path, entry):
    """Decoded (x, z) of one written segment, for independent verification."""
    from PIL import Image
    data = np.asarray(Image.open(path), dtype=float)
    row = data[0]
    a, b = entry['x_m']; low, high = entry['z_m']
    return np.linspace(a, b, len(row)), low+row/65535*(high-low)
