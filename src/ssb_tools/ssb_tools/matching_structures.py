"""Bounded image-only exclusion of long circumferential dark structures.

No ring period, scene, defect labels, pose truth, or evaluation inputs are used.
Exclusions affect correspondence estimation only; raw pixels and scoring stay intact.
"""
import cv2
import numpy as np
from scipy.ndimage import label, median_filter


STRUCTURE_MODEL = dict(model='circumferential_dark_band_v1', background_window_mm=60.,
    min_width_mm=2., max_width_mm=40., min_contrast_dn=6., row_support_fraction=.6,
    valid_column_fraction=.8, minimum_coarse_support_fraction=.7)
LK_WINDOW = 21
LK_LEVELS = 3
# Includes the level-3 patch, pyramid 5x5 blur accumulated through all levels,
# and a gradient stencil. A centre outside the groove alone is insufficient.
LK_STRUCTURE_RADIUS = (LK_WINDOW//2+1)*2**LK_LEVELS+2*(2**LK_LEVELS-1)


def column_nanmedian(values):
    """np.nanmedian(values, axis=0), bit-identical, without its masked-array sort.

    NumPy sorts masked arrays for axes shorter than 600 samples. Complete columns
    use the ordinary partition median; columns with NaN keep the original routine.
    """
    full = ~np.isnan(values).any(axis=0)
    profile = np.empty(values.shape[1], values.dtype)
    if full.any():
        profile[full] = np.median(np.ascontiguousarray(values[:, full].T), axis=1)
    if not full.all():
        profile[~full] = np.nanmedian(values[:, ~full], axis=0)
    return profile


def long_dark_mask(image, valid, pitch_m):
    image, valid = np.asarray(image), np.asarray(valid, bool)
    if (image.ndim != 2 or image.shape != valid.shape or image.size > 1 << 20 or
            not np.isfinite(pitch_m) or pitch_m <= 0):
        raise ValueError('bounded image, validity mask and positive grid pitch required')
    mask = np.zeros(image.shape, bool)
    if not valid.any():
        return mask, dict(bands_px=[], threshold_dn=None, fraction=0.)
    if not np.isfinite(image[valid]).all():
        raise ValueError('nonfinite valid image')
    count = valid.sum(axis=0)
    values = np.where(valid, image, np.nan)
    values[:, count == 0] = np.median(image[valid])
    profile = column_nanmedian(values)
    smooth = cv2.GaussianBlur(profile.astype(np.float32)[None, :], (0, 0), 1.5)[0]
    size = int(np.ceil(STRUCTURE_MODEL['background_window_mm']/(1000*pitch_m))) | 1
    # Bound filtering work for malformed or unexpectedly fine grids.
    size = min(size, 2*image.shape[1]+1)
    background = median_filter(smooth, size=size, mode='reflect')
    noise = profile-cv2.GaussianBlur(profile.astype(np.float32)[None, :], (0, 0), 3.)[0]
    threshold = max(STRUCTURE_MODEL['min_contrast_dn'],
                    4*1.4826*np.median(abs(noise-np.median(noise))))
    active = ((background-smooth > threshold) &
              (count >= STRUCTURE_MODEL['valid_column_fraction']*image.shape[0]))
    active = cv2.morphologyEx(active.astype(np.uint8)[None, :], cv2.MORPH_CLOSE,
                             np.ones((1, 7), np.uint8))[0].astype(bool)
    components, number = label(active)
    bands = []
    for identifier in range(1, number+1):
        columns = np.flatnonzero(components == identifier)
        left, right = int(columns[0]), int(columns[-1]+1)
        width_mm = len(columns)*pitch_m*1000
        if not STRUCTURE_MODEL['min_width_mm'] <= width_mm <= STRUCTURE_MODEL['max_width_mm']:
            continue
        dark = ((background[None, left:right]-image[:, left:right] > threshold/2) &
                valid[:, left:right])
        if dark.mean() < STRUCTURE_MODEL['row_support_fraction']:
            continue
        left, right = max(0, left-3), min(image.shape[1], right+3)
        mask[:, left:right] = True
        bands.append([left, right])
    return mask, dict(bands_px=bands, threshold_dn=float(threshold), fraction=float(mask.mean()))


def unsafe_structure_footprint(mask):
    return cv2.dilate(np.asarray(mask, np.uint8),
        np.ones((2*LK_STRUCTURE_RADIUS+1, 2*LK_STRUCTURE_RADIUS+1), np.uint8)).astype(bool)


def safe_structure_points(unsafe, points):
    finite = np.isfinite(points).all(axis=1)
    p = np.where(np.isfinite(points), points, -1).astype(np.float32)
    hit = cv2.remap(unsafe.astype(np.float32), p[:, 0:1], p[:, 1:2], cv2.INTER_LINEAR,
                   borderMode=cv2.BORDER_CONSTANT, borderValue=1)[:, 0]
    return finite & (hit <= 1e-6)


def masked_coarse_shift(a, b, valid_a, valid_b, max_shift, settings):
    """Pearson NCC over the actual overlapping background at each candidate shift.

    Filling the groove with black or zero and correlating it would create a new
    feature at its edge. Masked sums remove both images' excluded samples instead.
    """
    factor = settings.coarse_factor
    shape = (a.shape[1]//factor, a.shape[0]//factor)
    images, masks = [], []
    for image, valid in ((a, valid_a), (b, valid_b)):
        weight = valid.astype(np.float32)
        local = cv2.GaussianBlur(image*weight, (0, 0), 4.)/np.maximum(
            cv2.GaussianBlur(weight, (0, 0), 4.), 1e-6)
        interior = cv2.erode(weight, np.ones((9, 9), np.uint8))
        images.append(cv2.resize((image-local)*interior, shape, interpolation=cv2.INTER_AREA))
        masks.append((cv2.resize(interior, shape, interpolation=cv2.INTER_AREA) >= 1-1e-6).astype(np.float32))
    margin = int(np.ceil(max_shift/factor))+2
    if min(shape)-2*margin < 16:
        return None, dict(reason='search range exceeds window support')
    template, tm = images[0][margin:-margin, margin:-margin], masks[0][margin:-margin, margin:-margin]
    search, sm = images[1], masks[1]
    if tm.sum() < 256 or np.std(template[tm.astype(bool)]) < .2:
        return None, dict(reason='weak masked coarse texture or support')
    def correlate(x, y):
        return cv2.matchTemplate(x.astype(np.float32), y.astype(np.float32), cv2.TM_CCORR).astype(np.float64)
    count = correlate(sm, tm)
    sa, sb = correlate(sm, template*tm), correlate(search*sm, tm)
    aa = correlate(sm, template*template*tm)-sa*sa/np.maximum(count, 1)
    bb = correlate(search*search*sm, tm)-sb*sb/np.maximum(count, 1)
    ab = correlate(search*sm, template*tm)-sa*sb/np.maximum(count, 1)
    enough = ((count >= STRUCTURE_MODEL['minimum_coarse_support_fraction']*tm.sum()) &
              (aa > 1e-6) & (bb > 1e-6))
    if not enough.any():
        return None, dict(reason='no common masked coarse support')
    response = np.where(enough, np.clip(ab/np.sqrt(np.maximum(aa*bb, 1e-12)), -1, 1), -1)
    _, peak, _, position = cv2.minMaxLoc(response)
    x, y = position
    runner_up = response.copy()
    runner_up[max(0, y-3):y+4, max(0, x-3):x+4] = -1
    second = float(runner_up.max())
    diagnostic = dict(coarse_ncc=float(peak), peak_gap=float(peak-second),
                      coarse_background_support=int(round(count[y, x])))
    if peak < settings.min_coarse_ncc or peak-second < settings.min_peak_gap:
        return None, dict(diagnostic, reason='weak or ambiguous coarse peak')
    shift = (np.asarray(position, float)-margin)*factor
    if np.max(abs(shift)) > max_shift:
        return None, dict(diagnostic, reason='coarse shift exceeds prior')
    return shift, diagnostic
