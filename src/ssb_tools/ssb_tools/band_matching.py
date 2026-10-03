"""Image-only D2 correspondences; local affine checks allow small body tilt.

Affine matrices are diagnostics, not estimated body attitudes or final warps.
Rejected/weak windows contribute no constraints to a future global optimizer.
"""
from dataclasses import dataclass
import cv2
import numpy as np
from .matching_structures import (long_dark_mask, masked_coarse_shift, unsafe_structure_footprint,
                                  safe_structure_points, LK_WINDOW, LK_LEVELS, LK_STRUCTURE_RADIUS)


@dataclass(frozen=True)
class MatchSettings:
    max_shift_mm: float = 40.
    coarse_factor: int = 4
    min_texture_dn: float = .8
    min_coarse_ncc: float = .30
    min_peak_gap: float = .04
    max_features: int = 200
    min_matches: int = 24
    max_fb_px: float = .5
    min_patch_ncc: float = .75
    ransac_px: float = .8
    max_jacobian_change: float = .03
    max_holdout_p95_px: float = 1.
    exclude_long_structures: bool = True

    def __post_init__(self):
        values = [self.max_shift_mm, self.min_texture_dn, self.min_coarse_ncc,
                  self.min_peak_gap, self.max_fb_px, self.min_patch_ncc,
                  self.ransac_px, self.max_jacobian_change, self.max_holdout_p95_px]
        if (type(self.exclude_long_structures) is not bool or not np.isfinite(values).all() or self.max_shift_mm <= 0 or self.min_texture_dn < 0 or
            not -1 <= self.min_coarse_ncc <= 1 or not 0 < self.min_peak_gap <= 2 or
            not -1 <= self.min_patch_ncc <= 1 or min(self.max_fb_px, self.ransac_px, self.max_holdout_p95_px) <= 0 or
            not 0 < self.max_jacobian_change <= .1 or
            not all(isinstance(v, int) for v in (self.coarse_factor, self.max_features, self.min_matches)) or
            not 1 <= self.coarse_factor <= 16 or not 12 <= self.min_matches <= self.max_features <= 512):
            raise ValueError('invalid matching settings')


def matching_image(image, valid):
    # Contrast normalization is for matching only; physical DN stays untouched.
    values = image[valid]
    median = float(np.median(values)); sigma = max(float(np.std(values)), 1.)
    normalized = np.clip((np.where(valid, image, median)-median)*32/sigma+128, 0, 255)
    return normalized.astype(np.uint8)


def coarse_shift(a, b, valid_a, valid_b, max_shift, settings):
    factor = settings.coarse_factor
    shape = (a.shape[1]//factor, a.shape[0]//factor)
    images = []
    for image, valid in ((a, valid_a), (b, valid_b)):
        image = image.astype(np.float32)
        highpass = image-cv2.GaussianBlur(image, (0, 0), 4.)
        highpass[~cv2.erode(valid.astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)] = 0
        images.append(cv2.resize(highpass, shape, interpolation=cv2.INTER_AREA))
    margin = int(np.ceil(max_shift/factor))+2
    if min(shape)-2*margin < 16: return None, dict(reason='search range exceeds window support')
    template = images[0][margin:-margin, margin:-margin]
    if np.std(template) < .2: return None, dict(reason='weak coarse texture')
    response = cv2.matchTemplate(images[1], template, cv2.TM_CCOEFF_NORMED)
    _, peak, _, position = cv2.minMaxLoc(response)
    x, y = position
    runner_up = response.copy()
    runner_up[max(0, y-3):y+4, max(0, x-3):x+4] = -1
    second = float(runner_up.max())
    diagnostic = dict(coarse_ncc=float(peak), peak_gap=float(peak-second))
    if peak < settings.min_coarse_ncc or peak-second < settings.min_peak_gap:
        return None, dict(diagnostic, reason='weak or ambiguous coarse peak')
    shift = (np.asarray(position, float)-margin)*factor
    if np.max(abs(shift)) > max_shift: return None, dict(diagnostic, reason='coarse shift exceeds prior')
    return shift, diagnostic


def patch_ncc(a, b, valid_a, valid_b, points_a, points_b, radius=10):
    delta = np.arange(-radius, radius+1, dtype=np.float32)
    dx, dy = np.meshgrid(delta, delta)
    scores = np.full(len(points_a), -1., np.float32)
    mask_a, mask_b = valid_a.astype(np.float32), valid_b.astype(np.float32)
    for i, (pa, pb) in enumerate(zip(points_a, points_b)):
        patches = []
        for image, valid, point in ((a, mask_a, pa), (b, mask_b, pb)):
            mx, my = dx+point[0], dy+point[1]
            mask = cv2.remap(valid, mx, my, cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            if not np.all(mask >= 1-1e-6): break
            patch = cv2.remap(image, mx, my, cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            patch = patch.astype(float); patches.append(patch-patch.mean())
        if len(patches) == 2:
            norm = np.linalg.norm(patches[0])*np.linalg.norm(patches[1])
            if norm > 1e-6: scores[i] = np.sum(patches[0]*patches[1])/norm
    return scores


def match_window(a, b, valid_a, valid_b, pitch_m, settings=MatchSettings()):
    if a.shape != b.shape or a.ndim != 2 or min(a.shape) < 64:
        raise ValueError('matching requires equal two-dimensional windows of at least 64 pixels')
    if not np.isfinite(pitch_m) or pitch_m <= 0: raise ValueError('positive matching pitch required')
    if np.shape(valid_a) != a.shape or np.shape(valid_b) != a.shape:
        raise ValueError('matching mask dimensions must equal image dimensions')
    if settings.coarse_factor < 1 or settings.max_features < settings.min_matches or settings.max_shift_mm <= 0:
        raise ValueError('invalid matching settings')
    valid_a = np.asarray(valid_a, bool) & np.isfinite(a)
    valid_b = np.asarray(valid_b, bool) & np.isfinite(b)
    empty = dict(points_a=np.empty((0, 2)), points_b=np.empty((0, 2)),
                 ncc=np.empty(0), fb_error=np.empty(0), residual=np.empty(0),
                 holdout=np.empty(0, bool), inlier=np.empty(0, bool))
    def rejected(reason, **extra):
        return dict(status='unmeasurable', reason=reason, **extra), empty
    if min(valid_a.mean(), valid_b.mean()) < .8: return rejected('insufficient valid image support')
    if min(np.std(a[valid_a]), np.std(b[valid_b])) < settings.min_texture_dn:
        return rejected('weak texture')
    joint_a, joint_b = np.zeros(a.shape, bool), np.zeros(b.shape, bool)
    structure_reports = []
    if settings.exclude_long_structures:
        joint_a, report_a = long_dark_mask(a, valid_a, pitch_m)
        joint_b, report_b = long_dark_mask(b, valid_b, pitch_m)
        structure_reports = [report_a, report_b]
    structured = joint_a.any() or joint_b.any()
    valid_a, valid_b = valid_a & ~joint_a, valid_b & ~joint_b
    if (not valid_a.any() or not valid_b.any() or
            min(np.std(a[valid_a]), np.std(b[valid_b])) < settings.min_texture_dn):
        return rejected('weak nonstructure background', structure_masks=structure_reports)
    image_a, image_b = matching_image(a, valid_a), matching_image(b, valid_b)
    max_shift = settings.max_shift_mm/(1000*pitch_m)
    shift, diagnostic = (masked_coarse_shift if structured else coarse_shift)(
        image_a, image_b, valid_a, valid_b, max_shift, settings)
    diagnostic.update(structure_masks=structure_reports, structure_lk_radius_px=LK_STRUCTURE_RADIUS)
    if shift is None: return rejected(**diagnostic)
    feature_mask = cv2.erode(valid_a.astype(np.uint8), np.ones((25, 25), np.uint8))
    feature_mask[:16] = 0; feature_mask[-16:] = 0; feature_mask[:, :16] = 0; feature_mask[:, -16:] = 0
    if structured:
        feature_mask[unsafe_structure_footprint(joint_a)] = 0
    points = cv2.goodFeaturesToTrack(image_a, settings.max_features, .025, 16,
                                    mask=feature_mask, blockSize=7)
    diagnostic['corners'] = 0 if points is None else len(points)
    if points is None or len(points) < settings.min_matches: return rejected('too few corners', **diagnostic)
    initial = (points+shift.astype(np.float32)).astype(np.float32)
    criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 40, .001)
    following, forward_ok, _ = cv2.calcOpticalFlowPyrLK(image_a, image_b, points, initial,
        winSize=(LK_WINDOW, LK_WINDOW), maxLevel=LK_LEVELS, criteria=criteria, flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
    usable = forward_ok[:, 0].astype(bool) & np.isfinite(following[:, 0]).all(axis=1)
    points, following = points[usable], following[usable]
    diagnostic['forward_tracks'] = len(points)
    if len(points) < settings.min_matches: return rejected('too few forward tracks', **diagnostic)
    reverse, reverse_ok, _ = cv2.calcOpticalFlowPyrLK(image_b, image_a, following, points.copy(),
        winSize=(LK_WINDOW, LK_WINDOW), maxLevel=LK_LEVELS, criteria=criteria, flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
    pa, pb = points[:, 0].astype(float), following[:, 0].astype(float)
    fb = np.linalg.norm(reverse[:, 0]-pa, axis=1)
    usable = reverse_ok[:, 0].astype(bool)
    usable &= np.isfinite(pb).all(axis=1) & (fb <= settings.max_fb_px)
    usable &= np.max(abs(pb-pa), axis=1) <= max_shift
    if structured:
        usable &= safe_structure_points(unsafe_structure_footprint(joint_b), pb)
    pa, pb, fb = pa[usable], pb[usable], fb[usable]
    diagnostic['bidirectional_tracks'] = len(pa)
    ncc = patch_ncc(a, b, valid_a, valid_b, pa, pb)
    good = ncc >= settings.min_patch_ncc
    pa, pb, fb, ncc = pa[good], pb[good], fb[good], ncc[good]
    diagnostic['photometric_matches'] = len(pa)
    if len(pa) < settings.min_matches:
        return rejected('too few bidirectional photometric matches', candidates=len(pa), **diagnostic)
    normalized = (pa-pa.mean(axis=0))/np.array([a.shape[1], a.shape[0]])
    covariance = np.linalg.eigvalsh(normalized.T@normalized/len(pa))
    diagnostic['spatial_covariance_eigenvalues'] = covariance.tolist()
    if covariance[0] < .002 or covariance[0]/covariance[1] < .04:
        return rejected('matches do not constrain a two-dimensional affine model', **diagnostic)
    holdout = np.arange(len(pa)) % 5 == 0
    cv2.setRNGSeed(0)
    affine, _ = cv2.estimateAffine2D(pa[~holdout], pb[~holdout], method=cv2.RANSAC,
        ransacReprojThreshold=settings.ransac_px, maxIters=2000, confidence=.995, refineIters=10)
    if affine is None: return rejected('affine estimation failed', **diagnostic)
    change = np.linalg.norm(affine[:, :2]-np.eye(2), ord=2)
    if change > settings.max_jacobian_change:
        return rejected('affine deformation exceeds small-tilt prior', jacobian_change=float(change), **diagnostic)
    residual = np.linalg.norm(pa@affine[:, :2].T+affine[:, 2]-pb, axis=1)
    inlier = residual <= settings.ransac_px
    held = residual[holdout]
    diagnostic.update(candidates=len(pa), inliers=int(inlier.sum()), heldout=len(held),
        holdout_inlier_fraction=float(inlier[holdout].mean()),
        holdout_p95_px=float(np.percentile(held, 95)), jacobian_change=float(change))
    support_checks = dict(inlier_count=int(inlier.sum()) >= settings.min_matches,
        holdout_count=len(held) >= 5, holdout_inlier_fraction=float(inlier[holdout].mean()) >= .8,
        holdout_p95=diagnostic['holdout_p95_px'] <= settings.max_holdout_p95_px)
    diagnostic['support_checks'] = support_checks
    if not all(support_checks.values()):
        return rejected('independent holdout or inlier support failed', **diagnostic)
    centre = np.array([(a.shape[1]-1)/2, (a.shape[0]-1)/2])
    centre_shift = affine[:, :2]@centre+affine[:, 2]-centre
    if np.max(abs(centre_shift)) > max_shift: return rejected('affine shift exceeds prior', **diagnostic)
    diagnostic.update(status='accepted', candidates=len(pa), inliers=int(inlier.sum()),
        heldout=len(held), holdout_p95_px=float(np.percentile(held, 95)),
        nominal_displacement_p95_px=float(np.percentile(np.linalg.norm(pb-pa, axis=1), 95)),
        centre_shift_px=centre_shift.tolist(), affine_a_to_b_px=affine.tolist(),
        jacobian_change=float(change), median_patch_ncc=float(np.median(ncc[inlier])))
    matches = dict(points_a=pa, points_b=pb, ncc=ncc, fb_error=fb, residual=residual,
                   holdout=holdout, inlier=inlier)
    return diagnostic, matches
