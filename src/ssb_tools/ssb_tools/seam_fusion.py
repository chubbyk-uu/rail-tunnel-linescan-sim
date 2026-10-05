"""Bounded public-image band gains and narrow feathering after geometric acceptance.

No offset, spatial tone field, pyramid, sharpening, hole filling or image warp.
Gain fitting uses a fixed sparse overlap grid; alternate angular rows are held out.
"""
from dataclasses import asdict, dataclass
import numpy as np
from .global_resample import sample_corrected
from .reconstruction_support import correction_reach_m


@dataclass(frozen=True)
class FusionSettings:
    feather_width_m: float = .002
    gain_limit: float = 1.08
    angular_samples: int = 64
    axial_samples: int = 65
    minimum_samples: int = 128
    minimum_dn: float = 25.
    maximum_dn: float = 230.
    maximum_gradient_dn: float = 6.

    def validate(self):
        if (not np.isfinite([self.feather_width_m, self.gain_limit, self.minimum_dn,
                             self.maximum_dn, self.maximum_gradient_dn]).all() or
                not 0 < self.feather_width_m <= .004 or not 1 <= self.gain_limit <= 1.08 or
                not 0 < self.minimum_dn < self.maximum_dn < 255 or self.maximum_gradient_dn <= 0):
            raise ValueError('invalid bounded fusion settings')
        for name, low, high in (('angular_samples', 8, 256), ('axial_samples', 9, 257),
                                ('minimum_samples', 8, 65536)):
            value = getattr(self, name)
            if type(value) is not int or not low <= value <= high:
                raise ValueError('invalid fusion sample budget')


def solve_gains(bands, edges, limit):
    """Independent connected components, zero mean log gain, explicit bounds.

    Median overlap ratios are sufficient statistics. A small identity prior
    prevents a poorly supported chain from accumulating an arbitrary tone drift.
    A component outside the gain bounds is uniformly attenuated, not locally
    clipped (which would destroy the zero-mean gauge).
    """
    logs = np.zeros(bands); seen = set(); components = []
    for first in range(bands):
        if first in seen: continue
        component = {first}; pending = [first]
        while pending:
            band = pending.pop()
            for a, b, _ in edges:
                if band in (a, b):
                    other = b if band == a else a
                    if other not in component:
                        component.add(other); pending.append(other)
        ids = sorted(component); seen.update(ids)
        if len(ids) == 1:
            components.append(dict(bands=ids, measured=False, attenuation=1.)); continue
        index = {band: i for i, band in enumerate(ids)}
        chosen = [edge for edge in edges if edge[0] in component]
        matrix = np.zeros((len(chosen), len(ids))); target = []
        for row, (a, b, ratio) in enumerate(chosen):
            matrix[row, index[a]] = 1.; matrix[row, index[b]] = -1.; target.append(ratio)
        # log(gain_a)-log(gain_b)=log(value_b/value_a)
        value = np.linalg.solve(matrix.T@matrix+.01*np.eye(len(ids)), matrix.T@target)
        value -= value.mean()
        largest = np.max(abs(value), initial=0.)
        attenuation = min(1., np.log(limit)/largest) if largest else 1.
        logs[ids] = value*attenuation
        components.append(dict(bands=ids, measured=True, attenuation=float(attenuation)))
    return np.exp(logs), components


def estimate(model, coefficients, grid, settings=FusionSettings()):
    settings.validate()
    n = len(model.sampler.segments); edges = []; records = []; holdouts = []
    sampler = model.sampler
    offsets = sampler.output_offsets[sampler.geometry_valid]
    if len(offsets) < 2: raise ValueError('no valid radiometric footprint')
    # Fixed angular/axial fractions chosen without inspecting output or truth.
    theta0, theta1 = grid['theta_rad']
    angles = theta0+(np.arange(settings.angular_samples)+.5)*(theta1-theta0)/settings.angular_samples
    fractions = (np.arange(settings.axial_samples)+.5)/settings.axial_samples
    for band in range(n-1):
        data = []
        for row, angle in enumerate(angles):
            axes = []
            for k in (band, band+1):
                a, b = sampler.bounds[k]
                axes.append(np.interp(angle, sampler.phases[k], sampler.projection['x_axis_m'][a:b]))
            left = max(axes)+offsets[0]+.002
            right = min(axes)+offsets[-1]-.002
            left = max(left, grid['target_x_m'][0]); right = min(right, grid['target_x_m'][1])
            if right <= left: continue
            xs = left+fractions*(right-left)
            va, oka, _ = sample_corrected(model, coefficients, band, np.array([angle*model.radius]), xs)
            vb, okb, _ = sample_corrected(model, coefficients, band+1, np.array([angle*model.radius]), xs)
            va, vb = va[0], vb[0]
            valid = oka[0] & okb[0] & np.isfinite(va) & np.isfinite(vb)
            valid &= ((va >= settings.minimum_dn) & (vb >= settings.minimum_dn) &
                      (va <= settings.maximum_dn) & (vb <= settings.maximum_dn))
            # Reject edges/grooves by public image gradients, for radiometry only.
            # These pixels remain in coverage, geometry checks and final output.
            ga = np.maximum(np.r_[np.inf, abs(np.diff(va))], np.r_[abs(np.diff(va)), np.inf])
            gb = np.maximum(np.r_[np.inf, abs(np.diff(vb))], np.r_[abs(np.diff(vb)), np.inf])
            valid &= (ga <= settings.maximum_gradient_dn) & (gb <= settings.maximum_gradient_dn)
            if valid.any():
                data.append((row, va[valid].copy(), vb[valid].copy()))
        train = [np.log(b/a) for row, a, b in data if row % 2 == 0]
        count = sum(len(v) for v in train)
        entry = dict(bands=[band, band+1], training_samples=count,
                     heldout_samples=sum(len(a) for row, a, _ in data if row % 2 == 1))
        if count >= settings.minimum_samples:
            ratios = np.concatenate(train); ratio = float(np.median(ratios))
            edges.append((band, band+1, ratio))
            entry.update(status='measured', median_log_b_over_a=ratio,
                         log_ratio_mad=float(np.median(abs(ratios-ratio))))
        else:
            entry.update(status='insufficient_support')
        records.append(entry); holdouts.append([(a, b) for row, a, b in data if row % 2 == 1])
        sampler.native.release()
    if not edges:
        raise ValueError('no measurable overlap brightness; fusion calibration is unmeasurable')
    gains, components = solve_gains(n, edges, settings.gain_limit)
    before = []; after = []
    for band, (record, samples) in enumerate(zip(records, holdouts)):
        original = [abs(b-a) for a, b in samples]
        corrected = [abs(b*gains[band+1]-a*gains[band]) for a, b in samples]
        if original:
            x = np.concatenate(original); y = np.concatenate(corrected)
            record['heldout'] = dict(before_median_dn=float(np.median(x)), after_median_dn=float(np.median(y)),
                                     before_p95_dn=float(np.percentile(x, 95)), after_p95_dn=float(np.percentile(y, 95)))
            if record['status'] == 'measured': before.append(x); after.append(y)
    if not before: raise ValueError('no heldout overlap evidence for brightness calibration')
    x, y = np.concatenate(before), np.concatenate(after)
    return dict(schema='ssb.seam_fusion.v1', settings=asdict(settings), gains=gains.tolist(),
        components=components, pairs=records, sampling=dict(angular='fixed cell-centre fractions; even rows train, odd rows heldout',
            axial='fixed fractions of public nominal overlap clipped to output ROI',
            exclusions='invalid/saturated, DN outside range and high image gradient; radiometry only'),
        heldout=dict(samples=len(x), before_median_dn=float(np.median(x)), after_median_dn=float(np.median(y)),
                     before_p95_dn=float(np.percentile(x, 95)), after_p95_dn=float(np.percentile(y, 95))),
        gauge='each measured component has geometric-mean gain 1; unsupported isolated bands have gain 1',
        limitations=['one constant gain per band cannot remove directional normal-map shading',
                     'heldout brightness is not geometric accuracy or sharpness acceptance'])


def calibration_values(record, model):
    if record.get('schema') != 'ssb.seam_fusion.v1': raise ValueError('unsupported seam fusion calibration')
    settings = FusionSettings(**record['settings']); settings.validate()
    if np.any(np.diff(model.sampler.segments) != 1):
        raise ValueError('fusion requires consecutive observed scan segments; cannot bridge a missing circle')
    gains = np.asarray(record['gains'], float)
    if (gains.shape != (len(model.sampler.segments),) or not np.isfinite(gains).all() or
            np.any(gains < 1/settings.gain_limit-1e-12) or np.any(gains > settings.gain_limit+1e-12)):
        raise ValueError('invalid bounded per-band gains')
    width = settings.feather_width_m/float(np.median(np.diff(model.sampler.offsets)))
    if not 0 < width <= len(model.sampler.offsets)/4:
        raise ValueError('feather exceeds measured footprint budget')
    return gains, width


class CpuFusionRaster:
    """Independent NumPy source selection and two-observation narrow feather reference."""
    def __init__(self, model, coefficients, calibration):
        self.model, self.coefficients = model, coefficients
        self.gains, self.width = calibration_values(calibration, model)
        self.band_x = model.band_extents(coefficients)
        self.margin = correction_reach_m(model.radius, model.height)

    def candidate_bands(self, xs):
        return [i for i, (lo, hi) in enumerate(self.band_x)
                if hi+self.margin >= xs[0] and lo-self.margin <= xs[-1]]

    def components(self, angles, xs, bands=None):
        shape = (len(angles), len(xs))
        image = np.full(shape, np.nan, np.float32); second = image.copy()
        best = np.full(shape, -np.inf, np.float32); runner = best.copy()
        source = np.full(shape, -1, np.int16); other = source.copy(); count = np.zeros(shape, np.uint16)
        for band in (self.candidate_bands(xs) if bands is None else bands):
            lo, hi = self.band_x[band]
            left = int(np.searchsorted(xs, lo-self.margin)); right = int(np.searchsorted(xs, hi+self.margin, side='right'))
            if right <= left: continue
            sl = np.s_[:, left:right]
            result, valid, score = sample_corrected(self.model, self.coefficients, band,
                                                   np.asarray(angles)*self.model.radius, xs[left:right])
            result = (result.astype(np.float64)*self.gains[band]).astype(np.float32)
            better = valid & (score > best[sl]); next_best = valid & ~better & (score > runner[sl])
            second[sl][better] = image[sl][better]; runner[sl][better] = best[sl][better]; other[sl][better] = source[sl][better]
            second[sl][next_best] = result[next_best]; runner[sl][next_best] = score[next_best]; other[sl][next_best] = band
            image[sl][better] = result[better]; best[sl][better] = score[better]; source[sl][better] = band
            count[sl] += valid
        difference = np.full(shape, np.inf, float)
        both = other >= 0
        difference[both] = best[both].astype(float)-runner[both]
        blend = both & (abs(source-other) == 1) & (difference < self.width)
        weight = np.ones(shape, float); weight[blend] = .5+difference[blend]/(2*self.width)
        image[blend] = (image[blend].astype(float)*weight[blend]+second[blend].astype(float)*(1-weight[blend])).astype(np.float32)
        return image, count, source, other, weight

    def tile(self, angles, xs, bands=None):
        return self.components(angles, np.asarray(xs), bands)[:3]

    def close(self): pass

    def describe(self):
        return dict(name='cpu', geometry_precision='float64', fusion_abi=1,
                    fusion='independent NumPy two-best sources; narrow linear feather',
                    surface_relief=self.model.relief is not None)


def run(unroll, trajectory, baseline, output, raw_root=None, backend='cuda', settings=FusionSettings()):
    """Generate a separate fused product; never rewrite the accepted unblended data."""
    import json
    from pathlib import Path
    import time
    from .global_mosaic import run as mosaic_run
    from .global_resample import load_global
    from .match_bands import verified_bands
    from .public_capture import confined_file
    from .session import read_json, sha256_file
    from .provenance import stage_record
    started = time.monotonic()
    baseline = Path(baseline).resolve(); output = Path(output).resolve()
    settings.validate()
    if output.exists() or any(output.is_relative_to(Path(p).resolve()) for p in (unroll, trajectory, baseline)):
        raise ValueError('fresh separate fusion output required')
    baseline_report = read_json(confined_file(baseline, 'report.json'))
    baseline_record = read_json(confined_file(baseline, 'provenance.json'))
    if (baseline_record.get('stage') != 'global_mosaic' or baseline_report.get('fusion') is not None or
            baseline_report.get('coverage_gate', {}).get('status') != 'pass'):
        raise ValueError('complete unblended baseline required before fusion')
    baseline_inputs = [confined_file(baseline, name) for name in
        ('report.json', 'provenance.json', 'optimized/mosaic_u16.npy',
         'optimized/mosaic_count.npy', 'optimized/coverage_runs.bin')]
    for path in baseline_inputs[2:]:
        if baseline_record['outputs'].get(str(path)) != sha256_file(path):
            raise ValueError('unblended baseline product hash mismatch')
    sampler, upstream, inputs = verified_bands(unroll, raw_root)
    try:
        model, coefficients, _, trajectory_inputs = load_global(trajectory, sampler, upstream, unroll)
        if baseline_report['grid'] != upstream['grid']:
            raise ValueError('baseline grid differs from fusion inputs')
        for path in inputs+trajectory_inputs:
            if baseline_record['inputs'].get(str(path)) != sha256_file(path):
                raise ValueError('baseline does not belong to these public observations/trajectory')
        calibration = estimate(model, coefficients, upstream['grid'], settings)
        calibration['baseline'] = dict(root=str(baseline), report_sha256=sha256_file(baseline_inputs[0]),
                                       provenance_sha256=sha256_file(baseline_inputs[1]))
    finally:
        if hasattr(sampler.native, 'close'): sampler.native.close()
    calibration_s = time.monotonic()-started
    report = mosaic_run(unroll, trajectory, output, raw_root, backend, comparison=False, fusion=calibration)
    # Exact full-grid equality, not a preview or handful of probes. The same
    # geometry and validity predicate must yield byte-identical counts/runs.
    for name in ('mosaic_count.npy', 'coverage_runs.bin'):
        if sha256_file(output/'optimized'/name) != sha256_file(baseline/'optimized'/name):
            raise ValueError('fusion changed full-grid observation coverage')
    result = dict(schema='ssb.seam_fusion_acceptance.v1', status='pass',
        unchanged_coverage=True, unchanged_trajectory=True, baseline=str(baseline),
        heldout=calibration['heldout'], settings=asdict(settings),
        performance=dict(calibration_and_identity_s=calibration_s,
                         mosaic_s=report['performance']['wall_s'], wall_s=time.monotonic()-started),
        limitations=['radiometric/coverage checks; geometric acceptance remains the unfused result',
                     'no independent sharpness measurement here; compare native crack/joint crops'])
    path = output/'fusion_report.json'; path.write_text(json.dumps(result, indent=2)+'\n')
    record = stage_record('seam_fusion', baseline_inputs+inputs+trajectory_inputs,
                         [output/'provenance.json', output/'report.json', path],
                         dict(settings=asdict(settings), backend=backend))
    (output/'fusion_provenance.json').write_text(json.dumps(record, indent=2)+'\n')
    return result


def main():
    import argparse
    import ctypes as ct
    import json
    from pathlib import Path
    import sys
    from ament_index_python.packages import get_package_prefix
    from . import public_audit
    from .session import sha256_file
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('unroll', 'trajectory', 'baseline', 'output', 'raw', 'public_root'):
        parser.add_argument('--'+name.replace('_', '-'), required=True)
    parser.add_argument('--backend', choices=('cuda', 'cpu'), default='cuda')
    args = parser.parse_args(); sys.excepthook = sys.__excepthook__
    # Prime native implementation identity before installing a data-only audit;
    # the explicitly recorded shared library is a code input, not scene data.
    code_inputs = []
    if args.backend == 'cuda':
        library = Path(get_package_prefix('ssb_core'))/'lib/libssb_unroll_cuda.so'
        ct.CDLL(str(library)); sha256_file(library)
        # ament reopens this exact installed code locator when a raster is built.
        # Admit the locator itself, never the whole install/ tree.
        resource = library.parent.parent/'share/ament_index/resource_index/packages/ssb_core'
        code_inputs.extend([library, resource])
    for folder in (args.unroll, args.trajectory, args.baseline, args.output, args.raw):
        if public_audit.private(Path(folder).resolve()):
            raise ValueError('fusion inputs/output must be outside private evaluation/')
    reads = set()
    audit = public_audit.install(args.public_root, args.raw, reads,
                                recorded=[args.unroll, args.trajectory, args.baseline, args.output, *code_inputs])
    result = run(args.unroll, args.trajectory, args.baseline, args.output, args.raw, args.backend)
    public_audit.verified_states(audit, [])
    evidence = dict(state=dict(audit), public_files_opened=sorted(reads),
                    private_input_opens=audit['blocked_reads'],
                    scope='Python-level opens in this process; native/external I/O not fully covered')
    (Path(args.output)/'public_audit.json').write_text(json.dumps(evidence, indent=2)+'\n')
    print(json.dumps(result))



def reference_positions(model, coefficients, grid, maximum_pairs=8):
    """Fixed public rule ensures CPU probes actually exercise the narrow seam.

    Locate feathered cells using public geometry before inspecting stored images.
    This is a source/weight implementation check, not a truth accuracy check.
    """
    from .initial_unroll import grid_axes
    angles, xs = grid_axes(grid); sampler = model.sampler
    pairs = np.unique(np.rint(np.linspace(0, len(sampler.segments)-2,
                                        min(maximum_pairs, len(sampler.segments)-1))).astype(int))
    q_bins = np.unique(np.rint(np.linspace(0, len(angles)-1, 4)).astype(int))
    # Unit gains do not affect source scores or feather weights.
    calibration = dict(schema='ssb.seam_fusion.v1', settings=asdict(FusionSettings()),
                       gains=[1.]*len(sampler.segments))
    reference = CpuFusionRaster(model, coefficients, calibration); positions = set()
    for band in pairs:
        for qi in q_bins:
            angle = angles[qi]; centres = []
            for k in (band, band+1):
                a, b = sampler.bounds[k]
                axis = np.interp(angle, sampler.phases[k], sampler.projection['x_axis_m'][a:b])
                nominal = axis+(sampler.offsets[0]+sampler.offsets[-1])/2
                point, _ = model.forward(k, np.array([nominal]), np.array([angle*model.radius]), coefficients)
                centres.append(point[0, 0])
            middle = np.mean(centres)
            xa = max(0, int(np.searchsorted(xs, middle-.04)))
            xb = min(len(xs), int(np.searchsorted(xs, middle+.04)))
            if xa >= xb: continue
            columns = np.arange(xa, xb, max(1, int(.0004/grid['dx_m'])))
            _, count, _, _, weight = reference.components(np.array([angle]), xs[columns])
            valid = (count[0] > 1) & (weight[0] < 1)
            if valid.any():
                selected = np.flatnonzero(valid)[np.argmin(weight[0, valid])]
                positions.add((int(qi), int(columns[selected])))
            sampler.native.release()
    return positions


if __name__ == '__main__': main()
