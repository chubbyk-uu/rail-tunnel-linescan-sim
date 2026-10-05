"""Evaluation-only whole-mosaic diagnostics; never a production rectification.

Consumes the independently traced, frozen source footprints. A constant offset
is reported separately, not used to alter the image or the acceptance evidence.
Sparse local derivatives are shape diagnostics, not subpixel crack metrology.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from .session import read_json, sha256_file
from .provenance import stage_record


def summarize(points, bands, xs, qs, radius):
    points, bands = np.asarray(points, float), np.asarray(bands)
    xs, qs = np.asarray(xs, float), np.asarray(qs, float)
    if (xs.ndim != 1 or qs.ndim != 1 or min(len(xs), len(qs)) < 3 or
            points.shape != (len(qs), len(xs), 2) or bands.shape != points.shape[:2] or
            not np.isfinite(points).all() or not np.isfinite(xs).all() or
            not np.isfinite(qs).all() or not np.all(np.diff(xs) > 0) or
            not np.all(np.diff(qs) > 0) or not np.isfinite(radius) or radius <= 0):
        raise ValueError('finite, ordered nonempty source-footprint grid required')
    valid = bands >= 0
    if not valid.all():
        raise ValueError('whole-shape diagnostics require all planned grid points; do not discard missing samples')
    xx, qq = np.meshgrid(xs, qs)
    errors = points-np.stack((xx, qq), axis=-1)
    offset = np.median(errors.reshape(-1, 2), axis=0)
    centered = errors-offset
    delta = errors[:, -1]-errors[:, 0]
    harmonic = np.column_stack((np.ones(len(qs)), np.sin(qs/radius), np.cos(qs/radius)))
    parameters = np.linalg.lstsq(harmonic, delta[:, 1], rcond=None)[0]
    # Fit describes the observed endpoint pattern only; never subtract it from
    # the product, never use a truth-derived shear as a reconstruction input.
    predicted = harmonic @ parameters
    jacobian = np.empty(points.shape[:2]+(2, 2))
    for field in range(2):
        jacobian[..., field, 0] = np.gradient(points[..., field], xs, axis=1, edge_order=2)
        jacobian[..., field, 1] = np.gradient(points[..., field], qs, axis=0, edge_order=2)
    singular = np.linalg.svd(jacobian, compute_uv=False)
    return dict(samples=points.shape[0]*points.shape[1], missing_samples=0,
        common_offset_m=offset.tolist(),
        translation_only_residual_p95_m=np.percentile(abs(centered), 95, axis=(0, 1)).tolist(),
        endpoint_mean_m=delta.mean(0).tolist(),
        endpoint_p95_abs_m=np.percentile(abs(delta), 95, axis=0).tolist(),
        axial_scale_p95_abs_percent=float(np.percentile(abs(delta[:, 0]), 95)/(xs[-1]-xs[0])*100),
        endpoint_q_harmonic_m=dict(constant=float(parameters[0]), sine=float(parameters[1]),
            cosine=float(parameters[2]), residual_rms=float(np.sqrt(np.mean((delta[:, 1]-predicted)**2)))),
        coarse_local_scale=dict(minimum=float(singular.min()), maximum=float(singular.max()),
            p95_abs_deviation=float(np.percentile(abs(singular-1), 95)),
            interpretation='sparse footprint derivatives; not crack-width or exhaustive image accuracy'),
        q_drift_by_angle_m=delta[:, 1].tolist(),
        mean_q_error_by_progress_m=errors[..., 1].mean(0).tolist())


def run(geometry, trajectory, output):
    geometry, trajectory, output = (Path(p).resolve() for p in (geometry, trajectory, output))
    if 'evaluation' not in output.parts:
        raise ValueError('shape truth must remain in evaluation/')
    if output.exists():
        raise ValueError('use a new shape evidence directory')
    source = geometry/'mapping_samples.npz'
    inputs = [geometry/'report.json', geometry/'provenance.json', source,
              trajectory/'report.json', trajectory/'trajectory.json']
    record = read_json(inputs[1])
    matches = [v for k, v in record['outputs'].items() if Path(k).name == source.name]
    if matches != [sha256_file(source)]:
        raise ValueError('source-footprint identity differs from geometry evidence')
    upstream, fit = read_json(inputs[0]), read_json(inputs[3])
    result = dict(schema='ssb.mosaic_shape_evaluation.v1', evaluation_only=True,
        status='diagnostic', grid=upstream['grid'],
        geometry_gates=upstream['gates'], observability=fit['observability'],
        limitations=['No truth-derived rectification, similarity, rotation or shear is applied.',
                     'Local seam gates do not certify absolute whole-mosaic shape.',
                     'A harmonic fit describes drift and is not an identified mechanical cause.',
                     'Coarse derivatives do not certify thin-crack width or continuity.'])
    with np.load(source) as data:
        for name in ('nominal', 'optimized'):
            result[name] = summarize(data[name+'_true_xq_m'], data[name+'_band'],
                                     data['output_x_m'], data['output_q_m'], upstream['grid']['radius_m'])
        axes = dict(output_x_m=data['output_x_m'].tolist(), output_q_m=data['output_q_m'].tolist())
    result['axes'] = axes
    output.mkdir(parents=True)
    report = output/'report.json'
    report.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    provenance = stage_record('mosaic_shape_evaluation', inputs, [report],
                              dict(reference='frozen optical-mesh source footprints', rectification=False))
    (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('geometry', 'trajectory', 'output'):
        parser.add_argument('--'+name, required=True)
    args = parser.parse_args()
    result = run(args.geometry, args.trajectory, args.output)
    print(json.dumps({name: result[name]['endpoint_mean_m'] for name in ('nominal', 'optimized')}))


if __name__ == '__main__':
    main()
