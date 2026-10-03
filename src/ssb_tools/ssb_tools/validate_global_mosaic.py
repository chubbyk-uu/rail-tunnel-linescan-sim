#!/usr/bin/env python3
"""Audit every stored mask/count/run and fixed CPU native samples; no private truth."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from ssb_tools.global_resample import load_global, corrected_tile
from ssb_tools.initial_unroll import grid_axes, release_pages, MOSAIC_INVALID, MOSAIC_SCALE
from ssb_tools.match_bands import verified_bands
from ssb_tools.public_capture import confined_file
from ssb_tools.session import read_json, sha256_file
from ssb_tools.wall_coverage import RUN_DTYPE
from ssb_tools.provenance import stage_record


def validate(unroll, trajectory, mosaic, output):
    started = time.monotonic()
    mosaic = Path(mosaic).resolve(); output = Path(output).resolve()
    if any(output.is_relative_to(Path(p).resolve()) for p in (unroll, trajectory, mosaic)):
        raise ValueError('validation output must be separate from production inputs')
    report = read_json(confined_file(mosaic, 'report.json'))
    record_path = confined_file(mosaic, 'provenance.json'); record = read_json(record_path)
    if record.get('stage') != 'global_mosaic' or report.get('schema') != 'ssb.global_mosaic.v1':
        raise ValueError('verified full-resolution D3 output required')
    for name, digest in record['outputs'].items():
        path = confined_file(mosaic, str(Path(name).relative_to(mosaic)))
        if sha256_file(path) != digest:
            raise ValueError('mosaic product hash mismatch: '+path.name)
    sampler, upstream, inputs = verified_bands(unroll)
    try:
        model, coefficients, _, trajectory_inputs = load_global(trajectory, sampler, upstream, unroll)
        for path in inputs+trajectory_inputs:
            if record['inputs'].get(str(path)) != sha256_file(path):
                raise ValueError('mosaic belongs to different verified public inputs')
        if report['grid'] != upstream['grid']:
            raise ValueError('mosaic grid differs from declared target')
        grid = report['grid']; nq, nx = grid['shape']; angles, xs = grid_axes(grid)
        values = np.load(confined_file(mosaic, 'optimized/mosaic_u16.npy'), mmap_mode='r')
        count = np.load(confined_file(mosaic, 'optimized/mosaic_count.npy'), mmap_mode='r')
        runs = np.fromfile(confined_file(mosaic, 'optimized/coverage_runs.bin'), RUN_DTYPE)
        try:
            if values.dtype != np.uint16 or count.dtype != np.uint8 or values.shape != (nq, nx) or count.shape != (nq, nx):
                raise ValueError('mosaic dimensions or storage type differ from declared grid')
            if (not len(runs) or np.any(runs['q_bin'] < 0) or np.any(runs['q_bin'] >= nq) or
                np.any(runs['x_begin'] < 0) or np.any(runs['x_end'] > nx) or
                np.any(runs['x_end'] <= runs['x_begin']) or np.any(runs['count'] < 0)):
                raise ValueError('invalid coverage runs')
            new_row = np.r_[True, runs['q_bin'][1:] != runs['q_bin'][:-1]]
            end_row = np.r_[new_row[1:], True]
            if (not np.array_equal(runs['q_bin'][new_row], np.arange(nq)) or
                    np.any(runs['x_begin'][new_row] != 0) or np.any(runs['x_end'][end_row] != nx) or
                    np.any(runs['x_begin'][1:][~new_row[1:]] != runs['x_end'][:-1][~new_row[1:]])):
                raise ValueError('coverage runs contain a gap, overlap or missing row')
            histogram = {}
            row_batch = max(1, min(64, (1 << 20)//nx))
            for a in range(0, nq, row_batch):
                b = min(nq, a+row_batch)
                part = runs[np.searchsorted(runs['q_bin'], a):np.searchsorted(runs['q_bin'], b)]
                exact = np.repeat(part['count'], part['x_end']-part['x_begin']).reshape(b-a, nx)
                if (not np.array_equal(np.minimum(exact, 255), count[a:b]) or
                        not np.array_equal(exact == 0, values[a:b] == MOSAIC_INVALID)):
                    raise ValueError('stored image/count/run disagree')
                for number, n in zip(*np.unique(exact, return_counts=True)):
                    histogram[int(number)] = histogram.get(int(number), 0)+int(n)
                release_pages(values); release_pages(count)
            stats = report['products']['optimized']['coverage']
            expected = {str(k): int(n) for k, n in sorted(histogram.items())}
            if (stats['count_histogram'] != expected or stats['total_pixels'] != nq*nx or
                    stats['missing_pixels'] != histogram.get(0, 0)):
                raise ValueError('reported coverage differs from stored pixels')
            # Fixed positions are declared here before inspecting images or errors.
            rng = np.random.default_rng(20261003)
            positions = {(0, 0), (0, nx-1), (nq-1, 0), (nq-1, nx-1)}
            positions.update(zip(rng.integers(0, nq, 64).tolist(), rng.integers(0, nx, 64).tolist()))
            maximum = 0; probes = []
            for q, x in sorted(positions):
                v, n = corrected_tile(model, coefficients, np.array([angles[q]*model.radius]), np.array([xs[x]]))
                actual = int(values[q, x]); valid = int(n[0, 0]) > 0
                expected_code = int(np.clip(np.rint(float(v[0, 0])*MOSAIC_SCALE), 0, MOSAIC_INVALID-1)) if valid else MOSAIC_INVALID
                error = abs(actual-expected_code)
                if int(count[q, x]) != min(int(n[0, 0]), 255) or ((actual != MOSAIC_INVALID) != valid) or error > 1:
                    raise ValueError(f'CPU native reference differs at ({q}, {x})')
                maximum = max(maximum, error)
                probes.append(dict(q_bin=q, x_bin=x, count=int(n[0, 0]), code_error=error))
        finally:
            values._mmap.close(); count._mmap.close()
        result = dict(schema='ssb.global_mosaic_validation.v1', status='pass',
            full_pixel_consistency=True, total_pixels=nq*nx, missing_pixels=histogram.get(0, 0),
            coverage_gate='pass' if not histogram.get(0, 0) else 'fail',
            cpu_reference=dict(seed=20261003, probes=probes, maximum_code_error=maximum,
                               tolerance_code=1, role='independent NumPy resampling, not true-geometry evaluation'),
            wall_s=time.monotonic()-started)
        output.mkdir(parents=True, exist_ok=False)
        (output/'report.json').write_text(json.dumps(result, indent=2)+'\n')
        (output/'provenance.json').write_text(json.dumps(stage_record('global_mosaic_validation',
            inputs+trajectory_inputs+[record_path, mosaic/'report.json'], [output/'report.json'],
            dict(seed=20261003, full_pixel_consistency=True)), indent=2)+'\n')
        return result
    finally:
        if hasattr(sampler.native, 'close'):
            sampler.native.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ('unroll', 'trajectory', 'mosaic', 'output'):
        parser.add_argument('--'+arg, required=True)
    result = validate(**vars(parser.parse_args()))
    print(json.dumps({k: result[k] for k in ('status', 'total_pixels', 'missing_pixels', 'coverage_gate', 'wall_s')}))
    raise SystemExit(0 if result['coverage_gate'] == 'pass' else 1)


if __name__ == '__main__':
    main()
