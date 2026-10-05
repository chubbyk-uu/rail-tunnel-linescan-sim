#!/usr/bin/env python3
"""CPU kernel thread probe on verified public matches, not a full D3 timing."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from ssb_tools import global_geometry as geometry, optimize_bands as optimizer, public_audit
from ssb_tools.fast_geometry import backend
from ssb_tools.match_bands import verified_bands
from ssb_tools.parallel_budget import resolve_workers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--public', required=True)
    parser.add_argument('--matches', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    public, matches, output = (Path(p).resolve() for p in (args.public, args.matches, args.output))
    if output.is_relative_to(public) or output.is_relative_to(matches):
        parser.error('output must be separate from public inputs')
    output.mkdir(parents=True, exist_ok=False)
    identity = backend()[1]
    audit = public_audit.install(public, public/'raw', set(), recorded=[matches, output])
    sampler, upstream, _ = verified_bands(public/'d1', public/'raw')
    saved_threads = geometry.THREADS
    try:
        table, _, _ = optimizer.verified_matches(matches, public/'d1', upstream, sampler)
        training = table[table['inlier'].astype(bool) & ~table['holdout'].astype(bool)]
        selected = training[np.linspace(0, len(training)-1, min(len(training), 262144), dtype=int)]
        height, _ = optimizer.public_robot(public/'config/observable_config.json',
            upstream['source_observation_hashes']['config/observable_config.json'], upstream['grid']['radius_m'])
        model = geometry.Trajectory(sampler, upstream['grid']['radius_m'], height,
                                   geometry.reconstruction_settings(.02, True, True, True, 'cpu'))
        plan = optimizer.FixedJacobian(model.native_side(selected, 'a'), model.native_side(selected, 'b'),
                                       optimizer.regularizer(model))
        coefficients = np.sin(np.arange(model.size)+.3)*.7
        current = np.ones((len(selected), 2))
        pitch = np.array([upstream['grid']['dx_m'], upstream['grid']['dq_m']])
        plan.parameters(coefficients)  # Hold serial spline work fixed in this kernel-only probe.
        measurements, reference = [], None
        for requested in (4, 8, 16):
            geometry.THREADS = resolve_workers(requested)
            samples = []
            for repeat in range(4):
                started = time.monotonic()
                actual = plan(coefficients, current, pitch)
                difference = plan.numeric.difference(plan.parameters(coefficients))
                elapsed = time.monotonic()-started
                if reference is None:
                    reference = (actual.data.copy(), difference.copy())
                else:
                    np.testing.assert_array_equal(actual.data, reference[0])
                    np.testing.assert_array_equal(difference, reference[1])
                if repeat: samples.append(elapsed)
            measurements.append(dict(workers=requested, kernel_wall_s=samples, median_s=float(np.median(samples))))
        public_audit.verified_states(audit, [])
        report = dict(scope='representative CPU kernels only; cached spline parameters, not full D3',
                      public_training_matches=len(selected), backend=identity, measurements=measurements,
                      private_input_opens=audit['blocked_reads'], unchanged_numeric_arrays=True)
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report))
    finally:
        geometry.THREADS = saved_threads
        sampler.native.close()


if __name__ == '__main__':
    main()
