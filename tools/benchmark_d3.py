#!/usr/bin/env python3
"""Profile installed D3 on verified public D1/D2; no capture or matching rerun."""
import argparse
import functools
import hashlib
import json
from pathlib import Path
import sys
import time

from ssb_tools import global_geometry as geometry, optimize_bands as optimizer, public_audit
from ssb_tools.fast_normal import backend
from ssb_tools.global_geometry import reconstruction_settings


def main():
    # Ubuntu's optional exception hook reads unrelated host files under the audit.
    sys.excepthook = sys.__excepthook__
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--public', required=True, help='existing public_run/public folder')
    parser.add_argument('--matches', required=True, help='verified D2 folder')
    parser.add_argument('--output', required=True, help='fresh output folder')
    parser.add_argument('--adaptive-attitude', action='store_true')
    parser.add_argument('--slow-translation', action='store_true')
    parser.add_argument('--relative-encoder-scale', action='store_true')
    parser.add_argument('--attitude-spacing-m', type=float, default=.02)
    parser.add_argument('--geometry-backend', choices=('numpy', 'cpu', 'cuda'), default='cuda')
    parser.add_argument('--geometry-workers', type=int, help='explicit CPU geometry thread budget')
    args = parser.parse_args()
    public, matches, output = (Path(p).resolve() for p in (args.public, args.matches, args.output))
    if output.is_relative_to(public) or output.is_relative_to(matches):
        parser.error('output must be separate from public inputs')
    output.mkdir(exist_ok=False, parents=True)
    sources = [Path(module.__file__).resolve() for module in (optimizer, geometry)]
    from ssb_tools import fast_geometry
    sources.append(Path(fast_geometry.__file__).resolve())
    sources.append(Path(__file__).resolve())
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    backend()  # Bind the installed CPU implementation before restricting reads.
    if args.geometry_backend != 'numpy':
        from ssb_tools.fast_geometry import backend as ray_backend
        ray_backend()
        if args.geometry_backend == 'cuda':
            from ssb_tools.fast_geometry import cuda_backend
            cuda_backend()
    if args.geometry_workers is not None:
        from ssb_tools.parallel_budget import resolve_workers
        geometry.THREADS = resolve_workers(args.geometry_workers)
    metrics, passes = {}, []

    def timed(obj, name, key=None):
        original = getattr(obj, name)
        label = key or name
        @functools.wraps(original)
        def wrapped(*args, **kwargs):
            started = time.monotonic()
            try:
                return original(*args, **kwargs)
            finally:
                value = metrics.setdefault(label, dict(calls=0, wall_s=0.))
                value['calls'] += 1
                value['wall_s'] += time.monotonic()-started
        setattr(obj, name, wrapped)

    for name in ('window_weights', 'observability', 'normal_equations', 'bounded_normal_step',
                 'symmetric_normal_solve', 'damped_solve', 'verified_matches'):
        if hasattr(optimizer, name):
            timed(optimizer, name)
    timed(optimizer.FixedJacobian, '__init__', 'jacobian_plan')
    timed(optimizer.FixedJacobian, '__call__', 'jacobian_assembly')
    timed(optimizer.FixedJacobian, 'difference', 'training_ray_difference')
    timed(optimizer.FixedJacobian, 'parameters', 'training_ray_parameters')
    for name in ('hits', 'derivatives', 'jacobian'):
        timed(geometry.RaySet, name, 'ray_'+name)
    timed(geometry.Trajectory, 'native_side', 'native_sources')
    original_fit = optimizer.fit

    @functools.wraps(original_fit)
    def fitting(model, *args, **kwargs):
        prior = {key: value.copy() for key, value in metrics.items()}
        started = time.monotonic()
        result = original_fit(model, *args, **kwargs)
        delta = {key: dict(calls=value['calls']-prior.get(key, {}).get('calls', 0),
                           wall_s=value['wall_s']-prior.get(key, {}).get('wall_s', 0.))
                 for key, value in metrics.items()}
        passes.append(dict(size=model.size, wall_s=time.monotonic()-started,
                           solver=result[3], metrics=delta))
        return result

    optimizer.fit = fitting
    reads = set()
    audit = public_audit.install(public, public/'raw', reads, recorded=[matches, output])
    started = time.monotonic()
    settings = reconstruction_settings(args.attitude_spacing_m, args.adaptive_attitude,
                                       args.slow_translation, args.relative_encoder_scale, args.geometry_backend)
    report = optimizer.run(public/'d1', matches, public/'config/observable_config.json',
                           output/'pose', settings, public/'raw')
    public_audit.verified_states(audit, [])
    summary = dict(
        scope='Same-data performance regression, not an unseen holdout. Timers are inclusive '
              'and nested; do not sum all metrics.',
        geometry_threads=geometry.THREADS, wall_s=time.monotonic()-started,
        loaded_source_hashes=hashes, metrics=metrics, passes=passes, audit=dict(audit),
        private_input_opens=audit['blocked_reads'], image_consistency=report['image_consistency'],
        coefficients=report['coefficients'], performance=report['performance'])
    (output/'profile.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(dict(wall_s=summary['wall_s'], metrics=metrics,
                          private_input_opens=audit['blocked_reads'])))


if __name__ == '__main__':
    main()
