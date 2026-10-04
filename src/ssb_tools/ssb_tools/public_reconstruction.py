"""D2 + D3 run once, from staged public inputs only, under the public-input audit.

The production run itself is the isolation evidence, so no second public replay
of D3 is needed. Only copied D1 products, the observable configuration and
hash-checked raw blocks are staged; truth, scene, world and evaluation files are
never staged, and Python opens of them fail in this process and its D2 workers.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import time

from . import public_audit
from .band_matching import MatchSettings
from .global_geometry import GeometrySettings, reconstruction_settings
from .match_bands import run as match
from .optimize_bands import run as optimize
from .session import read_json, sha256_file
from .parallel_budget import resolve_workers

D1_PRODUCTS = ('report.json', 'provenance.json', 'bands.json', 'projection.npy', 'mapping.npz',
               'native_source.json')


def stage(unroll, observable, public, raw_root=None):
    d1, config, raw = public/'d1', public/'config', public/'raw'
    for path in (d1, config, raw):
        path.mkdir(parents=True)
    for name in D1_PRODUCTS:
        shutil.copyfile(Path(unroll)/name, d1/name)
    shutil.copyfile(observable, config/'observable_config.json')
    source = read_json(d1/'native_source.json')
    source_raw = Path(raw_root) if raw_root else Path(source['session_root'])/'raw'
    for block in source['blocks']:
        path = source_raw/block['file']
        if not path.resolve().is_relative_to(source_raw.resolve()) or sha256_file(path) != block['sha256']:
            raise ValueError('invalid public raw block before staging')
        # Hard links avoid a second copy of the raw blocks on the same filesystem.
        try:
            os.link(path, raw/block['file'])
        except OSError:
            shutil.copyfile(path, raw/block['file'])
    return d1, config/'observable_config.json', raw, len(source['blocks'])


def run(unroll, observable, root, raw_root=None, workers=None, spacing_m=.2,
        settings=GeometrySettings(attitude_spacing_m=.02, observed_knots=True), height=512, max_q_shift_mm=None,
        surface_relief=False):
    started = time.monotonic()
    workers = resolve_workers(workers)
    root = Path(root).resolve()
    record, matches, fit = root/'public_run', root/'matches', root/'fit'
    pose = root/'pose' if surface_relief else fit
    if type(surface_relief) is not bool:
        raise ValueError('surface_relief must be Boolean')
    if any(p.exists() for p in (record, matches, fit, pose)):
        raise ValueError('public reconstruction outputs must be fresh')
    if public_audit.private(root):
        raise ValueError('production outputs must be outside evaluation/')
    if settings.coarse_translation:
        # Resolve the installed implementation and bind its identity before the
        # data-read audit. The cached backend consumes only public arrays later.
        from .fast_normal import backend
        backend()
    d1, config, raw, blocks = stage(unroll, observable, record/'public', raw_root)
    staged_s = time.monotonic()-started
    reads = set()
    audit = public_audit.install(record/'public', raw, reads, recorded=[matches, fit, pose])
    d2 = match(d1, matches, spacing_m, height, 1024, MatchSettings(max_q_shift_mm=max_q_shift_mm), .25, raw, workers)
    if d2['status'] == 'unmeasurable':
        raise ValueError('D2 found no usable matches')
    d3 = optimize(d1, matches, config, pose, settings, raw)
    if surface_relief:
        from .surface_relief import run as estimate_relief
        d3 = estimate_relief(d1, pose, fit, raw)
    outputs = {str(path): sha256_file(path) for directory in (matches, fit)
               for path in sorted(directory.iterdir()) if path.is_file()}
    states = public_audit.verified_states(audit, d2['worker_audits'])
    report = dict(schema='ssb.public_reconstruction.v3', status='complete', audit_status='pass',
        quality_status=d3['image_consistency_gate']['status'],
        quality_scope='pose image-consistency gate only; independent geometry and coverage required separately',
        public_raw_blocks=blocks, public_files_opened=sorted(reads), outputs=outputs,
        private_input_opens=sum(s['blocked_reads'] for s in states), audit_states=states,
        d2=dict(status=d2['status'], windows=d2['windows'], matches=d2['matches']['total']),
        d3=dict(status=d3['status'], image_consistency_gate=d3['image_consistency_gate']['status'],
                surface_relief=surface_relief),
        performance=dict(staging_s=staged_s, d2_wall_s=d2['performance']['wall_s'],
                         d3_wall_s=d3['performance']['wall_s']+d3['performance'].get('relief_wall_s', 0.),
                         surface_relief_wall_s=d3['performance'].get('relief_wall_s', 0.),
                         wall_s=time.monotonic()-started),
        limitation='audit covers Python opens in this process and D2 workers; hard links retain raw contents')
    (record/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--unroll', required=True); parser.add_argument('--observable', required=True)
    parser.add_argument('--root', required=True, help='session root; writes matches/, fit/ and public_run/')
    parser.add_argument('--raw', help='relocated public raw directory of the source capture (hash-checked)')
    parser.add_argument('--workers', type=int, help='default min(8, available CPUs)')
    parser.add_argument('--spacing-m', type=float, default=.2)
    parser.add_argument('--height', type=int, default=512, help='matching window height; 256 with 0.1 m spacing keeps angular gaps')
    parser.add_argument('--max-q-shift-mm', type=float, help='explicit circumferential search prior; otherwise inherits 40 mm axial prior')
    parser.add_argument('--attitude-spacing-m', type=float, default=.02)
    parser.add_argument('--adaptive-attitude', action='store_true',
                        help='training-only bounded local refinement; use --spacing-m .1 for finer supported nodes')
    parser.add_argument('--slow-translation', action='store_true',
                        help='bounded image-derived lateral/heave on 0.6 m knots; nominal priors, no mount truth')
    parser.add_argument('--relative-encoder-scale', action='store_true',
                        help='single image-constrained relative progress scale; bounded local dx remains separate')
    parser.add_argument('--surface-relief', action='store_true', help='estimate shared radial depth from public stereo images after the pose fit')
    parser.add_argument('--strict', action='store_true', help='exit nonzero if the pose image-consistency gate is not pass')
    args = parser.parse_args()
    report = run(args.unroll, args.observable, args.root, args.raw, args.workers, args.spacing_m,
                 reconstruction_settings(args.attitude_spacing_m,args.adaptive_attitude,args.slow_translation,args.relative_encoder_scale), args.height, args.max_q_shift_mm,
                 args.surface_relief)
    print(json.dumps({k: report[k] for k in ('status', 'audit_status', 'quality_status', 'public_raw_blocks', 'd2', 'd3', 'performance')}))
    if args.strict and report['quality_status'] != 'pass': raise SystemExit(1)


if __name__ == '__main__':
    main()
