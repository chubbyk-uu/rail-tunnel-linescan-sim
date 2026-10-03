"""Private holdout declaration and evidence checks; never a reconstruction input."""
import argparse
from dataclasses import asdict
import datetime
import json
from pathlib import Path
import subprocess

import yaml

from .band_matching import MatchSettings
from .global_geometry import GeometrySettings
from .optical_identity import check_calibration
from .session import Session, read_json, sha256_file
from .evaluate_global_geometry import SAMPLING_SCHEMA


def evaluation_path(path):
    path = Path(path).resolve()
    if 'evaluation' not in path.parts:
        raise ValueError('protocol records belong in evaluation/')
    return path


def declare(workspace, demo, output, start, length):
    workspace, demo, output = Path(workspace).resolve(), Path(demo).resolve(), evaluation_path(output)
    state = subprocess.check_output(['sh', str(workspace/'src/ssb_core/cmake/source_state.sh'),
                                     str(workspace)], text=True).split()
    if state[1] != '0':
        raise ValueError('commit changes before declaring a frozen holdout')
    if output.exists():
        raise ValueError('a protocol cannot be redeclared or overwritten')
    from .mission_plan import wall_plan
    config = yaml.safe_load((demo/'capture.yaml').read_text())
    wall_plan(config, start, length, read_json(demo/'calibration.json'))
    check_calibration(demo/'capture.yaml', demo/'calibration.json')
    settings = GeometrySettings(attitude_spacing_m=.02, observed_knots=True)
    sources = [workspace/'src/ssb_tools/ssb_tools'/name for name in
               ('match_bands.py', 'band_matching.py', 'matching_structures.py', 'optimize_bands.py',
                'global_geometry.py', 'initial_unroll.py', 'global_resample.py',
                'evaluate_global_geometry.py', 'public_audit.py', 'public_reconstruction.py')]
    record = dict(schema='ssb.d3_holdout_protocol.v4',
        declared_at=datetime.datetime.now(datetime.timezone.utc).isoformat(), code_commit=state[0],
        holdout_roi_m=[start, start+length],
        d2=dict(spacing_m=.2, height=512, max_width=1024, halo_m=.25, settings=asdict(MatchSettings())),
        d3=asdict(settings), sampling=dict(schema=SAMPLING_SCHEMA, spacing_q_m=.2,
            phase_fractions=[.25, .75], samples_across=9, column_guard_pixels=2,
            original_nominal_probes_retained=True, angular_gaps_not_trimmed=True,
            outside_target_requires_observed_common_interval=True,
            exact_plan_saved_before_truth=True),
        required_evidence=['binary_matches_source', 'stage_b_acceptance', 'public_only_production_run'],
        input_hashes={str(demo/name): sha256_file(demo/name)
                      for name in ('capture.yaml', 'calibration.json', 'bundle.json')},
        production_sources={str(p): sha256_file(p) for p in sources})
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(record, indent=2)+'\n')
    return record


def capture_checks(session, expected_commit):
    path = session.root/'config/provenance.json'
    provenance = read_json(path)
    identity_ok = session.summary.get('files', {}).get('config/provenance.json') == sha256_file(path)
    build, run = provenance.get('build', {}), provenance.get('source_at_run', {})
    matches = (identity_ok and provenance.get('binary_matches_source') is True and
               build.get('git_head') == run.get('git_head') == expected_commit and
               isinstance(build.get('source_digest'), str) and
               len(build['source_digest']) == 64 and build['source_digest'] == run.get('source_digest'))
    report_path = session.root/'evaluation/reports/stage_b_smoke.json'
    report = read_json(report_path) if report_path.exists() else {}
    checks = report.get('checks', [])
    # An overall pass with absent, duplicated, failed or unmeasurable checks is
    # not acceptable evidence. In particular a missing replay must not pass.
    required = {'binary_matches_source', 'stored_hashes_verify', 'reimaging_byte_identical'}
    names = [c.get('name') for c in checks]
    stage_b_ok = (report.get('schema') == 'ssb.stage_b_smoke_report.v1' and
                  report.get('overall') == 'pass' and required.issubset(names) and
                  len(names) == len(set(names)) and all(c.get('state') == 'pass' for c in checks))
    return dict(capture_provenance_hash_valid=identity_ok, binary_matches_source=matches,
                stage_b_acceptance=bool(stage_b_ok))


def public_run_valid(root):
    """The scored D2/D3 products are exactly those of the audited public-only run."""
    path = root/'public_run/report.json'
    if not path.is_file():
        return False
    report = read_json(path)
    outputs = report.get('outputs', {})
    current = {str(p.resolve()): sha256_file(p) for name in ('matches', 'fit')
               for p in sorted((root/name).iterdir()) if p.is_file()} if all(
                   (root/name).is_dir() for name in ('matches', 'fit')) else {}
    return (report.get('schema') == 'ssb.public_reconstruction.v1' and report.get('status') == 'pass' and
            report.get('private_input_opens') == 0 and bool(current) and
            {str(Path(k).resolve()): v for k, v in outputs.items()} == current)


def verify(protocol_file, root, output):
    protocol_file, root, output = Path(protocol_file), Path(root), evaluation_path(output)
    if output.exists():
        raise ValueError('verification records must be fresh; preserve earlier evidence')
    protocol = read_json(protocol_file)
    session = Session(root/'capture')
    provenance = read_json(session.root/'config/provenance.json')
    stages = [read_json(root/name/'provenance.json')['source'] for name in ('unroll', 'matches', 'fit')]
    source = provenance.get('source_at_run', {})
    def hashes_match(entries):
        return bool(entries) and all(Path(p).is_file() and sha256_file(p) == digest
                                     for p, digest in entries.items())
    checks = dict(
        clean_worktree=all(s.get('git_dirty') is False for s in [source, *stages]),
        code_commit_unchanged=all(s.get('git_head') == protocol['code_commit'] for s in [source, *stages]),
        d2_settings_unchanged=read_json(root/'matches/report.json')['settings'] == protocol['d2']['settings'],
        d3_settings_unchanged=read_json(root/'fit/report.json')['settings'] == protocol['d3'],
        holdout_roi_correct=read_json(root/'unroll/report.json')['grid']['target_x_m'] == protocol['holdout_roi_m'],
        input_hashes_unchanged=hashes_match(protocol['input_hashes']),
        production_sources_unchanged=hashes_match(protocol['production_sources']))
    if protocol.get('schema') == 'ssb.d3_holdout_protocol.v4':
        checks['public_only_production_run'] = public_run_valid(root)
    else:
        checks['public_replay_identical'] = read_json(root/'public_replay/report.json').get(
            'geometry_and_residuals_identical') is True
    checks.update(capture_checks(session, protocol['code_commit']))
    if protocol.get('schema') in ('ssb.d3_holdout_protocol.v3', 'ssb.d3_holdout_protocol.v4'):
        checks['sampling_protocol_unchanged'] = protocol['sampling'] == dict(
            schema=SAMPLING_SCHEMA, spacing_q_m=.2, phase_fractions=[.25, .75], samples_across=9,
            column_guard_pixels=2, original_nominal_probes_retained=True,
            outside_target_requires_observed_common_interval=True,
            angular_gaps_not_trimmed=True, exact_plan_saved_before_truth=True)
    checks['capture_complete'] = session.summary.get('status') == 'complete'
    report = dict(schema='ssb.d3_holdout_verification.v2', status='pass' if all(checks.values()) else 'fail',
        checks=checks, protocol_sha256=sha256_file(protocol_file),
        capture_build=provenance.get('build'), capture_source_at_run=source,
        interpretation='unchanged C++ code is explanatory evidence, not an exemption from build identity')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    declaration = sub.add_parser('declare')
    for name in ('workspace', 'demo', 'output'):
        declaration.add_argument('--'+name, required=True)
    declaration.add_argument('--start', type=float, required=True)
    declaration.add_argument('--length', type=float, required=True)
    verification = sub.add_parser('verify')
    for name in ('protocol', 'root', 'output'):
        verification.add_argument('--'+name, required=True)
    args = parser.parse_args()
    if args.action == 'declare':
        declare(args.workspace, args.demo, args.output, args.start, args.length)
        return 0
    report = verify(args.protocol, args.root, args.output)
    print(json.dumps(report))
    return 0 if report['status'] == 'pass' else 1


if __name__ == '__main__':
    raise SystemExit(main())
