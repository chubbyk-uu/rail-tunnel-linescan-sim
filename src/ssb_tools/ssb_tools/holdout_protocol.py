"""Private holdout declaration and evidence checks; never a reconstruction input."""
import argparse
from dataclasses import asdict
import datetime
import json
from pathlib import Path
import subprocess

import numpy as np
import yaml

from .band_matching import MatchSettings
from .global_geometry import add_reconstruction_arguments, reconstruction_settings
from .optical_identity import check_calibration
from .session import Session, read_json, sha256_file
from .evaluate_global_geometry import SAMPLING_SCHEMA
from .public_audit import POLICY
from .reconstruction_support import matching_halo_m


def evaluation_path(path):
    path = Path(path).resolve()
    if 'evaluation' not in path.parts:
        raise ValueError('protocol records belong in evaluation/')
    return path


def declare(workspace, demo, output, start, length, spacing_m=.2, adaptive_attitude=False, height=512, max_q_shift_mm=None,
            surface_relief=False, slow_translation=False, relative_encoder_scale=False, geometry_backend='cpu',
            fit_axis_yaw=None):
    workspace, demo, output = Path(workspace).resolve(), Path(demo).resolve(), evaluation_path(output)
    state = subprocess.check_output(['sh', str(workspace/'src/ssb_core/cmake/source_state.sh'),
                                     str(workspace)], text=True).split()
    if state[1] != '0':
        raise ValueError('commit changes before declaring a frozen holdout')
    if output.exists():
        raise ValueError('a protocol cannot be redeclared or overwritten')
    from .mission_plan import wall_plan
    config = yaml.safe_load((demo/'capture.yaml').read_text())
    wall_plan(config, start, length, read_json(demo/'calibration.json'), relative_encoder_scale=relative_encoder_scale)
    check_calibration(demo/'capture.yaml', demo/'calibration.json')
    settings = reconstruction_settings(.02, adaptive_attitude, slow_translation, relative_encoder_scale,
                                       geometry_backend, fit_axis_yaw)
    settings.validate()
    if not isinstance(spacing_m, (int, float)) or not 0 < spacing_m <= .4:
        raise ValueError('holdout matching spacing must be positive and at most 0.4 m')
    if type(height) is not int or height not in (256, 512):
        raise ValueError('holdout matching height must be 256 or 512 pixels')
    match_settings = MatchSettings(max_q_shift_mm=max_q_shift_mm)
    sources = [workspace/'src/ssb_tools/ssb_tools'/name for name in
               ('match_bands.py', 'band_matching.py', 'matching_structures.py', 'optimize_bands.py',
                'global_geometry.py', 'initial_unroll.py', 'global_resample.py', 'reconstruction_support.py',
                'reconstruction_budget.py', 'mission_plan.py', 'global_cuda.py', 'global_mosaic.py',
                'evaluate_global_geometry.py', 'public_audit.py', 'public_reconstruction.py', 'parallel_budget.py',
                'validate_stage_b.py', 'validate_stage_a.py', 'ref_geometry.py', 'session.py')]
    if type(surface_relief) is not bool:
        raise ValueError('surface_relief must be Boolean')
    if surface_relief:
        sources.append(workspace/'src/ssb_tools/ssb_tools/surface_relief.py')
    if slow_translation:
        sources.extend([workspace/'src/ssb_tools/ssb_tools/fast_normal.py',
                        workspace/'src/ssb_core/src/normal_equations.cpp',workspace/'src/ssb_core/CMakeLists.txt'])
    if geometry_backend != 'numpy':
        sources.extend([workspace/'src/ssb_tools/ssb_tools/fast_geometry.py',
                        workspace/'src/ssb_core/src/ray_numeric.cpp',
                        workspace/'src/ssb_core/include/ssb_core/ray_numeric.hpp',
                        workspace/'src/ssb_core/CMakeLists.txt'])
        if geometry_backend == 'cuda':
            sources.append(workspace/'src/ssb_core/src/ray_numeric_cuda.cu')
    record = dict(schema='ssb.d3_holdout_protocol.v7' if surface_relief else 'ssb.d3_holdout_protocol.v6',
        declared_at=datetime.datetime.now(datetime.timezone.utc).isoformat(), code_commit=state[0],
        holdout_roi_m=[start, start+length],
        d2=dict(spacing_m=spacing_m, height=height, max_width=1024, halo_m=.25,
                halo_policy=('public_relative_scale_span_v1' if relative_encoder_scale else 'fixed_v1'),
                settings=asdict(match_settings)),
        d3=asdict(settings), sampling=dict(schema=SAMPLING_SCHEMA, spacing_q_m=spacing_m,
            phase_fractions=[.25, .75], samples_across=9, column_guard_pixels=2,
            original_nominal_probes_retained=True, angular_gaps_not_trimmed=True,
            outside_target_requires_public_footprint_proof=True,
            exact_plan_saved_before_truth=True),
        required_evidence=['binary_matches_source', 'stage_b_acceptance', 'stage_b_report_hash_valid', 'public_only_production_run'],
        input_hashes={str(demo/name): sha256_file(demo/name)
                      for name in ('capture.yaml', 'calibration.json', 'bundle.json')},
        production_sources={str(p): sha256_file(p) for p in sources})
    if surface_relief:
        from .surface_relief import ReliefSettings
        record['surface_relief'] = asdict(ReliefSettings())
    if slow_translation:
        from .fast_normal import backend
        record['normal_backend_sha256']=backend()[1]['library_sha256']
    if geometry_backend != 'numpy':
        from .fast_geometry import backend, cuda_backend
        record['geometry_backend_sha256'] = (cuda_backend if geometry_backend == 'cuda' else backend)()[1]['library_sha256']
        record['geometry_diagnostic_backend_sha256'] = backend()[1]['library_sha256']
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(record, indent=2)+'\n')
    return record


def capture_checks(session, expected_commit, require_report_identity=False):
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
    result = dict(capture_provenance_hash_valid=identity_ok, binary_matches_source=matches,
                  stage_b_acceptance=bool(stage_b_ok))
    if require_report_identity:
        from .validate_stage_b import identity_path
        identity_file = identity_path(report_path)
        identity = read_json(identity_file) if identity_file.is_file() else {}
        result['stage_b_report_hash_valid'] = bool(
            report_path.is_file() and identity.get('schema') == 'ssb.stage_b_identity.v1' and
            identity.get('report_sha256') == sha256_file(report_path) and
            identity.get('session_json_sha256') == sha256_file(session.root/'session.json') and
            report.get('evidence') == {key: identity.get(key) for key in
                ('session_json_sha256', 'compare_session_json_sha256')} and
            isinstance(identity.get('compare_session_json_sha256'), str) and
            len(identity['compare_session_json_sha256']) == 64)
    return result


def relocated_outputs(outputs, root, relocation):
    """Verify an explicit content-preserving move; never rewrite old evidence."""
    manifest = read_json(relocation)
    if manifest.get('schema') != 'ssb.review_data_relocation.v1' or manifest.get('status') != 'complete':
        raise ValueError('completed relocation manifest required')
    records = manifest.get('files', [])
    entries = {item['original']: item for item in records}
    if len(entries) != len(records) or len({item['durable'] for item in records}) != len(records):
        raise ValueError('duplicate relocation identity')
    translated = {}
    for source, digest in outputs.items():
        original = Path(source).resolve()
        if original.is_relative_to(root):
            destination = original
        else:
            item = entries.get(source)
            if item is None or item.get('sha256') != digest:
                raise ValueError('relocation does not bind the audited product hash')
            destination = Path(item['durable']).resolve()
            moves = [Path(move['durable']).resolve()/original.relative_to(Path(move['original']).resolve())
                     for move in manifest.get('roots', []) if original.is_relative_to(Path(move['original']).resolve())]
            if len(moves) != 1 or moves[0] != destination:
                raise ValueError('relocation file differs from declared root move')
        if destination.parent not in (root/'matches', root/'fit') or str(destination) in translated:
            raise ValueError('relocation escapes or duplicates the audited product directory')
        translated[str(destination)] = digest
    return translated


def public_run_valid(root, strong=True, relocation=None):
    """The scored D2/D3 products are exactly those of the audited public-only run."""
    root = Path(root).resolve()
    path = root/'public_run/report.json'
    if not path.is_file():
        return False
    report = read_json(path)
    outputs = report.get('outputs', {})
    current = {str(p.resolve()): sha256_file(p) for name in ('matches', 'fit')
               for p in sorted((root/name).iterdir()) if p.is_file()} if all(
                   (root/name).is_dir() for name in ('matches', 'fit')) else {}
    states = report.get('audit_states', [])
    audited = (report.get('schema') in ('ssb.public_reconstruction.v2', 'ssb.public_reconstruction.v3') and bool(states) and
               all(s.get('policy') == POLICY and s.get('installed') is True and
                   type(s.get('blocked_reads')) is int and s['blocked_reads'] == 0 and
                   type(s.get('data_reads')) is int and s['data_reads'] > 0 for s in states))
    if not strong and report.get('schema') == 'ssb.public_reconstruction.v1':
        audited = True  # historical v4 evidence retains its original, weaker meaning
    completed = (report.get('status') == 'complete' and report.get('audit_status') == 'pass'
                 if report.get('schema') == 'ssb.public_reconstruction.v3' else report.get('status') == 'pass')
    expected = (relocated_outputs(outputs, root, relocation) if relocation is not None else
                {str(Path(k).resolve()): v for k, v in outputs.items()})
    return (audited and completed and
            report.get('private_input_opens') == 0 and bool(current) and
            expected == current)


def expected_matching_plan(protocol,root):
    """halo_m declares the base; frozen scale policy uses recorded public span."""
    expected={k:protocol['d2'][k] for k in ('spacing_m','height','max_width','halo_m')}
    relative=protocol['d3'].get('relative_encoder_scale',False)
    if type(relative) is not bool:
        raise ValueError('Boolean relative scale policy required')
    policy='public_relative_scale_span_v1' if relative else 'fixed_v1'
    if protocol['d2'].get('halo_policy',policy)!=policy:
        raise ValueError('matching halo policy differs from declared geometry model')
    if relative:
        projection=np.load(Path(root)/'unroll/projection.npy',allow_pickle=False,mmap_mode='r')
        axes=projection['x_axis_m']
        expected['halo_m']=matching_halo_m([float(axes.min()),float(axes.max())],True,expected['halo_m'])
    return expected


def relocated_sources(entries, source_root):
    """Map frozen absolute production sources into an explicit checkout.

    The frozen workspace is derived from the entries themselves (each lies under
    <workspace>/src/ssb_*); a path that cannot be mapped is an error, never a
    fallback to the current workspace.
    """
    source_root = Path(source_root).resolve()
    if not entries or not source_root.is_dir():
        raise ValueError('frozen production sources and an existing source root are required')
    workspaces, mapped = set(), {}
    for frozen in entries:
        parts = Path(frozen).parts
        if not Path(frozen).is_absolute() or '..' in parts:
            raise ValueError('frozen production source must be an absolute normalized path: '+frozen)
        index = next((i for i in range(1, len(parts)-1) if parts[i] == 'src' and parts[i+1].startswith('ssb_')), None)
        if index is None:
            raise ValueError('frozen production source is outside <workspace>/src/ssb_*: '+frozen)
        workspaces.add(Path(*parts[:index]))
        mapped[frozen] = source_root.joinpath(*parts[index:])
    if len(workspaces) != 1:
        raise ValueError('frozen production sources span several workspaces')
    return workspaces.pop(), mapped


def source_checkout(source_root):
    """HEAD and dirty state of the checkout that supplies relocated sources."""
    git = ['git', '-C', str(Path(source_root).resolve())]
    head = subprocess.check_output(git+['rev-parse', 'HEAD'], text=True).strip()
    dirty = bool(subprocess.check_output(git+['status', '--porcelain', '--untracked-files=no'], text=True).strip())
    return head, dirty


def verify(protocol_file, root, output, source_root=None):
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
    production = protocol['production_sources']
    relocation = None
    if source_root is not None:
        workspace, mapped = relocated_sources(production, source_root)
        head, dirty = source_checkout(source_root)
        production = {str(mapped[p]): digest for p, digest in protocol['production_sources'].items()}
        relocation = dict(frozen_workspace=str(workspace), source_root=str(Path(source_root).resolve()),
                          head=head, dirty=dirty, mapped={p: str(m) for p, m in mapped.items()})
    checks = dict(
        clean_worktree=all(s.get('git_dirty') is False for s in [source, *stages]),
        code_commit_unchanged=all(s.get('git_head') == protocol['code_commit'] for s in [source, *stages]),
        d2_settings_unchanged=read_json(root/'matches/report.json')['settings'] == protocol['d2']['settings'],
        d3_settings_unchanged=read_json(root/'fit/report.json')['settings'] == protocol['d3'],
        holdout_roi_correct=read_json(root/'unroll/report.json')['grid']['target_x_m'] == protocol['holdout_roi_m'],
        input_hashes_unchanged=hashes_match(protocol['input_hashes']),
        production_sources_unchanged=hashes_match(production))
    if relocation is not None:
        checks['source_checkout_matches_protocol'] = (relocation['head'] == protocol['code_commit']
                                                      and not relocation['dirty'])
    if protocol.get('schema') in ('ssb.d3_holdout_protocol.v6', 'ssb.d3_holdout_protocol.v7'):
        checks['d2_planning_unchanged'] = read_json(root/'matches/report.json').get('planning') == expected_matching_plan(protocol,root)
    if protocol.get('schema') == 'ssb.d3_holdout_protocol.v7':
        checks['surface_relief_settings_unchanged'] = read_json(root/'fit/report.json').get(
            'surface_relief', {}).get('settings') == protocol['surface_relief']
    if 'normal_backend_sha256' in protocol:
        checks['normal_backend_identity'] = read_json(root/'fit/report.json').get(
            'normal_backend',{}).get('library_sha256') == protocol['normal_backend_sha256']
    if 'geometry_backend_sha256' in protocol:
        actual = read_json(root/'fit/report.json').get('geometry_backend', {})
        diagnostic = actual.get('host_diagnostic_backend', actual)
        checks['geometry_backend_identity'] = (
            actual.get('library_sha256') == protocol['geometry_backend_sha256'] and
            diagnostic.get('library_sha256') == protocol.get('geometry_diagnostic_backend_sha256'))
    if protocol.get('schema') in ('ssb.d3_holdout_protocol.v4', 'ssb.d3_holdout_protocol.v5', 'ssb.d3_holdout_protocol.v6', 'ssb.d3_holdout_protocol.v7'):
        checks['public_only_production_run'] = public_run_valid(root,
            strong=protocol['schema'] != 'ssb.d3_holdout_protocol.v4')
    else:
        checks['public_replay_identical'] = read_json(root/'public_replay/report.json').get(
            'geometry_and_residuals_identical') is True
    checks.update(capture_checks(session, protocol['code_commit'],
        require_report_identity=protocol.get('schema') in ('ssb.d3_holdout_protocol.v5', 'ssb.d3_holdout_protocol.v6', 'ssb.d3_holdout_protocol.v7')))
    if protocol.get('schema') in ('ssb.d3_holdout_protocol.v3', 'ssb.d3_holdout_protocol.v4', 'ssb.d3_holdout_protocol.v5', 'ssb.d3_holdout_protocol.v6', 'ssb.d3_holdout_protocol.v7'):
        checks['sampling_protocol_unchanged'] = protocol['sampling'] == dict(
            schema=SAMPLING_SCHEMA, spacing_q_m=(protocol['d2']['spacing_m']
                if protocol.get('schema') in ('ssb.d3_holdout_protocol.v6', 'ssb.d3_holdout_protocol.v7') else .2),
            phase_fractions=[.25, .75], samples_across=9,
            column_guard_pixels=2, original_nominal_probes_retained=True,
            outside_target_requires_public_footprint_proof=True,
            angular_gaps_not_trimmed=True, exact_plan_saved_before_truth=True)
    checks['capture_complete'] = session.summary.get('status') == 'complete'
    report = dict(schema='ssb.d3_holdout_verification.v2', status='pass' if all(checks.values()) else 'fail',
        checks=checks, protocol_sha256=sha256_file(protocol_file),
        validator_sha256=sha256_file(__file__),
        matching_halo_contract='halo_m is base; relative policy adds 3% of half the public exposure span',
        capture_build=provenance.get('build'), capture_source_at_run=source,
        interpretation='unchanged C++ code is explanatory evidence, not an exemption from build identity',
        informational=dict(irls_converged=read_json(root/'fit/report.json').get('irls_all_passes_converged'),
                           note='robust-weight convergence is reported, not gated; absent (null) before 2026-10-06'))
    if relocation is not None:
        report['source_relocation'] = dict(relocation,
            scope='production source hashes read from this checkout; all other checks read archived records')
    if protocol.get('schema') in ('ssb.d3_holdout_protocol.v5', 'ssb.d3_holdout_protocol.v6', 'ssb.d3_holdout_protocol.v7'):
        from .validate_stage_b import identity_path
        report_file = session.root/'evaluation/reports/stage_b_smoke.json'
        identity_file = identity_path(report_file)
        report['stage_b_evidence'] = {str(p): sha256_file(p) if p.is_file() else None
                                       for p in (report_file, identity_file)}
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
    declaration.add_argument('--spacing-m', type=float, default=.2)
    declaration.add_argument('--height', type=int, default=512)
    declaration.add_argument('--max-q-shift-mm', type=float)
    declaration.add_argument('--surface-relief', action='store_true')
    add_reconstruction_arguments(declaration, attitude_spacing=False)
    verification = sub.add_parser('verify')
    for name in ('protocol', 'root', 'output'):
        verification.add_argument('--'+name, required=True)
    verification.add_argument('--source-root',
        help='clean checkout of the protocol code_commit (e.g. a worktree of tag milestone-20m-code)')
    audit = sub.add_parser('audit-public', help='verify retained public products, optionally through an explicit relocation')
    audit.add_argument('--root', required=True)
    audit.add_argument('--relocation')
    audit.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.action == 'declare':
        declare(args.workspace, args.demo, args.output, args.start, args.length,
                args.spacing_m, args.adaptive_attitude, args.height, args.max_q_shift_mm, args.surface_relief,
                args.slow_translation, args.relative_encoder_scale, args.geometry_backend, args.fit_axis_yaw)
        return 0
    if args.action == 'audit-public':
        output = evaluation_path(args.output)
        if output.exists(): raise ValueError('use a fresh audit record; preserve earlier evidence')
        root = Path(args.root).resolve()
        valid = public_run_valid(root, relocation=args.relocation)
        inputs = [root/'public_run/report.json']
        if args.relocation: inputs.append(Path(args.relocation).resolve())
        report = dict(schema='ssb.public_run_identity_check.v1', status='pass' if valid else 'fail',
                      input_hashes={str(p): sha256_file(p) for p in inputs},
                      scope='audited product content identity only; not geometric quality or a new frozen capture')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report))
        return 0 if valid else 1
    report = verify(args.protocol, args.root, args.output, args.source_root)
    print(json.dumps(report))
    return 0 if report['status'] == 'pass' else 1


if __name__ == '__main__':
    raise SystemExit(main())
