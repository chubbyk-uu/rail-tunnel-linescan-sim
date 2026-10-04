import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from test_wall_coverage import nominal

from ssb_tools.holdout_protocol import capture_checks, public_run_valid
from ssb_tools.session import sha256_file


def fixture(tmp_path, **overrides):
    provenance = dict(binary_matches_source=True,
        build=dict(git_head='frozen', source_digest='a'*64),
        source_at_run=dict(git_head='frozen', source_digest='a'*64))
    provenance.update(overrides)
    path = tmp_path/'config/provenance.json'
    path.parent.mkdir()
    path.write_text(json.dumps(provenance))
    report = tmp_path/'evaluation/reports/stage_b_smoke.json'
    report.parent.mkdir(parents=True)
    report.write_text(json.dumps(dict(schema='ssb.stage_b_smoke_report.v1', overall='pass',
        checks=[dict(name=n, state='pass') for n in
                ('binary_matches_source', 'stored_hashes_verify', 'reimaging_byte_identical')])))
    session = SimpleNamespace(root=tmp_path,
        summary=dict(files={'config/provenance.json': sha256_file(path)}))
    return session, path, report


def test_capture_binary_and_stage_b_are_both_required(tmp_path):
    session, _, _ = fixture(tmp_path)
    assert all(capture_checks(session, 'frozen').values())


@pytest.mark.parametrize('flag', [False, None, 1, 'true'])
def test_stale_missing_or_nonboolean_binary_match_cannot_pass(tmp_path, flag):
    session, _, _ = fixture(tmp_path, binary_matches_source=flag)
    assert capture_checks(session, 'frozen')['binary_matches_source'] is False


@pytest.mark.parametrize('field,value', [('git_head', 'older'), ('source_digest', 'b'*64)])
def test_true_flag_does_not_override_mismatched_build_identity(tmp_path, field, value):
    build = dict(git_head='frozen', source_digest='a'*64)
    build[field] = value
    session, _, _ = fixture(tmp_path, build=build)
    assert capture_checks(session, 'frozen')['binary_matches_source'] is False


def test_modified_capture_provenance_is_not_accepted(tmp_path):
    session, path, _ = fixture(tmp_path)
    path.write_text(path.read_text()+'\n')
    checks = capture_checks(session, 'frozen')
    assert checks['capture_provenance_hash_valid'] is False
    assert checks['binary_matches_source'] is False


@pytest.mark.parametrize('change', ['missing', 'failed', 'unmeasurable', 'missing_replay', 'duplicate'])
def test_missing_or_incomplete_stage_b_evidence_is_rejected(tmp_path, change):
    session, _, path = fixture(tmp_path)
    if change == 'missing':
        path.unlink()
    else:
        report = json.loads(path.read_text())
        if change in ('failed', 'unmeasurable'):
            report['checks'][-1]['state'] = change
        elif change == 'missing_replay':
            report['checks'].pop()
        else:
            report['checks'].append(report['checks'][0])
        # Deliberately leave overall='pass': an inconsistent summary must fail.
        path.write_text(json.dumps(report))
    assert capture_checks(session, 'frozen')['stage_b_acceptance'] is False


@pytest.mark.parametrize('start,length,valid', [(12., 3., True), (19., 3., False), (12., .5, False)])
@pytest.mark.parametrize('spacing,adaptive,height', [(.2, False, 512), (.1, True, 256)])
@pytest.mark.parametrize('relief', [False, True])
def test_declaration_uses_measured_calibration_and_rejects_invalid_wall_task(tmp_path, monkeypatch, nominal, start, length, valid, spacing, adaptive, height, relief):
    import ssb_tools.holdout_protocol as module
    config, calibration = nominal
    demo = tmp_path/'demo'
    demo.mkdir()
    (demo/'capture.yaml').write_text(yaml.safe_dump(config))
    (demo/'calibration.json').write_text(json.dumps(calibration))
    (demo/'bundle.json').write_text('{}')
    source = tmp_path/'src/ssb_tools/ssb_tools'
    source.mkdir(parents=True)
    for name in ('match_bands.py', 'band_matching.py', 'matching_structures.py', 'optimize_bands.py', 'global_geometry.py',
                 'initial_unroll.py', 'global_resample.py', 'evaluate_global_geometry.py',
                 'public_audit.py', 'public_reconstruction.py', 'parallel_budget.py', 'validate_stage_b.py',
                 'validate_stage_a.py', 'ref_geometry.py', 'session.py', 'surface_relief.py'):
        (source/name).write_text('# fixture source\n')
    monkeypatch.setattr(module.subprocess, 'check_output', lambda *a, **k: 'frozen 0 '+('a'*64))
    monkeypatch.setattr(module, 'check_calibration', lambda *a: 'measured-rig')
    output = tmp_path/'evaluation/protocol.json'
    if valid:
        record = module.declare(tmp_path, demo, output, start, length, spacing, adaptive, height,
                                10. if adaptive else None, relief)
        assert record['holdout_roi_m'] == [12., 15.]
        assert record['required_evidence'] == ['binary_matches_source', 'stage_b_acceptance',
                                               'stage_b_report_hash_valid', 'public_only_production_run']
        assert record['schema'] == ('ssb.d3_holdout_protocol.v7' if relief else 'ssb.d3_holdout_protocol.v6')
        if relief:
            from ssb_tools.surface_relief import ReliefSettings
            from dataclasses import asdict
            assert record['surface_relief'] == asdict(ReliefSettings())
            assert any(name.endswith('surface_relief.py') for name in record['production_sources'])
        assert record['d2']['spacing_m'] == spacing
        assert record['d2']['height'] == height
        assert record['d2']['settings']['max_q_shift_mm'] == (10. if adaptive else None)
        assert record['sampling']['spacing_q_m'] == spacing
        assert record['d3']['adaptive_attitude'] is adaptive
        assert record['code_commit'] == 'frozen' and output.exists()
        assert record['sampling']['schema'] == 'ssb.public_common_overlap.v4'
        assert record['sampling']['exact_plan_saved_before_truth'] is True
        assert any(name.endswith('evaluate_global_geometry.py') for name in record['production_sources'])
    else:
        with pytest.raises(ValueError):
            module.declare(tmp_path, demo, output, start, length, spacing, adaptive, height)
        assert not output.exists()


def public_run_fixture(root):
    for name in ('matches', 'fit'):
        (root/name).mkdir(parents=True)
        (root/name/'report.json').write_text(name)
    outputs = {str(p): sha256_file(p) for name in ('matches', 'fit') for p in sorted((root/name).iterdir())}
    (root/'public_run').mkdir()
    from ssb_tools.public_audit import POLICY
    report = dict(schema='ssb.public_reconstruction.v3', status='complete', audit_status='pass',
                  quality_status='pass', private_input_opens=0, outputs=outputs,
        audit_states=[dict(policy=POLICY, installed=True, blocked_reads=0, data_reads=3)])
    (root/'public_run/report.json').write_text(json.dumps(report))
    return report


@pytest.mark.parametrize('change', [None, 'missing', 'replaced_output', 'extra_output', 'failed',
                                  'missing_audit', 'blocked_read', 'legacy'])
def test_scored_products_must_be_those_of_the_audited_public_run(tmp_path, change):
    report = public_run_fixture(tmp_path)
    if change == 'missing':
        (tmp_path/'public_run/report.json').unlink()
    elif change == 'replaced_output':
        (tmp_path/'fit/report.json').write_text('rerun outside the audit')
    elif change == 'extra_output':
        (tmp_path/'fit/trajectory.json').write_text('{}')
    elif change == 'failed':
        report['status'] = 'fail'
        (tmp_path/'public_run/report.json').write_text(json.dumps(report))
    elif change in ('missing_audit', 'blocked_read', 'legacy'):
        if change == 'missing_audit': report['audit_states'] = []
        elif change == 'blocked_read': report['audit_states'][0]['blocked_reads'] = 1
        else: report.update(schema='ssb.public_reconstruction.v1', status='pass')
        (tmp_path/'public_run/report.json').write_text(json.dumps(report))
        if change == 'legacy': assert public_run_valid(tmp_path, strong=False)
    assert public_run_valid(tmp_path) is (change is None)


def test_previous_public_audit_schema_retains_its_status_meaning(tmp_path):
    report = public_run_fixture(tmp_path)
    report.update(schema='ssb.public_reconstruction.v2', status='pass')
    report.pop('audit_status'); report.pop('quality_status')
    (tmp_path/'public_run/report.json').write_text(json.dumps(report))
    assert public_run_valid(tmp_path)


@pytest.mark.parametrize('change', [None, 'digest', 'missing', 'duplicate', 'escape', 'root', 'product'])
def test_relocated_audit_requires_exact_root_move_and_original_product_hashes(tmp_path, change):
    old, new = tmp_path/'old', tmp_path/'new'
    old.mkdir()
    report = public_run_fixture(old)
    files = [dict(original=p, durable=str(new/Path(p).relative_to(old)), sha256=h)
             for p, h in report['outputs'].items()]
    old.rename(new)
    manifest = dict(schema='ssb.review_data_relocation.v1', status='complete', files=files,
                    roots=[dict(original=str(old), durable=str(new))])
    if change == 'digest': files[0]['sha256'] = 'f'*64
    elif change == 'missing': files.pop()
    elif change == 'duplicate': files.append(dict(files[0]))
    elif change == 'escape': files[0]['durable'] = str(tmp_path/'elsewhere')
    elif change == 'root': manifest['roots'][0]['durable'] = str(tmp_path/'wrong')
    elif change == 'product': Path(files[0]['durable']).write_text('changed')
    relocation = tmp_path/'relocation.json'; relocation.write_text(json.dumps(manifest))
    assert not public_run_valid(new)
    if change in ('digest', 'missing', 'duplicate', 'escape', 'root'):
        with pytest.raises(ValueError): public_run_valid(new, relocation=relocation)
    else:
        assert public_run_valid(new, relocation=relocation) is (change is None)


@pytest.mark.parametrize('relief,change', [(r,c) for r in (False,True) for c in
    (None, 'spacing_m', 'height', 'max_width', 'halo_m', 'missing')]+[(True,'relief_settings')])
def test_v6_verification_detects_changed_matching_density_or_window_plan(tmp_path, monkeypatch, change, relief):
    import ssb_tools.holdout_protocol as module
    root = tmp_path/'run'
    source = dict(git_head='frozen', git_dirty=False)
    planning = dict(spacing_m=.1, height=512, max_width=1024, halo_m=.25)
    sampling = dict(schema=module.SAMPLING_SCHEMA, spacing_q_m=.1, phase_fractions=[.25, .75],
        samples_across=9, column_guard_pixels=2, original_nominal_probes_retained=True,
        outside_target_requires_public_footprint_proof=True, angular_gaps_not_trimmed=True,
        exact_plan_saved_before_truth=True)
    protocol = dict(schema='ssb.d3_holdout_protocol.v6', code_commit='frozen', holdout_roi_m=[12.,15.],
        d2=dict(planning, settings={}), d3=dict(adaptive_attitude=True), sampling=sampling)
    if relief:
        from ssb_tools.surface_relief import ReliefSettings
        from dataclasses import asdict
        protocol.update(schema='ssb.d3_holdout_protocol.v7', surface_relief=asdict(ReliefSettings()))
    marker = tmp_path/'marker'; marker.write_text('unchanged input and source')
    protocol['input_hashes'] = protocol['production_sources'] = {str(marker): sha256_file(marker)}
    for name in ('unroll', 'matches', 'fit'):
        (root/name).mkdir(parents=True)
        (root/name/'provenance.json').write_text(json.dumps(dict(source=source)))
    (root/'unroll/report.json').write_text(json.dumps(dict(grid=dict(target_x_m=[12.,15.]))))
    report = dict(settings={}, planning=planning.copy())
    if change == 'missing': report.pop('planning')
    elif change is not None and change != 'relief_settings': report['planning'][change] *= 2
    (root/'matches/report.json').write_text(json.dumps(report))
    fit = dict(settings=protocol['d3'])
    if relief:
        fit['surface_relief'] = dict(settings=protocol['surface_relief'].copy())
        if change == 'relief_settings':
            fit['surface_relief']['settings']['max_cycle_px'] = 2.
    (root/'fit/report.json').write_text(json.dumps(fit))
    (root/'capture/config').mkdir(parents=True)
    (root/'capture/config/provenance.json').write_text(json.dumps(dict(source_at_run=source)))
    protocol_file = tmp_path/'evaluation/protocol.json'; protocol_file.parent.mkdir()
    protocol_file.write_text(json.dumps(protocol))
    # Other identity checks have their own tests above; exercise the actual
    # verifier here, so a planning check omitted from it cannot silently pass.
    monkeypatch.setattr(module, 'Session', lambda path: SimpleNamespace(root=path, summary=dict(status='complete')))
    monkeypatch.setattr(module, 'capture_checks', lambda *a, **k: dict(binary_matches_source=True))
    monkeypatch.setattr(module, 'public_run_valid', lambda *a, **k: True)
    result = module.verify(protocol_file, root, tmp_path/'evaluation/verification.json')
    assert result['checks']['d2_planning_unchanged'] is (change in (None,'relief_settings'))
    if relief:
        assert result['checks']['surface_relief_settings_unchanged'] is (change != 'relief_settings')
    assert result['status'] == ('pass' if change is None else 'fail')
