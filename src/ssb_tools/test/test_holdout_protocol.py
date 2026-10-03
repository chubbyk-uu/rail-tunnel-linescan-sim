import json
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
def test_declaration_uses_measured_calibration_and_rejects_invalid_wall_task(tmp_path, monkeypatch, nominal, start, length, valid):
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
                 'public_audit.py', 'public_reconstruction.py', 'parallel_budget.py'):
        (source/name).write_text('# fixture source\n')
    monkeypatch.setattr(module.subprocess, 'check_output', lambda *a, **k: 'frozen 0 '+('a'*64))
    monkeypatch.setattr(module, 'check_calibration', lambda *a: 'measured-rig')
    output = tmp_path/'evaluation/protocol.json'
    if valid:
        record = module.declare(tmp_path, demo, output, start, length)
        assert record['holdout_roi_m'] == [12., 15.]
        assert record['required_evidence'] == ['binary_matches_source', 'stage_b_acceptance',
                                               'public_only_production_run']
        assert record['schema'] == 'ssb.d3_holdout_protocol.v5'
        assert record['code_commit'] == 'frozen' and output.exists()
        assert record['sampling']['schema'] == 'ssb.public_common_overlap.v2'
        assert record['sampling']['exact_plan_saved_before_truth'] is True
        assert any(name.endswith('evaluate_global_geometry.py') for name in record['production_sources'])
    else:
        with pytest.raises(ValueError):
            module.declare(tmp_path, demo, output, start, length)
        assert not output.exists()


def public_run_fixture(root):
    for name in ('matches', 'fit'):
        (root/name).mkdir(parents=True)
        (root/name/'report.json').write_text(name)
    outputs = {str(p): sha256_file(p) for name in ('matches', 'fit') for p in sorted((root/name).iterdir())}
    (root/'public_run').mkdir()
    from ssb_tools.public_audit import POLICY
    report = dict(schema='ssb.public_reconstruction.v2', status='pass', private_input_opens=0, outputs=outputs,
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
        else: report['schema'] = 'ssb.public_reconstruction.v1'
        (tmp_path/'public_run/report.json').write_text(json.dumps(report))
        if change == 'legacy': assert public_run_valid(tmp_path, strong=False)
    assert public_run_valid(tmp_path) is (change is None)
