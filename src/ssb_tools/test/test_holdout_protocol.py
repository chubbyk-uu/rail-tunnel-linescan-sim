import json
from types import SimpleNamespace

import pytest

from ssb_tools.holdout_protocol import capture_checks
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
