import json
from types import SimpleNamespace

import pytest

from ssb_tools.holdout_protocol import capture_checks
from ssb_tools.session import sha256_file
from ssb_tools.validate_stage_b import identity_path, main, report_destination, write_report


def evidence(tmp_path):
    root = tmp_path/'capture'
    root.mkdir()
    (root/'session.json').write_text('{}')
    compare = tmp_path/'reimage'
    compare.mkdir()
    (compare/'session.json').write_text('{"status":"complete"}')
    prov = root/'config/provenance.json'
    prov.parent.mkdir()
    prov.write_text(json.dumps(dict(binary_matches_source=True,
        build=dict(git_head='frozen', source_digest='a'*64),
        source_at_run=dict(git_head='frozen', source_digest='a'*64))))
    session = SimpleNamespace(root=root, summary=dict(files={'config/provenance.json': sha256_file(prov)}))
    report = report_destination(root)
    result = dict(schema='ssb.stage_b_smoke_report.v1', overall='pass', checks=[
        dict(name=name, state='pass') for name in
        ('binary_matches_source', 'stored_hashes_verify', 'reimaging_byte_identical')])
    write_report(result, report, root, compare)
    return session, report, result, compare


def test_report_identity_and_legacy_checks(tmp_path):
    session, report, _, compare = evidence(tmp_path)
    assert all(capture_checks(session, 'frozen', True).values())
    identity = json.loads(identity_path(report).read_text())
    assert identity['compare_session_json_sha256'] == sha256_file(compare/'session.json')


@pytest.mark.parametrize('change', ['report', 'session', 'identity', 'missing'])
def test_changed_or_missing_sealed_evidence_fails(tmp_path, change):
    session, report, _, _ = evidence(tmp_path)
    path = {'report': report, 'session': session.root/'session.json',
            'identity': identity_path(report), 'missing': identity_path(report)}[change]
    if change == 'missing':
        path.unlink()
    else:
        path.write_text(path.read_text()+'\n') if change != 'identity' else path.write_text('{}')
    assert capture_checks(session, 'frozen', True)['stage_b_report_hash_valid'] is False
    assert capture_checks(session, 'frozen')['stage_b_acceptance'] is True


def test_repeat_validation_never_overwrites_original(tmp_path):
    session, report, result, compare = evidence(tmp_path)
    before = {p: p.read_bytes() for p in (report, identity_path(report))}
    with pytest.raises(FileExistsError):
        write_report(result, report, session.root, compare)
    assert before == {p: p.read_bytes() for p in before}
    new = report_destination(session.root, report.parent/'repeat.json')
    write_report(result, new, session.root, compare)
    assert before == {p: p.read_bytes() for p in before}


@pytest.mark.parametrize('sidecar', [False, True])
def test_dangling_evidence_link_is_not_replaced(tmp_path, sidecar):
    root = tmp_path/'capture'
    report = root/'evaluation/reports/stage_b_smoke.json'
    report.parent.mkdir(parents=True)
    path = identity_path(report) if sidecar else report
    path.symlink_to(tmp_path/'missing')
    with pytest.raises(FileExistsError):
        report_destination(root)
    assert path.is_symlink()


def test_reports_stay_on_evaluation_side(tmp_path):
    with pytest.raises(ValueError, match='evaluation'):
        report_destination(tmp_path, tmp_path/'public/report.json')


def test_read_only_main_does_not_touch_existing_evidence(tmp_path, monkeypatch, capsys):
    import ssb_tools.validate_stage_b as module
    session, report, _, _ = evidence(tmp_path)
    session.summary['status'] = 'complete'
    monkeypatch.setattr(module, 'Session', lambda _: session)
    def stop(*args):
        raise ValueError('fixture ends here')
    monkeypatch.setattr(module, 'verify_hashes', stop)
    before = {p: p.read_bytes() for p in session.root.rglob('*') if p.is_file()}
    assert main([str(session.root), '--read-only']) == 1
    assert 'validation_error fail' in capsys.readouterr().out
    assert before == {p: p.read_bytes() for p in session.root.rglob('*') if p.is_file()}


def test_existing_report_is_rejected_before_session_io(tmp_path, monkeypatch):
    import ssb_tools.validate_stage_b as module
    session, _, _, _ = evidence(tmp_path)
    monkeypatch.setattr(module, 'Session', lambda _: pytest.fail('unnecessary session IO'))
    with pytest.raises(FileExistsError):
        main([str(session.root)])
