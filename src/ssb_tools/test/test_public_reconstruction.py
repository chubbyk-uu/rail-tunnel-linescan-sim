"""Public audit success cannot be confused with reconstruction quality success."""
import sys
import pytest
from ssb_tools import public_reconstruction as module


@pytest.mark.parametrize('gate', ['pass', 'fail', 'unmeasurable'])
def test_public_report_separates_completed_computation_audit_and_quality(tmp_path, monkeypatch, gate):
    def stage(*args):
        (tmp_path/'public_run').mkdir()
        return tmp_path/'d1', tmp_path/'config', tmp_path/'raw', 1
    def match(*args):
        (tmp_path/'matches').mkdir()
        return dict(status='complete', windows=1, matches=dict(total=12), worker_audits=[], performance=dict(wall_s=0.))
    def optimize(*args):
        (tmp_path/'fit').mkdir()
        return dict(status='complete', image_consistency_gate=dict(status=gate), performance=dict(wall_s=0.))
    monkeypatch.setattr(module, 'stage', stage)
    monkeypatch.setattr(module, 'match', match)
    monkeypatch.setattr(module, 'optimize', optimize)
    monkeypatch.setattr(module.public_audit, 'install', lambda *a, **kw: {})
    monkeypatch.setattr(module.public_audit, 'verified_states', lambda *a: [dict(blocked_reads=0)])
    report = module.run(tmp_path/'input', tmp_path/'observable', tmp_path, workers=1)
    assert report['status'] == 'complete' and report['audit_status'] == 'pass'
    assert report['quality_status'] == gate and report['d3']['image_consistency_gate'] == gate
    assert (tmp_path/'public_run/report.json').is_file()


@pytest.mark.parametrize('gate', ['pass', 'fail', 'unmeasurable'])
def test_strict_public_cli_fails_after_emitting_quality_diagnostics(monkeypatch, capsys, gate):
    report = dict(status='complete', audit_status='pass', quality_status=gate,
        public_raw_blocks=1, d2={}, d3={}, performance={})
    monkeypatch.setattr(module, 'run', lambda *a, **kw: report)
    monkeypatch.setattr(sys, 'argv', ['public_reconstruction', '--unroll', 'd1', '--observable', 'config',
        '--root', 'new', '--strict'])
    if gate == 'pass': module.main()
    else:
        with pytest.raises(SystemExit) as error: module.main()
        assert error.value.code == 1
    assert f'"quality_status": "{gate}"' in capsys.readouterr().out
