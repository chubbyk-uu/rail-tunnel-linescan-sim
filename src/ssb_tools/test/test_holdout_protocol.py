import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
import numpy as np
from test_wall_coverage import nominal  # noqa: F401 -- pytest fixture

from ssb_tools.holdout_protocol import capture_checks, public_run_valid
from ssb_tools.session import sha256_file


@pytest.mark.parametrize('explicit_policy',[False,True])
def test_relative_matching_protocol_checks_actual_public_span_not_fixed_halo(tmp_path,explicit_policy):
    from ssb_tools.holdout_protocol import expected_matching_plan
    protocol=dict(d2=dict(spacing_m=.1,height=256,max_width=1024,halo_m=.25),
                  d3=dict(relative_encoder_scale=True))
    if explicit_policy:protocol['d2']['halo_policy']='public_relative_scale_span_v1'
    (tmp_path/'unroll').mkdir()
    projection=np.array([(-1.2,),(21.2,)],dtype=[('x_axis_m','f8')])
    np.save(tmp_path/'unroll/projection.npy',projection)
    plan=expected_matching_plan(protocol,tmp_path)
    assert plan['halo_m']==pytest.approx(.586)
    old=dict(plan,halo_m=.25)
    assert old!=plan
    # A future actual exposure span is measured from public rows, not guessed
    # from the target length or recovered from true wheel parameters.
    projection['x_axis_m'][-1]=22.2;np.save(tmp_path/'unroll/projection.npy',projection)
    assert expected_matching_plan(protocol,tmp_path)['halo_m']==pytest.approx(.601)
    protocol['d2']['halo_policy']='fixed_v1'
    with pytest.raises(ValueError,match='policy'):
        expected_matching_plan(protocol,tmp_path)


def test_nominal_matching_protocol_does_not_require_public_span_file(tmp_path):
    from ssb_tools.holdout_protocol import expected_matching_plan
    d2=dict(spacing_m=.2,height=512,max_width=1024,halo_m=.25)
    assert expected_matching_plan(dict(d2=d2,d3={}),tmp_path)==d2


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
@pytest.mark.parametrize('slow,yaw', [(False, None), (True, None), (True, False)])
def test_declaration_uses_measured_calibration_and_rejects_invalid_wall_task(tmp_path, monkeypatch, nominal, start, length, valid, spacing, adaptive, height, relief, slow, yaw):
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
                 'initial_unroll.py', 'global_resample.py', 'reconstruction_support.py', 'evaluate_global_geometry.py',
                 'reconstruction_budget.py', 'mission_plan.py', 'global_cuda.py', 'global_mosaic.py',
                 'public_audit.py', 'public_reconstruction.py', 'parallel_budget.py', 'validate_stage_b.py',
                 'validate_stage_a.py', 'ref_geometry.py', 'session.py', 'surface_relief.py','fast_normal.py', 'fast_geometry.py'):
        (source/name).write_text('# fixture source\n')
    (tmp_path/'src/ssb_core/src').mkdir(parents=True)
    (tmp_path/'src/ssb_core/src/normal_equations.cpp').write_text('// fixture source\n')
    (tmp_path/'src/ssb_core/src/ray_numeric.cpp').write_text('// fixture source\n')
    (tmp_path/'src/ssb_core/include/ssb_core').mkdir(parents=True)
    (tmp_path/'src/ssb_core/include/ssb_core/ray_numeric.hpp').write_text('// fixture source\n')
    (tmp_path/'src/ssb_core/CMakeLists.txt').write_text('# fixture build\n')
    monkeypatch.setattr(module.subprocess, 'check_output', lambda *a, **k: 'frozen 0 '+('a'*64))
    monkeypatch.setattr(module, 'check_calibration', lambda *a: 'measured-rig')
    output = tmp_path/'evaluation/protocol.json'
    if valid:
        record = module.declare(tmp_path, demo, output, start, length, spacing, adaptive, height,
                                10. if adaptive else None, relief, slow, fit_axis_yaw=yaw)
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
        assert record['d3']['coarse_translation'] is slow
        assert record['d3']['fit_translation'] is slow
        assert record['d3']['translation_bound_mm'] == (30. if slow else 5.)
        assert record['d3']['fit_axis_yaw'] is (slow if yaw is None else yaw)
        if slow:assert len(record['normal_backend_sha256'])==64
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
    (None, 'spacing_m', 'height', 'max_width', 'halo_m', 'missing','normal_backend',
     'geometry_backend', 'geometry_diagnostic_backend')]+[(True,'relief_settings')])
@pytest.mark.parametrize('relative',[False,True])
def test_v6_verification_detects_changed_matching_density_or_window_plan(tmp_path, monkeypatch, change, relief,relative):
    import ssb_tools.holdout_protocol as module
    root = tmp_path/'run'
    source = dict(git_head='frozen', git_dirty=False)
    planning = dict(spacing_m=.1, height=512, max_width=1024, halo_m=.25)
    sampling = dict(schema=module.SAMPLING_SCHEMA, spacing_q_m=.1, phase_fractions=[.25, .75],
        samples_across=9, column_guard_pixels=2, original_nominal_probes_retained=True,
        outside_target_requires_public_footprint_proof=True, angular_gaps_not_trimmed=True,
        exact_plan_saved_before_truth=True)
    protocol = dict(schema='ssb.d3_holdout_protocol.v6', code_commit='frozen', holdout_roi_m=[12.,15.],
        d2=dict(planning, settings={}), d3=dict(adaptive_attitude=True), sampling=sampling,
        normal_backend_sha256='a'*64, geometry_backend_sha256='c'*64,
        geometry_diagnostic_backend_sha256='d'*64)
    if relative:protocol['d3']['relative_encoder_scale']=True
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
    if relative:
        np.save(root/'unroll/projection.npy',np.array([(10.,),(16.8,)],dtype=[('x_axis_m','f8')]))
        report['planning']['halo_m']=.25+.03*(16.8-10.)/2
    if change == 'missing': report.pop('planning')
    elif change is not None and change not in ('relief_settings','normal_backend', 'geometry_backend', 'geometry_diagnostic_backend'):
        report['planning'][change] *= 2
    (root/'matches/report.json').write_text(json.dumps(report))
    fit = dict(settings=protocol['d3'],normal_backend=dict(library_sha256='b'*64 if change=='normal_backend' else 'a'*64))
    fit['geometry_backend'] = dict(library_sha256=('e' if change=='geometry_backend' else 'c')*64,
        host_diagnostic_backend=dict(library_sha256=('e' if change=='geometry_diagnostic_backend' else 'd')*64))
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
    assert result['checks']['d2_planning_unchanged'] is (change in (None,'relief_settings','normal_backend', 'geometry_backend', 'geometry_diagnostic_backend'))
    assert result['checks']['normal_backend_identity'] is (change!='normal_backend')
    assert result['checks']['geometry_backend_identity'] is (change not in ('geometry_backend', 'geometry_diagnostic_backend'))
    if relief:
        assert result['checks']['surface_relief_settings_unchanged'] is (change != 'relief_settings')
    assert result['status'] == ('pass' if change is None else 'fail')


def relocation_fixture(tmp_path, monkeypatch):
    """A frozen protocol whose recorded workspace no longer exists, plus a git checkout."""
    import subprocess
    import ssb_tools.holdout_protocol as module
    checkout = tmp_path/'checkout'
    names = ('src/ssb_tools/ssb_tools/optimize_bands.py', 'src/ssb_core/CMakeLists.txt')
    for name in names:
        (checkout/name).parent.mkdir(parents=True, exist_ok=True)
        (checkout/name).write_text('# frozen '+name+'\n')
    git = ['git', '-C', str(checkout), '-c', 'user.name=t', '-c', 'user.email=t@t']
    subprocess.run(['git', 'init', '-q', str(checkout)], check=True)
    subprocess.run(git+['add', '.'], check=True)
    subprocess.run(git+['commit', '-q', '-m', 'frozen'], check=True)
    commit = subprocess.check_output(git+['rev-parse', 'HEAD'], text=True).strip()
    frozen = tmp_path/'gone/workspace'
    root = tmp_path/'run'
    source = dict(git_head=commit, git_dirty=False)
    for name in ('unroll', 'matches', 'fit'):
        (root/name).mkdir(parents=True)
        (root/name/'provenance.json').write_text(json.dumps(dict(source=source)))
    (root/'unroll/report.json').write_text(json.dumps(dict(grid=dict(target_x_m=[12., 15.]))))
    (root/'matches/report.json').write_text(json.dumps(dict(settings={})))
    (root/'fit/report.json').write_text(json.dumps(dict(settings={})))
    (root/'capture/config').mkdir(parents=True)
    (root/'capture/config/provenance.json').write_text(json.dumps(dict(source_at_run=source)))
    marker = tmp_path/'input'; marker.write_text('unchanged input')
    protocol = dict(schema='ssb.d3_holdout_protocol.v5', code_commit=commit, holdout_roi_m=[12., 15.],
        d2=dict(spacing_m=.2, settings={}), d3={}, input_hashes={str(marker): sha256_file(marker)},
        sampling=dict(schema=module.SAMPLING_SCHEMA, spacing_q_m=.2, phase_fractions=[.25, .75],
            samples_across=9, column_guard_pixels=2, original_nominal_probes_retained=True,
            outside_target_requires_public_footprint_proof=True, angular_gaps_not_trimmed=True,
            exact_plan_saved_before_truth=True),
        production_sources={str(frozen/name): sha256_file(checkout/name) for name in names})
    protocol_file = tmp_path/'evaluation/protocol.json'; protocol_file.parent.mkdir()
    protocol_file.write_text(json.dumps(protocol))
    monkeypatch.setattr(module, 'Session', lambda path: SimpleNamespace(root=path, summary=dict(status='complete')))
    monkeypatch.setattr(module, 'capture_checks', lambda *a, **k: dict(binary_matches_source=True))
    monkeypatch.setattr(module, 'public_run_valid', lambda *a, **k: True)
    return module, protocol_file, root, checkout, frozen, git


@pytest.mark.parametrize('change', [None, 'byte', 'missing', 'commit'])
def test_relocated_source_root_reads_only_the_named_checkout(tmp_path, monkeypatch, change):
    import subprocess
    module, protocol_file, root, checkout, frozen, git = relocation_fixture(tmp_path, monkeypatch)
    target = checkout/'src/ssb_tools/ssb_tools/optimize_bands.py'
    if change == 'byte':
        target.write_text(target.read_text()+' ')
    elif change == 'missing':
        target.unlink()
    elif change == 'commit':
        (checkout/'later.txt').write_text('later\n')
        subprocess.run(git+['add', 'later.txt'], check=True)
        subprocess.run(git+['commit', '-q', '-m', 'later'], check=True)
    # The frozen workspace is gone: without relocation the source check cannot pass.
    plain = module.verify(protocol_file, root, tmp_path/'evaluation/plain.json')
    assert plain['checks']['production_sources_unchanged'] is False and 'source_relocation' not in plain
    result = module.verify(protocol_file, root, tmp_path/'evaluation/relocated.json', checkout)
    assert result['checks']['production_sources_unchanged'] is (change in (None, 'commit'))
    assert result['checks']['source_checkout_matches_protocol'] is (change is None)
    assert result['status'] == ('pass' if change is None else 'fail')
    assert result['informational']['irls_converged'] is None and 'irls_converged' not in result['checks']
    relocation = result['source_relocation']
    assert relocation['frozen_workspace'] == str(frozen)
    assert relocation['mapped'][str(frozen/'src/ssb_core/CMakeLists.txt')] == str(
        checkout.resolve()/'src/ssb_core/CMakeLists.txt')


@pytest.mark.parametrize('entries', [
    {}, {'relative/src/ssb_core/a.cpp': 'x'}, {'/ws/lib/a.py': 'x'},
    {'/ws/src/ssb_core/../../escape.py': 'x'},
    {'/ws1/src/ssb_core/a.cpp': 'x', '/ws2/src/ssb_tools/b.py': 'x'}])
def test_unmappable_frozen_sources_are_rejected(tmp_path, entries):
    from ssb_tools.holdout_protocol import relocated_sources
    with pytest.raises(ValueError):
        relocated_sources(entries, tmp_path)
