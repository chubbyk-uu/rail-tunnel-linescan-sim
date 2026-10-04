"""Preflight uses nominal public geometry and allocates no images."""
import copy
import json
from pathlib import Path
import runpy
from types import SimpleNamespace

import pytest
import yaml

from ssb_tools.global_geometry import reconstruction_settings
from ssb_tools.reconstruction_budget import plan
from test_wall_coverage import nominal


@pytest.mark.parametrize('bad_source',['domain','hash'])
def test_reused_combined_lining_rejects_bad_inputs_before_work(tmp_path,bad_source):
    tool=Path(__file__).resolve().parents[3]/'tools/prepare_combined_20m.py'
    build=runpy.run_path(str(tool))['build']
    demo=tmp_path/'demo';demo.mkdir()
    bounds=[-1.5,21.5] if bad_source=='domain' else [-2.5,22.5]
    (demo/'capture.yaml').write_text(yaml.safe_dump(dict(tunnel=dict(x_min_m=bounds[0],x_max_m=bounds[1]))))
    if bad_source=='hash':
        (demo/'asset.bin').write_bytes(b'corrupted')
        (demo/'bundle.json').write_text(json.dumps(dict(schema='ssb.demo_bundle.v1',files={'asset.bin':'0'*64})))
    work,output=tmp_path/'work',tmp_path/'output'
    with pytest.raises(ValueError,match='domain' if bad_source=='domain' else 'hash mismatch'):
        build(demo,tmp_path/'sources',work,output,20261129,True)
    assert not work.exists() and not output.exists()


def test_budget_keeps_truth_out_and_reports_live_image_and_coefficient_caps():
    config, calibration = nominal.__wrapped__()
    settings = reconstruction_settings(.02, True, True, True)
    first = plan(config, calibration, 8., 3., settings)
    changed = copy.deepcopy(config)
    changed['truth'] = dict(odo_left_diameter_m=.081, mount=dict(dy_m=.02, tilt_y_rad=.001))
    second = plan(changed, calibration, 8., 3., settings)
    first.pop('wall_s'); second.pop('wall_s')
    assert first == second
    assert first['matching']['image_windows'] <= 4096
    assert first['matching']['descriptors'] <= 8192
    assert first['trajectory_coefficients'] < 2048
    assert first['available_refinement_coefficients'] == 2048-first['trajectory_coefficients']
    assert first['storage']['estimated_raw_bytes'] == first['estimated_rows']*4096
    assert 'odo_left_diameter_m' not in json.dumps(first)


def test_large_nominal_lattice_is_rejected_before_allocating(monkeypatch):
    import ssb_tools.reconstruction_budget as budget
    config, calibration = nominal.__wrapped__()
    monkeypatch.setattr(budget, 'MAXIMUM_PLANNING_LATTICE_ROWS', 10)
    monkeypatch.setattr(budget.np, 'arange', lambda *a, **k: pytest.fail('allocated before budget check'))
    with pytest.raises(ValueError, match='bounded partitions'):
        plan(config, calibration, 8., 3., reconstruction_settings(.02, True, True, True))


def test_cli_records_failed_storage_preflight_before_capture(tmp_path, monkeypatch):
    import ssb_tools.reconstruction_budget as budget
    config, calibration = nominal.__wrapped__()
    demo = tmp_path/'demo'; demo.mkdir()
    (demo/'capture.yaml').write_text(yaml.safe_dump(config))
    (demo/'calibration.json').write_text(json.dumps(calibration))
    output = tmp_path/'resource_plan.json'
    monkeypatch.setattr(budget.shutil, 'disk_usage', lambda _: SimpleNamespace(free=1))
    monkeypatch.setattr('sys.argv', ['budget', '--demo', str(demo), '--output', str(output),
                                   '--start', '8', '--length', '3', '--relative-encoder-scale'])
    with pytest.raises(SystemExit, match='insufficient free storage'):
        budget.main()
    report = json.loads(output.read_text())
    assert report['status'] == 'fail'
    assert report['storage']['available_bytes'] == 1
    assert report['storage']['approximate_free_bytes_required'] > 1
