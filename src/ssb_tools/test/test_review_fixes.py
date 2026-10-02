import json
import pytest
import numpy as np
import yaml
from pathlib import Path
from ssb_tools.stage_b_robot import make_robot
from ssb_tools.ref_geometry import head_pose
from ssb_tools.session import sha256_file
from ssb_tools.demo_bundle import export_demo
import shutil
import subprocess
import xml.etree.ElementTree as ET
from ssb_tools.optical_identity import ensure_optical_key
from ssb_tools.validate_stage_b import runtime_source_budget


def test_stage_a_config_matches_original_world_and_optical_centre():
    repo=Path(__file__).resolve().parents[3]
    c=yaml.safe_load((repo/'src/ssb_core/config/stage_a.yaml').read_text())
    car=ET.parse(repo/'src/ssb_gazebo/worlds/stage_a.sdf').find("world/model[@name='scan_car']")
    base_z=float(car.findtext("link[@name='base']/pose").split()[2])
    head_z=float(car.findtext("link[@name='head']/pose").split()[2])
    assert c['robot']['base_reference_z_m']==pytest.approx(base_z)
    assert c['robot']['scan_axis_height_m']==pytest.approx(head_z-base_z)
    truth=dict(c['truth'],tunnel=c['tunnel'],robot=c['robot'])
    origin,_,_=head_pose(np.zeros(1),np.zeros(1),truth)
    assert origin[0,2]==pytest.approx(1.97)


def test_key_is_random_persistent_and_rejects_invalid_values():
    a, b = {'truth': {}}, {'truth': {}}
    ensure_optical_key(a); ensure_optical_key(b)
    assert a != b
    saved = a['truth']['optical_key']
    ensure_optical_key(a)
    assert a['truth']['optical_key'] == saved and len(saved) == 64
    with pytest.raises(ValueError): ensure_optical_key({'truth': {'optical_key': 'public'}})


@pytest.mark.parametrize('absolute', [False, True])
def test_runtime_budget_resolves_against_scene_not_cwd(tmp_path, monkeypatch, absolute):
    assets = tmp_path/'assets'; assets.mkdir()
    surface = assets/'surface.json'
    surface.write_text(json.dumps({'resources': {'gpu_source_budget_bytes': 1234}}))
    elsewhere = tmp_path/'elsewhere'; elsewhere.mkdir(); monkeypatch.chdir(elsewhere)
    scene = {'surface': {'file': str(surface) if absolute else 'surface.json'}}
    assert runtime_source_budget(scene, assets/'scene.json') == 1234


def test_changed_base_frame_matches_sdf_and_independent_reference(tmp_path):
    repo=Path(__file__).resolve().parents[3]
    c=yaml.safe_load((repo/'src/ssb_core/config/stage_b.yaml').read_text())
    spec=yaml.safe_load((repo/'src/ssb_tools/config/stage_b_scene.yaml').read_text())
    c['robot']={'base_reference_z_m':.37,'scan_axis_height_m':1.645}
    car=make_robot(tmp_path,c,spec)
    base_z=float(car.find("link[@name='base']/pose").text.split()[2])
    head_z=float(car.find("link[@name='head']/pose").text.split()[2])
    assert base_z==pytest.approx(.37) and head_z==pytest.approx(2.015)
    for lamp in car.findall("link[@name='base']/light"):
        assert base_z+float(lamp.findtext('pose').split()[2])==pytest.approx(.45)
    body=np.zeros(1,dtype=[(name,'f8') for name in ('body_valid','y','z','roll','pitch','yaw')])
    body['body_valid']=1;body['z']=base_z;body['roll']=np.pi/2
    truth=dict(c['truth'],tunnel=c['tunnel'],robot=c['robot'])
    origin,_,_=head_pose(np.zeros(1),np.zeros(1),truth,body)
    np.testing.assert_allclose(origin[0],[0,-(head_z-base_z),base_z],atol=1e-12)


def test_crack_coordinates_keep_submicrometer_precision_at_150m(tmp_path):
    from ssb_tools.stage_b_defects import SEGMENT, LEGACY_SEGMENT, read_segments
    x=150.000123456789
    path=tmp_path/'segments.bin'
    np.array([(x,.2,x+.002,.201,.0002,.0002)],SEGMENT).tofile(path)
    precise=read_segments(path,{'grid':{'segment_format':'xq64_radius32_le'}})
    assert abs(precise['x0'][0]-x)<1e-10
    np.array([(3.,.2,3.002,.201,.0002,.0002)],LEGACY_SEGMENT).tofile(path)
    legacy=read_segments(path,{})
    assert legacy.dtype==LEGACY_SEGMENT and len(legacy)==1


def test_bundle_relocates_dependencies_and_guard_hashes(tmp_path):
    source=tmp_path/'original';(source/'world').mkdir(parents=True)
    blob=source/'segments.bin';blob.write_bytes(b'crack data')
    defects=source/'defects.json'
    defects.write_text(json.dumps({'files':{'segments.bin':{'file':str(blob),'sha256':sha256_file(blob)}}}))
    surface=source/'surface.json'
    surface.write_text(json.dumps({'schema':'ssb.surface_runtime.v1',
        'adaptive_defects_sha256':sha256_file(defects),
        'inputs':{'config':{'file':str(source/'deleted_historical_config.yaml'),'sha256':'original'}}}))
    scene=source/'scene.json';scene.write_text(json.dumps({
        'surface':{'file':str(surface),'sha256':sha256_file(surface)},
        'defects':{'file':str(defects),'sha256':sha256_file(defects)}}))
    mesh=source/'world/wall.obj';mesh.write_text('v 0 0 0\n')
    (source/'world/world.sdf').write_text(f'<sdf><world><model><link><visual><geometry><mesh><uri>{mesh}</uri></mesh></geometry></visual></link></model></world></sdf>')
    (source/'capture.yaml').write_text(yaml.safe_dump({'truth':{'optical_key':'0'*64},'render':{'optical_scene':str(scene)}}))
    (source/'calibration.json').write_text('{}');(source/'gui.config').write_text('<gui/>')
    bundle=tmp_path/'bundle';export_demo(source,bundle)
    shutil.rmtree(source)
    moved=tmp_path/'new/location';moved.parent.mkdir();shutil.move(bundle,moved)
    config=yaml.safe_load((moved/'capture.yaml').read_text())
    scene_path=moved/config['render']['optical_scene'];packed=json.loads(scene_path.read_text())
    for entry in packed.values():
        assert sha256_file(scene_path.parent/entry['file'])==entry['sha256']
    guard=json.loads((scene_path.parent/packed['surface']['file']).read_text())
    assert guard['adaptive_defects_sha256']==packed['defects']['sha256']
    assert guard['inputs']['config']['sha256']=='original'
    assert 'file' not in guard['inputs']['config']
    assert str(source) not in (moved/'world/world.sdf').read_text()


def test_resource_plan_matches_cpp_record_abi():
    from ssb_tools.stage_b_scene import resource_plan, load_spec
    repo=Path(__file__).resolve().parents[3]
    c=yaml.safe_load((repo/'src/ssb_core/config/stage_b.yaml').read_text())
    spec=load_spec(repo/'src/ssb_tools/config/stage_b_scene.yaml')
    plan=resource_plan(c,spec)
    layout=json.loads(subprocess.check_output([str(repo/'install/ssb_core/lib/ssb_core/ssb_optical_identity'),'--record-layout'],text=True))
    assert plan['pose_record_bytes']==layout['pose_record_bytes']
    assert plan['row_job_bytes']==layout['row_job_bytes']
    assert plan['render_job_queue_bytes']==layout['row_job_bytes']*c['render']['batch_rows']*c['render']['max_queued_batches']


def test_demo_generation_preserves_lens_key_and_preflight_rejects_other_rig(tmp_path):
    from ssb_tools.optical_identity import check_calibration
    repo=Path(__file__).resolve().parents[3]
    config=yaml.safe_load((repo/'src/ssb_core/config/stage_b.yaml').read_text())
    config['motion']['start_theta_deg']=-130.  # legacy input must not set the demo's initial pose
    ensure_optical_key(config)
    scene=tmp_path/'scene.json'
    scene.write_text(json.dumps({'lamp':{},'response_gain':3.2,'indirect_fill_relative':.002,'limitations':''}))
    config['render']['optical_scene']=str(scene)
    cfg=tmp_path/'input.yaml';cfg.write_text(yaml.safe_dump(config))
    executable=repo/'install/ssb_core/lib/ssb_core/ssb_optical_identity'
    identity=subprocess.check_output([str(executable),'--config',str(cfg)],text=True).strip()
    cal=tmp_path/'calibration.json';cal.write_text(json.dumps({'optical_signature':identity}))
    world=tmp_path/'input.sdf'
    world.write_text('<sdf version="1.9"><world name="stage_b"><scene><ambient>0 0 0 1</ambient><shadows>false</shadows></scene><model name="track"/><model name="scan_car"/></world></sdf>')
    demo=tmp_path/'demo'
    subprocess.run(['python3',str(repo/'tools/prepare_contact_demo.py'),'--world',str(world),
                    '--config',str(cfg),'--spec',str(repo/'src/ssb_tools/config/stage_b_scene.yaml'),
                    '--output',str(demo),'--calibration',str(cal)],check=True,capture_output=True)
    generated=yaml.safe_load((demo/'capture.yaml').read_text())
    assert generated['motion']['start_theta_deg']==180.
    assert generated['truth']['lens_k1']==.006
    assert generated['truth']['optical_key']==config['truth']['optical_key']
    assert check_calibration(demo/'capture.yaml',demo/'calibration.json')==identity
    generated['truth']['lens_k1']=.008
    wrong=tmp_path/'wrong.yaml';wrong.write_text(yaml.safe_dump(generated))
    with pytest.raises(subprocess.CalledProcessError):check_calibration(wrong,cal)
