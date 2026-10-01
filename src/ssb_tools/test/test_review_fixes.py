import json
import pytest
import numpy as np
import yaml
from pathlib import Path
from ssb_tools.stage_b_robot import make_robot
from ssb_tools.ref_geometry import head_pose
from ssb_tools.optical_identity import ensure_optical_key
from ssb_tools.validate_stage_b import runtime_source_budget


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
