"""Independent rigid transforms: external mount, shaft and camera share one frame."""
import copy
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from ssb_tools.stage_b_scene import make_world
from ssb_tools.physical_world import check
from ssb_tools.optical_bench import prepare

from test_physical_world import built, failed


@pytest.mark.parametrize('field,value', [('dy_m',.02),('dz_m',-.02),
                                        ('tilt_y_rad',.001),('tilt_z_rad',-.001)])
def test_nonzero_mount_agrees_with_independent_shaft_transform(built,tmp_path,field,value):
    _, base, spec = built
    config = copy.deepcopy(base); config['truth']['mount'][field] = value
    make_world(tmp_path,config,spec)
    report = check(config,spec,tmp_path/'world.sdf')
    assert report['passed'], failed(report)
    car = ET.parse(tmp_path/'world.sdf').find("world/model[@name='scan_car']")
    h = np.array(list(map(float,car.findtext("link[@name='head']/pose").split())))
    m = config['truth']['mount']; z = sum(config['robot'].values())
    expected_rotation = Rotation.from_rotvec([0,0,m['tilt_z_rad']])*Rotation.from_rotvec([0,m['tilt_y_rad'],0])
    np.testing.assert_allclose(h[:3],[config['truth']['head_mount_x_m'],m['dy_m'],z+m['dz_m']],atol=1e-12)
    for theta in (-2.,0.,.7,2.):
        actual = Rotation.from_euler('xyz',h[3:])*Rotation.from_rotvec([-theta,0,0])
        expect = expected_rotation*Rotation.from_rotvec([-theta,0,0])
        np.testing.assert_allclose(actual.as_matrix(),expect.as_matrix(),atol=1e-12)
    # Stator bearing centre follows the same mount, while base lights stay fixed.
    p = np.array(list(map(float,car.findtext("link[@name='base']/visual[@name='front_bearing']/pose").split())))
    centre = np.array([spec['robot']['head_cradle_length_m']/2,0,0])
    expected = h[:3]+expected_rotation.apply(centre)-[0,0,config['robot']['base_reference_z_m']]
    np.testing.assert_allclose(p[:3],expected,atol=5e-10)
    assert car.findtext("joint[@name='scan']/axis/xyz")=='-1 0 0'
    # Editing the config alone must not silently change the optical mount.
    config['truth']['mount'][field] += value
    assert 'scanner_assembly_matches_truth' in failed(check(config,spec,tmp_path/'world.sdf'))


@pytest.mark.parametrize('part', ['head','axis','axis_frame'])
def test_world_mount_tampering_is_rejected(built,tmp_path,part):
    _,config,spec = built
    make_world(tmp_path,config,spec)
    tree = ET.parse(tmp_path/'world.sdf'); car=tree.find("world/model[@name='scan_car']")
    if part=='head':
        node=car.find("link[@name='head']/pose"); p=list(map(float,node.text.split())); p[5]=.001
        node.text=' '.join(map(str,p))
    else:
        node=car.find("joint[@name='scan']/axis/xyz")
        if part=='axis':node.text='1 0 0'
        else:node.set('expressed_in','__model__')
    tree.write(tmp_path/'world.sdf')
    assert 'scanner_assembly_matches_truth' in failed(check(config,spec,tmp_path/'world.sdf'))


def test_centered_bench_preserves_mount_identity_but_does_not_publish_extrinsics(tmp_path):
    from test_sensor_noise_demo import bundle
    import yaml
    root=tmp_path/'rig';bundle(root)
    cfg=yaml.safe_load((root/'capture.yaml').read_text())
    cfg['truth']['mount'].update(dy_m=.02,dz_m=-.013,tilt_y_rad=.001,tilt_z_rad=-.0015)
    (root/'capture.yaml').write_text(yaml.safe_dump(cfg))
    meta=prepare(root/'capture.yaml',tmp_path/'bench')
    assert meta['centered_bench']
    assert 'dy_m' not in json.dumps(meta) and 'tilt_z_rad' not in json.dumps(meta)
    for target in meta['targets'].values():
        generated=yaml.safe_load((tmp_path/'bench'/target['config']).read_text())
        assert generated['truth']['mount']==cfg['truth']['mount']


def test_rviz_rotates_about_the_mounted_shaft_not_the_vehicle_axis(built,tmp_path):
    from ssb_tools.mission_preview import Preview
    from builtin_interfaces.msg import Time
    _, base, spec = built
    cfg=copy.deepcopy(base)
    cfg['truth']['mount'].update(dy_m=.02,dz_m=-.013,tilt_y_rad=.001,tilt_z_rad=-.0015)
    make_world(tmp_path,cfg,spec)
    preview=Preview(tmp_path/'world.sdf',tmp_path/'cache')
    for theta in (-2.,0.,.7,2.):
        status=dict(base_pose=[8.,0.,.3,0.,0.,0.,1.],scan=theta,wheel_angles=[0.]*4)
        frame=next(f for f in preview.frames(status,Time()) if f.child_frame_id=='sim_truth/head')
        rot=frame.transform.rotation
        actual=Rotation.from_quat([rot.x,rot.y,rot.z,rot.w])
        expect=(Rotation.from_rotvec([0,0,-.0015])*Rotation.from_rotvec([0,.001,0])*
                Rotation.from_rotvec([-theta,0,0]))
        np.testing.assert_allclose(actual.as_matrix(),expect.as_matrix(),atol=1e-12)
        translation=frame.transform.translation
        np.testing.assert_allclose([translation.x,translation.y,translation.z],[0,.02,1.702],atol=1e-12)
