import copy
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest
from scipy.spatial.transform import Rotation

from ssb_tools.mission_plan import plan
from ssb_tools.mission_preview import Preview, textured_dae


@pytest.fixture
def config():
    return dict(tunnel=dict(x_min_m=-1.5, x_max_m=21.5), camera=dict(fov_at_nominal_m=.85),
                scan_encoder=dict(ppr=2500, edges_per_cycle=4),
                rescaler=dict(multiply=128, divide=15),
                motion=dict(advance_per_rev_m=.6, line_rate_hz=28444.444444444445), acceptance={})


@pytest.mark.parametrize('distance', [.05, .1, .2, 3., 20.])
def test_distance_profile_integrates_to_requested_travel(config, distance):
    before = copy.deepcopy(config)
    c, task = plan(config, 0., distance)
    p = np.array(c['motion']['profile'])
    integral = np.sum(np.diff(p[:, 0])*(p[1:, 1]+p[:-1, 1])/2)
    assert integral*task['speed_m_s'] == pytest.approx(distance, abs=1e-12)
    assert task['speed_m_s'] == pytest.approx(.2)
    assert np.all(np.diff(p[:, 0]) > 0)
    assert config == before


@pytest.mark.parametrize('start,distance', [(-.001, 1), (19., 2.), (0., 0.), (0., .01), (0., float('nan')), (float('inf'), 1)])
def test_out_of_bounds_and_nonfinite_tasks_rejected(config, start, distance):
    with pytest.raises(ValueError): plan(config, start, distance)


def test_preview_head_follows_body_attitude_and_negative_scan_axis(tmp_path):
    pytest.importorskip('geometry_msgs')
    world = tmp_path/'world.sdf'
    world.write_text('''<sdf><world><model name="scan_car"><pose>3 0 0 0 0 0</pose>
      <link name="base"><pose>0 0 .25 0 0 0</pose></link>
      <link name="head"><pose>0 0 1.97 0 0 0</pose></link></model></world></sdf>''')
    p = Preview(world, tmp_path/'cache')
    from builtin_interfaces.msg import Time
    body = Rotation.from_euler('xyz', [.1, -.2, .3])
    state = dict(base_pose=[4, .02, .251, *body.as_quat()], scan=.7, wheel_angles=[0.]*4)
    frames = p.frames(state, Time())
    assert frames[1].header.frame_id == 'sim_truth/base'
    tr = frames[1].transform
    assert tr.translation.z == pytest.approx(1.72)
    q = tr.rotation
    head = body*Rotation.from_quat([q.x, q.y, q.z, q.w])
    expected = body*Rotation.from_rotvec([-.7, 0, 0])
    np.testing.assert_allclose(head.as_matrix(), expected.as_matrix(), atol=1e-12)


def test_textured_preview_preserves_uv_winding_and_caps_image(tmp_path):
    mesh = tmp_path/'m.obj'; texture = tmp_path/'t.png'
    mesh.write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 0 1\nf 1/1 2/2 3/3\n')
    Image.new('RGB', (1200, 800), (70, 90, 80)).save(texture)
    target = textured_dae(mesh, texture, tmp_path/'cache')
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    doc = ET.parse(target)
    assert doc.findtext('.//c:image/c:init_from', namespaces=ns) == target.with_suffix('.png').name
    assert doc.findtext('.//c:triangles/c:p', namespaces=ns) == '0 0 1 1 2 2'
    with Image.open(target.with_suffix('.png')) as image: assert image.size == (512, 341)
    before = target.stat().st_mtime_ns
    assert textured_dae(mesh, texture, tmp_path/'cache') == target
    assert target.stat().st_mtime_ns == before


@pytest.mark.parametrize('message', ['[]', 'null', '{"id":"x","action":"unknown"}', '{'])
def test_invalid_command_cannot_crash_manager_or_poison_queue(message):
    pytest.importorskip('rclpy')
    import queue
    import threading
    from types import SimpleNamespace
    from std_msgs.msg import String
    from ssb_tools.mission_manager import MissionManager
    fake = SimpleNamespace(commands=queue.Queue(), lock=threading.RLock(), error='')
    MissionManager.enqueue(fake, String(data=message))
    assert fake.error and fake.commands.empty()
    MissionManager.enqueue(fake, String(data='{"id":"good","action":"pause"}'))
    assert fake.commands.get_nowait()['id'] == 'good'
