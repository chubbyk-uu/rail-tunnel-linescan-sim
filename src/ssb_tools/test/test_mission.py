import copy
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest
from scipy.spatial.transform import Rotation

from ssb_tools.mission_plan import plan, start_values
from ssb_tools.mission_preview import Preview, textured_dae


@pytest.fixture
def config():
    return dict(tunnel=dict(x_min_m=-1.5, x_max_m=21.5), camera=dict(fov_at_nominal_m=.85),
                scan_encoder=dict(ppr=2500, edges_per_cycle=4),
                rescaler=dict(multiply=128, divide=15),
                gate=dict(start_deg=-120., end_deg=120.),
                motion=dict(advance_per_rev_m=.6, line_rate_hz=28444.444444444445,
                            start_theta_deg=180.), acceptance={})


@pytest.mark.parametrize('distance', [1., 1.2, 3., 20.])
def test_distance_profile_integrates_to_requested_travel(config, distance):
    before = copy.deepcopy(config)
    c, task = plan(config, 0., distance)
    p = np.array(c['motion']['profile'])
    integral = np.sum(np.diff(p[:, 0])*(p[1:, 1]+p[:-1, 1])/2)
    assert integral*task['speed_m_s'] == pytest.approx(distance, abs=1e-12)
    assert task['speed_m_s'] == pytest.approx(.2)
    assert np.all(np.diff(p[:, 0]) > 0)
    assert config == before


@pytest.mark.parametrize('distance', [1., 1.2, 3., 20.])
@pytest.mark.parametrize('phase', [180., -130., 0., 120.])
def test_exposure_acceptance_is_inside_nominal_gate_span(config, distance, phase):
    config['motion']['start_theta_deg'] = phase
    c, task = plan(config, 3. if distance <= 3. else 0., distance)
    lo, hi = c['acceptance']['valid_x_m']
    # Independent dense spatial samples: a line is available only inside the 240° gate.
    x = np.linspace(0., distance, 100001)
    angle = (phase+360*x/.6+120.) % 360.
    exposed = x[angle < 240.]
    assert exposed.min() < lo-task['start_m'] < hi-task['start_m'] < exposed.max()
    assert lo >= task['start_m']+.11-1e-12
    assert hi <= task['end_m']-.21+1e-12
    assert task['exposure_acceptance_x_m'] == [lo, hi]


def test_acceptance_guard_does_not_hide_an_interior_missing_row(config):
    from ssb_tools.validate_stage_a import valid_region
    c, _ = plan(config, 3., 3.)
    poses = np.array([(0., 3., 1.), (3., 6., 1.)],
                     dtype=[('t', 'f8'), ('x', 'f8'), ('v', 'f8')])
    truth = np.array([(3.10021,), (4.,), (5.90003,)], dtype=[('x', 'f8')])
    dropped = np.zeros(0, dtype=[('gated', 'u1'), ('t_lo', 'f8'), ('t_hi', 'f8'), ('reason', 'u1')])
    assert valid_region(c, poses, [], truth, dropped)['state'] == 'pass'
    missing = np.array([(1, 1., 1.001, 2)], dtype=dropped.dtype)
    check = valid_region(c, poses, [], truth, missing)
    assert check['state'] == 'fail' and check['missing_in_region'] == 1


@pytest.mark.parametrize('start,distance', [(-.001, 1), (19., 2.), (0., 0.), (0., .05), (0., .1), (0., .12), (0., .2), (0., .999999), (0., float('nan')), (float('inf'), 1)])
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


@pytest.mark.parametrize('angle,direction', [(180., [0., 0., -1.]),
                                          (-130., [0., -.766044443118978, -.642787609686539])])
def test_idle_and_starting_scan_pose_use_configured_angle(tmp_path, angle, direction):
    pytest.importorskip('rclpy')
    from ssb_tools.mission_manager import initial_state
    from builtin_interfaces.msg import Time
    world = tmp_path/'world.sdf'
    world.write_text('''<sdf><world><model name="scan_car"><pose>0 0 0 0 0 0</pose>
      <link name="base"><pose>0 0 .3 0 0 0</pose></link>
      <link name="head"><pose>0 0 2.015 0 0 0</pose></link></model></world></sdf>''')
    state = initial_state({'motion': {'start_theta_deg': angle}}, [0., 0., .3, 0., 0., 0., 1.])
    q = Preview(world, tmp_path/'cache').frames(state, Time())[1].transform.rotation
    beam = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([0., 0., 1.])
    np.testing.assert_allclose(beam, direction, atol=1e-12)


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


@pytest.mark.parametrize('value', [None, [], {}, True, 'invalid', float('nan'), float('inf')])
@pytest.mark.parametrize('key', ['start_m', 'distance_m'])
def test_malformed_start_rejected_before_queueing_without_failing_active_task(value, key):
    pytest.importorskip('rclpy')
    import json
    import queue
    import threading
    from types import SimpleNamespace
    from std_msgs.msg import String
    from ssb_tools.mission_manager import MissionManager
    fake = SimpleNamespace(commands=queue.Queue(), lock=threading.RLock(), error='',
                           state='running', command_result={})
    command = dict(id='bad', action='start', start_m=3., distance_m=1.)
    command[key] = value
    MissionManager.enqueue(fake, String(data=json.dumps(command)))
    assert fake.commands.empty() and fake.state == 'running'
    assert fake.command_result['id'] == 'bad' and not fake.command_result['ok']
    with pytest.raises(ValueError): start_values(command)


def test_full_command_queue_returns_rejection_with_matching_id():
    pytest.importorskip('rclpy')
    import queue
    import threading
    from types import SimpleNamespace
    from std_msgs.msg import String
    from ssb_tools.mission_manager import MissionManager
    fake = SimpleNamespace(commands=queue.Queue(maxsize=1), lock=threading.RLock(),
                           error='', state='paused', command_result={})
    fake.commands.put({'id': 'previous', 'action': 'pause'})
    MissionManager.enqueue(fake, String(data='{"id":"full","action":"resume"}'))
    assert fake.state == 'paused' and fake.commands.qsize() == 1
    assert fake.command_result['id'] == 'full' and not fake.command_result['ok']
    assert 'queue is full' in fake.command_result['error']


def test_preview_moves_sprung_axles_and_measuring_sliders(tmp_path):
    pytest.importorskip('geometry_msgs')
    from builtin_interfaces.msg import Time
    world = tmp_path/'world.sdf'
    world.write_text('''<sdf><world><model name="scan_car"><pose>3 0 0 0 0 0</pose>
      <link name="base"><pose>0 0 .3 0 0 0</pose></link>
      <link name="wheel_2_axle"><pose>-.35 -.754 .1 0 0 0</pose></link>
      <link name="wheel_2"><pose>-.35 -.754 .1 0 0 0</pose></link>
      <link name="measure_left_slider"><pose>-.52 .754 .04 0 0 0</pose></link>
      <link name="measure_left_wheel"><pose>-.52 .754 .04 0 0 0</pose></link></model></world></sdf>''')
    state = dict(base_pose=[3, 0, .3, 0, 0, 0, 1], scan=0., wheel_angles=[0.]*4, measure_angles=[0.]*2,
                 suspension=[0., 0., .0004, 0.], measure_slides=[-.002, 0.])
    frames = {f.child_frame_id: f.transform.translation for f in Preview(world, tmp_path/'cache').frames(state, Time())}
    assert frames['sim_truth/wheel_2_axle'].z == pytest.approx(.1-.3+.0004)
    assert frames['sim_truth/wheel_2'].z == pytest.approx(.1-.3+.0004)
    assert frames['sim_truth/measure_left_slider'].z == pytest.approx(.04-.3-.002)
    assert frames['sim_truth/measure_left_wheel'].z == pytest.approx(.04-.3-.002)
