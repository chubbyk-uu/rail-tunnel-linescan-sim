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
    return dict(tunnel=dict(x_min_m=-1.5, x_max_m=21.5),
                  mission=dict(inspection_x_m=[0.,20.],vehicle_half_length_m=.56,safety_margin_m=.09,minimum_distance_m=1.), camera=dict(fov_at_nominal_m=.85),
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


@pytest.mark.parametrize('diameter', [.079, .08, .081])
def test_contact_task_uses_public_distance_stop_independent_of_truth(config, diameter):
    config['contact']={'enabled':True}
    config['truth']={'odo_left_diameter_m':diameter,'odo_right_diameter_m':diameter}
    c,task=plan(config,3.,3.)
    stop=c['motion']['distance_stop']
    assert stop['target_m']==3. and stop['brake_distance_m']==pytest.approx(.1)
    assert stop['timeout_s']>task['duration_s']
    assert task['completion_basis']=='dual_encoder_distance_and_park'
    assert task['timeout_s']==stop['timeout_s']
    assert c['truth']==config['truth']


@pytest.mark.parametrize('fault', ['target', 'rate', 'reported_distance', 'completion'])
def test_distance_acceptance_recomputes_encoder_hold_without_world_pose(fault):
    from types import SimpleNamespace
    from ssb_tools.validate_stage_b import distance_stop_check
    poses=np.zeros(1001,dtype=[('t','f8'),('wheel','f8'),('right_wheel','f8'),
        ('wheel_omega','f8'),('right_wheel_omega','f8'),('x','f8')])
    poses['t']=np.arange(1001)*.001
    target=3.;count=10000;indices=np.floor(target/(np.pi*.08/count))
    # A moving prefix followed by a 0.6 s held, quantized final distance.
    angles=indices*2*np.pi/count
    poses['wheel']=poses['right_wheel']=np.minimum(poses['t']/.4,1.)*angles
    achieved=indices*np.pi*.08/count
    stop=dict(target_m=target,hold_s=.5,tolerance_m=.0001,speed_tolerance_m_s=.001)
    cfg=dict(motion=dict(distance_stop=stop,sample_period_s=.001),
        calibration=dict(odo_left_diameter_m=.08,odo_right_diameter_m=.08),
        odometer=dict(ppr=2500,edges_per_cycle=4,gear_ratio=1.))
    motion=dict(complete=True,completion_basis='dual_encoder_distance_and_park',estimated_distance_m=achieved)
    session=SimpleNamespace(summary={'motion':motion})
    poses['x']=1e9 # ignored: teleporting a body cannot prove encoder completion
    assert distance_stop_check(session,cfg,poses)['state']=='pass'
    if fault=='target':stop['target_m']+=.001
    elif fault=='rate':poses['right_wheel_omega'][-100:]=1.
    elif fault=='reported_distance':motion['estimated_distance_m']+=.001
    else:motion['completion_basis']='timed_profile'
    assert distance_stop_check(session,cfg,poses)['state']=='fail'


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


@pytest.mark.parametrize('mode', ['travel', 'wall'])
@pytest.mark.parametrize('start,distance', [(-1., 1.), (19., 2.), (3., .5)])
def test_rejected_plan_preserves_completed_display(config, tmp_path, mode, start, distance):
    pytest.importorskip('rclpy')
    import threading
    from types import SimpleNamespace
    from ssb_tools.mission_manager import MissionManager
    config['robot'] = {'base_reference_z_m': .3}
    demo = tmp_path/'demo'; demo.mkdir()
    (demo/'calibration.json').write_text('{}')
    fake = SimpleNamespace(lock=threading.RLock(), state='complete', session=tmp_path/'completed',
        inputs=tmp_path/'previous_inputs', task={'start_m': 3., 'distance_m': 3.},
        latest={'sim_time': 20., 'base_pose': [6., 0., .3], 'capture': {'rows_saved': 100}},
        events=[{'state': 'complete'}], error='', drain_activity=22, shutdown_deadline=123.,
        demo=demo, config=config, data_root=tmp_path/'data')
    previous = copy.deepcopy({k: v for k, v in vars(fake).items() if k != 'lock'})
    with pytest.raises(ValueError):
        MissionManager.launch(fake, {'start_m': start, 'distance_m': distance, 'mode': mode})
    assert {k: v for k, v in vars(fake).items() if k != 'lock'} == previous
    assert not fake.data_root.exists()


@pytest.mark.parametrize('enabled', [False, True])
def test_manager_wall_preflight_and_generation_use_identical_scale_support(tmp_path, monkeypatch, enabled):
    pytest.importorskip('rclpy')
    import threading
    from types import SimpleNamespace
    import ssb_tools.mission_manager as manager
    demo = tmp_path/'demo'; demo.mkdir()
    (demo/'calibration.json').write_text('{}')
    calls = []
    def preflight(*args, **kwargs):
        calls.append(('preflight', kwargs['relative_encoder_scale']))
    def generation(*args, **kwargs):
        calls.append(('generation', kwargs['relative_encoder_scale']))
        raise RuntimeError('stop after checking the generation policy')
    monkeypatch.setattr(manager, 'wall_plan', preflight)
    monkeypatch.setattr(manager, 'prepare', generation)
    monkeypatch.setattr(manager, 'initial_state', lambda *args: {})
    fake = SimpleNamespace(state='complete', lock=threading.RLock(), demo=demo,
        config={'robot': {'base_reference_z_m': .3}}, data_root=tmp_path/'data',
        args=SimpleNamespace(relative_encoder_scale=enabled))
    with pytest.raises(RuntimeError, match='generation policy'):
        manager.MissionManager.launch(fake, {'mode':'wall', 'start_m':0., 'distance_m':20.})
    assert calls == [('preflight', enabled), ('generation', enabled)]


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


def test_unknown_task_mode_is_rejected_before_queueing():
    pytest.importorskip('rclpy')
    import json
    import queue
    import threading
    from types import SimpleNamespace
    from std_msgs.msg import String
    from ssb_tools.mission_manager import MissionManager
    fake = SimpleNamespace(commands=queue.Queue(), lock=threading.RLock(), error='', state='idle', command_result={})
    command = dict(id='bad-mode', action='start', start_m=3., distance_m=3., mode='unknown')
    MissionManager.enqueue(fake, String(data=json.dumps(command)))
    assert fake.commands.empty() and fake.state == 'idle' and not fake.command_result['ok']


@pytest.mark.parametrize('state', ['running', 'paused'])
def test_stale_gazebo_telemetry_fails_even_when_process_is_alive(monkeypatch, state):
    import threading
    from types import SimpleNamespace
    import ssb_tools.mission_manager as module
    fake = SimpleNamespace(state=state, server=SimpleNamespace(poll=lambda: None),
        lock=threading.RLock(), last_received=100., args=SimpleNamespace(status_timeout_s=10.),
        latest=dict(capture={}, motion_complete=False))
    monkeypatch.setattr(module.time, 'monotonic', lambda: 111.)
    with pytest.raises(RuntimeError, match='telemetry expired'):
        module.MissionManager.check_active(fake)
    # A recent paused heartbeat does not require progress in simulation time.
    fake.last_received = 110.
    module.MissionManager.check_active(fake)


@pytest.mark.parametrize('state', ['idle', 'starting', 'draining', 'complete', 'stopped', 'failed'])
def test_telemetry_watchdog_does_not_replace_startup_or_drain_deadlines(state):
    from types import SimpleNamespace
    from ssb_tools.mission_manager import MissionManager
    MissionManager.check_active(SimpleNamespace(state=state, server=SimpleNamespace(poll=lambda: None)))


def test_worker_marks_stale_task_failed_and_fresh_messages_cannot_resume_it(monkeypatch):
    import queue
    import threading
    from types import SimpleNamespace
    import ssb_tools.mission_manager as module
    fake = SimpleNamespace(state='running', server=SimpleNamespace(poll=lambda: None),
        lock=threading.RLock(), last_received=0., args=SimpleNamespace(status_timeout_s=10.),
        latest=dict(capture={}, motion_complete=False), closing=threading.Event(), commands=queue.Queue())
    fake.check_active = lambda: module.MissionManager.check_active(fake)
    errors = []
    def fail(error):
        errors.append(str(error)); fake.state = 'failed'; fake.closing.set()
    fake.fail = fail
    monkeypatch.setattr(module.time, 'monotonic', lambda: 11.)
    module.MissionManager.work(fake)
    assert fake.state == 'failed' and len(errors) == 1 and 'telemetry expired' in errors[0]
    fake.last_received = 11.
    module.MissionManager.check_active(fake)
    assert fake.state == 'failed'


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


def test_configured_150m_domain_and_vehicle_margin_replace_fixed_constants(config):
    from ssb_tools.mission_plan import mission_limits
    config['tunnel'].update(x_min_m=-1.5,x_max_m=151.5)
    config['mission']['inspection_x_m']=[0.,150.]
    c,task=plan(config,149.,1.)
    assert task['end_m']==150.
    assert mission_limits(c)['clearance_m']==pytest.approx(.65)
    with pytest.raises(ValueError):plan(config,149.1,1.)
    config['mission'].update(vehicle_half_length_m=1.5,safety_margin_m=.1)
    with pytest.raises(ValueError,match='margin'):plan(config,0.,1.)
    config['mission'].update(vehicle_half_length_m=.56,safety_margin_m=.09)
    config['camera']['fov_at_nominal_m']=4.
    assert mission_limits(config)['clearance_m']==pytest.approx(2.09)
    with pytest.raises(ValueError,match='margin'):plan(config,0.,1.)


@pytest.mark.parametrize('change', [None,{'inspection_x_m':[0.,float('inf')]},
    {'inspection_x_m':[20.,0.]},{'inspection_x_m':[-2.,20.]},
    {'vehicle_half_length_m':0.},{'safety_margin_m':-.1},{'minimum_distance_m':.5}])
def test_missing_or_invalid_public_mission_limits_rejected(config,change):
    if change is None:config.pop('mission')
    else:config['mission'].update(change)
    with pytest.raises(ValueError):plan(config,3.,1.)
