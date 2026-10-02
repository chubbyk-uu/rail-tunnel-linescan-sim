import json
import subprocess
import sys
import time
import pytest
from ssb_tools.process_drain import stop_group, positive_timeout


@pytest.mark.parametrize('ignore_term', [False, True])
def test_unresponsive_owned_server_has_bounded_escalation(tmp_path, ignore_term):
    ready = tmp_path/'ready'
    code = '''import signal,time,sys
signal.signal(signal.SIGINT, signal.SIG_IGN)
if sys.argv[2]=='1': signal.signal(signal.SIGTERM, signal.SIG_IGN)
open(sys.argv[1],'w').close()
while True: time.sleep(.05)
'''
    proc = subprocess.Popen([sys.executable, '-c', code, str(ready), str(int(ignore_term))],
                            start_new_session=True)
    try:
        deadline = time.monotonic()+5
        while not ready.exists() and time.monotonic() < deadline: time.sleep(.01)
        assert ready.exists()
        result = stop_group(proc, .1, .1, 2.)
        assert result['forced'] and result['elapsed_s'] < 3.
        assert result['signals'] == ['SIGINT','SIGTERM']+(['SIGKILL'] if ignore_term else [])
        assert proc.poll() is not None
    finally:
        if proc.poll() is None: proc.kill(); proc.wait(timeout=3)


@pytest.mark.parametrize('value', [0., -1., float('inf'), float('nan')])
def test_unbounded_or_invalid_timeout_rejected(value):
    with pytest.raises(ValueError): positive_timeout(value)


def test_manager_marks_failed_before_waiting_and_preserves_capture_status(tmp_path):
    pytest.importorskip('rclpy')
    import threading
    from types import SimpleNamespace, MethodType
    from ssb_tools.mission_manager import MissionManager
    ready=tmp_path/'ready'; session=tmp_path/'session'; session.mkdir()
    (session/'session.json').write_text('{"status":"running"}')
    proc=subprocess.Popen([sys.executable,'-c',
        "import signal,time,sys; signal.signal(signal.SIGINT,signal.SIG_IGN); open(sys.argv[1],'w').close(); time.sleep(20)",
        str(ready)],start_new_session=True)
    fake=SimpleNamespace(lock=threading.RLock(),state='running',error='',events=[],latest={'sim_time':1.},
        session=session,server=proc,drain_activity=0,shutdown_deadline=0.,
        args=SimpleNamespace(drain_timeout_s=.3,terminate_timeout_s=.2,kill_timeout_s=1.))
    fake.transition=MethodType(MissionManager.transition,fake)
    fake.stop_server=MethodType(MissionManager.stop_server,fake)
    fake.get_logger=lambda:SimpleNamespace(error=lambda _:None)
    worker=threading.Thread(target=lambda:MissionManager.fail(fake,RuntimeError('injected error')))
    try:
        deadline=time.monotonic()+5
        while not ready.exists() and time.monotonic()<deadline:time.sleep(.01)
        assert ready.exists()
        worker.start()
        deadline=time.monotonic()+.2
        while fake.state!='failed' and time.monotonic()<deadline:time.sleep(.01)
        assert fake.state=='failed' and proc.poll() is None
        worker.join(timeout=3);assert not worker.is_alive() and fake.server is None
        failure=json.loads((session/'evaluation/mission_failure.json').read_text())
        assert failure['status']=='failed' and failure['shutdown']['forced']
        assert json.loads((session/'session.json').read_text())['status']=='running'
    finally:
        if proc.poll() is None:proc.kill();proc.wait(timeout=2)
        if worker.ident:worker.join(timeout=3)


def test_exited_launcher_does_not_leave_ignoring_child_alive(tmp_path):
    import os, signal
    ready=tmp_path/'child'
    proc=subprocess.Popen([sys.executable,'-c', '''import signal,os,time,sys
signal.signal(signal.SIGINT,signal.SIG_IGN)
if os.fork()==0:
 signal.signal(signal.SIGTERM,signal.SIG_IGN)
 open(sys.argv[1],'w').write(str(os.getpid()))
while True:time.sleep(.05)
''',str(ready)],start_new_session=True)
    try:
        deadline=time.monotonic()+5
        while not ready.exists() and time.monotonic()<deadline:time.sleep(.01)
        assert ready.exists()
        result=stop_group(proc,.1,.1,2.)
        assert result['signals']==['SIGINT','SIGTERM','SIGKILL']
        assert result['forced'] and proc.returncode==-signal.SIGTERM
    finally:
        try:os.killpg(proc.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        proc.wait(timeout=3)


@pytest.mark.parametrize('failure_at', ['prepare', 'calibration', 'shutdown', 'wall_read'])
def test_new_attempt_failure_never_modifies_completed_session(tmp_path, monkeypatch, failure_at):
    pytest.importorskip('rclpy')
    import threading
    from types import SimpleNamespace, MethodType
    import ssb_tools.mission_manager as module
    old = tmp_path/'completed'; (old/'evaluation').mkdir(parents=True)
    (old/'session.json').write_text('{"status":"complete"}')
    (old/'evaluation/mission.json').write_text('{"events":["old"]}')
    before = {str(p.relative_to(old)): p.read_bytes() for p in old.rglob('*') if p.is_file()}
    closing = threading.Event()
    if failure_at == 'shutdown': closing.set()
    config = {'motion': {'start_theta_deg': 180.}, 'robot': {'base_reference_z_m': .3}}
    fake = SimpleNamespace(lock=threading.RLock(), state='complete', session=old,
        inputs=tmp_path/'old_inputs', events=[{'old': True}], error='', task={'old': True},
        latest={'sim_time': 99.}, data_root=tmp_path, root=tmp_path/'sessions',
        demo=tmp_path/'missing_demo', config=config, server=None, closing=closing)
    fake.transition = MethodType(module.MissionManager.transition, fake)
    fake.get_logger = lambda: SimpleNamespace(error=lambda _: None)
    monkeypatch.setattr(module, 'plan', lambda *args: None)
    def prepare(*args, **kwargs):
        if failure_at == 'prepare': raise OSError('prepare failed')
        return config, {'start_m': 3.}
    monkeypatch.setattr(module, 'prepare', prepare)
    def calibration(*args):
        if failure_at == 'calibration': raise subprocess.CalledProcessError(2, 'identity')
    monkeypatch.setattr(module, 'check_calibration', calibration)
    command = {'start_m': 3., 'distance_m': 1., 'mode': 'wall' if failure_at == 'wall_read' else 'travel'}
    with pytest.raises((OSError, subprocess.CalledProcessError, RuntimeError)) as raised:
        module.MissionManager.launch(fake, command)
    module.MissionManager.fail(fake, raised.value)
    after = {str(p.relative_to(old)): p.read_bytes() for p in old.rglob('*') if p.is_file()}
    assert after == before
    assert fake.session is None and not fake.root.exists()
    report = json.loads((fake.inputs/'mission_failure.json').read_text())
    assert report['status'] == 'failed' and len(report['events']) == 1
    assert report['events'][0]['state'] == 'failed' and report['events'][0]['sim_time'] == 0.
    assert fake.task['start_m'] == 3. and 'old' not in fake.task


def test_failure_before_capture_does_not_create_phantom_session(tmp_path):
    pytest.importorskip('rclpy')
    import threading
    from types import SimpleNamespace, MethodType
    from ssb_tools.mission_manager import MissionManager
    fake = SimpleNamespace(lock=threading.RLock(), state='starting', error='', events=[],
        latest={'sim_time': 0.}, session=tmp_path/'not_created', inputs=tmp_path/'attempt', server=None)
    fake.transition = MethodType(MissionManager.transition, fake)
    fake.get_logger = lambda: SimpleNamespace(error=lambda _: None)
    MissionManager.fail(fake, RuntimeError('startup failed'))
    assert not fake.session.exists()
    assert (fake.inputs/'mission_failure.json').is_file()


def test_rejected_start_preserves_active_attempt(tmp_path):
    pytest.importorskip('rclpy')
    from types import SimpleNamespace
    from ssb_tools.mission_manager import MissionManager
    fake = SimpleNamespace(state='running', session=tmp_path/'active', events=[{'active': True}])
    with pytest.raises(ValueError, match='already active'):
        MissionManager.launch(fake, {'start_m': 3., 'distance_m': 1.})
    assert fake.session == tmp_path/'active' and fake.events == [{'active': True}]


@pytest.mark.parametrize('advances', [True, False])
def test_drain_extends_for_work_but_not_heartbeats(tmp_path, advances):
    import threading
    # Work notifications use an anonymous pipe, not persistent small files.
    proc = subprocess.Popen([sys.executable, '-u', '-c', '''import signal,time
def drain(*args):
 for i in range(12):
  print(i+1,flush=True);time.sleep(.06)
 raise SystemExit(0)
signal.signal(signal.SIGINT,drain)
print('ready',flush=True)
while True:time.sleep(.05)
'''], stdout=subprocess.PIPE, text=True, start_new_session=True)
    assert proc.stdout.readline().strip() == 'ready'
    counter = [0]
    def receive():
        for line in proc.stdout:
            counter[0] = int(line) if advances else 0
    reader = threading.Thread(target=receive); reader.start()
    try:
        result = stop_group(proc, .25, .2, 1., progress=lambda: counter[0])
        if advances:
            assert not result['forced'] and result['returncode'] == 0
            assert result['elapsed_s'] > .6 and result['progress_extensions'] > 1
        else:
            assert result['forced'] and result['progress_extensions'] == 0
            assert result['elapsed_s'] < .6
    finally:
        if proc.poll() is None: proc.kill(); proc.wait(timeout=3)
        reader.join(timeout=3); proc.stdout.close()


def test_capture_reports_merge_without_replacing_pose_and_ignore_stale_sessions(tmp_path):
    pytest.importorskip('rclpy')
    import threading
    from types import SimpleNamespace
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from ssb_tools.mission_manager import MissionManager
    fake = SimpleNamespace(lock=threading.RLock(), session=tmp_path/'new',
        latest={'sim_time': 12., 'base_pose': [1, 2, 3]}, drain_activity=0)
    def report(session, sequence, rows):
        MissionManager.receive_capture(fake, StringMsg(data=json.dumps(dict(session=str(session),
            capture=dict(activity_sequence=sequence, rows_saved=rows, phase='syncing')))))
    report(fake.session, 10, 100)
    report(tmp_path/'old', 1000, 999)
    report(fake.session, 9, 99)
    assert fake.drain_activity == 10 and fake.latest['capture']['rows_saved'] == 100
    report(fake.session, 11, 100)  # durable work with no new rows
    assert fake.drain_activity == 11 and fake.latest['sim_time'] == 12.
    assert fake.latest['base_pose'] == [1, 2, 3]


def test_close_respects_extended_drain_deadline(monkeypatch):
    pytest.importorskip('rclpy')
    import threading
    from types import SimpleNamespace
    import ssb_tools.mission_manager as module
    now = [0.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    fake = SimpleNamespace(closing=threading.Event(), lock=threading.RLock(),
        shutdown_deadline=0., server=None, gui=None, log_handles=[],
        ownership=SimpleNamespace(close=lambda: None),
        args=SimpleNamespace(drain_timeout_s=1., terminate_timeout_s=1., kill_timeout_s=1.))
    class Worker:
        def is_alive(self): return now[0] < 100.
        def join(self, timeout):
            now[0] += timeout
            fake.shutdown_deadline = now[0]+20.
    fake.worker = Worker()
    fake.fail = lambda error: pytest.fail(str(error))
    module.MissionManager.close(fake)
    assert now[0] == 100. and fake.closing.is_set()
