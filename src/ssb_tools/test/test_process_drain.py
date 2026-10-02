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
        session=session,server=proc,args=SimpleNamespace(drain_timeout_s=.3,terminate_timeout_s=.2,kill_timeout_s=1.))
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
