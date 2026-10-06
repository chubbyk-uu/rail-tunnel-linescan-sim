#!/usr/bin/env python3
"""Real paused Gazebo heartbeat and SIGSTOP telemetry-loss regression.

Uses dynamics-only mode so a deliberately frozen process cannot damage raw
capture evidence. Run with the ROS/OptiX environment and redirect output to a log.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import psutil
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def run(output):
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[2]
    env = dict(os.environ, GZ_PARTITION='ssb_telemetry_'+uuid.uuid4().hex,
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(repo/'install/ssb_gazebo/lib')+':'+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''))
    os.environ['GZ_PARTITION'] = env['GZ_PARTITION']
    latest = {}; frozen = []
    rclpy.init(); node = rclpy.create_node('ssb_telemetry_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def receive(message):
        latest.clear(); latest.update(json.loads(message.data))
    node.create_subscription(String, '/ssb/mission/status', receive, qos)
    publisher = node.create_publisher(String, '/ssb/mission/command', 10)
    with (output/'manager.log').open('w') as log:
        manager = subprocess.Popen(['python3', '-m', 'ssb_tools.mission_manager', '--dynamics-only',
            '--output-root', str(output/'sessions'), '--data-root', str(output/'data'),
            '--status-timeout-s', '1.5', '--drain-timeout-s', '5', '--terminate-timeout-s', '2',
            '--kill-timeout-s', '2'], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def wait(predicate, timeout=90):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                assert manager.poll() is None, 'manager exited; inspect manager.log'
                if predicate(): return
            raise AssertionError('telemetry test timeout: '+json.dumps(latest))
        def command(action, **kwargs):
            identifier = uuid.uuid4().hex
            publisher.publish(String(data=json.dumps(dict(id=identifier, action=action, **kwargs))))
            wait(lambda: latest.get('command_result', {}).get('id') == identifier)
            assert latest['command_result']['ok'], latest
        try:
            wait(lambda: latest.get('state') == 'idle' and publisher.get_subscription_count() > 0)
            command('start', start_m=0., distance_m=20.)
            wait(lambda: latest['state'] == 'running')
            command('pause'); wait(lambda: latest['state'] == 'paused')
            until = time.monotonic()+2.5
            wait(lambda: time.monotonic() >= until)
            assert latest['state'] == 'paused' and latest['status_age_s'] < 1.5 and not latest['telemetry_stale']
            paused = dict(latest)
            command('resume'); wait(lambda: latest['state'] == 'running')
            # Freeze the owned Gazebo family, including its launcher. The manager
            # remains live and must distinguish stale telemetry from exit.
            family = psutil.Process(manager.pid).children(recursive=True)
            frozen = [p for p in family if p.name().startswith('gz') or 'gz-sim' in p.name() or p.name() == 'ruby']
            assert frozen, 'no Gazebo child to freeze'
            for child in frozen: child.send_signal(signal.SIGSTOP)
            started = time.monotonic()
            wait(lambda: latest['state'] == 'failed', timeout=10)
            detection_s = time.monotonic()-started
            assert 'telemetry expired' in latest['error']
            failed = dict(latest)
            for child in frozen:
                try: child.send_signal(signal.SIGCONT)
                except psutil.NoSuchProcess: pass
            frozen.clear()
            failure_path = output/'data/mission_runs'/Path(latest['output']).name/'mission_failure.json'
            wait(failure_path.is_file, timeout=15)
            failure = json.loads(failure_path.read_text())
            assert failure['status'] == 'failed'
            until = time.monotonic()+.5
            wait(lambda: time.monotonic() >= until)
            assert latest['state'] == 'failed'
            report = dict(status='pass', paused_heartbeat=paused, failed=failed,
                detection_s=detection_s, failure_record=failure,
                limitation='development dynamics-only fault injection; not raw-capture or frozen holdout acceptance')
            (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps(dict(status='pass', paused_heartbeat=True, stale_failed=True)))
        finally:
            for child in frozen:
                try: child.send_signal(signal.SIGCONT)
                except psutil.NoSuchProcess: pass
            if manager.poll() is None: os.killpg(manager.pid, signal.SIGINT)
            try: manager.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(manager.pid, signal.SIGKILL); manager.wait(timeout=5)
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--output', required=True)
    run(**vars(parser.parse_args()))
