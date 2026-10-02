#!/usr/bin/env python3
"""Real RViz command watchdog regression with a fake manager and no Gazebo.

Run inside the Mesa/ROS environment, with no other mission manager or RViz open.
Full output belongs in a log. The lost-ack test waits the actual 90 second timeout.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[1]
    rclpy.init(); node = rclpy.create_node('ssb_panel_watchdog_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status = node.create_publisher(String, '/ssb/mission/status', qos)
    requests = node.create_publisher(String, '/ssb/mission/review', 10)
    commands = []
    # Receive commands but deliberately do not acknowledge them.
    receiver = node.create_subscription(String, '/ssb/mission/command',
        lambda message: commands.append(json.loads(message.data)), 10)
    state = dict(state='idle', scan_pitch_m=.6, scan_rad=3.141592653589793,
                 output='', command_result={}, error='', task={})
    publishing = [True]
    timer = node.create_timer(.1, lambda: status.publish(String(data=json.dumps(state)))
                             if publishing[0] else None)

    def wait(predicate, timeout=20):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if predicate(): return
        raise AssertionError('Panel test timed out')

    def review(name, predicate):
        last = [0.]; path = output/(name+'_panel.json')
        def ready():
            if time.monotonic()-last[0] > .5:
                requests.publish(String(data=name)); last[0] = time.monotonic()
            try: return predicate(json.loads(path.read_text()))
            except (FileNotFoundError, ValueError): return False
        wait(ready)
        return json.loads(path.read_text())

    def press_start(panel):
        # Use the actual button position; window focus/shortcuts can be unreliable on WSLg.
        subprocess.run(['xdotool', 'mousemove', str(panel['begin_center_x']),
                        str(panel['begin_center_y']), 'click', '1'], check=True)

    def click_start(panel):
        expected = len(commands)+1; last = [0.]
        def received():
            # A WSLg activation click can be consumed before reaching the button.
            # Once a command is sent the button is disabled, so later clicks cannot resend it.
            if time.monotonic()-last[0] > .5:
                press_start(panel); last[0] = time.monotonic()
            return len(commands) >= expected
        wait(received, timeout=10)
        assert len(commands) == expected

    report = {}
    with (output/'rviz.log').open('w') as log:
        viewer = subprocess.Popen(['rviz2', '-d', str(repo/'install/ssb_rviz/share/ssb_rviz/config/mission.rviz')],
            env=dict(os.environ, SSB_RVIZ_REVIEW_DIR=str(output)),
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            initial = review('initial', lambda value: value['connected'] and value['begin_enabled'])
            assert initial['command_timeout_ms'] == 90000
            assert initial['task_mode'] == 'travel'
            # WSLg can consume the first click to activate the window.
            retry_at = [0.]
            def select_wall(value):
                if value.get('task_mode') == 'wall': return True
                if time.monotonic()-retry_at[0] > 1.:
                    subprocess.run(['xdotool', 'mousemove', str(value['mode_center_x']),
                                    str(value['mode_center_y']), 'click', '1', 'key', 'End', 'Return'], check=True)
                    retry_at[0] = time.monotonic()
                return False
            initial = review('wall_selected', select_wall)
            assert 'Target wall: 3.000' in initial['extent']
            assert 'overscan' in initial['extent']
            click_start(initial)
            assert commands[-1]['mode'] == 'wall'
            pending = review('pending_disconnect', lambda value: bool(value['pending']))
            assert not pending['begin_enabled'] and pending['command_timer_active']
            publishing[0] = False
            disconnected = review('disconnected', lambda value: not value['connected'])
            assert not disconnected['pending'] and not disconnected['command_timer_active']
            assert not disconnected['begin_enabled']
            publishing[0] = True
            report['reconnected'] = review('reconnected', lambda value: value['connected'] and value['begin_enabled'])

            click_start(report['reconnected'])
            state['command_result'] = dict(id=commands[-1]['id'], ok=False, error='Test rejection')
            state['error'] = 'Test rejection'
            acknowledged = review('acknowledged', lambda value: not value['pending'] and value['begin_enabled'])
            assert not acknowledged['command_timer_active'] and acknowledged['error'] == 'Test rejection'
            report['rejection_ack'] = acknowledged

            node.destroy_subscription(receiver)
            until = time.monotonic()+3.
            wait(lambda: time.monotonic() >= until)
            press_start(acknowledged)
            absent = review('receiver_absent', lambda value: 'receiver unavailable' in value['error'])
            assert not absent['pending'] and absent['begin_enabled'] and len(commands) == 2
            report['receiver_absent'] = absent
            receiver = node.create_subscription(String, '/ssb/mission/command',
                lambda message: commands.append(json.loads(message.data)), 10)

            click_start(absent)
            started = time.monotonic()
            pending = review('pending', lambda value: bool(value['pending']))
            assert not pending['begin_enabled'] and pending['command_timer_active']
            wait(lambda: time.monotonic()-started >= 91., timeout=100)
            restored = review('timeout', lambda value: not value['pending'] and value['begin_enabled'])
            assert 'timed out' in restored['error'] and not restored['command_timer_active']
            report['lost_ack'] = dict(elapsed_s=time.monotonic()-started, restored=restored)
            click_start(restored)
            state['command_result'] = dict(id=commands[-1]['id'], ok=False, error='Test rejection')
            report['post_timeout_command'] = review('post_timeout', lambda value: not value['pending'] and value['begin_enabled'])
            report['status'] = 'passed'
            (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print('Panel watchdog: lost acknowledgement, disconnect/reconnect, rejection and missing receiver passed')
        finally:
            if viewer.poll() is None: os.killpg(viewer.pid, signal.SIGINT)
            viewer.wait(timeout=30)
            (output/'received_commands.json').write_text(json.dumps(commands,indent=2)+'\n')
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
