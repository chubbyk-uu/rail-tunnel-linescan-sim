#!/usr/bin/env python3
"""Installed mission stack integration: pause, drain, restart, replay and GUI rates.

Run inside tools/with_mesa_runtime.py + tools/with_optix_runtime.sh, sourcing ROS and
install/setup.bash inside the wrappers. Full stdout belongs in a saved log.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import numpy as np
import psutil
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from PIL import Image
from ssb_tools.session import Session, sha256_file


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--performance', action='store_true')
    a = p.parse_args(); output = a.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[1]
    os.environ['GZ_PARTITION'] = 'ssb_mission_test_'+uuid.uuid4().hex
    processes, logs = [], []
    def spawn(name, command, env=None):
        log = (output/(name+'.log')).open('w'); logs.append(log)
        proc = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc); return proc
    def stop(proc):
        if proc.poll() is None: os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=180)
    manager = spawn('manager', ['python3', '-m', 'ssb_tools.mission_manager', '--output-root', str(output/'sessions')])
    rclpy.init(); node = rclpy.create_node('ssb_mission_test')
    latest = {}; history = []; peak = [0]
    def receive(msg):
        value = json.loads(msg.data); latest.clear(); latest.update(value); history.append(value)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/ssb/mission/status', receive, qos)
    commands = node.create_publisher(String, '/ssb/mission/command', 10)
    reviews = node.create_publisher(String, '/ssb/mission/review', 10)
    def wait(predicate, timeout=120):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if manager.poll() is not None: raise AssertionError('manager exited; inspect manager.log')
            try:
                family = [psutil.Process(manager.pid), *psutil.Process(manager.pid).children(recursive=True)]
                family += [psutil.Process(x.pid) for x in processes[1:] if x.poll() is None]
                peak[0] = max(peak[0], sum(x.memory_info().rss for x in family if x.is_running()))
            except psutil.Error: pass
            if predicate(): return
        raise AssertionError('timeout; last status: '+json.dumps(latest, ensure_ascii=False))
    def command(action, **kwargs):
        identifier = uuid.uuid4().hex
        message = dict(id=identifier, action=action, **kwargs)
        commands.publish(String(data=json.dumps(message)))
        wait(lambda: latest.get('command_result', {}).get('id') == identifier)
        return latest['command_result']
    def verify(session):
        with (output/'checks.log').open('a') as log:
            result = subprocess.run(['python3', str(repo/'tools/check_session.py'), str(session)], stdout=log, stderr=subprocess.STDOUT)
        s = Session(session)
        assert not (session/'processed/optical').exists(), 'capture must leave correction until before stitching'
        assert s.summary['status'] == 'complete'
        assert result.returncode == (0 if s.summary['motion']['complete'] else 1)
        for path, digest in s.summary['files'].items(): assert sha256_file(session/path) == digest
        for block in s.raw_index()['blocks']: assert sha256_file(session/'raw'/block['file']) == block['sha256']
        rows = s.metadata('rows'); truth = s.evaluation('row_truth')
        assert len(rows) and np.array_equal(rows['sequence'], np.arange(len(rows)))
        assert np.all(np.diff(rows['t_center']) > 0)
        poses = s.evaluation('pose_stream')
        assert np.allclose(np.diff(poses['t']), .001, atol=1e-10)
        return dict(rows=len(rows), last_row_x=float(truth['x'][-1]),
                    motion_complete=json.loads((session/'session.json').read_text())['motion']['complete'])
    def review_scene(directory, name, state):
        # Read Ogre's framebuffer; WSLg X11 window grabs can be black even when
        # the actual scene renders correctly. Diagnostics are explicitly opt-in.
        panel = directory/(name+'_panel.json')
        last_request = [0.]
        def ready():
            if time.monotonic()-last_request[0] > .5:
                reviews.publish(String(data=name)); last_request[0] = time.monotonic()
            try:
                value = json.loads(panel.read_text())
                return value['state'] == state and not value['pending']
            except (FileNotFoundError, ValueError): return False
        wait(ready, timeout=20)
        with Image.open(directory/(name+'_scene.png')) as im:
            spread = float(np.asarray(im.convert('RGB'), dtype=float).std())
        assert spread > 5., 'RViz scene is blank; inspect framebuffer screenshot'
        return spread
    report = dict(checks={}, performance={})
    try:
        wait(lambda: latest.get('state') == 'idle')
        assert not command('start', start_m=19., distance_m=2.)['ok']
        assert latest['state'] == 'idle' and latest['output'] == ''
        report['checks']['bounds_rejection'] = True
        assert command('start', start_m=2., distance_m=.6)['ok']
        wait(lambda: latest['distance_estimated_m'] > .13 and latest['rows_generated'] > 1000)
        assert command('pause')['ok']
        paused = dict(latest)
        deadline = time.monotonic()+1.5
        wait(lambda: time.monotonic() >= deadline, timeout=5)
        assert latest['state'] == 'paused'
        assert latest['distance_estimated_m'] == paused['distance_estimated_m']
        assert latest['scan_rad'] == paused['scan_rad']
        frozen_generated = latest['rows_generated']
        deadline = time.monotonic()+.6
        wait(lambda: time.monotonic() >= deadline, timeout=3)
        assert latest['rows_generated'] == frozen_generated
        assert latest['rows_saved'] >= paused['rows_saved']
        assert not command('start', start_m=7., distance_m=1.)['ok']
        assert latest['output'] == paused['output'] and latest['state'] == 'paused'
        assert command('resume')['ok']
        wait(lambda: latest['state'] in ('complete', 'failed'))
        assert latest['state'] == 'complete', latest
        first = Path(latest['output']); report['checks']['pause_resume'] = verify(first)
        replay = output/'replay'
        with (output/'replay.log').open('w') as log:
            subprocess.run([str(repo/'install/ssb_core/lib/ssb_core/ssb_render'),
                            '--config', str(first/'evaluation/config_source.yaml'), '--session', str(replay),
                            '--poses', str(first/'evaluation/pose_stream.bin'), '--batch-rows', '333'],
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        originals = list((first/'raw').glob('*.u8')) + list((first/'metadata').glob('*.bin'))
        for path in originals: assert sha256_file(path) == sha256_file(replay/path.relative_to(first))
        report['checks']['replay_identical_files'] = len(originals)
        assert command('start', start_m=4., distance_m=2.)['ok']
        wait(lambda: latest['distance_estimated_m'] > .16 and latest['rows_generated'] > 1000)
        assert command('stop')['ok']
        assert latest['state'] == 'stopped'
        second = Path(latest['output']); early = verify(second)
        assert not early['motion_complete']
        report['checks']['early_stop'] = early
        assert command('start', start_m=6., distance_m=.2)['ok']
        wait(lambda: latest['state'] in ('complete', 'failed'))
        assert latest['state'] == 'complete', latest
        third = Path(latest['output']); report['checks']['restart'] = verify(third)
        assert len({str(first), str(second), str(third)}) == 3
        if a.performance:
            for mode in ('gz', 'rviz', 'both'):
                viewers = []
                if mode in ('gz', 'both'):
                    viewers.append(spawn(mode+'_gz', ['gz', 'sim', '-g', '-v', '3', '--gui-config',
                                                      str(repo/'local_data/stage_b/contact_demo/gui.config')]))
                if mode in ('rviz', 'both'):
                    review = output/(mode+'_review')
                    viewers.append(spawn(mode+'_rviz', ['rviz2', '-d', str(repo/'install/ssb_rviz/share/ssb_rviz/config/mission.rviz'),
                                                        '--ros-args', '-p', 'use_sim_time:=true'],
                                          env=dict(os.environ, SSB_RVIZ_REVIEW_DIR=str(review))))
                    wait(lambda: (review/'panel.json').exists())
                    panel = json.loads((review/'panel.json').read_text())
                    assert panel['connected'] and panel['begin_enabled']
                peak[0] = 0
                assert command('start', start_m=3., distance_m=3.)['ok']
                spread = None
                if mode in ('rviz', 'both'):
                    wait(lambda: latest['rows_generated'] > 1000)
                    spread = review_scene(review, 'running', 'running')
                wait(lambda: latest['state'] in ('complete', 'failed'), timeout=180)
                assert latest['state'] == 'complete', latest
                assert all(v.poll() is None for v in viewers), 'viewer exited'
                s = Path(latest['output']); verify(s)
                perf = json.loads((s/'logs/performance.json').read_text())['summary']
                report['performance'][mode] = dict(dynamics_rtf=perf['dynamics_rtf'],
                                                   imaging_rtf=perf['imaging_progress_rtf'],
                                                   peak_rss_bytes=peak[0], rows=perf['rows'], session=str(s))
                if spread is not None:
                    report['performance'][mode]['rendered_pixel_std'] = spread
                    report['performance'][mode]['complete_pixel_std'] = review_scene(review, 'complete', 'complete')
                for viewer in viewers: stop(viewer)
                if mode in ('rviz', 'both'):
                    assert '[ERROR]' not in (output/(mode+'_rviz.log')).read_text()
        report['status'] = 'passed'
        (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False))
    finally:
        for proc in reversed(processes): stop(proc)
        node.destroy_node(); rclpy.shutdown()
        for log in logs: log.close()


if __name__ == '__main__': main()
