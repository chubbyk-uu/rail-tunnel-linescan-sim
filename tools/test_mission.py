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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from PIL import Image
from ssb_tools.session import Session, sha256_file


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--performance', action='store_true')
    p.add_argument('--wall-only', action='store_true', help='accept one full wall target, without the short mission suite')
    p.add_argument('--wall-start', type=float, default=0.)
    p.add_argument('--wall-length', type=float, default=20.)
    p.add_argument('--viewers', choices=('none', 'gz', 'rviz', 'both'), default='both')
    a = p.parse_args(); output = a.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[1]
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(repo/'install/ssb_gazebo/lib')+(
        ':'+os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] if os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH') else '')
    os.environ['GZ_GUI_PLUGIN_PATH'] = str(repo/'install/ssb_gazebo/lib')+(
        ':'+os.environ['GZ_GUI_PLUGIN_PATH'] if os.environ.get('GZ_GUI_PLUGIN_PATH') else '')
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
    latest = {}; history = []; peak = [0]; resources = []; sampled_at = [0.]
    monitor_started = time.monotonic()
    def receive(msg):
        value = json.loads(msg.data); latest.clear(); latest.update(value); history.append(value)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/ssb/mission/status', receive, qos)
    commands = node.create_publisher(String, '/ssb/mission/command', 10)
    latest_image = {}
    def receive_preview(message):
        assert len(message.data) < 512*1024
        latest_image.clear(); latest_image.update(json.loads(message.data))
    node.create_subscription(String, '/ssb/mission/preview', receive_preview,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    reviews = node.create_publisher(String, '/ssb/mission/review', 10)
    def wait(predicate, timeout=120):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if manager.poll() is not None: raise AssertionError('manager exited; inspect manager.log')
            if time.monotonic()-sampled_at[0] >= .5:
                sample = dict(wall_s=time.monotonic()-monitor_started, state=latest.get('state'),
                              distance_estimated_m=latest.get('distance_estimated_m'),
                              rows_generated=latest.get('rows_generated'), rows_saved=latest.get('rows_saved'),
                              imaging_lag_s=latest.get('imaging_lag_s'), rss_by_role={})
                for i, proc in enumerate(processes):
                    if proc.poll() is not None: continue
                    try:
                        parent = psutil.Process(proc.pid)
                        family = [parent, *parent.children(recursive=True)]
                    except psutil.Error: continue
                    rss = 0
                    for member in family:
                        try: rss += member.memory_info().rss
                        except psutil.Error: pass
                    sample['rss_by_role']['manager' if i == 0 else str(proc.args[0])] = rss
                sample['rss_bytes'] = sum(sample['rss_by_role'].values())
                peak[0] = max(peak[0], sample['rss_bytes'])
                if a.wall_only:
                    try:
                        gpu = subprocess.run(['nvidia-smi', '--query-gpu=memory.used,utilization.gpu',
                                              '--format=csv,noheader,nounits'],
                                             capture_output=True, text=True, timeout=2, check=True)
                        memory, utilization = map(float, gpu.stdout.splitlines()[0].split(','))
                        sample.update(global_gpu_used_bytes=int(memory*1024**2), global_gpu_utilization=utilization)
                    except (OSError, ValueError, subprocess.SubprocessError): pass
                    resources.append(sample)
                sampled_at[0] = time.monotonic()
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
                return (value['state'] == state and not value['pending']
                        and value.get('preview_ready') and value.get('preview_output') == latest['output'])
            except (FileNotFoundError, ValueError): return False
        wait(ready, timeout=20)
        with Image.open(directory/(name+'_scene.png')) as im:
            spread = float(np.asarray(im.convert('RGB'), dtype=float).std())
        assert spread > 5., 'RViz scene is blank; inspect framebuffer screenshot'
        return spread
    def replay_and_validate(session, name):
        replay = output/name
        def run(command, log, stage):
            if not a.wall_only:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
                return
            started = time.monotonic(); rss_peak = 0
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                while proc.poll() is None:
                    try:
                        parent = psutil.Process(proc.pid)
                        family = [parent, *parent.children(recursive=True)]
                    except psutil.Error: family = []
                    rss = 0
                    for member in family:
                        try: rss += member.memory_info().rss
                        except psutil.Error: pass
                    rss_peak = max(rss_peak, rss)
                    time.sleep(.5)
                assert proc.returncode == 0, f'{stage} failed; inspect {name}.log'
            finally:
                if proc.poll() is None: os.killpg(proc.pid, signal.SIGINT); proc.wait(timeout=180)
            report.setdefault('offline_stages', {})[stage] = dict(wall_s=time.monotonic()-started,
                                                               sampled_process_tree_peak_rss_bytes=rss_peak)
        with (output/(name+'.log')).open('w') as log:
            run([str(repo/'install/ssb_core/lib/ssb_core/ssb_render'),
                            '--config', str(session/'evaluation/config_source.yaml'), '--session', str(replay),
                            '--poses', str(session/'evaluation/pose_stream.bin'), '--batch-rows', '333'],
                           log, 'independent_replay')
            run([str(repo/'install/ssb_tools/lib/ssb_tools/validate_stage_b_smoke'),
                            str(session), '--compare', str(replay)],
                           log, 'stage_b_validation')
        originals = list((session/'raw').glob('*.u8')) + list((session/'metadata').glob('*.bin'))
        for path in originals: assert sha256_file(path) == sha256_file(replay/path.relative_to(session))
        smoke = json.loads((session/'evaluation/reports/stage_b_smoke.json').read_text())
        return dict(identical_files=len(originals), acceptance=smoke)
    report = dict(checks={}, performance={})
    try:
        wait(lambda: latest.get('state') == 'idle')
        assert abs(latest['scan_rad']-np.pi) < 1e-12, 'default camera must face down before launch'
        report['checks']['initial_scan_rad'] = latest['scan_rad']
        if a.wall_only:
            viewers = []; review = output/'rviz_review'
            if a.viewers in ('gz', 'both'):
                viewers.append(spawn('wall_gz', ['gz', 'sim', '-g', '-v', '3', '--gui-config',
                                                str(repo/'local_data/stage_b/contact_demo/gui.config')]))
            if a.viewers in ('rviz', 'both'):
                viewers.append(spawn('wall_rviz', ['rviz2', '-d', str(repo/'install/ssb_rviz/share/ssb_rviz/config/mission.rviz'),
                                                   '--ros-args', '-p', 'use_sim_time:=true'],
                                      env=dict(os.environ, SSB_RVIZ_REVIEW_DIR=str(review))))
                wait(lambda: (review/'panel.json').exists())
                assert json.loads((review/'panel.json').read_text())['connected']
            began = time.monotonic()
            assert command('start', start_m=a.wall_start, distance_m=a.wall_length, mode='wall')['ok'], latest
            wall = Path(latest['output']); task = dict(latest['task'])
            expected_target = [a.wall_start, a.wall_start+a.wall_length]
            assert task['target_x_m'] == expected_target
            if a.viewers in ('rviz', 'both'):
                wait(lambda: latest['distance_estimated_m'] >= task['distance_m']/2 or latest['state'] == 'failed',
                     timeout=600)
                assert latest['state'] != 'failed', latest
                report['checks']['rviz_running_pixel_std'] = review_scene(review, 'running', 'running')
            wait(lambda: latest['state'] in ('complete', 'failed'), timeout=600)
            finished = time.monotonic()
            assert latest['state'] == 'complete', latest
            assert all(viewer.poll() is None for viewer in viewers), 'viewer exited during capture'
            report['checks']['full_wall_capture'] = verify(wall)
            if a.viewers in ('rviz', 'both'):
                report['checks']['rviz_complete_pixel_std'] = review_scene(review, 'complete', 'complete')
                panel = json.loads((review/'complete_panel.json').read_text())
                assert panel['preview_ready'] and panel['preview_output'] == str(wall)
                assert '[ERROR]' not in (output/'wall_rviz.log').read_text()
            for viewer in viewers: stop(viewer)
            stop(manager)
            perf = json.loads((wall/'logs/performance.json').read_text())['summary']
            active = [sample for sample in resources if sample['state'] in ('starting', 'running', 'draining')]
            report['performance'] = dict(viewers=a.viewers, task=task, capture_end_to_end_s=finished-began,
                dynamics_rtf=perf['dynamics_rtf'], imaging_rtf=perf['imaging_progress_rtf'],
                imaging_finish_lag_s=perf['wall_seconds_after_finish'], render_rows_per_second=perf['render_rows_per_second'],
                peak_rss_bytes=max((sample['rss_bytes'] for sample in active), default=0),
                global_gpu_used_peak_bytes=max((sample.get('global_gpu_used_bytes', 0) for sample in active), default=0),
                rss_note='Sampled sum of process-tree RSS; shared pages may be counted more than once.',
                gpu_note='Global device usage includes unrelated applications, not only this capture.',
                samples=len(resources), raw_bytes=sum(path.stat().st_size for path in (wall/'raw').glob('*.u8')),
                session=str(wall), backend_final=perf['backend_final'])
            (output/'resources.json').write_text(json.dumps(resources, indent=2)+'\n')
            from ssb_tools.wall_coverage import inspect_session
            coverage = inspect_session(wall, repo/'local_data/stage_b/contact_demo/calibration.json',
                                       wall/'reconstruction/coverage')
            report['checks']['nominal_coverage'] = {k: coverage[k] for k in
                ('nominal_complete', 'recorded_rows', 'missing_pixels', 'missing_fraction', 'overlap_pixels')}
            assert coverage['nominal_complete'], coverage['missing_bounds_bins']
            replay = replay_and_validate(wall, 'wall_replay')
            report['checks']['stage_b'] = replay['acceptance']
            report['checks']['identical_replay_files'] = replay['identical_files']
            from ssb_tools.validate_contact import validate
            contact = validate(Path(str(wall)+'_dynamics'), wall/'evaluation/config_source.yaml')
            report['checks']['contact'] = contact
            assert contact['passed'], contact
            assert perf['imaging_progress_rtf'] >= .6, report['performance']
            report['acceptance_end_to_end_s'] = time.monotonic()-monitor_started
            report['status'] = 'passed'
            evaluation = output/'evaluation'; evaluation.mkdir(exist_ok=True)
            (evaluation/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            from ssb_tools.provenance import stage_record
            provenance = stage_record('full_wall_acceptance',
                [wall/'session.json', wall/'raw/index.json', wall/'logs/performance.json',
                 wall/'reconstruction/coverage/report.json', wall/'evaluation/reports/stage_b_smoke.json',
                 Path(str(wall)+'_dynamics')/'validation.json'],
                [evaluation/'report.json', output/'resources.json'],
                dict(target_x_m=expected_target, viewers=a.viewers, replay_batch_rows=333))
            (evaluation/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
            print(json.dumps({k: report['performance'][k] for k in
                ('session', 'dynamics_rtf', 'imaging_rtf', 'peak_rss_bytes', 'global_gpu_used_peak_bytes')}))
            return
        assert not command('start', start_m=19., distance_m=2.)['ok']
        assert latest['state'] == 'idle' and latest['output'] == ''
        report['checks']['bounds_rejection'] = True
        assert not command('start', start_m=2., distance_m=.999)['ok']
        assert latest['state'] == 'idle' and latest['output'] == ''
        report['checks']['short_travel_rejection'] = True
        assert not command('start', start_m=None, distance_m=1.)['ok']
        assert latest['state'] == 'idle' and latest['output'] == ''
        report['checks']['malformed_start_rejection'] = True
        assert command('start', start_m=2., distance_m=1.)['ok']
        def entered_gate():
            if latest['scan_rad'] < 4*np.pi/3:
                assert latest['rows_generated'] == 0, 'exposure before right lower gate'
            return latest['distance_estimated_m'] > .13 and latest['rows_generated'] > 1000
        wait(entered_gate)
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
        wait(lambda: latest_image.get('status') == 'ready' and latest_image.get('output') == paused['output'])
        import base64, io
        with Image.open(io.BytesIO(base64.b64decode(latest_image['png']))) as image:
            image.load()
            assert list(image.size) == latest_image['preview_size']
            assert max(image.size) <= 512
        assert latest_image['last_row'] < latest['rows_saved']
        report['checks']['raw_thumbnail'] = {k:v for k,v in latest_image.items() if k != 'png'}
        assert not command('start', start_m=7., distance_m=1.)['ok']
        assert latest['output'] == paused['output'] and latest['state'] == 'paused'
        assert command('resume')['ok']
        wait(lambda: latest['state'] in ('complete', 'failed'))
        assert latest['state'] == 'complete', latest
        first = Path(latest['output']); report['checks']['pause_resume'] = verify(first)
        first_angle = float(Session(first).evaluation('row_truth')['theta'][0])
        assert 4*np.pi/3 <= first_angle < 4*np.pi/3+.001, first_angle
        report['checks']['first_exposure_rad'] = first_angle
        replay_check = replay_and_validate(first, 'replay')
        report['checks']['replay_identical_files'] = replay_check['identical_files']
        report['checks']['pause_resume_smoke'] = replay_check['acceptance']
        assert command('start', start_m=4., distance_m=2.)['ok']
        wait(lambda: latest['distance_estimated_m'] > .16 and latest['rows_generated'] > 1000)
        assert command('stop')['ok']
        assert latest['state'] == 'stopped'
        second = Path(latest['output']); early = verify(second)
        assert not early['motion_complete']
        report['checks']['early_stop'] = early
        assert command('start', start_m=6., distance_m=1.)['ok']
        wait(lambda: latest['state'] in ('complete', 'failed'))
        assert latest['state'] == 'complete', latest
        third = Path(latest['output']); report['checks']['restart'] = verify(third)
        assert len({str(first), str(second), str(third)}) == 3
        report['checks']['minimum_travel'] = report['checks']['restart']
        assert command('start', start_m=3., distance_m=3.)['ok']
        wait(lambda: latest['state'] in ('complete', 'failed'))
        assert latest['state'] == 'complete', latest
        fourth = Path(latest['output']); report['checks']['three_m_travel'] = verify(fourth)
        report['checks']['three_m_smoke'] = replay_and_validate(fourth, 'three_m_replay')['acceptance']
        assert command('start', start_m=3., distance_m=3., mode='wall')['ok']
        wait(lambda: latest['state'] in ('complete', 'failed'))
        assert latest['state'] == 'complete', latest
        wall = Path(latest['output']); report['checks']['three_m_wall'] = verify(wall)
        assert latest['task']['target_x_m'] == [3., 6.]
        assert latest['task']['start_m'] < 3. and latest['task']['end_m'] > 6.
        from ssb_tools.wall_coverage import inspect_session
        coverage = inspect_session(wall, repo/'local_data/stage_b/contact_demo/calibration.json',
                                   wall/'reconstruction/coverage')
        assert coverage['nominal_complete'] and coverage['missing_pixels'] == 0
        report['checks']['wall_coverage'] = {k: coverage[k] for k in
                                           ('nominal_complete', 'recorded_rows', 'missing_pixels', 'overlap_pixels')}
        report['checks']['wall_smoke'] = replay_and_validate(wall, 'wall_replay')['acceptance']
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
