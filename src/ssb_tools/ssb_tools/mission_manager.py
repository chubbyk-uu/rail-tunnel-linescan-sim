"""One owner for Gazebo lifetime, mission commands and raw capture drain."""
import argparse
from collections import deque
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray
import yaml
from gz.transport13 import Node as GzNode
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.world_control_pb2 import WorldControl

from .mission_plan import prepare, plan, start_values, wall_plan, mission_limits
from .mission_preview import Preview, pose_values, transform
from .mission_image_preview import RawImagePreview
from .optical_identity import check_calibration
from .session import sha256_file
from .process_drain import stop_group, positive_timeout


TERMINAL = ('idle', 'complete', 'stopped', 'failed')


def initial_state(config, base_pose):
    return dict(sim_time=0., scan=math.radians(config['motion']['start_theta_deg']),
                wheel_angles=[0.]*4, measure_angles=[0.]*2, measure_slides=[0.]*2, suspension=[], base_pose=base_pose,
                s_hat=0., speed=0., scan_rate=0., motion_complete=False, capture={})


def merge_capture(previous, incoming):
    """Merge same-session snapshots; late delivery cannot undo capture work."""
    if not isinstance(incoming, dict): raise ValueError('Invalid capture snapshot')
    sequence = incoming.get('activity_sequence', -1)
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < -1:
        raise ValueError('Invalid capture activity counter')
    old_sequence = previous.get('activity_sequence', -1)
    merged = dict(previous)
    if sequence >= old_sequence:
        merged.update(incoming)
        # A snapshot can observe updated rows before the work counter advances.
        phases = ('capturing', 'draining', 'syncing', 'joining', 'finalizing', 'complete')
        old_phase, new_phase = previous.get('phase'), incoming.get('phase')
        if sequence == old_sequence and old_phase in phases and new_phase in phases:
            if phases.index(old_phase) > phases.index(new_phase): merged['phase'] = old_phase
    for name in ('rows_generated', 'rows_saved', 'sim_time_pushed', 'sim_time_written'):
        if name in previous and name in incoming:
            merged[name] = max(previous[name], incoming[name])
    # Runtime producer failure may reach the physics topic before the work
    # counter changes; never discard that failure because its snapshot is older.
    if previous.get('failed') or incoming.get('failed'):
        merged['failed'] = True
        if incoming.get('failed') and incoming.get('error'): merged['error'] = incoming['error']
    return merged


class MissionManager(Node):
    def __init__(self, args):
        super().__init__('ssb_mission_manager')
        self.args = args
        self.data_root = Path(args.data_root).resolve()
        self.data_root.mkdir(parents=True, exist_ok=True)
        self.demo = Path(args.demo).resolve()
        self.config = yaml.safe_load((self.demo/'capture.yaml').read_text())
        self.limits = mission_limits(self.config)
        check_calibration(self.demo/'capture.yaml', self.demo/'calibration.json')
        self.root = Path(args.output_root).resolve(); self.root.mkdir(parents=True, exist_ok=True)
        self.ownership = (self.data_root/'mission_manager.lock').open('a')
        fcntl.flock(self.ownership, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.lock = threading.RLock()
        self.state = 'idle'; self.error = ''; self.task = {}; self.session = None
        self.inputs = None
        self.drain_activity = 0
        self.shutdown_deadline = 0.
        self.server = None; self.gui = None; self.log_handles = []
        self.commands = queue.Queue(maxsize=16); self.seen = deque(maxlen=256)
        self.command_result = {}; self.events = []
        self.last_received = time.monotonic(); self.expected_exit = False
        self.preview = Preview(self.demo/'world/world.sdf', self.data_root/'rviz_preview')
        self.latest = initial_state(self.config,
            pose_values(transform(self.preview.car.findtext('pose'))@self.preview.base))
        self.truth_clock = self.create_publisher(Clock, '/clock', 10)
        self.tf = TransformBroadcaster(self)
        self.joints = self.create_publisher(JointState, '/ssb/sim_truth/joint_states', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/ssb/mission/status', qos)
        self.image_preview = self.create_publisher(String, '/ssb/mission/preview',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.image_sampler = RawImagePreview()
        self.image_timer = self.create_timer(1., self.publish_image_preview)
        self.markers = self.create_publisher(MarkerArray, '/ssb/sim_truth/scene', qos)
        self.scene_markers = self.preview.markers()
        self.markers.publish(self.scene_markers)
        # RViz clears markers on time rewind or display reset without resubscribing.
        # Reuse the cached preview; this does not reread meshes or optical textures.
        self.scene_timer = self.create_timer(1., lambda: self.markers.publish(self.scene_markers))
        self.sub = self.create_subscription(String, '/ssb/mission/command', self.enqueue, 10)
        self.gz = GzNode()
        self.world_name = ET.parse(self.demo/'world/world.sdf').getroot().find('world').get('name')
        self.status_topic = '/ssb/mission/contact_status'
        if not self.gz.subscribe(StringMsg, self.status_topic, self.receive):
            raise RuntimeError('Gazebo status subscription failed')
        if not self.gz.subscribe(StringMsg, self.status_topic+'/capture', self.receive_capture):
            raise RuntimeError('Gazebo capture progress subscription failed')
        self.closing = threading.Event()
        self.worker = threading.Thread(target=self.work, daemon=True); self.worker.start()
        self.timer = self.create_timer(1/30, self.tick)   # TF and status at the RViz frame rate

    def receive(self, message):
        try:
            data = json.loads(message.data)
            with self.lock:
                if self.session is None or data.get('session') != str(self.session): return
                data['capture'] = merge_capture(self.latest.get('capture', {}), data.get('capture', {}))
                self.drain_activity = max(self.drain_activity, data['capture'].get('activity_sequence', 0))
                self.latest = data; self.last_received = time.monotonic()
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(str(e))

    def enqueue(self, message):
        identifier = None
        try:
            command = json.loads(message.data)
            if not isinstance(command, dict): raise ValueError('Command must be a JSON object')
            if not isinstance(command.get('id'), str) or not command['id']:
                raise ValueError('Command ID is missing')
            identifier = command['id']
            if command.get('action') not in ('start', 'pause', 'resume', 'stop'):
                raise ValueError('Unknown mission command')
            if command['action'] == 'start':
                command['start_m'], command['distance_m'] = start_values(command)
                if command.get('mode', 'travel') not in ('travel', 'wall'):
                    raise ValueError('Unknown task mode')
            self.commands.put_nowait(command)
        except (ValueError, queue.Full) as e:
            error = 'Command queue is full; try again later' if isinstance(e, queue.Full) else str(e)
            with self.lock:
                self.error = error
                if identifier:
                    self.command_result = dict(id=identifier, ok=False, error=error)

    def receive_capture(self, message):
        try:
            data = json.loads(message.data)
            capture = data['capture']
            sequence = capture['activity_sequence']
            if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
                raise ValueError('Invalid capture activity counter')
            with self.lock:
                if self.session is None or data.get('session') != str(self.session): return
                merged = merge_capture(self.latest.get('capture', {}), capture)
                self.drain_activity = max(self.drain_activity, sequence)
                self.latest['capture'] = merged
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().error(str(e))

    def transition(self, state):
        with self.lock:
            self.state = state
            self.events.append(dict(state=state, wall_monotonic_s=time.monotonic(),
                                    sim_time=self.latest['sim_time']))

    def control(self, paused):
        ok, reply = self.gz.request('/world/'+self.world_name+'/control',
                                    WorldControl(pause=paused), WorldControl, Boolean, 3000)
        if not ok or not reply.data:
            raise RuntimeError('Gazebo pause/resume request was not acknowledged')
        # Confirm the simulation has actually applied the request, not only queued it.
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            with self.lock:
                if self.latest.get('paused') == paused: return
            time.sleep(.02)
        raise RuntimeError('Timed out waiting for Gazebo pause state')

    def launch(self, command):
        if self.state not in TERMINAL:
            raise ValueError('A mission is already active')
        start, distance = start_values(command)
        mode = command.get('mode', 'travel')
        if mode not in ('travel', 'wall'): raise ValueError('Unknown task mode')
        token = time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        inputs = self.data_root/'mission_runs'/token
        def begin_attempt():
            # Only an accepted plan or an unexpected preparation error owns a
            # new attempt. A rejected command leaves the last displayed result.
            with self.lock:
                self.session = None
                self.inputs = inputs
                self.task = dict(start_m=start, distance_m=distance, mode=mode)
                self.events = []
                self.error = ''
                self.drain_activity = 0
                self.shutdown_deadline = 0.
                self.latest = initial_state(self.config,
                    [start, 0., self.config['robot']['base_reference_z_m'], 0., 0., 0., 1.])
        try:
            if mode == 'wall':
                wall_plan(self.config, start, distance, json.loads((self.demo/'calibration.json').read_text()))
            else:
                plan(self.config, start, distance)  # Reject before creating any files/processes.
        except (ValueError, KeyError):
            raise
        except Exception:
            # Preflight I/O failure must still be isolated from the old session.
            begin_attempt()
            raise
        begin_attempt()
        config, task = prepare(self.demo, inputs, start, distance, mode=mode)
        check_calibration(inputs/'capture.yaml', self.demo/'calibration.json')
        from .physical_world import check as physical_check, spec_for
        physical = (physical_check(config, spec_for(inputs/'capture.yaml'), inputs/'world.sdf')
                    if config.get('contact', {}).get('enabled') else dict(passed=True))
        if not physical['passed']:
            raise ValueError('physical world differs from the configuration: '+
                             ', '.join(n for n, v in physical['checks'].items() if not v['passed']))
        if self.closing.is_set(): raise RuntimeError('Manager is shutting down')
        self.session = self.root/token; self.task = task; self.events = []
        self.error = ''; self.expected_exit = False
        self.inputs = inputs
        self.latest = initial_state(config,
            [task['start_m'], 0., config['robot']['base_reference_z_m'], 0., 0., 0., 1.])
        self.transition('starting')
        env = dict(os.environ, SSB_CONFIG=str(inputs/'capture.yaml'), SSB_WORLD=str(inputs/'world.sdf'),
                   SSB_SESSION=str(self.session), SSB_MISSION_STATUS_TOPIC=self.status_topic)
        if self.args.dynamics_only: env['SSB_DYNAMICS_ONLY'] = '1'
        else: env.pop('SSB_DYNAMICS_ONLY', None)
        log = (inputs/'server.log').open('w'); self.log_handles.append(log)
        self.server = subprocess.Popen(['gz', 'sim', '-s', '-v', '3', str(inputs/'world.sdf')],
                                       env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        if self.args.gz_gui and (self.gui is None or self.gui.poll() is not None):
            log = (inputs/'gui.log').open('w'); self.log_handles.append(log)
            self.gui = subprocess.Popen(['gz', 'sim', '-g', '-v', '3', '--gui-config',
                                          str(self.demo/'gui.config')], env=env, stdout=log,
                                         stderr=subprocess.STDOUT, start_new_session=True)
        started_at = time.monotonic(); deadline = started_at+60
        while time.monotonic() < deadline:
            if self.closing.is_set(): raise RuntimeError('Manager is shutting down')
            if self.server.poll() is not None: raise RuntimeError('Gazebo failed to start; see '+str(inputs/'server.log'))
            with self.lock:
                if self.last_received > started_at: break
            time.sleep(.05)
        else: raise RuntimeError('Gazebo did not publish mission status')
        self.control(False)
        self.transition('running')

    def finish(self, early=False):
        if self.server is None: raise ValueError('No active mission')
        self.transition('draining')
        self.expected_exit = True
        if self.server.poll() is None:
            self.control(True)
            shutdown = self.stop_server()
            if shutdown['forced']:
                raise RuntimeError('Gazebo exceeded raw drain deadline; forced shutdown: '+str(shutdown))
        if self.server.returncode != 0:
            raise RuntimeError('Gazebo exited abnormally; check mission server.log')
        self.server = None
        report = dict(schema='ssb.mission.v1', task=self.task, early_stop=early,
                      simulation_display_only=True, output=str(self.session), events=self.events,
                      final_sim_truth=self.latest,
                      input_config_sha256=sha256_file(self.inputs/'capture.yaml'),
                      input_world_sha256=sha256_file(self.inputs/'world.sdf'))
        if self.args.dynamics_only:
            destination = Path(str(self.session)+'_dynamics')
            report['capture_mode'] = 'dynamics_only_no_images'
        else:
            destination = self.session
            if not (destination/'session.json').exists():
                # A stop during settling can finish without starting capture.
                if not early: raise RuntimeError('Mission did not create a capture session')
                destination = Path(str(self.session)+'_dynamics')
                report['capture_mode'] = 'stopped_before_capture'
            else:
                summary = json.loads((destination/'session.json').read_text())
                if summary['status'] != 'complete': raise RuntimeError('Capture failed: '+summary.get('error', ''))
                report['motion_complete'] = summary['motion']['complete']
                report['rows'] = summary['rows']
                report['optical_correction'] = 'deferred_before_stitching'
                with self.lock:
                    self.latest['capture'].update(rows_generated=summary['rows'], rows_saved=summary['rows'],
                                                  sim_time_written=self.latest['capture'].get('sim_time_pushed', 0))
        self.transition('stopped' if early else 'complete')
        report['events'] = self.events; report['status'] = self.state
        report['final_sim_truth'] = self.latest
        destination.mkdir(parents=True, exist_ok=True)
        (destination/'evaluation').mkdir(exist_ok=True)
        (destination/'evaluation/mission.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')

    def execute(self, command):
        action = command['action']
        if action == 'start': self.launch(command)
        elif action == 'pause':
            if self.state != 'running': raise ValueError('Only a running mission can be paused')
            self.control(True); self.transition('paused')
        elif action == 'resume':
            if self.state != 'paused': raise ValueError('Mission is not paused')
            self.control(False); self.transition('running')
        elif action == 'stop':
            if self.state not in ('running', 'paused'): raise ValueError('No active mission to stop')
            self.finish(early=True)

    def work(self):
        while not self.closing.is_set():
            try: self.check_active()
            except Exception as e:
                self.fail(e)
                continue
            try: command = self.commands.get(timeout=.1)
            except queue.Empty:
                continue
            if command['id'] in self.seen: continue
            self.seen.append(command['id'])
            try:
                with self.lock: self.error = ''
                self.execute(command)
                with self.lock: self.command_result = dict(id=command['id'], ok=True)
            except (ValueError, KeyError) as e:
                with self.lock:
                    self.error = str(e); self.command_result = dict(id=command['id'], ok=False, error=str(e))
            except Exception as e:
                self.fail(e)
                with self.lock: self.command_result = dict(id=command['id'], ok=False, error=str(e))

    def check_active(self):
        # Paused Gazebo still publishes wall-clock heartbeats. Startup has its
        # own 60 s deadline; draining is monitored by completed writer work.
        if not self.server or self.state not in ('running', 'paused'): return
        if self.server.poll() is not None: raise RuntimeError('Gazebo server exited unexpectedly')
        with self.lock:
            latest = dict(self.latest); age = time.monotonic()-self.last_received
        if age > self.args.status_timeout_s:
            raise RuntimeError(f'Gazebo mission telemetry expired ({age:.2f} s; limit {self.args.status_timeout_s:g} s)')
        if latest.get('capture', {}).get('failed'): raise RuntimeError('Imaging pipeline failed')
        if latest.get('motion_complete'): self.finish()

    def stop_server(self):
        def progress():
            with self.lock: return self.drain_activity
        def deadline_changed(deadline):
            with self.lock:
                # close() must allow the same progress lease plus escalation.
                self.shutdown_deadline = max(self.shutdown_deadline,
                    deadline+self.args.terminate_timeout_s+self.args.kill_timeout_s+5.)
        result = stop_group(self.server, self.args.drain_timeout_s,
                            self.args.terminate_timeout_s, self.args.kill_timeout_s,
                            progress=progress, deadline_changed=deadline_changed)
        self.events.append(dict(shutdown=result, wall_monotonic_s=time.monotonic()))
        return result

    def fail(self, error):
        with self.lock: self.error = str(error)
        self.transition('failed')  # Visible even while the writer is draining.
        shutdown = None
        try:
            if self.server is not None: shutdown = self.stop_server()
        except Exception as stop_error:
            with self.lock: self.error += '; shutdown: '+str(stop_error)
        if self.server is None or self.server.poll() is not None: self.server = None
        # Do not relabel or modify raw capture evidence after a forced exit.
        destination = None
        if self.session is not None and (self.session/'session.json').is_file():
            destination = self.session/'evaluation'
        elif self.inputs is not None:
            destination = self.inputs
        if destination is not None:
            destination.mkdir(parents=True, exist_ok=True)
            (destination/'mission_failure.json').write_text(json.dumps(dict(
                status='failed', error=self.error, shutdown=shutdown, events=self.events), indent=2)+'\n')
        self.get_logger().error(self.error)

    def tick(self):
        with self.lock:
            latest = dict(self.latest); state = self.state
            age = time.monotonic()-self.last_received
            capture = latest.get('capture', {})
            status = dict(state=state, error=self.error, task=self.task,
                          mission_limits=self.limits,
                          scan_pitch_m=self.config['motion']['advance_per_rev_m'],
                          distance_estimated_m=latest['s_hat'], speed_m_s=latest['speed'],
                          scan_rad=latest['scan'], scan_rate_rad_s=latest['scan_rate'],
                          rows_generated=capture.get('rows_generated', 0), rows_saved=capture.get('rows_saved', 0),
                          imaging_lag_s=max(0., capture.get('sim_time_pushed', 0)-capture.get('sim_time_written', 0)),
                          output=str(self.session or ''), command_result=self.command_result,
                          dynamics_only=self.args.dynamics_only,
                          status_age_s=age, status_timeout_s=self.args.status_timeout_s,
                          telemetry_stale=state in ('running', 'paused') and age > self.args.status_timeout_s)
        self.status.publish(String(data=json.dumps(status, ensure_ascii=False)))
        sim = max(0., latest['sim_time']); clock = Clock()
        clock.clock.sec = int(sim); clock.clock.nanosec = int((sim-int(sim))*1e9)
        self.truth_clock.publish(clock)
        self.tf.sendTransform(self.preview.frames(latest, clock.clock))
        js = JointState(); js.header.stamp = clock.clock
        js.name = ['scan', 'odometer', 'wheel_joint_1', 'wheel_joint_2', 'wheel_joint_3', 'measure_left', 'measure_right']
        js.position = [latest['scan'], *latest['wheel_angles'], *latest.get('measure_angles', [0., 0.])]
        self.joints.publish(js)

    def publish_image_preview(self):
        with self.lock:
            session = self.session
        try:
            data = self.image_sampler.sample(session, self.config['camera']['width'],
                self.config['storage']['block_rows'], enabled=not self.args.dynamics_only)
        except (OSError, ValueError) as e:
            # Observer failures must not mark the acquisition pipeline as failed.
            data = dict(schema='ssb.raw_preview.v1', output=str(session or ''),
                        status='error', error=str(e))
        self.image_preview.publish(String(data=json.dumps(data)))

    def close(self):
        self.closing.set()
        bound = 70+self.args.drain_timeout_s+self.args.terminate_timeout_s+self.args.kill_timeout_s
        initial_deadline = time.monotonic()+bound
        while self.worker.is_alive():
            with self.lock: deadline = max(initial_deadline, self.shutdown_deadline)
            remaining = deadline-time.monotonic()
            if remaining <= 0: break
            self.worker.join(timeout=min(1., remaining))
        if self.worker.is_alive():
            self.fail(RuntimeError('Mission worker exceeded shutdown deadline'))
            # A stuck Python preparation operation must not keep the process alive.
            # Its daemon thread is abandoned; leave its handles open until process exit.
            if self.gui: stop_group(self.gui, 15., self.args.terminate_timeout_s, self.args.kill_timeout_s)
            return
        if self.server is not None:
            try: self.finish(early=True)
            except Exception as e: self.fail(e)
        if self.gui and self.gui.poll() is None:
            stop_group(self.gui, 15., self.args.terminate_timeout_s, self.args.kill_timeout_s)
        for log in self.log_handles: log.close()
        self.ownership.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--demo', default='local_data/stage_b/contact_demo_buffered')
    p.add_argument('--output-root', default='sessions/mission')
    p.add_argument('--data-root', default='local_data', help='writable assets/cache/lock root; Linux filesystem preferred')
    p.add_argument('--gz-gui', action='store_true')
    p.add_argument('--dynamics-only', action='store_true', help='test mode, NO image acquisition')
    p.add_argument('--status-timeout-s', type=positive_timeout, default=10.,
                   help='maximum wall-clock age of Gazebo telemetry while running or paused')
    p.add_argument('--drain-timeout-s', type=positive_timeout, default=180.,
                   help='maximum seconds without completed capture/commit work during drain')
    p.add_argument('--terminate-timeout-s', type=positive_timeout, default=10.)
    p.add_argument('--kill-timeout-s', type=positive_timeout, default=5.)
    args = p.parse_args()
    rclpy.init(); node = MissionManager(args)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
