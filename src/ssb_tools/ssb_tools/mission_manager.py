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

from .mission_plan import prepare, plan, start_values
from .mission_preview import Preview, pose_values, transform
from .mission_image_preview import RawImagePreview
from .optical_identity import check_calibration
from .session import sha256_file


TERMINAL = ('idle', 'complete', 'stopped', 'failed')


def initial_state(config, base_pose):
    return dict(sim_time=0., scan=math.radians(config['motion']['start_theta_deg']),
                wheel_angles=[0.]*4, measure_angles=[0.]*2, measure_slides=[0.]*2, suspension=[], base_pose=base_pose,
                s_hat=0., speed=0., scan_rate=0., motion_complete=False, capture={})


class MissionManager(Node):
    def __init__(self, args):
        super().__init__('ssb_mission_manager')
        self.args = args
        self.repo = Path(__file__).resolve().parents[3]
        self.demo = Path(args.demo).resolve()
        self.config = yaml.safe_load((self.demo/'capture.yaml').read_text())
        check_calibration(self.demo/'capture.yaml', self.demo/'calibration.json')
        self.root = Path(args.output_root).resolve(); self.root.mkdir(parents=True, exist_ok=True)
        self.ownership = (self.repo/'local_data/mission_manager.lock').open('a')
        fcntl.flock(self.ownership, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.lock = threading.RLock()
        self.state = 'idle'; self.error = ''; self.task = {}; self.session = None
        self.server = None; self.gui = None; self.log_handles = []
        self.commands = queue.Queue(maxsize=16); self.seen = deque(maxlen=256)
        self.command_result = {}; self.events = []
        self.last_received = time.monotonic(); self.expected_exit = False
        self.preview = Preview(self.demo/'world/world.sdf', self.repo/'local_data/rviz_preview')
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
        self.closing = threading.Event()
        self.worker = threading.Thread(target=self.work, daemon=False); self.worker.start()
        self.timer = self.create_timer(.1, self.tick)

    def receive(self, message):
        try:
            data = json.loads(message.data)
            with self.lock:
                if data.get('session') != str(self.session): return
                self.latest = data; self.last_received = time.monotonic()
        except (ValueError, KeyError) as e:
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
            self.commands.put_nowait(command)
        except (ValueError, queue.Full) as e:
            error = 'Command queue is full; try again later' if isinstance(e, queue.Full) else str(e)
            with self.lock:
                self.error = error
                if identifier:
                    self.command_result = dict(id=identifier, ok=False, error=error)

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
        plan(self.config, start, distance)  # Reject before creating any files/processes.
        token = time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        inputs = self.repo/'local_data/mission_runs'/token
        config, task = prepare(self.demo, inputs, start, distance)
        check_calibration(inputs/'capture.yaml', self.demo/'calibration.json')
        from .physical_world import check as physical_check, spec_for
        physical = (physical_check(config, spec_for(inputs/'capture.yaml'), inputs/'world.sdf')
                    if config.get('contact', {}).get('enabled') else dict(passed=True))
        if not physical['passed']:
            raise ValueError('physical world differs from the configuration: '+
                             ', '.join(n for n, v in physical['checks'].items() if not v['passed']))
        self.session = self.root/token; self.task = task; self.events = []
        self.error = ''; self.expected_exit = False
        self.inputs = inputs
        self.latest = initial_state(config,
            [start, 0., config['robot']['base_reference_z_m'], 0., 0., 0., 1.])
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
            os.killpg(self.server.pid, signal.SIGINT)
            # Preserve tail blocks and error evidence; do not kill a draining writer.
            self.server.wait()
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
            try: command = self.commands.get(timeout=.1)
            except queue.Empty:
                if self.server and self.state in ('running', 'paused'):
                    try:
                        if self.server.poll() is not None: raise RuntimeError('Gazebo server exited unexpectedly')
                        with self.lock: latest = dict(self.latest)
                        if latest.get('capture', {}).get('failed'): raise RuntimeError('Imaging pipeline failed')
                        if latest.get('motion_complete'):
                            self.finish()
                    except Exception as e: self.fail(e)
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

    def fail(self, error):
        with self.lock: self.error = str(error)
        if self.server and self.server.poll() is None:
            os.killpg(self.server.pid, signal.SIGINT); self.server.wait()
        self.server = None
        self.transition('failed')
        self.get_logger().error(str(error))

    def tick(self):
        with self.lock:
            latest = dict(self.latest); state = self.state
            capture = latest.get('capture', {})
            status = dict(state=state, error=self.error, task=self.task,
                          scan_pitch_m=self.config['motion']['advance_per_rev_m'],
                          distance_estimated_m=latest['s_hat'], speed_m_s=latest['speed'],
                          scan_rad=latest['scan'], scan_rate_rad_s=latest['scan_rate'],
                          rows_generated=capture.get('rows_generated', 0), rows_saved=capture.get('rows_saved', 0),
                          imaging_lag_s=max(0., capture.get('sim_time_pushed', 0)-capture.get('sim_time_written', 0)),
                          output=str(self.session or ''), command_result=self.command_result,
                          dynamics_only=self.args.dynamics_only,
                          status_age_s=time.monotonic()-self.last_received)
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
        self.closing.set(); self.worker.join()
        if self.server is not None:
            try: self.finish(early=True)
            except Exception as e: self.get_logger().error(str(e))
        if self.gui and self.gui.poll() is None:
            os.killpg(self.gui.pid, signal.SIGINT); self.gui.wait()
        for log in self.log_handles: log.close()
        self.ownership.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--demo', default='local_data/stage_b/contact_demo')
    p.add_argument('--output-root', default='sessions/mission')
    p.add_argument('--gz-gui', action='store_true')
    p.add_argument('--dynamics-only', action='store_true', help='test mode, NO image acquisition')
    args = p.parse_args()
    rclpy.init(); node = MissionManager(args)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
