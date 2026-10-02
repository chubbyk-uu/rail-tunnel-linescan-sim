"""Static GUI shadow diagnostic; never produces reconstruction inputs."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image as PillowImage, ImageDraw

from .stage_b_scene import sub
from .session import sha256_file


ANGLES = (150, 160, 164, 165, 170, 180, 190, 195, 196, 200, 210)
STATES = ('off', 'strip', 'work', 'both')


def floor_coverage(car, angle, samples=64):
    """Independent cone-rim intersection with the actual nominal cradle box.

    Does not predict visibility through other robot components or shadow-map behavior.
    The floor is horizontal in the static diagnostic, and scan rotates about -x.
    """
    base = car.find("link[@name='base']")
    head = car.find("link[@name='head']")
    floor = base.find("visual[@name='u_cradle_floor']")
    origin = np.array(list(map(float, base.findtext('pose').split()[:3])))
    center = origin + np.array(list(map(float, floor.findtext('pose').split()[:3])))
    size = np.array(list(map(float, floor.findtext('geometry/box/size').split())))
    axis = car.findtext("joint[@name='scan']/axis/xyz").split()
    if list(map(float, axis)) != [-1., 0., 0.]:
        raise ValueError('diagnostic requires the current -x scan axis')
    theta = -math.radians(angle)
    rotation = np.array([[1, 0, 0], [0, math.cos(theta), -math.sin(theta)],
                         [0, math.sin(theta), math.cos(theta)]])
    head_origin = np.array(list(map(float, head.findtext('pose').split()[:3])))
    hits = []
    for light in head.findall('light'):
        if not light.get('name', '').startswith('cob_strip_'):
            continue
        emitter = head_origin + rotation @ np.array(list(map(float, light.findtext('pose').split()[:3])))
        direction = np.array(list(map(float, light.findtext('direction').split())))
        direction /= np.linalg.norm(direction)
        a = np.cross(direction, [0, 1, 0]); a /= np.linalg.norm(a)
        b = np.cross(direction, a)
        half = float(light.findtext('spot/outer_angle')) / 2
        # Include each cone centre as well as its boundary.
        rays = [direction]
        rays.extend(direction * math.cos(half) + (a * math.cos(p) + b * math.sin(p)) * math.sin(half)
                    for p in np.linspace(0, 2 * math.pi, samples, endpoint=False))
        for local_ray in rays:
            ray = rotation @ local_ray
            distance = (center[2] + size[2]/2 - emitter[2]) / ray[2]
            if distance <= 0:
                raise ValueError('probe does not face the cradle plane')
            hits.append(emitter + distance * ray)
    if not hits:
        raise ValueError('no strip lights in world')
    points = np.array(hits)
    inside = np.all(np.abs(points[:, :2] - center[:2]) <= size[:2]/2, axis=1)
    return dict(angle_deg=angle, tested_rays=len(hits), blocked_rays=int(inside.sum()),
                floor_plane_xy_min_m=points[:, :2].min(axis=0).tolist(),
                floor_plane_xy_max_m=points[:, :2].max(axis=0).tolist(),
                note='Nominal direct-ray floor test; not an Ogre shadow validation')


def diagnostic_world(source, output, fix=False):
    tree = ET.parse(source); world = tree.getroot().find('world')
    world.set('name', 'strip_check')
    for plugin in list(world.findall('plugin')):
        world.remove(plugin)
    for filename, name in (('gz-sim-physics-system', 'Physics'),
                           ('gz-sim-user-commands-system', 'UserCommands'),
                           ('gz-sim-scene-broadcaster-system', 'SceneBroadcaster')):
        plugin = sub(world, 'plugin', filename=filename, name='gz::sim::systems::'+name)
    car = world.find("model[@name='scan_car']")
    original = copy.deepcopy(car)
    car_origin = list(map(float, car.findtext('pose').split()[:3]))
    for model in world.findall('model'):
        static = model.find('static')
        if static is None: static = sub(model, 'static')
        static.text = 'true'
        for plugin in list(model.findall('plugin')): model.remove(plugin)
        for joint in list(model.findall('joint')): model.remove(joint)
        for frame in list(model.findall('frame')): model.remove(frame)
        for link in model.findall('link'):
            for collision in list(link.findall('collision')): link.remove(collision)
    head = car.find("link[@name='head']")
    head_pose = list(map(float, head.findtext('pose').split()[:3]))
    car.remove(head)
    scanner = sub(world, 'model', name='scanner'); sub(scanner, 'static', 'true')
    position = [a+b for a, b in zip(car_origin, head_pose)]
    sub(scanner, 'pose', ' '.join(map(str, position))+' 0 0 0')
    head.find('pose').text = '0 0 0 0 0 0'; scanner.append(head)
    eye = np.array([car_origin[0]+1.8, -2.3, 3.3]); target = np.array([car_origin[0], 0., 1.25])
    direction = target-eye
    yaw = math.atan2(direction[1], direction[0])
    pitch = math.atan2(-direction[2], math.hypot(*direction[:2]))
    camera_pose = ' '.join(map(str, [*eye, 0., pitch, yaw]))
    output.with_suffix('.config').write_text(f'''<window><width>960</width><height>720</height><dialog_on_exit>false</dialog_on_exit></window>
<plugin filename="MinimalScene" name="3D View"><gz-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui><engine>ogre2</engine><scene>scene</scene><ambient_light>0.6 0.6 0.6</ambient_light><background_color>0.25 0.25 0.25</background_color><camera_pose>{camera_pose}</camera_pose></plugin>
<plugin filename="GzSceneManager" name="Scene Manager"/>
<plugin filename="SsbShadowSettings" name="SsbShadowSettings"><apply_fix>{str(fix).lower()}</apply_fix><diagnostic>true</diagnostic><gz-gui><property key="state" type="string">floating</property><property key="width" type="double">1</property><property key="height" type="double">1</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
''')
    ET.indent(tree); tree.write(output, encoding='unicode', xml_declaration=True)
    return original, position


def below_floor_row(car, width, height):
    """Screen boundary below the entire floor, from the fixture camera geometry."""
    origin = np.array(list(map(float, car.findtext('pose').split()[:3])))
    base = car.find("link[@name='base']")
    floor = base.find("visual[@name='u_cradle_floor']")
    center = origin + np.array(list(map(float, base.findtext('pose').split()[:3])))
    center += np.array(list(map(float, floor.findtext('pose').split()[:3])))
    half = np.array(list(map(float, floor.findtext('geometry/box/size').split()))) / 2
    eye = np.array([origin[0]+1.8, -2.3, 3.3]); target = np.array([origin[0], 0., 1.25])
    forward = target-eye; forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.]); right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    focal = width / (2 * math.tan(1.05/2))
    rows = []
    for x in (-1, 1):
        for y in (-1, 1):
            for z in (-1, 1):
                relative = center + half*np.array([x, y, z])-eye
                rows.append(height/2-focal*np.dot(relative, up)/np.dot(relative, forward))
    return math.ceil(max(rows))+10


def run(source, output, fix=False):
    # Configure the transport environment before loading the bindings.
    os.environ['GZ_PARTITION'] = 'ssb_strip_'+str(os.getpid())
    from gz.transport13 import Node
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.light_pb2 import Light
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.scene_pb2 import Scene
    from google.protobuf import text_format
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    source = Path(source).resolve()
    car, position = diagnostic_world(source, output/'world.sdf', fix)
    environment = dict(os.environ)
    node = Node(); condition = threading.Condition(); frames = [0, None]
    def receive(message):
        pixels = np.frombuffer(message.data, dtype='u1').reshape(message.height, message.step)
        pixels = pixels[:, :message.width*3].reshape(message.height, message.width, 3).copy()
        with condition:
            frames[0] += 1; frames[1] = pixels; condition.notify_all()
    node.subscribe(Image, '/strip_check/image', receive)
    lights_publisher = node.advertise('/world/strip_check/light_config', Light)
    def fresh_image():
        with condition:
            sequence = frames[0]
            if not condition.wait_for(lambda: frames[0] >= sequence+6, timeout=30):
                raise RuntimeError('camera frames timed out; inspect server.log')
            return frames[1].copy()
    images = []; measures = []
    with (output/'server.log').open('w') as log, (output/'gui.log').open('w') as gui_log:
        server = subprocess.Popen(['gz', 'sim', '-s', '-r', '-v', '3', str(output/'world.sdf')],
                                  env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        gui = subprocess.Popen(['gz', 'sim', '-g', '-v', '3', '--gui-config', str(output/'world.config')],
                               env=environment, stdout=gui_log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            fresh_image()
            # Blocking Python RPC contends with image callbacks in this binding.
            # A single CLI request releases the Python GIL while waiting.
            response = subprocess.run(['gz', 'service', '-s', '/world/strip_check/scene/info',
                                       '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
                                       '--timeout', '10000', '--req', ''],
                                      env=environment, capture_output=True, text=True, timeout=15, check=True)
            scene = Scene(); text_format.Parse(response.stdout, scene)
            lights = {}
            def collect(model):
                for link in model.link:
                    for light in link.light: lights[light.name] = (light.id, link.id)
                for child in model.model: collect(child)
            for model in scene.model: collect(model)
            for angle in ANGLES:
                pose = Pose(name='scanner'); pose.position.x, pose.position.y, pose.position.z = position
                pose.orientation.x = -math.sin(math.radians(angle)/2)
                pose.orientation.w = math.cos(math.radians(angle)/2)
                response = subprocess.run(['gz', 'service', '-s', '/world/strip_check/set_pose',
                                           '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                                           '--timeout', '10000', '--req', text_format.MessageToString(pose)],
                                          env=environment, capture_output=True, text=True, timeout=15, check=True)
                reply = Boolean(); text_format.Parse(response.stdout, reply)
                if not reply.data: raise RuntimeError('scanner pose was not accepted')
                cases = []
                for state in STATES:
                    for link in car.findall('link'):
                        for light in link.findall('light'):
                            strip = light.get('name').startswith('cob_strip_')
                            enabled = state == 'both' or state == ('strip' if strip else 'work')
                            # Gazebo's single-light service resolves the unscoped
                            # component name. Names are unique in this fixture.
                            message = Light(name=light.get('name'), type=Light.SPOT)
                            message.id, message.parent_id = lights[light.get('name')]
                            message.cast_shadows = True
                            message.direction.x, message.direction.y, message.direction.z = map(float, light.findtext('direction').split())
                            for field in ('diffuse', 'specular'):
                                color = getattr(message, field)
                                color.r, color.g, color.b, color.a = map(float, light.findtext(field).split())
                            message.range = float(light.findtext('attenuation/range'))
                            for field in ('constant', 'linear', 'quadratic'):
                                setattr(message, 'attenuation_'+field, float(light.findtext('attenuation/'+field)))
                            message.spot_inner_angle = float(light.findtext('spot/inner_angle'))
                            message.spot_outer_angle = float(light.findtext('spot/outer_angle'))
                            message.spot_falloff = float(light.findtext('spot/falloff'))
                            message.intensity = float(light.findtext('intensity')) if enabled else 0.
                            lights_publisher.publish(message)
                    cases.append(fresh_image())
                images.append(cases); measures.append(floor_coverage(car, angle))
        finally:
            for process in (gui, server):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try: process.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=10)
            if server.returncode != 0 or gui.returncode != 0:
                raise RuntimeError(f'diagnostic process failed: server={server.returncode}, gui={gui.returncode}')
    pictures = np.array(images)
    if not np.any(pictures[:, 1] != pictures[:, 0]) or not np.any(pictures[:, 2] != pictures[:, 0]):
        raise RuntimeError('light switching had no visible effect; diagnostic cannot pass')
    boundary = below_floor_row(car, pictures.shape[3], pictures.shape[2])
    failures = []
    for index, geometry in enumerate(measures):
        difference = np.abs(pictures[index, 1].astype('i2')-pictures[index, 0].astype('i2')).max(axis=2)
        maximum = int(difference[boundary:].max())
        geometry.update(below_floor_max_delta_dn=maximum,
                        below_floor_changed_pixels=int((difference[boundary:] > 3).sum()))
        if geometry['blocked_rays'] == geometry['tested_rays'] and maximum > 2:
            failures.append(geometry['angle_deg'])
    np.savez_compressed(output/'frames.npz', angles=ANGLES, states=STATES, images=pictures)
    sheet = PillowImage.new('RGB', (4*480, len(ANGLES)*380), '#202020'); draw = ImageDraw.Draw(sheet)
    for i, angle in enumerate(ANGLES):
        for j, state in enumerate(STATES):
            sheet.paste(PillowImage.fromarray(pictures[i, j]).resize((480, 360)), (j*480, i*380+20))
            draw.text((j*480+8, i*380+3), f'{angle} deg / {state}', fill='white')
    sheet.save(output/'comparison.jpg', quality=93)
    report = dict(source=str(source), source_sha256=sha256_file(source), geometry=measures, fix=fix,
                  below_floor_screen_row=boundary, leakage_angles_deg=failures, passed=not failures,
                  states=STATES, angles=ANGLES, frame_shape=list(pictures.shape),
                  note='Static actual Gazebo GUI diagnostic; acquisition plugins removed, no raw session generated.')
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if fix and failures: raise RuntimeError('fully blocked poses still leak: '+str(failures))
    print(json.dumps(dict(output=str(output), angles=len(ANGLES), images=len(ANGLES)*len(STATES))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', required=True); parser.add_argument('--output', required=True)
    parser.add_argument('--fix', action='store_true')
    args = parser.parse_args(); run(args.world, args.output, args.fix)


if __name__ == '__main__': main()
