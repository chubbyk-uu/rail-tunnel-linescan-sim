"""SDF visual preview and simulation-truth TF; never used by reconstruction."""
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation


def transform(text='0 0 0 0 0 0'):
    v = list(map(float, text.split()))
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', v[3:]).as_matrix()
    result[:3, 3] = v[:3]
    return result


def pose_values(matrix):
    return [*matrix[:3, 3], *Rotation.from_matrix(matrix[:3, :3]).as_quat()]


def textured_dae(mesh, texture, output):
    """Keep preview geometry/UVs, cap texture to 512 px; cache by input identity."""
    mesh, texture, output = Path(mesh), Path(texture), Path(output)
    identity = hashlib.sha256(b'ssb-rviz-dae-v3'+mesh.read_bytes()+texture.read_bytes()).hexdigest()
    target = output/f'{identity}.dae'
    if target.exists():
        return target
    output.mkdir(parents=True, exist_ok=True)
    image = output/f'{identity}.png'
    with Image.open(texture) as src:
        src.thumbnail((512, 512), Image.Resampling.LANCZOS)
        src.save(image)
    vertices, uv, faces = [], [], []
    for line in mesh.read_text().splitlines():
        fields = line.split()
        if not fields: continue
        if fields[0] == 'v': vertices.extend(fields[1:4])
        elif fields[0] == 'vt': uv.extend(fields[1:3])
        elif fields[0] == 'f':
            points = [p.split('/') for p in fields[1:]]
            for i in range(1, len(points)-1):
                faces.extend([int(x)-1 for p in (points[0], points[i], points[i+1]) for x in p[:2]])
    if not vertices or not uv or not faces:
        raise ValueError('textured preview mesh requires vertices, UVs and faces')
    # Collada embedded materials are supported by RViz mesh_resource; OBJ's SDF PBR is not.
    document = f'''<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_images><image id="image"><init_from>{image.name}</init_from></image></library_images>
<library_effects><effect id="effect"><profile_COMMON>
<newparam sid="surface"><surface type="2D"><init_from>image</init_from></surface></newparam>
<newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam>
<technique sid="common"><lambert><ambient><color>.7 .7 .7 1</color></ambient><diffuse><texture texture="sampler" texcoord="UV"/></diffuse></lambert></technique>
</profile_COMMON></effect></library_effects>
<library_materials><material id="material"><instance_effect url="#effect"/></material></library_materials>
<library_geometries><geometry id="geometry"><mesh>
<source id="positions"><float_array id="p" count="{len(vertices)}">{' '.join(vertices)}</float_array><technique_common><accessor source="#p" count="{len(vertices)//3}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="uv"><float_array id="t" count="{len(uv)}">{' '.join(uv)}</float_array><technique_common><accessor source="#t" count="{len(uv)//2}" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles material="material" count="{len(faces)//6}"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><p>{' '.join(map(str,faces))}</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node id="node"><instance_geometry url="#geometry"><bind_material><technique_common><instance_material symbol="material" target="#material"><bind_vertex_input semantic="UV" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#scene"/></scene></COLLADA>'''
    target.write_text(document)
    return target


class Preview:
    def __init__(self, world, cache):
        self.world = Path(world).resolve()
        self.models = ET.parse(self.world).getroot().find('world').findall('model')
        self.car = next(m for m in self.models if m.get('name') == 'scan_car')
        self.base = transform(self.car.find("link[@name='base']").findtext('pose'))
        self.cache = Path(cache)

    def markers(self):
        from visualization_msgs.msg import Marker, MarkerArray
        result = MarkerArray()
        for model in self.models:
            moving = model is self.car
            model_pose = transform(model.findtext('pose', '0 0 0 0 0 0'))
            for link in model.findall('link'):
                lp = transform(link.findtext('pose', '0 0 0 0 0 0'))
                for visual in link.findall('visual'):
                    marker = Marker()
                    marker.header.frame_id = 'sim_truth/'+link.get('name') if moving else 'world'
                    marker.frame_locked = moving
                    marker.ns = 'vehicle' if moving else model.get('name')
                    marker.id = len(result.markers)
                    vp = transform(visual.findtext('pose', '0 0 0 0 0 0'))
                    pv = pose_values(vp if moving else model_pose@lp@vp)
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pv[:3]
                    marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = pv[3:]
                    colour = list(map(float, visual.findtext('material/diffuse', '.5 .5 .5 1').split()))
                    marker.color.r, marker.color.g, marker.color.b, marker.color.a = colour
                    geometry = visual.find('geometry')
                    if geometry.find('box') is not None:
                        marker.type = Marker.CUBE
                        scale = list(map(float, geometry.findtext('box/size').split()))
                    elif geometry.find('cylinder') is not None:
                        marker.type = Marker.CYLINDER
                        r = float(geometry.findtext('cylinder/radius'))
                        scale = [2*r, 2*r, float(geometry.findtext('cylinder/length'))]
                    elif geometry.find('sphere') is not None:
                        marker.type = Marker.SPHERE
                        scale = [2*float(geometry.findtext('sphere/radius'))]*3
                    elif geometry.find('ellipsoid') is not None:
                        marker.type = Marker.SPHERE
                        scale = [2*float(r) for r in geometry.findtext('ellipsoid/radii').split()]
                    elif geometry.find('mesh') is not None:
                        marker.type = Marker.MESH_RESOURCE
                        mesh = (self.world.parent/geometry.findtext('mesh/uri')).resolve()
                        texture = visual.findtext('material/pbr/metal/albedo_map')
                        if texture:
                            mesh = textured_dae(mesh, (self.world.parent/texture).resolve(), self.cache)
                            marker.mesh_use_embedded_materials = True
                        marker.mesh_resource = mesh.as_uri()
                        scale = list(map(float, geometry.findtext('mesh/scale', '1 1 1').split()))
                    else:
                        raise ValueError('unsupported RViz preview geometry')
                    marker.scale.x, marker.scale.y, marker.scale.z = scale
                    result.markers.append(marker)
        return result

    def frames(self, status, stamp):
        from geometry_msgs.msg import TransformStamped
        base_pose = status['base_pose']
        world = np.eye(4)
        world[:3, 3] = base_pose[:3]
        world[:3, :3] = Rotation.from_quat(base_pose[3:]).as_matrix()
        wheel_names = ['odometer_wheel', 'wheel_1', 'wheel_2', 'wheel_3']
        measure_names = ['measure_left_wheel', 'measure_right_wheel']
        # Vertical slide displacements along the base z axis (sprung axles, measuring-wheel sliders).
        lift = {}
        for links, q in zip((('odometer_wheel_axle', 'odometer_wheel'), ('wheel_1_axle', 'wheel_1'),
                             ('wheel_2_axle', 'wheel_2'), ('wheel_3_axle', 'wheel_3')), status.get('suspension') or []):
            for link in links: lift[link] = q
        for side, q in zip(('left', 'right'), status.get('measure_slides') or []):
            for link in (f'measure_{side}_slider', f'measure_{side}_wheel'): lift[link] = q
        frames = []
        for link in self.car.findall('link'):
            name = link.get('name')
            relative = np.linalg.inv(self.base)@transform(link.findtext('pose', '0 0 0 0 0 0'))
            if name == 'head':
                relative[:3, :3] = Rotation.from_rotvec([-status['scan'], 0, 0]).as_matrix()@relative[:3, :3]
            elif name in wheel_names or name in measure_names:
                angle = (status['wheel_angles'][wheel_names.index(name)] if name in wheel_names else
                         status.get('measure_angles', [0., 0.])[measure_names.index(name)])
                relative[:3, :3] = relative[:3, :3]@Rotation.from_rotvec([0, angle, 0]).as_matrix()
            if name in lift:
                relative[2, 3] += lift[name]
            values = pose_values(world if name == 'base' else relative)
            tf = TransformStamped(); tf.header.stamp = stamp
            tf.header.frame_id = 'world' if name == 'base' else 'sim_truth/base'
            tf.child_frame_id = 'sim_truth/'+name
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = values[:3]
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = values[3:]
            frames.append(tf)
        return frames
