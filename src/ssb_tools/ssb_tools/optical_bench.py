"""Render known cylindrical optical targets through the production OptiX shader."""
import argparse
import copy
import json
import math
from pathlib import Path
import subprocess
import yaml
from .session import sha256_file
from .optical_identity import ensure_optical_key


def prepare(config_path, output):
    config_path = Path(config_path).resolve(); output = Path(output).resolve()
    config = yaml.safe_load(config_path.read_text())
    if not config['truth'].get('optical_key'):
        raise ValueError('prepare a persistent optical key in the capture configuration before calibrating')
    ensure_optical_key(config)
    mount = config['truth']['mount']
    if any(mount[k] for k in ('e_m','tangential_m','twist_rad')):
        raise ValueError('bench currently supports external axis placement errors only')
    scene_path = Path(config['render']['optical_scene'])
    if not scene_path.is_absolute():
        scene_path = config_path.parent/scene_path
    scene = json.loads(scene_path.read_text())
    # Resolve asset entries before placing the scene snapshot in a new directory.
    def resolve_entries(value):
        if isinstance(value, dict):
            if 'file' in value:
                value['file'] = str((scene_path.parent/value['file']).resolve())
            for child in value.values(): resolve_entries(child)
        elif isinstance(value, list):
            for child in value: resolve_entries(child)
    resolve_entries(scene)
    output.mkdir(parents=True, exist_ok=False)
    tunnel = config['tunnel']; radius = tunnel['radius_m']; z0 = tunnel['axis_z_m']; facets = 4096
    mesh = output/'target.obj'
    with mesh.open('w') as f:
        for x in (tunnel['x_min_m'], tunnel['x_max_m']):
            for i in range(facets):
                a = 2*math.pi*i/facets
                f.write(f'v {x:.12f} {radius*math.sin(a):.12f} {z0+radius*math.cos(a):.12f}\n')
        for i in range(facets):
            a, b = i+1, (i+1)%facets+1
            f.write(f'f {a} {b} {b+facets}\nf {a} {b+facets} {a+facets}\n')
    scene['meshes'] = [dict(file=str(mesh), sha256=sha256_file(mesh), material=0)]
    scene['sampling'].update(integrated_cracks=False, adaptive_area=False, area_axis_samples=16, area_pattern='rooks')
    scene['convex_panel_visibility'] = False
    x = (tunnel['x_min_m']+tunnel['x_max_m'])/2
    pitch = .02; fov = config['camera']['fov_at_nominal_m']; targets = {}
    for target_index, (name, kind, shift, albedo, theta) in enumerate( [('dark',3,0,.22,0), ('bright',1,0,.22,0),
            ('bars',2,0,.22,0), ('bright_holdout',1,0,.18,.19), ('bars_holdout',2,.007,.22,0)]):
        target_scene = copy.deepcopy(scene)
        target_scene['calibration_target'] = dict(kind=kind, origin_x_m=x+shift, pitch_m=pitch, bar_width_m=.003, albedo=albedo)
        target_json = output/(name+'.json'); target_json.write_text(json.dumps(target_scene, indent=2)+'\n')
        target_config = copy.deepcopy(config); target_config['render']['optical_scene'] = str(target_json)
        noise = target_config['truth'].get('sensor_noise', {})
        if noise.get('enabled'):
            # Separate temporal realizations; fixed PRNU is the same physical rig.
            noise['realization_seed'] = (noise['realization_seed']+target_index+1) % (1 << 64)
        target_yaml = output/(name+'.yaml'); target_yaml.write_text(yaml.safe_dump(target_config, sort_keys=False))
        # All designed stripes in the camera's nominal field; incomplete edge stripes are excluded.
        positions = [i*pitch+shift for i in range(-100,101) if abs(i*pitch+shift) < .99*fov/2]
        targets[name] = dict(config=target_yaml.name, capture=name, theta_rad=theta,
                             positions_m=positions, camera_x_m=x-config['truth']['head_mount_x_m'])
    meta = dict(schema='ssb.optical_bench.v1', fov_m=fov, targets=targets,
                config_sha256=sha256_file(config_path), target_sha256=sha256_file(mesh),
                sensor_noise_enabled=bool(config['truth'].get('sensor_noise', {}).get('enabled')),
                centered_bench=True,
                description='Camera aligned in a separate known-distance jig; external vehicle mount is absent. '
                            'Images estimate intrinsics/flat only, not vehicle extrinsics; independent shifted bars and gray/angle.')
    (output/'bench.json').write_text(json.dumps(meta, indent=2)+'\n')
    return meta


def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--config', required=True)
    p.add_argument('--output', required=True); p.add_argument('--render', action='store_true')
    a = p.parse_args(); meta = prepare(a.config, a.output); root = Path(a.output).resolve()
    if a.render:
        from .package_paths import probe_command
        for name, target in meta['targets'].items():
            with (root/(name+'.log')).open('w') as log:
                subprocess.run(probe_command()+[ '--config', str(root/target['config']),
                    '--output',str(root/target['capture']), '--rows','256', '--x',str(target['camera_x_m']),
                    '--theta',str(target['theta_rad']), '--centered-bench'], stdout=log, stderr=subprocess.STDOUT, check=True)
    print(json.dumps(dict(bench=str(root/'bench.json'), rendered=a.render)))

if __name__ == '__main__':
    main()
