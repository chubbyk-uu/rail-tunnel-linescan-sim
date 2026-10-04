#!/usr/bin/env python3
"""Generate one comprehensive 20 m case with scale support buffers and noise.

First build the nominal bundle from downloaded sources using
build_demo_from_sources.py. All working/output directories must be new.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

import yaml

from ssb_tools.demo_bundle import export_demo
from ssb_tools.public_capture import confined_file
from ssb_tools.session import sha256_file
from ssb_tools.sensor_noise_demo import prepare as noise_prepare, calibrate
from ssb_tools.stage_b_defects import regrid
from ssb_tools.stage_b_gui import prepare as gui
from ssb_tools.stage_b_optics import prepare as optics
from ssb_tools.stage_b_runtime_surface import prepare_set
from ssb_tools.stage_b_scene import prepare as geometry

REPO = Path(__file__).resolve().parents[1]


def build(demo, sources, work, output, seed, reuse_lining=False):
    demo, sources, work, output = (Path(p).resolve() for p in (demo, sources, work, output))
    if work.exists() or output.exists():
        raise ValueError('work and output must be fresh')
    if work == output or work in output.parents or output in work.parents:
        raise ValueError('work and output must be separate, non-nested directories')
    if type(seed) is not int or not 0 <= seed <= ((1 << 64)-3)//100:
        raise ValueError('nonnegative bounded integer seed required')
    started = time.monotonic()
    config = yaml.safe_load((demo/'capture.yaml').read_text())
    if reuse_lining and [config['tunnel']['x_min_m'],config['tunnel']['x_max_m']] != [-2.5,22.5]:
        raise ValueError('reused lining must already cover the full [-2.5,22.5] m construction domain')
    if reuse_lining:
        manifest=json.loads((demo/'bundle.json').read_text())
        if manifest.get('schema')!='ssb.demo_bundle.v1' or not manifest.get('files'):
            raise ValueError('reuse requires a complete portable source bundle')
        for name,digest in manifest['files'].items():
            if sha256_file(confined_file(demo,name))!=digest:
                raise ValueError('source bundle hash mismatch: '+name)
    config['tunnel'].update(x_min_m=-2.5, x_max_m=22.5)
    spec = demo/'spec.yaml'
    scene_path = demo/config['render']['optical_scene']
    scene = json.loads(scene_path.read_text())
    resource = lambda entry: (scene_path.parent/entry['file']).resolve()
    work.mkdir(parents=True)
    config_path = work/'generation.yaml'
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    try:
        if reuse_lining:
            # Only the physical track/vehicle and sensor realization change.
            # Optical scene identities are verified by the normal preparation
            # and portable export; source buffers must already be present.
            world_input=demo/'world/world.sdf'
            config_input=demo/'capture.yaml'
            calibration_args=['--calibration',str(demo/'calibration.json')]
        else:
            prepare_set(REPO/'src/ssb_tools/config/stage_b_material_set.yaml', sources,
                        config_path, spec, work/'surface')
            geometry(config_path, spec, work/'geometry')
            # Preserve existing crack paths/widths/depths; extend only empty buffers.
            regrid(resource(scene['defects']), work/'defects', .01, config_path)
            optics(config_path, work/'geometry', work/'surface/surface.json', work/'defects/defects.json',
                   work/'optics', 16, 3, spec, False, True, 2, False, resource(scene['filler']), 'rooks', 64)
            gui(work/'optics/scene.json', work/'geometry/world.sdf', work/'gui')
            world_input=work/'gui/world.sdf'
            config_input=work/'optics/capture.yaml'
            calibration_args=['--calibrate']
        subprocess.run([sys.executable, str(REPO/'tools/prepare_contact_demo.py'),
            '--demo', str(demo), '--world', str(world_input),
            '--config', str(config_input), '--spec', str(spec),
            '--output', str(work/'prepared'), '--odo-truth-mm', '81', '81',
            '--odo-calibration-mm', '80', '80', '--mount-offset-mm', '20', '-20',
            '--mount-tilt-mrad', '1', '-1', '--track-chord-mm', '2',
            '--track-cross-level-mm', '2', '--track-seed', str(seed),
            '--response-gain', '2.4', *calibration_args], check=True)
        export_demo(work/'prepared', work/'base_bundle')
        profile = yaml.safe_load((REPO/'src/ssb_core/config/sensor_noise_assumed.yaml').read_text())
        profile.update(pattern_seed=seed*100+1, realization_seed=seed*100+2)
        noise_prepare(work/'base_bundle', output, profile)
        result = calibrate(output, 512)
        summary = dict(schema='ssb.combined_20m_preparation.v1',
                       track_seed=seed, output=str(output), tunnel_x_m=[-2.5,22.5],
                       wall_target_x_m=[0,20], wall_s=time.monotonic()-started,
                       calibration=result['validation'], noise_profile='assumed, not measured',
                       reused_lining=reuse_lining)
        (work/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        return summary
    except Exception:
        (work/'FAILED').write_text('Incomplete generation; inspect the redirected generation log.\n')
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', type=Path, default=REPO/'local_data/stage_b/contact_demo_buffered')
    parser.add_argument('--sources', type=Path, default=REPO/'local_data/stage_b/sources')
    parser.add_argument('--work', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seed', required=True, type=int)
    parser.add_argument('--reuse-lining',action='store_true',
                        help='reuse an existing 25 m bundle lining; regenerate track, vehicle and noise calibration')
    args = parser.parse_args()
    print(json.dumps(build(args.demo, args.sources, args.work, args.output, args.seed,args.reuse_lining)))


if __name__ == '__main__':
    main()
