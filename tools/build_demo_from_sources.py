#!/usr/bin/env python3
"""Build a new relocatable 20 m demo from downloaded maps and tracked crack assets."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

REPO = Path(__file__).resolve().parents[1]


def build(sources, work, output, runtime):
    sources, work, output = (Path(p).resolve() for p in (sources, work, output))
    if work.exists() or output.exists():
        raise ValueError('work and output must be new directories; existing demos are never overwritten')
    if work == output or work in output.parents or output in work.parents:
        raise ValueError('work and output must be separate, non-nested directories')
    work.mkdir(parents=True)
    config = REPO/'src/ssb_core/config/stage_b.yaml'
    spec = REPO/'src/ssb_tools/config/stage_b_scene.yaml'
    log_path = work/'generation.log'

    def run(label, command):
        print(label, flush=True)
        with log_path.open('a') as log:
            log.write(f'\n{label}\n')
            log.flush()
            subprocess.run([str(x) for x in command], cwd=REPO, stdout=log,
                           stderr=subprocess.STDOUT, check=True)

    def module(name, *args):
        return [sys.executable, '-m', 'ssb_tools.'+name, *args]

    try:
        run('1/8 Verify downloaded sources', [sys.executable, REPO/'tools/download_demo_sources.py',
                                            '--output', sources, '--verify-only'])
        run('2/8 Prepare Concrete034 surface', module('stage_b_runtime_surface', 'prepare-set',
            '--set', REPO/'src/ssb_tools/config/stage_b_material_set.yaml', '--sources', sources,
            '--config', config, '--spec', spec, '--output', work/'surface'))
        run('2/8 Prepare joint filler', module('stage_b_runtime_surface', 'prepare-filler',
            '--downloads', sources/'grey_plaster/downloads.json', '--output', work/'filler'))

        # Old catalog paths are provenance from authoring. Bind identical tracked PNGs
        # to this checkout without changing the tracked catalogs or their image hashes.
        from ssb_tools.session import sha256_file
        catalogs = work/'catalogs'
        catalogs.mkdir()
        for name in ('long_crack_candidates_v1', 'crack_candidates_v1'):
            data = json.loads((REPO/f'assets/cracks/generated/{name}.catalog.json').read_text())
            image = REPO/f'assets/cracks/generated/{name}.png'
            if sha256_file(image) != data['source_sha256']:
                raise ValueError(f'tracked crack image differs from catalog: {image}')
            data['source_file'] = str(image)
            (catalogs/(name+'.catalog.json')).write_text(json.dumps(data, indent=2)+'\n')
        run('3/8 Generate cracks', module('stage_b_defects', '--config', config, '--spec', spec,
            '--long-catalog', catalogs/'long_crack_candidates_v1.catalog.json',
            '--short-catalog', catalogs/'crack_candidates_v1.catalog.json', '--output', work/'cracks_base'))
        run('3/8 Refine cracks', module('stage_b_defects', '--refine', work/'cracks_base',
                                      '--spec', spec, '--output', work/'cracks_refined'))
        run('3/8 Apply crack depth', module('stage_b_defects', '--depth', work/'cracks_refined',
                                           '--spec', spec, '--output', work/'cracks'))
        run('4/8 Generate tunnel geometry', module('stage_b_scene', '--config', config,
                                                  '--spec', spec, '--output', work/'geometry'))
        run('5/8 Bind optical scene', module('stage_b_optics', '--config', config, '--spec', spec,
            '--geometry', work/'geometry', '--surface', work/'surface/surface.json',
            '--defects', work/'cracks/defects.json', '--filler', work/'filler/filler.json',
            '--integrated', '--area-samples', '16', '--area-pattern', 'rooks',
            '--time-samples', '3', '--crack-area-samples', '64', '--output', work/'optics'))
        run('6/8 Generate GUI textures', module('stage_b_gui', '--scene', work/'optics/scene.json',
                                               '--world', work/'geometry/world.sdf', '--output', work/'gui'))
        run('7/8 Build contact robot and rails', [sys.executable, REPO/'tools/prepare_contact_demo.py',
            # Every input is explicit: no fallback to an existing contact_demo.
            '--demo', work/'optics', '--world', work/'gui/world.sdf',
            '--config', work/'optics/capture.yaml', '--spec', spec,
            '--track-chord-mm', '2', '--track-cross-level-mm', '2', '--wheel-deflection-mm', '.2',
            '--gate-margin-deg', '5', '--response-gain', '2.4',
            '--output', work/'demo'])
        run('8/8 Prepare calibration targets', module('optical_bench', '--config', work/'demo/capture.yaml',
                                                     '--output', work/'demo/bench'))
        from ssb_tools.package_paths import core_executable
        bench = json.loads((work/'demo/bench/bench.json').read_text())
        for name, target in bench['targets'].items():
            command = [core_executable('ssb_probe')]
            if runtime == 'wsl':
                command = ['bash', REPO/'tools/with_optix_runtime.sh', *command]
            run(f'8/8 Render {name} target', [*command, '--config', work/'demo/bench'/target['config'],
                '--output', work/'demo/bench'/target['capture'], '--rows', '256',
                '--x', str(target['camera_x_m']), '--theta', str(target['theta_rad'])])
        run('8/8 Fit calibration', module('optical_calibration', 'fit', '--bench', work/'demo/bench/bench.json',
                                         '--output', work/'demo/calibration.json'))
        from ssb_tools.optical_identity import check_calibration
        check_calibration(work/'demo/capture.yaml', work/'demo/calibration.json')
        run('8/8 Export self-contained demo', module('demo_bundle', '--demo', work/'demo', '--output', output))
        print(f'Demo ready: {output}\nFull generation log: {log_path}', flush=True)
    except Exception:
        (work/'FAILED').write_text(f'Generation failed; inspect {log_path}\n')
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sources', type=Path, default=REPO/'local_data/stage_b/sources')
    parser.add_argument('--work', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=REPO/'local_data/stage_b/contact_demo_buffered')
    parser.add_argument('--runtime', choices=('wsl', 'native'), required=True)
    args = parser.parse_args()
    build(args.sources, args.work, args.output, args.runtime)


if __name__ == '__main__':
    main()
