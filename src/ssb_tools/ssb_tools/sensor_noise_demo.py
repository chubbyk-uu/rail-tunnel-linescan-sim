"""Prepare a separate, portable noisy demo and independently measured calibration."""
import argparse
import copy
import errno
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import yaml
from .session import sha256_file
from .public_capture import confined_file
from .package_paths import share_file, probe_command
from .optical_bench import prepare as prepare_bench
from .optical_calibration import fit


def validate_profile(noise):
    keys = {'model', 'enabled', 'pattern_seed', 'realization_seed', 'electrons_per_dn',
            'read_noise_e', 'dark_current_e_per_s', 'bias_dn', 'prnu_fraction'}
    if set(noise) != keys or noise['model'] != 'shot_read_prnu_v1' or noise['enabled'] is not True:
        raise ValueError('complete enabled generator noise profile required')
    for key in ('pattern_seed', 'realization_seed'):
        if type(noise[key]) is not int or not 0 <= noise[key] < 1 << 64:
            raise ValueError('noise seeds must be uint64 integers')
    for key, lower, upper in [('electrons_per_dn', 1, 10000), ('read_noise_e', 0, 1000),
                              ('dark_current_e_per_s', 0, 1e9), ('bias_dn', 0, 64), ('prnu_fraction', 0, .1)]:
        value = noise[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError('invalid noise parameter: '+key)


def prepare(demo, output, noise):
    validate_profile(noise)
    demo, output = Path(demo).resolve(), Path(output).resolve()
    if output.exists() or output.is_relative_to(demo):
        raise ValueError('noise variant must be fresh and separate from the source demo')
    manifest = json.loads((demo/'bundle.json').read_text())
    if manifest.get('schema') != 'ssb.demo_bundle.v1':
        raise ValueError('portable source demo bundle required')
    # Verify once per preparation; never put hashes into row/preview loops.
    sources = []
    for name, digest in manifest['files'].items():
        source = confined_file(demo, name)
        if sha256_file(source) != digest:
            raise ValueError('source demo hash mismatch: '+name)
        sources.append((name, source))
    output.mkdir(parents=True)
    for name, source in sources:
        if name in ('capture.yaml', 'calibration.json'):
            continue  # new files must not share a mutable inode with the baseline
        target = output/name; target.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.link(source, target)
        except OSError as error:
            if error.errno != errno.EXDEV:
                raise
            shutil.copyfile(source, target)
    config = yaml.safe_load((demo/'capture.yaml').read_text())
    config['truth']['sensor_noise'] = copy.deepcopy(noise)
    (output/'capture.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    # No calibration or ready bundle exists until the new target images pass.
    (output/'preparation.json').write_text(json.dumps(dict(schema='ssb.noise_demo_preparation.v1',
        source_bundle_sha256=sha256_file(demo/'bundle.json'), source_config_sha256=sha256_file(demo/'capture.yaml'),
        status='awaiting_independent_calibration', note='Shared immutable assets; private generator seeds are not reconstruction inputs.'), indent=2)+'\n')
    return config


def calibrate(output, rows=512):
    if type(rows) is not int or not 256 <= rows <= 16384:
        raise ValueError('calibration rows must be in [256,16384]')
    output = Path(output).resolve()
    meta = prepare_bench(output/'capture.yaml', output/'bench')
    for name, target in meta['targets'].items():
        with (output/'bench'/(name+'.log')).open('w') as log:
            subprocess.run(probe_command()+['--config',str(output/'bench'/target['config']),
                '--output',str(output/'bench'/target['capture']), '--rows',str(rows),
                '--x',str(target['camera_x_m']), '--theta',str(target['theta_rad'])],
                stdout=log, stderr=subprocess.STDOUT, check=True)
    result = fit(output/'bench/bench.json', output/'calibration.json')
    from .optical_identity import check_calibration
    check_calibration(output/'capture.yaml', output/'calibration.json')
    preparation = json.loads((output/'preparation.json').read_text())
    preparation.update(status='complete', calibration_sha256=sha256_file(output/'calibration.json'), rows_per_target=rows)
    (output/'preparation.json').write_text(json.dumps(preparation,indent=2)+'\n')
    # Independent bench data stays beside the bundle, explicitly as generator evidence.
    files = {str(p.relative_to(output)):sha256_file(p) for p in sorted(output.rglob('*')) if p.is_file()}
    manifest = dict(schema='ssb.demo_bundle.v1',files=files,
        note='Private simulation generator truth; assumed sensor profile, not manufacturer measurements. Runtime dependencies are relative and immutable assets may be hard-linked.')
    (output/'bundle.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo',required=True); parser.add_argument('--output',required=True)
    parser.add_argument('--profile',default=str(share_file('config/sensor_noise_assumed.yaml','ssb_core')))
    parser.add_argument('--rows',type=int,default=512)
    args = parser.parse_args()
    if not 256 <= args.rows <= 16384:
        raise ValueError('calibration rows must be in [256,16384]')
    prepare(args.demo,args.output,yaml.safe_load(Path(args.profile).read_text()))
    result = calibrate(args.output,args.rows)
    print(json.dumps(dict(output=str(Path(args.output).resolve()),validation=result['validation'])))


if __name__ == '__main__':
    main()
