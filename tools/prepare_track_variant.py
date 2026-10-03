#!/usr/bin/env python3
"""Clone immutable demo assets and regenerate only the physical irregular rails.

Generator truth stays in the local bundle. Calibration/optical assets are unchanged;
capture and reconstruction still use the normal public-observation boundary.
"""
import argparse
import errno
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml

from ssb_tools.optical_identity import check_calibration
from ssb_tools.physical_world import check, write_manifest
from ssb_tools.provenance import stage_record
from ssb_tools.public_capture import confined_file
from ssb_tools.session import read_json, sha256_file
from ssb_tools.stage_b_track import replace_track


def prepare(demo, output, seed):
    demo, output = Path(demo).resolve(), Path(output).resolve()
    if output.exists() or output.is_relative_to(demo):
        raise ValueError('track variant must be fresh and separate from its source')
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError('uint32 track seed required')
    manifest = read_json(demo/'bundle.json')
    if manifest.get('schema') != 'ssb.demo_bundle.v1':
        raise ValueError('portable source bundle required')
    sources = []
    for name, digest in manifest['files'].items():
        source = confined_file(demo, name)
        if sha256_file(source) != digest:
            raise ValueError('source bundle identity mismatch: '+name)
        sources.append((name, source, digest))
    config = yaml.safe_load((demo/'capture.yaml').read_text())
    old_seed = config['truth']['track_irregularity']['seed']
    if seed == old_seed:
        raise ValueError('new track seed must differ from the source')
    config['truth']['track_irregularity']['seed'] = seed
    spec = yaml.safe_load((demo/'spec.yaml').read_text())
    output.mkdir(parents=True)
    mutable = {'capture.yaml', 'world/world.sdf', 'world/physical_manifest.json',
               'preparation.json', 'generation_provenance.json'}
    identities = {}
    for name, source, digest in sources:
        # A variant can itself be a source bundle. Regeneration writes these
        # files in place, so inheriting their hard links would corrupt the source.
        if name in mutable or name.startswith('world/track/'):
            continue
        target = output/name
        target.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.link(source, target)
        except OSError as error:
            if error.errno != errno.EXDEV:
                raise
            shutil.copyfile(source, target)
        identities[name] = digest
    (output/'capture.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    world = ET.parse(demo/'world/world.sdf')
    (output/'world').mkdir(exist_ok=True)
    replace_track(world.getroot().find('world'), output/'world', config, spec)
    # Newly generated track resources become relative; inherited bundled resources
    # retain their relative paths. Mutable files never share a source inode.
    for node in world.getroot().iter('uri'):
        if node.text and Path(node.text).is_absolute():
            asset = Path(node.text).resolve()
            if not asset.is_relative_to(output):
                raise ValueError('generated resource escaped the variant bundle')
            node.text = os.path.relpath(asset, output/'world')
    world_path = output/'world/world.sdf'
    world.write(world_path, encoding='unicode', xml_declaration=True)
    write_manifest(world_path, config, spec)
    result = check(config, spec, world_path)
    if not result['passed']:
        raise ValueError('regenerated physical world differs from track configuration')
    check_calibration(output/'capture.yaml', output/'calibration.json')
    preparation = dict(schema='ssb.track_variant.v1', source=str(demo),
        source_bundle_sha256=sha256_file(demo/'bundle.json'), old_seed=old_seed, seed=seed,
        physical_world_check=result, optical_calibration_compatible=True,
        immutable_assets_shared=True, scope='generator truth, not reconstruction input')
    (output/'preparation.json').write_text(json.dumps(preparation, indent=2)+'\n')
    for path in sorted(output.rglob('*')):
        if path.is_file():
            name = str(path.relative_to(output))
            if name not in identities:
                identities[name] = sha256_file(path)
    (output/'bundle.json').write_text(json.dumps(dict(schema='ssb.demo_bundle.v1', files=identities,
        note='Private generator track variant; relative resources and shared immutable optical assets.'), indent=2)+'\n')
    proof = stage_record('track_variant', [demo/'bundle.json', demo/'capture.yaml', demo/'spec.yaml',
        Path(__file__).resolve()], [output/'preparation.json', output/'bundle.json'], dict(seed=seed))
    (output/'generation_provenance.json').write_text(json.dumps(proof, indent=2)+'\n')
    return preparation


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', required=True); parser.add_argument('--output', required=True)
    parser.add_argument('--seed', required=True, type=int)
    result = prepare(**vars(parser.parse_args()))
    print(json.dumps(dict(seed=result['seed'], physical_world_pass=result['physical_world_check']['passed'],
                          optical_calibration_compatible=result['optical_calibration_compatible'])))
