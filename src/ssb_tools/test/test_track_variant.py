"""Asset generation must preserve a source bundle, including a previous variant."""
import importlib.util
import json
from pathlib import Path

import pytest
import yaml

from ssb_tools.session import sha256_file


@pytest.mark.parametrize('previous_variant', [False, True])
def test_regenerating_rails_never_mutates_source(tmp_path, monkeypatch, previous_variant):
    repo = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location('prepare_track_variant',
                                                 repo/'tools/prepare_track_variant.py')
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    source, output = tmp_path/'source', tmp_path/'output'
    contents = {
        'capture.yaml': yaml.safe_dump({'truth': {'track_irregularity': {'seed': 10}}}),
        'spec.yaml': '{}\n',
        'world/world.sdf': '<sdf><world name="test"/></sdf>',
        'world/physical_manifest.json': 'old physical manifest',
        'calibration.json': 'unchanged calibration',
        'assets/texture.bin': 'unchanged optical texture',
    }
    if previous_variant:
        contents.update({'world/track/rail_profile.npz': 'old rail profile',
                         'world/track/rail_top_left_00.png': 'old heightmap',
                         'generation_provenance.json': 'old generation provenance'})
    for name, data in contents.items():
        path = source/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(data)
    identities = {name: sha256_file(source/name) for name in contents}
    (source/'bundle.json').write_text(json.dumps({'schema': 'ssb.demo_bundle.v1',
                                                'files': identities}))

    def regenerate(_world, world_folder, _config, _spec):
        folder = world_folder/'track'
        folder.mkdir(exist_ok=True)
        (folder/'rail_profile.npz').write_text('new rail profile')
        (folder/'rail_top_left_00.png').write_text('new heightmap')

    monkeypatch.setattr(generator, 'replace_track', regenerate)
    monkeypatch.setattr(generator, 'write_manifest',
                        lambda path, *_: (path.parent/'physical_manifest.json').write_text('new manifest'))
    monkeypatch.setattr(generator, 'check', lambda *_: {'passed': True})
    monkeypatch.setattr(generator, 'check_calibration', lambda *_: None)
    generator.prepare(source, output, 11)

    # The real copy/regeneration lifecycle runs; only geometry/optical checking
    # is stubbed so the fixture isolates source-file ownership, not rendering.
    assert {name: sha256_file(source/name) for name in identities} == identities
    assert (output/'world/track/rail_profile.npz').read_text() == 'new rail profile'
    assert (output/'assets/texture.bin').stat().st_ino == (source/'assets/texture.bin').stat().st_ino
    bundle = json.loads((output/'bundle.json').read_text())
    assert all(sha256_file(output/name) == digest for name, digest in bundle['files'].items())
