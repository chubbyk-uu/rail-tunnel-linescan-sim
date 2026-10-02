"""Exercise streaming extraction, cache reuse and corruption rejection without network."""
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile

import pytest


@pytest.fixture
def downloader(tmp_path):
    repo = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location('download_demo_sources', repo/'tools/download_demo_sources.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    remote = tmp_path/'remote'
    remote.mkdir()
    archive = remote/'maps.zip'
    channels = {}
    with zipfile.ZipFile(archive, 'w') as zipped:
        for name in ('diffuse', 'normal_gl', 'roughness'):
            content = (name*200).encode()
            filename = name+'.png'
            zipped.writestr(filename, content)
            channels[name] = dict(file=filename, bytes=len(content),
                                  sha256=hashlib.sha256(content).hexdigest())
        zipped.writestr('unneeded.png', b'not a runtime dependency')
    source = dict(asset='fixture', channels=channels,
                  package=dict(file=archive.name, url=archive.as_uri(), bytes=archive.stat().st_size,
                               sha256=module.sha256(archive)))
    metadata = tmp_path/'fixture_repo/assets/materials/demo_sources.json'
    metadata.parent.mkdir(parents=True)
    metadata.write_text(json.dumps(dict(sources={'fixture': source})))
    module.REPO = tmp_path/'fixture_repo'
    return module, source, archive, tmp_path/'cache'


def test_extract_only_named_maps_and_reuse_complete_offline_cache(downloader):
    module, source, archive, cache = downloader
    module.prepare(cache)
    assert not (cache/'fixture/unneeded.png').exists()
    assert json.loads((cache/'fixture/downloads.json').read_text()) == source
    archive.unlink()
    (cache/'fixture/maps.zip').unlink()
    before = {p.name: p.stat().st_mtime_ns for p in (cache/'fixture').iterdir()}
    module.prepare(cache, verify_only=True)
    module.prepare(cache)
    assert {p.name: p.stat().st_mtime_ns for p in (cache/'fixture').iterdir()} == before


def test_corrupt_download_never_becomes_verified_asset(downloader):
    module, source, archive, cache = downloader
    # Preserve length so the test actually exercises SHA verification.
    content = bytearray(archive.read_bytes())
    content[-1] ^= 1
    archive.write_bytes(content)
    with pytest.raises(ValueError, match='differs from pinned'):
        module.prepare(cache)
    assert not (cache/'fixture/maps.zip').exists()
    assert not (cache/'fixture/downloads.json').exists()
    assert not list(cache.rglob('*.part'))


def test_corrupt_cache_is_rejected_without_overwrite(downloader):
    module, source, archive, cache = downloader
    module.prepare(cache)
    target = cache/'fixture/diffuse.png'
    target.write_bytes(b'x'*source['channels']['diffuse']['bytes'])
    with pytest.raises(ValueError, match='differs from pinned'):
        module.prepare(cache)
    assert target.read_bytes() == b'x'*source['channels']['diffuse']['bytes']
