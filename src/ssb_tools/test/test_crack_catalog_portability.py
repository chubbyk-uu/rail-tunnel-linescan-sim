"""Crack catalogs record source identity, not the generating machine's directory layout."""
import json
from pathlib import Path

from ssb_tools.package_paths import share_file
from ssb_tools.stage_b_cracks import catalog, catalog_long
from ssb_tools.stage_b_scene import digest

GENERATED = Path(__file__).resolve().parents[3]/'assets/cracks/generated'


def test_new_catalogs_name_their_source_without_absolute_paths(tmp_path):
    spec = share_file('config/stage_b_scene.yaml')
    for build, name in ((catalog, 'crack_candidates_v1.png'), (catalog_long, 'long_crack_candidates_v1.png')):
        output = tmp_path/name.replace('.png', '')/'catalog.json'
        build(GENERATED/name, spec, output)
        record = json.loads(output.read_text())
        assert record['source_file'] == name
        assert record['source_sha256'] == digest(GENERATED/name)
        assert str(tmp_path) not in output.read_text() and str(GENERATED) not in output.read_text()
