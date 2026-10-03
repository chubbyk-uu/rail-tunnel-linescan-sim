import subprocess
import sys

import pytest

SCRIPT = """
import sys
from pathlib import Path
from ssb_tools import public_audit
root = Path(sys.argv[1]); public = root/'public'; (public/'raw').mkdir(parents=True)
for name in ('evaluation/truth.json', 'capture/scene.json', 'world.sdf', 'other/block.u8', 'public/raw/block.u8',
             'public/d1/report.json'):
    (root/name).parent.mkdir(parents=True, exist_ok=True); (root/name).write_text('x')
reads = set()
public_audit.install(public, public/'raw', reads)
open(public/'raw/block.u8').close(); open(public/'d1/report.json').close()
assert reads == {str(public/'raw/block.u8'), str(public/'d1/report.json')}, reads
open(root/sys.argv[2]).close()
"""


@pytest.mark.parametrize('target,message', [
    ('evaluation/truth.json', 'private input'), ('capture/scene.json', 'private input'),
    ('world.sdf', 'private input'), ('other/block.u8', 'original raw locator')])
def test_private_or_unstaged_inputs_cannot_be_opened(tmp_path, target, message):
    result = subprocess.run([sys.executable, '-c', SCRIPT, str(tmp_path), target],
                            capture_output=True, text=True)
    assert result.returncode != 0 and message in result.stderr, result.stderr


def test_public_staging_inside_evaluation_is_refused(tmp_path):
    from ssb_tools.public_audit import install
    with pytest.raises(ValueError, match='outside evaluation'):
        install(tmp_path/'evaluation/public', tmp_path/'evaluation/public/raw')
