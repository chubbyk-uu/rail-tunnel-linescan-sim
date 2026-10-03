import subprocess
import sys

import pytest

SCRIPT = """
import sys
from pathlib import Path
from ssb_tools import public_audit
root = Path(sys.argv[1]); public = root/'public'; (public/'raw').mkdir(parents=True)
for name in ('evaluation/truth.json', 'capture/scene.json', 'world.sdf', 'other/block.u8', 'public/raw/block.u8',
             'public/d1/report.json', 'local_data/capture.yaml'):
    (root/name).parent.mkdir(parents=True, exist_ok=True); (root/name).write_text('x')
(public/'alias.json').symlink_to(root/'evaluation/truth.json')
reads = set()
state = public_audit.install(public, public/'raw', reads)
open(public/'raw/block.u8').close(); open(public/'d1/report.json').close()
assert reads == {str(public/'raw/block.u8'), str(public/'d1/report.json')}, reads
assert state['data_reads'] == 2 and state['blocked_reads'] == 0
open(root/sys.argv[2]).close()
"""


@pytest.mark.parametrize('target,message', [
    ('evaluation/truth.json', 'private input'), ('capture/scene.json', 'private input'),
    ('world.sdf', 'private input'), ('other/block.u8', 'original raw locator'),
    ('public/alias.json', 'private input'), ('local_data/capture.yaml', 'outside public read allowlist')])
def test_private_or_unstaged_inputs_cannot_be_opened(tmp_path, target, message):
    result = subprocess.run([sys.executable, '-c', SCRIPT, str(tmp_path), target],
                            capture_output=True, text=True)
    assert result.returncode != 0 and message in result.stderr, result.stderr


def test_public_staging_inside_evaluation_is_refused(tmp_path):
    from ssb_tools.public_audit import install
    with pytest.raises(ValueError, match='outside evaluation'):
        install(tmp_path/'evaluation/public', tmp_path/'evaluation/public/raw')


def test_caught_rejection_remains_in_audit_counters(tmp_path):
    script = SCRIPT.replace('open(root/sys.argv[2]).close()', '''
try: open(root/sys.argv[2]).close()
except RuntimeError: pass
assert state['blocked_reads'] == 1
try: public_audit.verified_states(state, [])
except RuntimeError: pass
else: raise AssertionError('swallowed violation produced a pass')
''')
    result = subprocess.run([sys.executable, '-c', script, str(tmp_path), 'local_data/capture.yaml'],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_output_writes_do_not_grant_read_access(tmp_path):
    script = SCRIPT.replace('open(root/sys.argv[2]).close()', '''
path = root/'result.log'
path.write_text('allowed output')
try: path.read_text()
except RuntimeError: pass
else: raise AssertionError('writing a file granted it input access')
assert state['blocked_reads'] == 1
''')
    result = subprocess.run([sys.executable, '-c', script, str(tmp_path), 'unused'],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
