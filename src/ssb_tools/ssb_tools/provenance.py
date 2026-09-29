"""Stage provenance written by every Python stage (DESIGN.md §12.1)."""
import os
import subprocess
import sys
from pathlib import Path

from .session import sha256_file

import ssb_tools


def source_state():
    """(head, dirty, digest) of the source tree this module is running from."""
    here = Path(os.path.realpath(ssb_tools.__file__)).parent
    script = here.parent.parent / 'ssb_core' / 'cmake' / 'source_state.sh'
    try:
        out = subprocess.run(['sh', str(script), str(here)], capture_output=True, text=True, check=True).stdout.split()
        return {'git_head': out[0], 'git_dirty': out[1] == '1', 'source_digest': out[2], 'module': str(here)}
    except (OSError, subprocess.CalledProcessError, IndexError):
        return {'git_head': 'unknown', 'git_dirty': True, 'source_digest': 'unknown', 'module': str(here)}


def stage_record(stage, inputs, outputs, parameters):
    """inputs/outputs: iterables of paths; hashes are computed, never typed in."""
    return {
        'schema': 'ssb.stage_provenance.v1',
        'stage': stage,
        'argv': sys.argv,
        'python': sys.version.split()[0],
        'source': source_state(),
        'parameters': parameters,
        'inputs': {str(p): sha256_file(p) for p in inputs},
        'outputs': {str(p): sha256_file(p) for p in outputs},
    }
