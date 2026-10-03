"""Public-input isolation for production reconstruction runs.

Once installed, a Python open of evaluation/truth/scene/world files fails, as does a
raw block read outside the staged public raw directory. The setting is inherited
through the environment, and D2 worker processes install the same hook at start.
Audit hooks cover Python-level opens only; they cannot be removed once installed.
"""
import json
import os
from pathlib import Path
import sys

ENVIRONMENT = 'SSB_PUBLIC_AUDIT'


def private(path):
    path = Path(path)
    return 'evaluation' in path.parts or path.name in ('truth.json', 'scene.json') or path.suffix == '.sdf'


def install(public, raw, reads=None, recorded=()):
    """Fail private opens in this process and its later workers; collect recorded reads."""
    public, raw = Path(public).resolve(), Path(raw).resolve()
    recorded = [Path(p).resolve() for p in (public, *recorded)]
    if private(public) or not raw.is_relative_to(public):
        raise ValueError('public staging must be outside evaluation/ and contain the raw directory')

    def hook(event, arguments):
        if event != 'open' or not isinstance(arguments[0], (str, bytes, os.PathLike)):
            return
        path = Path(os.fsdecode(arguments[0])).absolute()
        if private(path):
            raise RuntimeError('private input opened during production reconstruction: '+str(path))
        if path.suffix == '.u8' and not path.resolve().is_relative_to(raw):
            raise RuntimeError('original raw locator was dereferenced instead of relocated public input')
        if reads is not None and any(path.is_relative_to(p) for p in recorded):
            reads.add(str(path))

    sys.addaudithook(hook)
    os.environ[ENVIRONMENT] = json.dumps(dict(public=str(public), raw=str(raw)))


def install_from_environment():
    """Called by worker processes; a no-op outside an audited run."""
    value = os.environ.get(ENVIRONMENT)
    if value:
        setting = json.loads(value)
        install(setting['public'], setting['raw'])
        return True
    return False
