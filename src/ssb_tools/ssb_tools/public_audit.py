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
import sysconfig

ENVIRONMENT = 'SSB_PUBLIC_AUDIT'
POLICY = 'resolved_read_allowlist.v1'


def private(path):
    path = Path(path)
    return 'evaluation' in path.parts or path.name in ('truth.json', 'scene.json') or path.suffix == '.sdf'


def install(public, raw, reads=None, recorded=()):
    """Fail private opens in this process and its later workers; collect recorded reads."""
    public, raw = Path(public).resolve(), Path(raw).resolve()
    recorded = [Path(p).resolve() for p in (public, *recorded)]
    if private(public) or not raw.is_relative_to(public):
        raise ValueError('public staging must be outside evaluation/ and contain the raw directory')

    # Interpreter/package resources (including PIL fonts) are implementation
    # inputs. Workspace data is never admitted merely because it is on sys.path.
    libraries = {Path(p).resolve() for p in sys.path if p and
                 (Path(p).is_relative_to(sys.base_prefix) or str(p).startswith('/opt/ros/'))}
    libraries.add(Path(sysconfig.get_path('stdlib')).resolve())
    code = Path(__file__).resolve().parent
    metrics = Path('/proc/self/status').resolve()
    state = dict(policy=POLICY, pid=os.getpid(), installed=True, data_reads=0,
                 runtime_reads=0, blocked_reads=0)

    def hook(event, arguments):
        if event != 'open' or not isinstance(arguments[0], (str, bytes, os.PathLike)):
            return
        # Writes of logs/results are not reads. O_RDWR/r+/a+ must still be checked.
        _, mode, flags = arguments
        readable = flags & os.O_ACCMODE != os.O_WRONLY
        if not readable:
            return
        path = Path(os.fsdecode(arguments[0])).absolute().resolve()
        reason = None
        if private(path):
            reason = 'private input'
        elif path.suffix == '.u8' and not path.is_relative_to(raw):
            reason = 'original raw locator'
        elif any(path.is_relative_to(p) for p in recorded):
            state['data_reads'] += 1
            if reads is not None:
                reads.add(str(path))
            return
        elif (path == metrics or any(path.is_relative_to(p) for p in libraries) or
              (path.is_relative_to(code) and path.suffix in ('.py', '.pyc', '.so'))):
            state['runtime_reads'] += 1
            return
        else:
            reason = 'input outside public read allowlist'
        state['blocked_reads'] += 1
        raise RuntimeError(reason+' opened during production reconstruction: '+str(path))

    sys.addaudithook(hook)
    os.environ[ENVIRONMENT] = json.dumps(dict(public=str(public), raw=str(raw),
                                             recorded=[str(p) for p in recorded[1:]]))
    return state


def install_from_environment():
    """Called by worker processes; a no-op outside an audited run."""
    value = os.environ.get(ENVIRONMENT)
    if value:
        setting = json.loads(value)
        return install(setting['public'], setting['raw'], recorded=setting.get('recorded', ()))
    return None


def verified_states(main, workers):
    """Collected counters, not a hard-coded assertion; swallowed violations fail too."""
    states = [dict(main), *workers]
    if any(s.get('policy') != POLICY or s.get('installed') is not True or
           type(s.get('blocked_reads')) is not int or s['blocked_reads'] != 0 or
           s.get('data_reads', 0) <= 0 for s in states):
        raise RuntimeError('public-input audit is incomplete or recorded a blocked read')
    return states
