"""Progress-aware drain, then bounded escalation for owned process groups."""
import math
import os
import signal
import subprocess
import time
from pathlib import Path


def _group_running(pgid):
    # procfs is memory-backed: this performs no persistent WSL disk I/O. A dead
    # launcher can leave live children; zombies cannot hold capture files open.
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit(): continue
        try:
            fields = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == pgid and fields[0] != 'Z': return True
        except (OSError, ValueError, IndexError):
            continue
    return False


def positive_timeout(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('shutdown timeouts must be finite and positive')
    return value


def stop_group(process, drain_s=180., terminate_s=10., kill_s=5., *,
               progress=None, deadline_changed=None):
    """SIGINT waits at most drain_s without completed work.

    progress returns a monotonic work counter, not a heartbeat. TERM/KILL have
    fixed deadlines regardless of progress. No disk polling is needed.
    """
    limits = [positive_timeout(v) for v in (drain_s, terminate_s, kill_s)]
    started = time.monotonic()
    sent = []
    extensions = 0
    previous = progress() if progress else None
    for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), limits):
        if process.poll() is not None and not _group_running(process.pid):
            break
        try:
            os.killpg(process.pid, sig)
            sent.append(sig.name)
        except ProcessLookupError:
            pass
        deadline = time.monotonic()+timeout
        if deadline_changed: deadline_changed(deadline)
        while time.monotonic() < deadline:
            if sig == signal.SIGINT and progress:
                current = progress()
                if current is not None and (previous is None or current > previous):
                    previous = current
                    deadline = time.monotonic()+timeout
                    extensions += 1
                    if deadline_changed: deadline_changed(deadline)
            if process.poll() is None:
                try: process.wait(timeout=min(.1, max(.001, deadline-time.monotonic())))
                except subprocess.TimeoutExpired: continue
            if not _group_running(process.pid): break
            time.sleep(.05)
        if process.poll() is not None and not _group_running(process.pid): break
    if process.poll() is None or _group_running(process.pid):
        raise TimeoutError(f'owned process {process.pid} did not exit after SIGKILL')
    return dict(returncode=process.returncode, signals=sent,
                forced=any(s != 'SIGINT' for s in sent), elapsed_s=time.monotonic()-started,
                progress_extensions=extensions)
