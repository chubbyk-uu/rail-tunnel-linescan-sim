"""One CPU budget for offline processes and geometry threads."""
import os


def available_cpus():
    try:
        affinity = os.sched_getaffinity(0)
        if affinity:
            return len(affinity)
    except (AttributeError, OSError):
        pass
    return os.cpu_count() or 1


def resolve_workers(requested=None):
    available = available_cpus()
    if requested is None:
        return min(8, available)
    if type(requested) is not int or not 1 <= requested <= available:
        raise ValueError('worker count must be between 1 and the available CPU count')
    return requested
