"""Shared diagnostic display grid; never changes reconstruction or acceptance inputs."""
from copy import deepcopy
import math
from numbers import Integral


def overview_stride(shape):
    if len(shape) != 2 or any(not isinstance(n, Integral) or isinstance(n, bool) or n <= 0 for n in shape):
        raise ValueError('positive two-dimensional display shape required')
    nq, nx = shape
    stride = max(1, math.ceil(max(shape)/1800), math.ceil(math.sqrt(nq*nx/(1 << 20))))
    while math.ceil(nq/stride)*math.ceil(nx/stride) > 1 << 20:
        stride += 1
    return stride


def display_grid(original, target_x=None):
    grid = deepcopy(original)
    if target_x is None:
        return grid
    if len(target_x) != 2:
        raise ValueError('two display interval boundaries required')
    a, b = map(float, target_x)
    lo, hi = grid['target_x_m']
    if not all(map(math.isfinite, (a, b))) or not lo <= a < b <= hi:
        raise ValueError('display interval must be inside the original target')
    indices = [(x-lo)/grid['dx_m'] for x in (a, b)]
    if any(abs(i-round(i)) > 1e-7 for i in indices):
        raise ValueError('display interval must align with original output pixel boundaries')
    first, end = map(round, indices)
    if not 0 <= first < end <= grid['shape'][1]:
        raise ValueError('nonempty display subset required')
    grid.update(target_x_m=[a, b], shape=[grid['shape'][0], end-first])
    return grid
