"""Conservative source support from nominal robot geometry, never simulation truth."""
import math


def correction_reach_m(radius, height):
    """Bound supported small-motion reconstruction beyond nominal footprints.

    30 mm translation and 40 mrad combined rotation are conservative bounds for
    the supported trajectory model. Source retention, task buffers and public
    edge proofs must use the same bound.
    """
    if not all(math.isfinite(v) and v > 0 for v in (radius, height)):
        raise ValueError('positive nominal radius and scan-axis support height required')
    return .03+.04*(radius+height)
