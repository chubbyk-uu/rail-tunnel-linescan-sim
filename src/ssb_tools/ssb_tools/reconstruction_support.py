"""Conservative source support from nominal robot geometry, never simulation truth."""
import math

# Declared reconstruction uncertainty, not an assumed true wheel diameter.
RELATIVE_SCALE_BOUND = .03


def relative_scale_reach_m(domain, bound):
    if (len(domain) != 2 or not all(math.isfinite(v) for v in domain) or domain[1] < domain[0] or
            not math.isfinite(bound) or not 0 <= bound <= RELATIVE_SCALE_BOUND):
        raise ValueError('finite progress domain and relative scale bound within 3 percent required')
    return bound*(domain[1]-domain[0])/2


def matching_halo_m(domain, relative_enabled, base=.25):
    """Recorded buffer matches must cover the declared relative scale support."""
    if type(relative_enabled) is not bool or not math.isfinite(base) or not 0 <= base <= .5:
        raise ValueError('finite base matching halo and Boolean scale policy required')
    return base+relative_scale_reach_m(domain,RELATIVE_SCALE_BOUND if relative_enabled else 0.)


def correction_reach_m(radius, height):
    """Bound supported small-motion reconstruction beyond nominal footprints.

    30 mm translation and 40 mrad combined rotation are conservative bounds for
    the supported trajectory model. Source retention, task buffers and public
    edge proofs must use the same bound.
    """
    if not all(math.isfinite(v) and v > 0 for v in (radius, height)):
        raise ValueError('positive nominal radius and scan-axis support height required')
    return .03+.04*(radius+height)
