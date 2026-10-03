"""D3 nominal-cylinder geometry, using measured rays and encoder progress only.

The fitted attitudes are regularized image-derived corrections, not measured poses.
No renderer, scene, reference geometry or evaluation module is imported here.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import math

import numpy as np
from scipy.interpolate import BSpline
from scipy.sparse import csr_matrix, diags, hstack
from .parallel_budget import resolve_workers


@dataclass(frozen=True)
class GeometrySettings:
    attitude_spacing_m: float = .05
    position_spacing_m: float = .6
    position_prior_mm: float = 10.
    attitude_prior_mrad: float = 5.
    position_curvature_mm: float = 2.
    attitude_curvature_mrad: float = .6
    noise_floor_px: float = .05
    effective_points_per_window: float = 16.
    max_irls: int = 5
    fit_translation: bool = False
    fit_heave: bool = False
    translation_prior_mm: float = 2.
    translation_curvature_mm: float = .5
    translation_bound_mm: float = 5.
    observed_knots: bool = False

    def validate(self):
        numbers = [v for k, v in vars(self).items() if k not in ('max_irls', 'fit_translation', 'fit_heave', 'observed_knots')]
        if not all(math.isfinite(v) and v > 0 for v in numbers):
            raise ValueError('positive finite optimization settings required')
        if not .01 <= self.attitude_spacing_m <= .6:
            raise ValueError('attitude node spacing outside supported range')
        if not .3 <= self.position_spacing_m <= 2.:
            raise ValueError('position node spacing outside supported range')
        if not isinstance(self.max_irls, int) or not 1 <= self.max_irls <= 10:
            raise ValueError('IRLS iteration budget outside supported range')
        if self.effective_points_per_window > 32:
            raise ValueError('window information budget exceeds 32')
        if not isinstance(self.fit_translation, bool) or not isinstance(self.fit_heave, bool):
            raise ValueError('translation model switches must be Boolean')
        if self.fit_translation and self.fit_heave:
            raise ValueError('choose heave-only or lateral/heave, not both')
        if self.translation_bound_mm > 30 or self.translation_prior_mm > self.translation_bound_mm:
            raise ValueError('dynamic translation prior/bounds exceed the supported small-motion model')
        if not isinstance(self.observed_knots, bool):
            raise ValueError('observed_knots must be Boolean')


def spline_knots(lower, upper, spacing):
    if not np.isfinite([lower, upper, spacing]).all() or upper <= lower or spacing <= 0:
        raise ValueError('nonempty finite encoder progress interval required')
    breaks = np.linspace(lower, upper, max(1, math.ceil((upper-lower)/spacing))+1)
    return np.r_[np.repeat(lower, 4), breaks[1:-1], np.repeat(upper, 4)]


def observed_spline_knots(sampler, lower, upper, spacing):
    """Fine knots on encoder exposure spans, coarse cubic continuation across gaps."""
    intervals = sorted((float(sampler.projection['x_axis_m'][a:b].min()),
                        float(sampler.projection['x_axis_m'][a:b].max())) for a, b in sampler.bounds)
    merged = []
    for lo, hi in intervals:
        if merged and lo <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])
    breaks = np.unique(np.concatenate([np.array([lower, upper])]+[
        np.linspace(lo, hi, max(1, math.ceil((hi-lo)/spacing))+1) for lo, hi in merged]))
    return np.r_[np.repeat(lower, 4), breaks[(breaks > lower) & (breaks < upper)], np.repeat(upper, 4)]


def curvature_stencil(knots, index, reference_spacing):
    """Second derivative on Greville progress, scaled to a declared reference step."""
    g = np.array([np.mean(knots[k+1:k+4]) for k in range(index, index+3)])
    left, right = np.diff(g)
    if left <= 0 or right <= 0:
        raise ValueError('positive Greville spacing required')
    return reference_spacing**2*np.array([2/(left*(left+right)), -2/(left*right),
                                         2/(right*(left+right))])


# Fixed chunks keep temporaries in cache; their order (not the thread count)
# defines every floating-point reduction, so results do not depend on threads.
CHUNK_RAYS = 1 << 16
THREADS = resolve_workers()


def in_chunks(function, count, factor=1.):
    """[function(start, stop)] over fixed consecutive ranges of CHUNK_RAYS*factor, in order."""
    size = max(1, int(CHUNK_RAYS*factor))
    ranges = [(start, min(start+size, count)) for start in range(0, count, size)] or [(0, 0)]
    if len(ranges) == 1 or THREADS == 1:
        return [function(*r) for r in ranges]
    with ThreadPoolExecutor(min(THREADS, len(ranges))) as pool:
        return list(pool.map(lambda r: function(*r), ranges))


def cylinder_points(axis, theta, tangent, correction, radius, height):
    """Exact Ry(pitch) Rx(roll) rays, including rotation of the upright support.

    correction columns: carriage dx [m], scan phase dq [m], roll/pitch [rad],
    optional heave [m] (five fields), or lateral/heave [m] (six fields).
    The cylinder axis is the nominal origin in y,z. No actual tunnel mesh is read.
    """
    correction = np.asarray(correction)
    if correction.ndim != 2 or correction.shape[1] not in (4, 5, 6) or not np.isfinite(correction).all():
        raise ValueError('finite four-, five- or six-component ray correction required')
    dx, dq, roll, pitch = correction[:, :4].T
    lateral = correction[:, 4] if correction.shape[1] == 6 else 0.
    vertical = correction[:, -1] if correction.shape[1] > 4 else 0.
    theta = np.asarray(theta)+dq/radius
    ca, sa = np.cos(roll), np.sin(roll)
    cb, sb = np.cos(pitch), np.sin(pitch)
    sy, sz = np.sin(theta), np.cos(theta)
    ry, rz = ca*sy-sa*sz, sa*sy+ca*sz
    vx = cb*tangent+sb*rz
    vy = ry
    vz = -sb*tangent+cb*rz
    ox = axis+dx+height*sb*ca
    oy = lateral-height*sa
    oz = vertical+height*(cb*ca-1)
    aa = vy*vy+vz*vz
    bb = 2*(oy*vy+oz*vz)
    cc = oy*oy+oz*oz-radius*radius
    discriminant = bb*bb-4*aa*cc
    if np.any(discriminant <= 0) or np.any(aa <= 0):
        raise ValueError('corrected ray does not intersect the nominal cylinder')
    length = (-bb+np.sqrt(discriminant))/(2*aa)
    return np.column_stack((ox+length*vx, radius*np.arctan2(oy+length*vy, oz+length*vz)))


def cylinder_derivatives(axis, theta, tangent, correction, radius, height):
    """Exact derivatives of cylinder_points with respect to each correction field.

    Returns (n, fields, 2): d(x, q)/d(dx, dq, roll, pitch[, lateral], [vertical]).
    The hit length follows from the implicit cylinder equation, so no finite step.
    """
    correction = np.asarray(correction)
    if correction.ndim != 2 or correction.shape[1] not in (4, 5, 6) or not np.isfinite(correction).all():
        raise ValueError('finite four-, five- or six-component ray correction required')
    dx, dq, roll, pitch = correction[:, :4].T
    lateral = correction[:, 4] if correction.shape[1] == 6 else 0.
    vertical = correction[:, -1] if correction.shape[1] > 4 else 0.
    theta = np.asarray(theta)+dq/radius
    tangent = np.asarray(tangent)
    ca, sa = np.cos(roll), np.sin(roll)
    cb, sb = np.cos(pitch), np.sin(pitch)
    sy, sz = np.sin(theta), np.cos(theta)
    ry, rz = ca*sy-sa*sz, sa*sy+ca*sz
    vx = cb*tangent+sb*rz
    vy = ry
    vz = -sb*tangent+cb*rz
    oy = lateral-height*sa
    oz = vertical+height*(cb*ca-1)
    aa = vy*vy+vz*vz
    bb = 2*(oy*vy+oz*vz)
    cc = oy*oy+oz*oz-radius*radius
    discriminant = bb*bb-4*aa*cc
    if np.any(discriminant <= 0) or np.any(aa <= 0):
        raise ValueError('corrected ray does not intersect the nominal cylinder')
    length = (-bb+np.sqrt(discriminant))/(2*aa)
    py, pz = oy+length*vy, oz+length*vz
    slope = py*vy+pz*vz  # = sqrt(discriminant)/2 > 0 on the exit root
    radial = py*py+pz*pz
    zero, one = np.zeros_like(length), np.ones_like(length)
    # (d origin x, y, z, d direction x, y, z) for each field.
    fields = [(one, zero, zero, zero, zero, zero),
              (zero, zero, zero, -sb*ry/radius, rz/radius, -cb*ry/radius),
              (-height*sb*sa, -height*ca, -height*cb*sa, sb*ry, -rz, cb*ry),
              (height*cb*ca, zero, -height*sb*ca, vz, zero, -vx)]
    if correction.shape[1] == 6:
        fields.append((zero, one, zero, zero, zero, zero))
    if correction.shape[1] > 4:
        fields.append((zero, zero, one, zero, zero, zero))
    result = np.empty((len(length), len(fields), 2))
    for k, (dox, doy, doz, dvx, dvy, dvz) in enumerate(fields):
        dl = -(py*(doy+length*dvy)+pz*(doz+length*dvz))/slope
        dpy, dpz = doy+dl*vy+length*dvy, doz+dl*vz+length*dvz
        result[:, k, 0] = dox+dl*vx+length*dvx
        result[:, k, 1] = radius*(pz*dpy-py*dpz)/radial
    return result


class Trajectory:
    scale = .001  # solve in mm and mrad, geometry always uses m and rad

    def __init__(self, sampler, radius, height, settings=GeometrySettings(), knots=None):
        settings.validate()
        if not np.isfinite([radius, height]).all() or radius <= 0 or height <= 0:
            raise ValueError('positive nominal radius and support height required')
        self.sampler, self.radius, self.height, self.settings = sampler, radius, height, settings
        axis = sampler.projection['x_axis_m']
        self.domain = [float(axis.min()), float(axis.max())]
        if knots is None:
            position = spline_knots(*self.domain, settings.position_spacing_m)
            attitude = (observed_spline_knots(sampler, *self.domain, settings.attitude_spacing_m)
                        if settings.observed_knots else spline_knots(*self.domain, settings.attitude_spacing_m))
            knots = [position, position, attitude, attitude]
            if settings.fit_translation:
                knots += [attitude, attitude]
            elif settings.fit_heave:
                knots += [attitude]
        self.fields = ['carriage_dx_m', 'scan_phase_dq_m', 'roll_rad', 'pitch_rad']
        if settings.fit_translation:
            self.fields += ['carriage_lateral_m', 'carriage_vertical_m']
        elif settings.fit_heave:
            self.fields += ['carriage_vertical_m']
        if len(knots) != len(self.fields):
            raise ValueError('trajectory fields differ from configured rigid-body model')
        self.knots = [np.asarray(k, float) for k in knots]
        self.sizes = [len(k)-4 for k in self.knots]
        self.starts = np.r_[0, np.cumsum(self.sizes)]
        self.size = int(self.starts[-1])
        if self.size > 2048:
            raise ValueError('trajectory exceeds 2048-coefficient resource budget')

    def bases(self, axis):
        return [csr_matrix(BSpline.design_matrix(axis, k, 3, extrapolate=False)) for k in self.knots]

    def parameters(self, coefficients, bases):
        return np.column_stack([b @ coefficients[self.starts[k]:self.starts[k+1]]
                                for k, b in enumerate(bases)])*self.scale

    def points(self, axis, theta, tangent, coefficients, bases=None):
        if bases is None:
            local = np.column_stack([BSpline(k, coefficients[self.starts[i]:self.starts[i+1]]*self.scale, 3,
                                             extrapolate=False)(axis) for i, k in enumerate(self.knots)])
        else:
            local = self.parameters(coefficients, bases)
        return cylinder_points(axis, theta, tangent, local,
                               self.radius, self.height)

    def native_side(self, table, side):
        """Four measured native pixel centres and their interpolation weights."""
        p = self.sampler.projection
        n = len(table)
        axis = np.empty((n, 4)); theta = np.empty((n, 4)); tangent = np.empty((n, 4))
        weights = np.empty((n, 4)); angular = table[side+'_angular_weight']
        for r, (name, row_weight) in enumerate((('lower', 1-angular), ('upper', angular))):
            sequence = table[side+'_'+name+'_sequence']
            ids = np.searchsorted(p['sequence'], sequence)
            if np.any(ids >= len(p)) or not np.array_equal(p['sequence'][ids], sequence):
                raise ValueError('match source exposure absent from D1 projection')
            if not np.array_equal(p['segment'][ids], self.sampler.segments[table['band_'+side]]):
                raise ValueError('match source belongs to a different band')
            col = table[side+'_'+name+'_column']
            if not np.isfinite(col).all() or np.any(col < 0) or np.any(col > len(self.sampler.offsets)-1):
                raise ValueError('invalid measured native match column')
            left = np.floor(col).astype(int); fraction = col-left
            for c, (column, cw) in enumerate(((left, 1-fraction),
                    (np.minimum(left+1, len(self.sampler.offsets)-1), fraction))):
                k = 2*r+c
                axis[:, k] = p['x_axis_m'][ids]
                theta[:, k] = p['theta_rad'][ids]-2*math.pi*p['segment'][ids]
                tangent[:, k] = self.sampler.offsets[column]/self.radius
                weights[:, k] = row_weight*cw
        if not np.isfinite(angular).all() or np.any((angular < 0) | (angular > 1)):
            raise ValueError('invalid native angular interpolation weight')
        nominal = np.column_stack(((weights*(axis+self.radius*tangent)).sum(1),
                                   (weights*self.radius*theta).sum(1)))
        claimed = np.column_stack((table['x_'+side+'_m'], table['q_'+side+'_m']))
        if not np.allclose(nominal, claimed, rtol=0, atol=1e-8):
            raise ValueError('native sources disagree with D2 nominal coordinates')
        return RaySet(self, axis, theta, tangent, weights)

    def forward(self, band, x, q, coefficients):
        """Map coordinates within recorded pixel footprints, not only row centres.

        Between contiguous rows the centre-ray interpolation stays unchanged.
        A nearest-row sample has a finite angular footprint: evaluate that
        direction at the recorded pose, while radiometry still uses that row.
        Snapping its geometry to the centre makes the map piecewise constant
        and prevents even a zero-correction inverse from being an identity.
        row_sources still rejects uncovered gaps and missing observations.
        """
        x, q = np.broadcast_arrays(x, q)
        lo, hi, w, supported = self.sampler.row_sources(band, q.ravel()/self.radius)
        result = np.zeros((x.size, 2))
        for ids, weight in ((lo, 1-w), (hi, w)):
            p = self.sampler.projection[ids]
            axis = p['x_axis_m']; theta = p['theta_rad']-2*math.pi*p['segment']
            theta = np.where(lo == hi, q.ravel()/self.radius, theta)
            tangent = (x.ravel()-axis)/self.radius
            result += weight[:, None]*self.points(axis, theta, tangent, coefficients)
        return result.reshape(x.shape+(2,)), supported.reshape(x.shape)

    def serialize(self, coefficients):
        extended = self.settings.fit_translation or self.settings.fit_heave
        return dict(schema='ssb.global_trajectory.v2' if extended else 'ssb.global_trajectory.v1',
                    radius_m=self.radius, support_height_m=self.height,
                    progress_domain_m=self.domain, degree=3, knots=[k.tolist() for k in self.knots],
                    coefficients=(np.asarray(coefficients)*self.scale).tolist(),
                    fields=self.fields, sizes=self.sizes,
                    model=('Ry(pitch) Rx(roll), upright support, '+('lateral/heave' if self.settings.fit_translation else 'heave')+' and nominal cylinder'
                           if extended else 'Ry(pitch) Rx(roll), upright support and nominal cylinder'))


class RaySet:
    def __init__(self, model, axis, theta, tangent, weights):
        self.model = model
        self.axis, self.theta, self.tangent = axis.ravel(), theta.ravel(), tangent.ravel()
        self.weights = weights
        self.bases = model.bases(self.axis)
        self.reducer = csr_matrix((np.ones(len(self.axis)),
            (np.repeat(np.arange(len(self.weights)), 4), np.arange(len(self.axis)))),
            shape=(len(self.weights), len(self.axis)))

    def hits(self, coefficients):
        model = self.model
        local = model.parameters(coefficients, self.bases)
        # Elementwise per ray: chunking leaves every hit bit-identical.
        points = np.concatenate(in_chunks(lambda a, b: cylinder_points(
            self.axis[a:b], self.theta[a:b], self.tangent[a:b], local[a:b], model.radius, model.height),
            len(self.axis)))
        return np.sum(points.reshape(-1, 4, 2)*self.weights[..., None], axis=1)

    def derivatives(self, coefficients):
        """Analytic d(x, q)/d(local correction field) of every native centre ray."""
        model = self.model
        local = model.parameters(coefficients, self.bases)
        return np.concatenate(in_chunks(lambda a, b: cylinder_derivatives(
            self.axis[a:b], self.theta[a:b], self.tangent[a:b], local[a:b], model.radius, model.height),
            len(self.axis)))

    def jacobian(self, coefficients):
        """Sparse derivative: analytic local ray derivatives, exact spline basis."""
        model = self.model
        matrices = [[], []]
        derivatives = self.derivatives(coefficients)
        for field, basis in enumerate(self.bases):
            derivative = derivatives[:, field]
            for direction in range(2):
                weighted = diags(derivative[:, direction]*self.weights.ravel()*model.scale) @ basis
                # Sum each consecutive group of four native centre rays.
                matrices[direction].append(self.reducer @ weighted)
        return [hstack(parts, format='csr') for parts in matrices]
