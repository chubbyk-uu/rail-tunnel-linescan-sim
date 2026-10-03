"""D3 nominal-cylinder geometry, using measured rays and encoder progress only.

The fitted attitudes are regularized image-derived corrections, not measured poses.
No renderer, scene, reference geometry or evaluation module is imported here.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.interpolate import BSpline
from scipy.sparse import csr_matrix, diags, hstack


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

    def validate(self):
        numbers = [v for k, v in vars(self).items() if k != 'max_irls']
        if not all(math.isfinite(v) and v > 0 for v in numbers):
            raise ValueError('positive finite optimization settings required')
        if not .025 <= self.attitude_spacing_m <= .6:
            raise ValueError('attitude node spacing outside supported range')
        if not .3 <= self.position_spacing_m <= 2.:
            raise ValueError('position node spacing outside supported range')
        if not isinstance(self.max_irls, int) or not 1 <= self.max_irls <= 10:
            raise ValueError('IRLS iteration budget outside supported range')
        if self.effective_points_per_window > 32:
            raise ValueError('window information budget exceeds 32')


def spline_knots(lower, upper, spacing):
    if not np.isfinite([lower, upper, spacing]).all() or upper <= lower or spacing <= 0:
        raise ValueError('nonempty finite encoder progress interval required')
    breaks = np.linspace(lower, upper, max(1, math.ceil((upper-lower)/spacing))+1)
    return np.r_[np.repeat(lower, 4), breaks[1:-1], np.repeat(upper, 4)]


def cylinder_points(axis, theta, tangent, correction, radius, height):
    """Exact Ry(pitch) Rx(roll) rays, including rotation of the upright support.

    correction columns: carriage dx [m], scan phase dq [m], roll/pitch [rad],
    and optional carriage lateral/vertical displacement [m].
    The cylinder axis is the nominal origin in y,z. No actual tunnel mesh is read.
    """
    correction = np.asarray(correction)
    if correction.ndim != 2 or correction.shape[1] not in (4, 6) or not np.isfinite(correction).all():
        raise ValueError('finite four- or six-component ray correction required')
    dx, dq, roll, pitch = correction[:, :4].T
    lateral = correction[:, 4] if correction.shape[1] > 4 else 0.
    vertical = correction[:, 5] if correction.shape[1] > 5 else 0.
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
            attitude = spline_knots(*self.domain, settings.attitude_spacing_m)
            knots = [position, position, attitude, attitude]
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
        """Map nominal band coordinates to corrected cylinder coordinates."""
        x, q = np.broadcast_arrays(x, q)
        lo, hi, w, supported = self.sampler.row_sources(band, q.ravel()/self.radius)
        result = np.zeros((x.size, 2))
        for ids, weight in ((lo, 1-w), (hi, w)):
            p = self.sampler.projection[ids]
            axis = p['x_axis_m']; theta = p['theta_rad']-2*math.pi*p['segment']
            tangent = (x.ravel()-axis)/self.radius
            result += weight[:, None]*self.points(axis, theta, tangent, coefficients)
        return result.reshape(x.shape+(2,)), supported.reshape(x.shape)

    def serialize(self, coefficients):
        return dict(schema='ssb.global_trajectory.v1', radius_m=self.radius, support_height_m=self.height,
                    progress_domain_m=self.domain, degree=3, knots=[k.tolist() for k in self.knots],
                    coefficients=(np.asarray(coefficients)*self.scale).tolist(),
                    fields=['carriage_dx_m', 'scan_phase_dq_m', 'roll_rad', 'pitch_rad'],
                    sizes=self.sizes, model='Ry(pitch) Rx(roll), upright support and nominal cylinder')


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
        points = self.model.points(self.axis, self.theta, self.tangent, coefficients, self.bases)
        return np.sum(points.reshape(-1, 4, 2)*self.weights[..., None], axis=1)

    def jacobian(self, coefficients):
        """Sparse derivative: numerical local ray derivatives, exact spline basis."""
        model = self.model
        local = model.parameters(coefficients, self.bases)
        matrices = [[], []]
        step = 1e-7
        for field, basis in enumerate(self.bases):
            plus, minus = local.copy(), local.copy()
            plus[:, field] += step; minus[:, field] -= step
            derivative = (cylinder_points(self.axis, self.theta, self.tangent, plus, model.radius, model.height)-
                          cylinder_points(self.axis, self.theta, self.tangent, minus, model.radius, model.height))/(2*step)
            for direction in range(2):
                weighted = diags(derivative[:, direction]*self.weights.ravel()*model.scale) @ basis
                # Sum each consecutive group of four native centre rays.
                matrices[direction].append(self.reducer @ weighted)
        return [hstack(parts, format='csr') for parts in matrices]
