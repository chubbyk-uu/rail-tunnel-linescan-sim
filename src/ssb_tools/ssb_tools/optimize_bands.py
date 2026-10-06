"""D3: robust continuous trajectory fitting from verified D1/D2 public products."""
import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.sparse import csr_matrix, diags, vstack
from scipy.sparse.linalg import splu

from .global_geometry import (GeometrySettings, Trajectory, add_reconstruction_arguments, curvature_stencil,
                              cylinder_derivatives, in_chunks, refined_attitude_coefficients, settings_from_arguments)
from .quality_targets import SEAM_P95_PX
from .match_bands import MATCH, verified_bands, graph_components
from .provenance import stage_record
from .public_capture import confined_file
from .session import read_json, sha256_file
from .stage_b_scene import peak_rss_bytes

IRLS_WEIGHT_TOLERANCE = .005


def verified_matches(root, d1_root, upstream, sampler):
    root = Path(root).resolve()
    names = ['provenance.json', 'report.json', 'windows.json', 'matches.npy']
    paths = [confined_file(root, name) for name in names]
    provenance = read_json(paths[0])
    if provenance.get('stage') != 'band_matching':
        raise ValueError('verified D2 provenance required')
    for path in paths[1:]:
        hashes = [v for k, v in provenance['outputs'].items() if Path(k).name == path.name]
        if len(hashes) != 1 or hashes[0] != sha256_file(path):
            raise ValueError('D2 product hash mismatch: '+path.name)
    report = read_json(paths[1])
    if (report.get('schema') != 'ssb.band_matches.v1' or
            report.get('optical_signature') != upstream['optical_signature'] or
            report.get('upstream_grid') != upstream['grid'] or
            report.get('source_observation_hashes') != upstream['source_observation_hashes']):
        raise ValueError('D1/D2 public observation identity mismatch')
    for name in ('report.json', 'bands.json', 'projection.npy', 'mapping.npz'):
        hashes = [v for k, v in provenance['inputs'].items() if Path(k).name == name]
        if len(hashes) != 1 or hashes[0] != sha256_file(Path(d1_root)/name):
            raise ValueError('D2 was produced from different D1 geometry: '+name)
    table = np.load(paths[3], allow_pickle=False)
    if table.dtype != MATCH or table.ndim != 1 or not 1 <= len(table) <= 2_100_000:
        raise ValueError('invalid or excessive D2 match table')
    windows = read_json(paths[2])
    accepted = {w['id']: w for w in windows if w['status'] == 'accepted'}
    if len(graph_components(len(sampler.segments), windows)) != 1:
        raise ValueError('disconnected match graph cannot define one global result')
    for w, ids in window_groups(table):
        if int(w) not in accepted:
            raise ValueError('matches from a rejected window')
        selected = table[ids]
        if not np.all(selected['band_a'] == accepted[int(w)]['bands'][0]) or not np.all(
                selected['band_b'] == accepted[int(w)]['bands'][1]):
            raise ValueError('match window band identity mismatch')
    for field in table.dtype.names:
        if table.dtype[field].kind == 'f' and not np.isfinite(table[field]).all():
            raise ValueError('nonfinite D2 match input')
    if np.any(table['inlier'] > 1) or np.any(table['holdout'] > 1):
        raise ValueError('invalid match membership flags')
    if any(np.any((table['band_'+s] < 0) | (table['band_'+s] >= len(sampler.segments))) for s in ('a', 'b')):
        raise ValueError('match band index out of range')
    return table, windows, paths


def public_robot(path, expected_hash, radius):
    path = Path(path).absolute()
    if path.name != 'observable_config.json' or path.parent.name != 'config':
        raise ValueError('only config/observable_config.json is accepted as nominal robot input')
    confined_file(path.parent, path.name)
    if not expected_hash or sha256_file(path) != expected_hash:
        raise ValueError('nominal robot public observation identity mismatch')
    config = read_json(path)
    if config.get('schema') != 'ssb.observable_config.v1' or config['calibration']['radius_m'] != radius:
        raise ValueError('nominal robot geometry mismatch')
    height = float(config['robot']['scan_axis_height_m'])
    if not np.isfinite(height) or height <= 0:
        raise ValueError('positive public scan-axis support height required')
    return height, path


def window_groups(table):
    """Stable row groups: one sort instead of a full-table scan per window."""
    order = np.argsort(table['window'], kind='stable')
    split = np.flatnonzero(np.diff(table['window'][order]))+1
    return [(int(table['window'][ids[0]]), ids) for ids in np.split(order, split) if len(ids)]


def window_weights(table, grid, settings):
    """Directional noise from training-only local scatter, not NCC or evaluation truth.

    Each window has capped total information, irrespective of feature count. Local
    affine detrending is used only for noise estimation, never as a final warp.
    """
    weights = np.zeros((len(table), 2)); descriptions = []
    train = table['inlier'].astype(bool) & ~table['holdout'].astype(bool)
    pitch = np.array([grid['dx_m'], grid['dq_m']])
    for window, ids in window_groups(table):
        selected = ids[train[ids]]
        if len(selected) < 12:
            raise ValueError('insufficient independent training support in match window')
        a = np.column_stack((table['x_a_m'][selected], table['q_a_m'][selected]))
        delta = np.column_stack((table['x_b_m'][selected]-a[:, 0], table['q_b_m'][selected]-a[:, 1]))/pitch
        spatial = (a-a.mean(0))/pitch
        design = np.column_stack((np.ones(len(a)), spatial))
        if np.linalg.matrix_rank(design) < 3:
            raise ValueError('degenerate spatial support in match window')
        local = np.linalg.lstsq(design, delta, rcond=None)[0]
        # An accidentally retained correspondence must not inflate the noise
        # estimate through a high-leverage ordinary affine least-squares fit.
        for _ in range(8):
            residual = delta-design @ local
            centre = np.median(residual, axis=0)
            scale = np.maximum(settings.noise_floor_px,
                1.4826*np.median(abs(residual-centre), axis=0))
            z = np.linalg.norm((residual-centre)/scale, axis=1)
            robust = np.sqrt(np.minimum(1., 2.5/np.maximum(z, 1e-12)))
            local = np.linalg.lstsq(design*robust[:, None], delta*robust[:, None], rcond=None)[0]
        residual = delta-design @ local
        sigma = np.maximum(settings.noise_floor_px,
            1.4826*np.median(abs(residual-np.median(residual, axis=0)), axis=0))
        multiplier = np.sqrt(min(settings.effective_points_per_window, len(selected))/len(selected))/sigma
        weights[selected] = multiplier
        descriptions.append(dict(window=int(window), training=len(selected), sigma_px=sigma.tolist(),
                                 effective_points=min(settings.effective_points_per_window, len(selected))))
    return train, weights, descriptions


@dataclass
class PreparedFit:
    """Fit-invariant public observations, shared only across one run's refinement.

    Robust weights, spline bases and priors are deliberately not cached here.
    Content identity prevents a changed table from silently reusing old noise.
    """
    table: np.ndarray
    digest: str
    identity: tuple
    train: np.ndarray
    holdout: np.ndarray
    training: np.ndarray
    base: np.ndarray
    descriptions: list
    members: list

    @staticmethod
    def content(table):
        return hashlib.sha256(memoryview(np.ascontiguousarray(table)).cast('B')).hexdigest()

    @staticmethod
    def key(grid, settings, bands):
        return (grid['dx_m'], grid['dq_m'], settings.noise_floor_px,
                settings.effective_points_per_window, bands)

    @classmethod
    def prepare(cls, model, table, grid):
        train, weights, descriptions = window_weights(table, grid, model.settings)
        holdout = table['inlier'].astype(bool) & table['holdout'].astype(bool)
        if not holdout.any():
            raise ValueError('independent held-out image matches required')
        for mask in (train, holdout):
            pairs = np.unique(np.column_stack((table['band_a'][mask], table['band_b'][mask])), axis=0)
            if len(pairs) != len(model.sampler.segments)-1:
                raise ValueError('training or held-out match graph is disconnected')
        training = table[train]
        by_window = dict(window_groups(training))
        members = [by_window[item['window']] for item in descriptions]
        return cls(table, cls.content(table), cls.key(grid, model.settings, len(model.sampler.segments)),
                   train, holdout, training, weights[train], descriptions, members)

    def validate(self, model, table, grid):
        if (table is not self.table or self.identity != self.key(grid, model.settings, len(model.sampler.segments))
                or self.digest != self.content(table)):
            raise ValueError('prepared fit observations changed')


def observability(a, b, coefficients, weights, pitch):
    ja, jb = a.jacobian(coefficients), b.jacobian(coefficients)
    matrix = vstack([diags(weights[:, k]/pitch[k]) @ (jb[k]-ja[k]) for k in range(2)], format='csr')
    eigenvalues = np.maximum(np.linalg.eigvalsh((matrix.T @ matrix).toarray()), 0.)
    singular = np.sqrt(eigenvalues)
    threshold = max(float(singular[-1])*1e-6, 1e-8)
    return dict(data_rank=int(np.count_nonzero(singular > threshold)), coefficients=len(coefficients),
                rank_relative_tolerance=1e-6, smallest_singular=float(singular[0]),
                largest_singular=float(singular[-1]),
                unsupported_coefficients=int(np.count_nonzero(np.asarray(abs(matrix).sum(0)).ravel() == 0)),
                interpretation='training-only Jacobian; null/weak modes are anchored by explicit nominal priors')


class FixedJacobian:
    """Weighted B-A data rows (axial/circumferential interleaved) above the priors.

    The sparsity follows only from the fixed exposure positions and knots, so the
    column pattern is built once; each Gauss-Newton step only fills values from
    analytic ray derivatives, in fixed blocks of matches computed in parallel.
    Rays sharing one exposure share their spline row.
    """
    def __init__(self, a, b, prior):
        model = a.model; n = len(a.weights)
        self.model, self.n, self.sides, self.blocks, keys = model, n, (a, b), [], []
        for side, (sign, rays) in enumerate(((-1., a), (1., b))):
            for k, basis in enumerate(rays.bases):
                basis = csr_matrix(basis)
                if basis.shape[0] != 4*n or np.any(np.diff(basis.indptr) != 4):
                    raise ValueError('cubic spline rows with four basis entries required')
                columns = basis.indices.reshape(n, 4, 4)
                values = basis.data.reshape(n, 4, 4)
                pairs = all(np.array_equal(columns[:, i], columns[:, j]) and np.array_equal(values[:, i], values[:, j])
                            for i, j in ((0, 1), (2, 3)))
                groups = [[0, 1], [2, 3]] if pairs else [[0], [1], [2], [3]]
                first = [g[0] for g in groups]
                self.blocks.append([sign, side, k, groups, values[:, first]*model.scale])
                keys.append((np.arange(n)[:, None, None]*model.size+model.starts[k]+columns[:, first]).ravel())
        if model.scale_index is not None:
            keys.append(np.arange(n)*model.size+model.scale_index)
        sizes = [len(k) for k in keys]
        keys, inverse = np.unique(np.concatenate(keys), return_inverse=True)
        for block, part in zip(self.blocks, np.split(inverse, np.cumsum(sizes)[:-1])):
            block.append(part.reshape(n, -1))
        self.scale_positions = np.split(inverse, np.cumsum(sizes)[:-1])[-1] if model.scale_index is not None else None
        self.scale_derivative = ((b.axis.reshape(n, 4)*b.weights).sum(1)-
                                 (a.axis.reshape(n, 4)*a.weights).sum(1))*model.scale
        self.nnz = len(keys)
        rows, columns = keys//model.size, keys % model.size
        counts = np.bincount(rows, minlength=n)
        self.starts = np.r_[0, np.cumsum(counts)]
        # Each match row appears twice (axial, circumferential) with the same columns.
        within = np.arange(self.nnz)-np.repeat(self.starts[:-1], counts)
        self.positions = [2*np.repeat(self.starts[:-1], counts)+d*np.repeat(counts, counts)+within for d in range(2)]
        prior = csr_matrix(prior)
        indices = np.empty(2*self.nnz, np.int64)
        for d in range(2):
            indices[self.positions[d]] = columns
        self.indices = np.r_[indices, prior.indices]
        rows_ptr = np.empty(2*n, np.int64)
        rows_ptr[0::2], rows_ptr[1::2] = 2*self.starts[:-1], 2*self.starts[:-1]+counts
        self.indptr = np.r_[rows_ptr, 2*self.nnz+prior.indptr]
        self.prior = prior.data
        self.shape = (2*n+prior.shape[0], model.size)
        self.columns = np.ascontiguousarray(columns, np.int32) if model.settings.geometry_backend == 'cuda' else None
        self.numeric = None
        if model.settings.geometry_backend != 'numpy':
            from .fast_geometry import CpuJacobian, CudaJacobian
            self.numeric = (CudaJacobian if model.settings.geometry_backend == 'cuda' else CpuJacobian)(self)
        self._coefficients, self._local = None, None
        self._difference_coefficients, self._difference = None, None

    def parameters(self, coefficients):
        if self._coefficients is None or not np.array_equal(coefficients, self._coefficients):
            self._local = [self.model.parameters(coefficients, rays.bases, rays.axis) for rays in self.sides]
            self._coefficients = coefficients.copy()
        return self._local

    def difference(self, coefficients):
        if self._difference_coefficients is None or not np.array_equal(coefficients, self._difference_coefficients):
            if self.numeric is None:
                a, b = self.sides
                self._difference = b.hits(coefficients)-a.hits(coefficients)
            else:
                parameters = coefficients if self.model.settings.geometry_backend == 'cuda' else self.parameters(coefficients)
                self._difference = self.numeric.difference(parameters)
            self._difference.setflags(write=False)
            self._difference_coefficients = coefficients.copy()
        return self._difference

    def __call__(self, coefficients, current, pitch):
        model = self.model
        local = coefficients if model.settings.geometry_backend == 'cuda' else self.parameters(coefficients)
        data = np.empty(2*self.nnz)

        if self.numeric is not None:
            current = np.ascontiguousarray(current, np.float64)
            pitch = np.ascontiguousarray(pitch, np.float64)
            if model.settings.geometry_backend == 'cuda':
                self.numeric.fill(0, self.n, local, current, pitch, data)
            else:
                in_chunks(lambda first, last: self.numeric.fill(first, last, local, current, pitch, data), self.n, .25)
            return csr_matrix((np.r_[data, self.prior], self.indices, self.indptr), shape=self.shape)

        def fill(first, last):
            derivatives = []
            for rays, parameters in zip(self.sides, local):
                a, b = 4*first, 4*last
                d = cylinder_derivatives(rays.axis[a:b], rays.theta[a:b], rays.tangent[a:b], parameters[a:b],
                                         model.radius, model.height)
                derivatives.append(d.reshape(last-first, 4, -1, 2)*rays.weights[first:last, :, None, None])
            lo, hi = self.starts[first], self.starts[last]
            scale = current[first:last]/pitch
            values = np.zeros((2, hi-lo))
            for sign, side, k, groups, basis, inverse in self.blocks:
                field = derivatives[side][:, :, k]
                grouped = np.stack([field[:, g].sum(axis=1) for g in groups], axis=1)*(sign*scale)[:, None, :]
                positions = inverse[first:last].ravel()-lo
                for d in range(2):
                    values[d] += np.bincount(positions, (grouped[..., d, None]*basis[first:last]).ravel(),
                                             minlength=hi-lo)
            for d in range(2):
                data[self.positions[d][lo:hi]] = values[d]
            if self.scale_positions is not None:
                positions = self.scale_positions[first:last]
                data[self.positions[0][positions]] = self.scale_derivative[first:last]*scale[:, 0]

        in_chunks(fill, self.n, .25)  # four native rays per match
        return csr_matrix((np.r_[data, self.prior], self.indices, self.indptr), shape=self.shape)


def normal_equations(matrix, residual, native=False):
    """J^T J and J^T r summed over fixed row chunks in order (thread-count independent)."""
    if native:
        from .fast_normal import accumulator
        part,cap=accumulator(matrix,residual)
    else:
        def part(first, last):
            block = matrix[first:last]
            return block.T @ block, block.T @ residual[first:last]
        cap=None
    parts = in_chunks(part, matrix.shape[0], 2., max_workers=cap)
    normal, gradient = parts[0]
    for product, vector in parts[1:]:
        normal = normal+product; gradient = gradient+vector
    return normal.tocsc(), np.asarray(gradient).ravel()


def regularizer(model):
    """Explicit nominal priors, smoothness, and a mean dx/dq coordinate gauge."""
    settings = model.settings
    prior = [settings.position_prior_mm]*2+[settings.attitude_prior_mrad]*2
    curvature = [settings.position_curvature_mm]*2+[settings.attitude_curvature_mrad]*2
    if settings.fit_translation or settings.fit_heave:
        extra = len(model.fields)-4-int(settings.fit_axis_yaw)
        prior += [settings.translation_prior_mm]*extra
        curvature += [settings.translation_curvature_mm]*extra
    if settings.fit_axis_yaw:
        prior.append(settings.axis_yaw_prior_mrad)
        curvature.append(settings.attitude_curvature_mrad)
    rows = []; cols = []; values = []; row = 0
    for k, size in enumerate(model.sizes):
        start = int(model.starts[k])
        for i in range(size):
            rows.append(row); cols.append(start+i); values.append(1/(prior[k]*np.sqrt(size)))
            row += 1
        for i in range(size-2):
            rows.extend([row]*3); cols.extend([start+i, start+i+1, start+i+2])
            stencil = (curvature_stencil(model.knots[k], i,
                settings.position_spacing_m if k < 2 or (k > 3 and settings.coarse_translation)
                else settings.attitude_spacing_m)
                if settings.observed_knots else np.array([1., -2., 1.]))
            values.extend(stencil/curvature[k])
            row += 1
        if k < 2:
            rows.extend([row]*size); cols.extend(range(start, start+size))
            values.extend([1e4/size]*size); row += 1
    if model.scale_index is not None:
        # Remove the affine component of local dx: it would duplicate the single
        # global scale. Greville abscissae reproduce linear cubic splines exactly.
        g = np.array([model.knots[0][i+1:i+4].mean() for i in range(model.sizes[0])])
        slope = g-g.mean()
        slope /= np.linalg.norm(slope)
        rows.extend([row]*len(slope)); cols.extend(range(model.sizes[0]))
        values.extend(1e4*slope); row += 1
        rows.append(row); cols.append(model.scale_index)
        values.append(model.scale/settings.relative_scale_prior_fraction); row += 1
    if settings.coarse_translation:
        # On a cylinder, a common roll can be exchanged for common lateral/
        # vertical offsets and a rotation of output q. Fix the public midpoint
        # coordinate frame rather than spending iterations along that near-null
        # mode. Four basis entries preserve sparsity (a dense mean would not).
        basis=model.bases(np.array([sum(model.domain)/2]))[2].tocoo()
        rows.extend([row]*len(basis.data))
        cols.extend((model.starts[2]+basis.col).tolist())
        values.extend((1e4*basis.data).tolist()); row+=1
    return csr_matrix((values, (rows, cols)), shape=(row, model.size))


def residual_summary(delta, pitch):
    values = np.asarray(delta)/np.asarray(pitch)
    if not len(values) or not np.isfinite(values).all():
        raise ValueError('empty or nonfinite optimization score')
    return dict(count=len(values), norm_px=dict(zip(('p50', 'p95', 'p99', 'max'),
        map(float, np.percentile(np.linalg.norm(values, axis=1), [50, 95, 99, 100])))),
        mean_px=values.mean(0).tolist(), std_px=values.std(0).tolist())


def symmetric_normal_solve(matrix, rhs):
    """Solve a regularized SPD normal system without general-LU row pivoting.

    Matching row/column ordering avoids fill from the differently scaled spline
    fields and gauges. Check componentwise backward error on the actual system;
    never accept a fast factorization merely because its result is finite.
    """
    matrix = matrix.tocsc()
    solution = splu(matrix, permc_spec='MMD_AT_PLUS_A', diag_pivot_thresh=0.,
                    options={'SymmetricMode': True}).solve(rhs)
    residual = matrix @ solution-rhs
    reference = abs(matrix) @ abs(solution)+abs(rhs)
    if (not np.isfinite(solution).all() or not np.isfinite(residual).all() or
            not np.isfinite(reference).all() or
            np.max(abs(residual)/np.maximum(reference, 1e-300)) > 1e-10):
        raise ValueError('symmetric normal solve failed its backward-error check')
    return solution


def bounded_normal_step(normal, gradient, current, bounds, damping):
    """Solve the damped box quadratic; clipping a coupled Newton step is not a solve."""
    diagonal=np.maximum(normal.diagonal(),1e-8)
    matrix=normal+diags(damping*diagonal)
    lower,upper=-bounds-current,bounds-current
    unconstrained=symmetric_normal_solve(matrix,-gradient)
    if not np.isfinite(unconstrained).all():
        raise ValueError('nonfinite bounded trajectory step')
    if np.all(unconstrained>=lower) and np.all(unconstrained<=upper):
        return unconstrained
    step=np.zeros_like(current)
    active=((lower>=-1e-10)&(gradient>0)) | ((upper<=1e-10)&(gradient<0))
    for iteration in range(min(256,max(20,2*len(step)))):
        derivative=matrix@step+gradient
        free=np.flatnonzero(~active)
        direction=np.zeros_like(step)
        if len(free):
            if iteration==0 and not active.any():
                direction=unconstrained
            else:
                direction[free]=symmetric_normal_solve(matrix[free][:,free],-derivative[free])
        if not np.isfinite(direction).all():
            raise ValueError('nonfinite bounded trajectory step')
        if np.max(abs(direction),initial=0.)<1e-9:
            wrong=(active & (((step<=lower+1e-9)&(derivative<0)) |
                             ((step>=upper-1e-9)&(derivative>0))))
            violation=np.where(wrong,abs(derivative)/np.sqrt(diagonal),0.)
            if np.max(violation,initial=0.)<1e-8:
                return step
            active[int(np.argmax(violation))]=False
            continue
        fraction=np.ones_like(step)
        positive=direction>1e-12; negative=direction<-1e-12
        fraction[positive]=(upper[positive]-step[positive])/direction[positive]
        fraction[negative]=(lower[negative]-step[negative])/direction[negative]
        alpha=max(0.,min(1.,float(fraction.min())))
        step=np.clip(step+alpha*direction,lower,upper)
        if alpha<1.:
            active |= ((positive | negative)&(fraction<=alpha+1e-12))
        else:
            # This is the exact unconstrained minimum on the current free face.
            derivative=matrix@step+gradient
            wrong=(active & (((step<=lower+1e-9)&(derivative<0)) |
                             ((step>=upper-1e-9)&(derivative>0))))
            violation=np.where(wrong,abs(derivative)/np.sqrt(diagonal),0.)
            if np.max(violation,initial=0.)<1e-8:
                return step
            active[int(np.argmax(violation))]=False
    raise ValueError('bounded normal step exceeded its active-set budget')


def damped_solve(fun, jac, initial, bounds, native=False):
    """Bounded, damped Gauss-Newton with a sparse normal system and backtracking.

    Explicit priors make the small coefficient system positive definite. Solving
    that system directly avoids hundreds of LSMR steps through weak pose modes.
    A step is accepted only if the full data-plus-prior objective decreases.
    """
    c = initial.copy(); damping = 1e-4; evaluations = 0; trace = []
    residual = fun(c); evaluations += 1
    cost = float(residual @ residual)/2
    for iteration in range(50):
        normal, gradient = normal_equations(jac(c), residual, native)
        diagonal = np.maximum(normal.diagonal(), 1e-8)
        active = ((c <= -bounds+1e-8) & (gradient > 0)) | ((c >= bounds-1e-8) & (gradient < 0))
        projected_gradient = np.where(active, 0., gradient)
        optimality = float(np.max(abs(projected_gradient)/np.sqrt(diagonal)))
        trace.append(dict(step=iteration, cost=cost, optimality=optimality, damping=damping,
                          active_bounds=int(np.count_nonzero(abs(c) >= bounds-1e-5))))
        if optimality < 1e-5:
            return c, dict(iterations=iteration, evaluations=evaluations, cost=cost, optimality=optimality,
                           active_bounds=int(active.sum()))
        step = bounded_normal_step(normal,gradient,c,bounds,damping)
        if not np.isfinite(step).all():
            raise ValueError('nonfinite damped trajectory step')
        accepted = False
        for fraction in (1., .5, .25, .125, .0625, .03125, .015625):
            trial = np.clip(c+fraction*step, -bounds, bounds)
            r = fun(trial); evaluations += 1
            value = float(r @ r)/2
            if value <= cost:
                change = np.max(abs(trial-c)); improvement = cost-value
                c, residual, cost = trial, r, value
                damping = max(damping/3, 1e-10); accepted = True
                if change < 1e-5 or improvement < 1e-8*max(1., cost):
                    return c, dict(iterations=iteration+1, evaluations=evaluations, cost=cost, optimality=optimality,
                                   active_bounds=int(np.count_nonzero(abs(c) >= bounds-1e-8)))
                break
        if not accepted:
            damping *= 10
            if damping > 1e6:
                raise ValueError('trajectory line search failed')
    raise ValueError('global trajectory solver exceeded 50 Gauss-Newton steps: '+json.dumps(trace[-5:]))


def fit(model, table, grid, initial=None, prepared=None):
    from .fast_geometry import numeric_scope
    with numeric_scope():
        return _fit(model, table, grid, initial, prepared)


def _fit(model, table, grid, initial=None, prepared=None):
    coefficients = np.zeros(model.size) if initial is None else np.asarray(initial, dtype=float).copy()
    if (coefficients.shape != (model.size,) or not np.isfinite(coefficients).all() or
            np.any(abs(coefficients) > model.coefficient_bounds())):
        raise ValueError('finite bounded initial trajectory coefficients required')
    if prepared is None:
        prepared = PreparedFit.prepare(model, table, grid)
    else:
        prepared.validate(model, table, grid)
    train, holdout, training = prepared.train, prepared.holdout, prepared.training
    descriptions, members = prepared.descriptions, prepared.members
    a, b = model.native_side(training, 'a'), model.native_side(training, 'b')
    pitch = np.array([grid['dx_m'], grid['dq_m']])
    base = prepared.base; current = base.copy()
    prior = regularizer(model)
    assembly = FixedJacobian(a, b, prior)
    history = []
    for iteration in range(model.settings.max_irls):
        def fun(c):
            d = assembly.difference(c)/pitch
            return np.r_[(d*current).ravel(), prior @ c]

        def jac(c):
            return assembly(c, current, pitch)

        bounds = model.coefficient_bounds()
        coefficients, solver = damped_solve(fun, jac, coefficients, bounds, native=model.settings.coarse_translation)
        delta = assembly.difference(coefficients)/pitch
        robust = np.ones(len(training))
        for item, ids in zip(descriptions, members):
            z = np.linalg.norm(delta[ids]/item['sigma_px'], axis=1)
            # Both individual outliers and coherently wrong windows lose influence.
            point = 1/np.sqrt(1+(z/3)**2)
            window = 1/np.sqrt(1+(float(np.median(z))/3)**2)
            robust[ids] = np.sqrt(point*window)
        updated = base*robust[:, None]
        change = float(np.max(abs(updated-current)/np.maximum(base, 1e-12)))
        history.append(dict(solver, iteration=iteration, minimum_window_weight=float(robust.min()),
                            max_relative_weight_change=change))
        if change < IRLS_WEIGHT_TOLERANCE:
            break
        current = updated
    # Holdout points were never passed into the solver, weighting or convergence.
    evidence = observability(a, b, coefficients, base, pitch)
    del a, b
    aa, bb = model.native_side(table, 'a'), model.native_side(table, 'b')
    before = bb.hits(np.zeros(model.size))-aa.hits(np.zeros(model.size))
    after = bb.hits(coefficients)-aa.hits(coefficients)
    scores = dict(training_before=residual_summary(before[train], pitch),
                  training_after=residual_summary(after[train], pitch),
                  heldout_before=residual_summary(before[holdout], pitch),
                  heldout_after=residual_summary(after[holdout], pitch))
    if assembly.numeric is not None:
        model.numeric_report = assembly.numeric.describe()
    return coefficients, scores, descriptions, history, before, after, evidence


def irls_summary(history, max_irls):
    """Whether robust reweighting settled, read from one fit's solver history.

    Each entry's weight change and minimum window weight describe the weights
    computed after that solve; the reported coefficients were solved with the
    weights of the previous entry. Reaching max_irls is reported, not hidden.
    """
    if not history or any('max_relative_weight_change' not in item for item in history):
        raise ValueError('IRLS history with recorded weight changes required')
    final = float(history[-1]['max_relative_weight_change'])
    return dict(converged=final < IRLS_WEIGHT_TOLERANCE, iterations=len(history), max_irls=int(max_irls),
                criterion_max_relative_weight_change=IRLS_WEIGHT_TOLERANCE,
                final_max_relative_weight_change=final,
                interpretation='informational: robust weights after the final solve; not an acceptance gate')


def refine_attitude_knots(model, table, residual_m, grid, minimum_step_m):
    """One bounded refinement from coherent TRAINING residuals, never holdout/truth.

    Split a supported interval only when both halves have at least two independent
    accepted windows. Keep the observation-spacing guard and the 2048 coefficient
    cap; rank scarce knots by training median error and report unallocated nodes.
    Position knots, raw pixels and geometry acceptance samples are unchanged.
    """
    train = table['inlier'].astype(bool) & ~table['holdout'].astype(bool)
    residual = np.asarray(residual_m) / [grid['dx_m'], grid['dq_m']]
    if (residual.shape != (len(table), 2) or not np.isfinite(residual[train]).all()
            or not np.isfinite(minimum_step_m) or minimum_step_m <= 0):
        raise ValueError('finite training residuals and positive observation spacing required')
    threshold = SEAM_P95_PX / 2
    centres, bad = {}, []
    p = model.sampler.projection
    for window in np.unique(table['window'][train]):
        ids = np.flatnonzero(train & (table['window'] == window))
        axes = []
        for side in ('a', 'b'):
            sequences = table[side+'_lower_sequence'][ids]
            rows = np.searchsorted(p['sequence'], sequences)
            if np.any(rows >= len(p)) or not np.array_equal(p['sequence'][rows], sequences):
                raise ValueError('refinement source exposure missing from D1')
            axes.append(float(np.median(p['x_axis_m'][rows])))
        centres[int(window)] = axes
        score = float(np.linalg.norm(np.median(residual[ids], axis=0)))
        if score > threshold:
            bad.append((score, int(window)))
    if not np.array_equal(model.knots[2], model.knots[3]):
        raise ValueError('paired attitude knots required for refinement')
    breaks = np.unique(model.knots[2])
    candidates = {}
    for score, window in bad:
        for axis in centres[window]:
            index = int(np.searchsorted(breaks, axis, side='right')-1)
            if not 0 <= index < len(breaks)-1:
                continue
            left, right = breaks[index:index+2]
            mid = (left+right)/2
            if min(mid-left, right-mid) < minimum_step_m:
                continue
            counts = [sum(any(lo <= x < hi for x in axes) for axes in centres.values())
                      for lo, hi in ((left, mid), (mid, right))]
            if min(counts) < 2:
                continue
            candidates[mid] = max(candidates.get(mid, 0.), score)
    capacity = max(0, (2048-model.size)//2)
    priority = sorted(candidates, key=lambda x: (-candidates[x], x))
    selected = priority[:capacity]
    knots = [k.copy() for k in model.knots]
    for field in (2, 3):
        knots[field] = np.sort(np.r_[knots[field], selected])
    report = dict(model='training_median_attitude_split_v1', passes=1, training_only=True,
        trigger_median_px=threshold, minimum_child_step_m=float(minimum_step_m),
        windows_per_child=2, triggered_windows=[w for _, w in bad],
        inserted_progress_m=sorted(selected), candidate_nodes=len(priority),
        budget_deferred_progress_m=sorted(priority[capacity:]), coefficient_limit=2048,
        before_coefficients=model.size, after_coefficients=model.size+2*len(selected))
    return knots, report


def run(unroll, matches, observable, output, settings=GeometrySettings(), raw_root=None):
    started = time.monotonic()
    settings.validate()
    output = Path(output).resolve()
    if any(output.is_relative_to(Path(p).resolve()) for p in (unroll, matches)):
        raise ValueError('D3 output must be separate from upstream inputs')
    sampler, upstream, inputs = verified_bands(unroll, raw_root)
    try:
        table, windows, match_inputs = verified_matches(matches, unroll, upstream, sampler)
        height, observable_path = public_robot(observable,
            upstream['source_observation_hashes'].get('config/observable_config.json'), upstream['grid']['radius_m'])
        # A quarter-revolution gap contains no images. Splines there are priors, not observations.
        spacing = np.median(np.diff(np.unique([w['q_center_m'] for w in windows if w['status'] == 'accepted'])))
        p = sampler.projection
        progress_per_q = np.median([(p['x_axis_m'][b-1]-p['x_axis_m'][a])/
            (upstream['grid']['radius_m']*(p['theta_rad'][b-1]-p['theta_rad'][a]))
            for a, b in sampler.bounds if b-a > 20])
        if not np.isfinite(spacing*progress_per_q) or settings.attitude_spacing_m < 2*spacing*progress_per_q:
            raise ValueError('attitude nodes finer than two observed window spacings')
        model = Trajectory(sampler, upstream['grid']['radius_m'], height, settings)
        check_s = time.monotonic()-started
        prepared = PreparedFit.prepare(model, table, upstream['grid'])
        fit_started = time.monotonic()
        coefficients, scores, noise, history, before, after, evidence = fit(
            model, table, upstream['grid'], prepared=prepared)
        fit_passes = [dict(coefficients=model.size, wall_s=time.monotonic()-fit_started,
                          initialization='zero', solver=history, irls=irls_summary(history, settings.max_irls))]
        refinement = None
        if settings.adaptive_attitude:
            knots, refinement = refine_attitude_knots(model, table, after, upstream['grid'],
                                                      2*spacing*progress_per_q)
            refinement['initial_image_consistency'] = scores
            if refinement['after_coefficients'] > model.size:
                refined = Trajectory(sampler, upstream['grid']['radius_m'], height, settings, knots)
                initial = refined_attitude_coefficients(model, refined, coefficients)
                model = refined
                fit_started = time.monotonic()
                coefficients, scores, noise, history, before, after, evidence = fit(
                    model, table, upstream['grid'], initial=initial, prepared=prepared)
                refinement['initialization'] = 'exact_training_curve_knot_insertion_v1'
                fit_passes.append(dict(coefficients=model.size, wall_s=time.monotonic()-fit_started,
                                       initialization=refinement['initialization'], solver=history,
                                       irls=irls_summary(history, settings.max_irls)))
        output.mkdir(parents=True, exist_ok=False)
        (output/'trajectory.json').write_text(json.dumps(model.serialize(coefficients), indent=2)+'\n')
        np.savez(output/'match_residuals.npz', before_m=before, after_m=after,
                 inlier=table['inlier'], holdout=table['holdout'], window=table['window'])
        per_window = []
        source_windows = {w['id']: w for w in windows if w['status'] == 'accepted'}
        heldout = np.flatnonzero(table['inlier'].astype(bool) & table['holdout'].astype(bool))
        order = heldout[np.argsort(table['window'][heldout], kind='stable')]
        split = np.flatnonzero(np.diff(table['window'][order]))+1
        by_window = dict(zip(table['window'][order][np.r_[0, split]].tolist(), np.split(order, split))) if len(order) else {}
        for item in noise:
            selected = by_window.get(item['window'], np.empty(0, int))
            if not len(selected):
                raise ValueError('accepted window has no held-out inlier support')
            extent = source_windows[item['window']]
            source_window = {k: extent[k] for k in ('bands', 'shape', 'x_first_m', 'q_first_m')}
            per_window.append(dict(item, source_window=source_window, heldout_before=residual_summary(before[selected],
                [upstream['grid']['dx_m'], upstream['grid']['dq_m']]), heldout_after=residual_summary(after[selected],
                [upstream['grid']['dx_m'], upstream['grid']['dq_m']])))
        (output/'windows.json').write_text(json.dumps(per_window, indent=2)+'\n')
        extended = settings.fit_translation or settings.fit_heave
        report = dict(schema=('ssb.global_optimization.v4' if settings.fit_axis_yaw else
                              'ssb.global_optimization.v3' if settings.relative_encoder_scale else
                              'ssb.global_optimization.v2' if extended else 'ssb.global_optimization.v1'),
            stage='D3', status='complete',
            optical_signature=upstream['optical_signature'], source_observation_hashes=upstream['source_observation_hashes'],
            grid=upstream['grid'], bands=len(sampler.segments), coefficients=model.size, settings=asdict(settings),
            image_consistency=scores, solver=history, irls=fit_passes[-1]['irls'],
            irls_all_passes_converged=all(item['irls']['converged'] for item in fit_passes), observability=evidence,
            image_consistency_gate=dict(threshold_p95_px=SEAM_P95_PX,
                status='pass' if scores['heldout_after']['norm_px']['p95'] <= SEAM_P95_PX else 'fail',
                scope='held-out image consistency only; optical-mesh seam acceptance is still required'),
            gauge=('mean carriage dx and scan phase dq anchored to zero; radius and measured lens mapping fixed'+
                   ('; roll at public encoder midpoint anchored to the output-cylinder frame' if settings.coarse_translation else '')),
            limitations=['image residuals are not independent optical-mesh seam acceptance',
                'no IMU; fitted attitudes and positions are prior-dependent image corrections, not measured body poses',
                'absolute scale, common deformation and photometric matching bias are not recovered from truth',
                'cubic trajectory cannot recover unobserved bottom-sector or high-frequency motion',
                ('constant axis yaw is an image-fitted direction correction; physical mount and body yaw remain prior-dependent'
                 if settings.fit_axis_yaw else 'yaw and fixed mounting parameters are not independently recovered; relative encoder scale is prior-dependent'
                 if settings.relative_encoder_scale else 'yaw, mounting errors and wheel scale are not independently fitted; absolute translation modes remain prior-dependent'
                 if extended else 'yaw, lateral motion, heave, mounting errors and wheel scale are not independently fitted in this first model'),
                'no seam blending; geometry, coverage and noisy-image robustness require independent acceptance'],
            performance=dict(input_check_s=check_s, wall_s=time.monotonic()-started,
                             peak_rss_bytes=peak_rss_bytes(), fit_passes=fit_passes))
        if refinement is not None:
            report['attitude_refinement'] = refinement
        if settings.coarse_translation:
            from .fast_normal import backend
            report['normal_backend']=backend()[1]
        if settings.geometry_backend != 'numpy':
            report['geometry_backend'] = model.numeric_report
        if settings.relative_encoder_scale:
            report['relative_encoder_scale'] = model.serialize(coefficients)['relative_encoder_scale']
            report['gauge'] += '; local dx linear mode anchored; global scale is relative to fixed radius/lens priors'
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        provenance = stage_record('global_optimization', inputs+match_inputs+[observable_path], sorted(output.iterdir()),
                                  dict(settings=asdict(settings)))
        (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
        return report
    finally:
        if hasattr(sampler.native, 'close'):
            sampler.native.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--unroll', required=True); parser.add_argument('--matches', required=True)
    parser.add_argument('--observable', required=True); parser.add_argument('--output', required=True)
    parser.add_argument('--raw', help='relocated public raw directory')
    add_reconstruction_arguments(parser)
    args = parser.parse_args()
    report = run(args.unroll, args.matches, args.observable, args.output, settings_from_arguments(args), args.raw)
    print(json.dumps({k: report[k] for k in ('status', 'coefficients', 'image_consistency', 'performance')}))

if __name__ == '__main__':
    main()
