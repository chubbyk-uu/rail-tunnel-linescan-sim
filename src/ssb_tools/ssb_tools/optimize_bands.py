"""D3: robust continuous trajectory fitting from verified D1/D2 public products."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np
from scipy.sparse import csr_matrix, diags, vstack
from scipy.sparse.linalg import spsolve

from .global_geometry import GeometrySettings, Trajectory
from .match_bands import MATCH, verified_bands, graph_components
from .provenance import stage_record
from .public_capture import confined_file
from .session import read_json, sha256_file
from .stage_b_scene import peak_rss_bytes


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
    for w in np.unique(table['window']):
        if int(w) not in accepted:
            raise ValueError('matches from a rejected window')
        selected = table[table['window'] == w]
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


def window_weights(table, grid, settings):
    """Directional noise from training-only local scatter, not NCC or evaluation truth.

    Each window has capped total information, irrespective of feature count. Local
    affine detrending is used only for noise estimation, never as a final warp.
    """
    weights = np.zeros((len(table), 2)); descriptions = []
    train = table['inlier'].astype(bool) & ~table['holdout'].astype(bool)
    pitch = np.array([grid['dx_m'], grid['dq_m']])
    for window in np.unique(table['window']):
        selected = np.flatnonzero(train & (table['window'] == window))
        if len(selected) < 12:
            raise ValueError('insufficient independent training support in match window')
        a = np.column_stack((table['x_a_m'][selected], table['q_a_m'][selected]))
        delta = np.column_stack((table['x_b_m'][selected]-a[:, 0], table['q_b_m'][selected]-a[:, 1]))/pitch
        spatial = (a-a.mean(0))/pitch
        design = np.column_stack((np.ones(len(a)), spatial))
        if np.linalg.matrix_rank(design) < 3:
            raise ValueError('degenerate spatial support in match window')
        residual = delta-design @ np.linalg.lstsq(design, delta, rcond=None)[0]
        sigma = np.maximum(settings.noise_floor_px,
            1.4826*np.median(abs(residual-np.median(residual, axis=0)), axis=0))
        multiplier = np.sqrt(min(settings.effective_points_per_window, len(selected))/len(selected))/sigma
        weights[selected] = multiplier
        descriptions.append(dict(window=int(window), training=len(selected), sigma_px=sigma.tolist(),
                                 effective_points=min(settings.effective_points_per_window, len(selected))))
    return train, weights, descriptions


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


def regularizer(model):
    """Explicit nominal priors, smoothness, and a mean dx/dq coordinate gauge."""
    settings = model.settings
    prior = [settings.position_prior_mm]*2+[settings.attitude_prior_mrad]*2
    curvature = [settings.position_curvature_mm]*2+[settings.attitude_curvature_mrad]*2
    rows = []; cols = []; values = []; row = 0
    for k, size in enumerate(model.sizes):
        start = int(model.starts[k])
        for i in range(size):
            rows.append(row); cols.append(start+i); values.append(1/(prior[k]*np.sqrt(size)))
            row += 1
        for i in range(size-2):
            rows.extend([row]*3); cols.extend([start+i, start+i+1, start+i+2])
            values.extend([1/curvature[k], -2/curvature[k], 1/curvature[k]])
            row += 1
        if k < 2:
            rows.extend([row]*size); cols.extend(range(start, start+size))
            values.extend([1e4/size]*size); row += 1
    return csr_matrix((values, (rows, cols)), shape=(row, model.size))


def residual_summary(delta, pitch):
    values = np.asarray(delta)/np.asarray(pitch)
    if not len(values) or not np.isfinite(values).all():
        raise ValueError('empty or nonfinite optimization score')
    return dict(count=len(values), norm_px=dict(zip(('p50', 'p95', 'p99', 'max'),
        map(float, np.percentile(np.linalg.norm(values, axis=1), [50, 95, 99, 100])))),
        mean_px=values.mean(0).tolist(), std_px=values.std(0).tolist())


def damped_solve(fun, jac, initial, bounds):
    """Bounded, damped Gauss-Newton with a sparse normal system and backtracking.

    Explicit priors make the small coefficient system positive definite. Solving
    that system directly avoids hundreds of LSMR steps through weak pose modes.
    A step is accepted only if the full data-plus-prior objective decreases.
    """
    c = initial.copy(); damping = 1e-4; evaluations = 0; trace = []
    residual = fun(c); evaluations += 1
    cost = float(residual @ residual)/2
    for iteration in range(50):
        matrix = jac(c)
        gradient = np.asarray(matrix.T @ residual).ravel()
        normal = (matrix.T @ matrix).tocsc()
        diagonal = np.maximum(normal.diagonal(), 1e-8)
        optimality = float(np.max(abs(gradient)/np.sqrt(diagonal)))
        trace.append(dict(step=iteration, cost=cost, optimality=optimality, damping=damping,
                          active_bounds=int(np.count_nonzero(abs(c) >= bounds-1e-5))))
        if optimality < 1e-5:
            return c, dict(iterations=iteration, evaluations=evaluations, cost=cost, optimality=optimality)
        step = spsolve(normal+diags(damping*diagonal), -gradient)
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
                    return c, dict(iterations=iteration+1, evaluations=evaluations, cost=cost, optimality=optimality)
                break
        if not accepted:
            damping *= 10
            if damping > 1e6:
                raise ValueError('trajectory line search failed')
    raise ValueError('global trajectory solver exceeded 50 Gauss-Newton steps: '+json.dumps(trace[-5:]))


def fit(model, table, grid):
    train, weights, descriptions = window_weights(table, grid, model.settings)
    holdout = table['inlier'].astype(bool) & table['holdout'].astype(bool)
    if not holdout.any():
        raise ValueError('independent held-out image matches required')
    training = table[train]
    # Train and held-out adjacency must both cover every link in the chain.
    for field_mask in (train, holdout):
        pairs = np.unique(np.column_stack((table['band_a'][field_mask], table['band_b'][field_mask])), axis=0)
        if len(pairs) != len(model.sampler.segments)-1:
            raise ValueError('training or held-out match graph is disconnected')
    a, b = model.native_side(training, 'a'), model.native_side(training, 'b')
    pitch = np.array([grid['dx_m'], grid['dq_m']])
    base = weights[train]; current = base.copy()
    prior = regularizer(model)
    coefficients = np.zeros(model.size)
    history = []
    for iteration in range(model.settings.max_irls):
        def fun(c):
            d = (b.hits(c)-a.hits(c))/pitch
            return np.r_[(d*current).ravel(), prior @ c]

        def jac(c):
            ja, jb = a.jacobian(c), b.jacobian(c)
            directions = [diags(current[:, k]/pitch[k]) @ (jb[k]-ja[k]) for k in range(2)]
            # Interleave axial/circumferential rows to agree with fun().
            stacked = vstack(directions, format='csr')
            order = np.column_stack((np.arange(len(training)), np.arange(len(training))+len(training))).ravel()
            return vstack((stacked[order], prior), format='csr')

        bounds = np.concatenate([np.full(size, 10. if k in (2, 3) else 30.) for k, size in enumerate(model.sizes)])
        coefficients, solver = damped_solve(fun, jac, coefficients, bounds)
        delta = (b.hits(coefficients)-a.hits(coefficients))/pitch
        robust = np.ones(len(training))
        for item in descriptions:
            ids = training['window'] == item['window']
            z = np.linalg.norm(delta[ids]/item['sigma_px'], axis=1)
            # Both individual outliers and coherently wrong windows lose influence.
            point = 1/np.sqrt(1+(z/3)**2)
            window = 1/np.sqrt(1+(float(np.median(z))/3)**2)
            robust[ids] = np.sqrt(point*window)
        updated = base*robust[:, None]
        history.append(dict(solver, iteration=iteration, minimum_window_weight=float(robust.min())))
        if np.max(abs(updated-current)/np.maximum(base, 1e-12)) < .005:
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
    return coefficients, scores, descriptions, history, before, after, evidence


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
        coefficients, scores, noise, history, before, after, evidence = fit(model, table, upstream['grid'])
        output.mkdir(parents=True, exist_ok=False)
        (output/'trajectory.json').write_text(json.dumps(model.serialize(coefficients), indent=2)+'\n')
        np.savez(output/'match_residuals.npz', before_m=before, after_m=after,
                 inlier=table['inlier'], holdout=table['holdout'], window=table['window'])
        per_window = []
        for item in noise:
            selected = (table['window'] == item['window']) & table['inlier'].astype(bool) & table['holdout'].astype(bool)
            if not selected.any():
                raise ValueError('accepted window has no held-out inlier support')
            per_window.append(dict(item, heldout_before=residual_summary(before[selected],
                [upstream['grid']['dx_m'], upstream['grid']['dq_m']]), heldout_after=residual_summary(after[selected],
                [upstream['grid']['dx_m'], upstream['grid']['dq_m']])))
        (output/'windows.json').write_text(json.dumps(per_window, indent=2)+'\n')
        report = dict(schema='ssb.global_optimization.v1', stage='D3', status='complete',
            optical_signature=upstream['optical_signature'], source_observation_hashes=upstream['source_observation_hashes'],
            grid=upstream['grid'], bands=len(sampler.segments), coefficients=model.size, settings=asdict(settings),
            image_consistency=scores, solver=history, observability=evidence,
            image_consistency_gate=dict(threshold_p95_px=.5,
                status='pass' if scores['heldout_after']['norm_px']['p95'] <= .5 else 'fail',
                scope='held-out image consistency only; optical-mesh seam acceptance is still required'),
            gauge='mean carriage dx and scan phase dq anchored to zero; radius and measured lens mapping fixed',
            limitations=['image residuals are not independent optical-mesh seam acceptance',
                'no IMU; fitted attitudes and positions are prior-dependent image corrections, not measured body poses',
                'absolute scale, common deformation and photometric matching bias are not recovered from truth',
                'cubic trajectory cannot recover unobserved bottom-sector or high-frequency motion',
                'yaw, lateral motion, heave, mounting errors and wheel scale are not independently fitted in this first model',
                'no seam blending and no noisy-image robustness acceptance yet'],
            performance=dict(input_check_s=check_s, wall_s=time.monotonic()-started, peak_rss_bytes=peak_rss_bytes()))
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
    parser.add_argument('--attitude-spacing-m', type=float, default=.05)
    args = parser.parse_args()
    report = run(args.unroll, args.matches, args.observable, args.output,
                 GeometrySettings(attitude_spacing_m=args.attitude_spacing_m), args.raw)
    print(json.dumps({k: report[k] for k in ('status', 'coefficients', 'image_consistency', 'performance')}))


if __name__ == '__main__':
    main()
