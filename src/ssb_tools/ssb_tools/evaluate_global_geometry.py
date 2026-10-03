"""Independent optical-mesh scoring of D3 output sampling; evaluation only.

Unlike a correspondence residual, a seam compares the true native footprints of
two bands sampling the SAME output coordinate. Truth is never fitted, or imported
by the production optimizer. Boundary checks are exact on the perimeter, while
the interior and seams are sampled; this does not claim complete image coverage.
"""
import argparse
import json
import multiprocessing
import os
from pathlib import Path
import time

import numpy as np
import yaml

from .evaluate_band_matches import mesh_points
from .global_resample import inverse_points, load_global, native_points
from .initial_unroll import grid_axes
from .match_bands import MATCH, verified_bands
from .optimize_bands import residual_summary
from .quality_targets import SEAM_P95_PX, CONTROLLED_SEAM_P95_PX
from .provenance import stage_record
from .ref_mesh import OpticalMesh
from .session import Session, read_json, sha256_file
from .stage_b_scene import peak_rss_bytes

SAMPLING_SCHEMA = 'ssb.public_common_overlap.v2'
_SHARED = None


def _apply(task):
    function, items = task
    return [function(item, **_SHARED) for item in items]


def parallel_map(function, items, shared, workers):
    """function(item, **shared) for each item, in order.

    Each item keeps exactly the calls (and inverse-mapping batches) of the serial
    loop, so the result cannot depend on the worker count. Forked workers share the
    read-only model, raw mappings and mesh instead of rebuilding them.
    """
    global _SHARED
    if workers <= 1 or len(items) < 2:
        return [function(item, **shared) for item in items]
    workers = min(workers, len(items))
    bounds = np.linspace(0, len(items), 4*workers+1).round().astype(int)
    tasks = [(function, items[a:b]) for a, b in zip(bounds[:-1], bounds[1:]) if b > a]
    _SHARED = shared
    try:
        with multiprocessing.get_context('fork').Pool(workers) as pool:
            return [result for chunk in pool.map(_apply, tasks) for result in chunk]
    finally:
        _SHARED = None


def column_clearances(model, coefficients, bands, x, q):
    """Sensor-domain distances only: no intensity, truth, or angular-gap selection.

    Angular support and bad/saturated pixels are deliberately checked later at
    the fixed samples. They must not be used to move a probe away from a gap.
    """
    x = np.asarray(x, float)
    q = np.full(x.shape, q)
    usable = model.sampler.output_offsets[model.sampler.geometry_valid]
    lower = max(usable[0], model.sampler.offsets[0])
    upper = min(usable[-1], model.sampler.offsets[-1])
    result = np.full(x.shape+(2,), np.inf)
    for c in (np.zeros(model.size), coefficients):
        for band in bands:
            if np.any(c):
                nx, nq, _ = inverse_points(model, c, band, x, q)
                projected, _ = model.forward(band, nx, nq, c)
                converged = np.isfinite(projected).all(axis=-1) & (
                    np.max(abs(projected-np.stack((x, q), axis=-1)), axis=-1) < 1e-8)
            else:
                nx, nq, converged = x, q, np.ones(x.shape, bool)
            lo, hi, _, _ = model.sampler.row_sources(band, nq/model.radius)
            axes = model.sampler.projection['x_axis_m'][np.stack((lo, hi), axis=-1)]
            delta = nx[..., None]-axes
            clearance = np.stack(((delta-lower).min(axis=-1),
                                  (upper-delta).min(axis=-1)), axis=-1)
            result = np.minimum(result, np.where(converged[..., None], clearance, -np.inf))
    return result


def column_interval(model, coefficients, bands, q, bounds, guard):
    """Bounded intersection of the two public sensor domains, ignoring intensity."""
    left, right = bounds
    if right <= left:
        return None
    def clear(x):
        return column_clearances(model, coefficients, bands, x, q)
    edges = clear([left, right])
    if edges[1, 0] < guard or edges[0, 1] < guard:
        return None
    if edges[0, 0] < guard:
        a, b = left, right
        for _ in range(20):
            mid = (a+b)/2
            if clear([mid])[0, 0] >= guard:
                b = mid
            else:
                a = mid
        left = b
    if edges[1, 1] < guard:
        a, b = left, right
        for _ in range(20):
            mid = (a+b)/2
            if clear([mid])[0, 1] >= guard:
                a = mid
            else:
                b = mid
        right = a
    if right <= left or np.min(clear([left, right])) < guard-1e-8:
        return None
    return [left, right]


def geometry_supported(model, coefficients, bands, x, q):
    """Proof from public exposure rows and calibration only, never brightness."""
    q = np.full(len(x), q)
    for c in (np.zeros(model.size), coefficients):
        for band in bands:
            nx, nq, valid = inverse_points(model, c, band, x, q)
            lo, hi, _, _ = model.sampler.row_sources(band, nq/model.radius)
            for ids in (lo, hi):
                delta = nx-model.sampler.projection['x_axis_m'][ids]
                col = np.interp(delta, model.sampler.output_offsets, model.sampler.columns)
                c0 = np.floor(col).astype(int)
                c1 = np.minimum(c0+1, len(model.sampler.columns)-1)
                valid &= model.sampler.geometry_valid[c0] & model.sampler.geometry_valid[c1]
            if not valid.all():
                return False
    return True


def shared_seam_plan(model, coefficients, grid, spacing_m=.2, workers=1):
    """Keep the fixed q lattices; bound x by BOTH public sampling maps.

    The algorithm is frozen before capture, the exact locations are written
    before opening truth. Original nominal boundary probes remain diagnostics.
    A truly missing common interval stays planned/unmeasurable. A seam can be
    outside the output target only if an unclipped common interval AND public
    exposure support are proved. Original probes stay in the diagnostic either
    way. Gaps and bad pixels inside the target still fail at the fixed samples.
    """
    coefficients = np.asarray(coefficients, float)
    if coefficients.shape != (model.size,) or not np.isfinite(coefficients).all():
        raise ValueError('finite verified trajectory required for a public seam plan')
    plan = seam_plan(model.sampler, grid, spacing_m)
    planned = [index for index, window in enumerate(plan) if window['status'] == 'planned']
    shared = dict(model=model, coefficients=coefficients, grid=grid, guard=2*grid['dx_m'])
    for index, window in zip(planned, parallel_map(plan_window, [plan[i] for i in planned], shared, workers)):
        plan[index] = window
    return plan


def plan_window(window, model, coefficients, grid, guard):
    window['nominal_probe_x_m'] = list(window['x_m'])
    bounds = column_interval(model, coefficients, window['bands'], window['q_center_m'],
                             window['x_m'], guard)
    if bounds is None:
        full = column_interval(model, coefficients, window['bands'], window['q_center_m'],
                               window['nominal_unclipped_x_m'], guard)
        target = grid['target_x_m']
        outside = full is not None and (full[1] <= target[0]+guard or full[0] >= target[1]-guard)
        if outside and geometry_supported(model, coefficients, window['bands'],
                                          np.linspace(*full, 9), window['q_center_m']):
            window.update(status='outside_target', shared_support='outside_target',
                          outside_target_shared_x_m=full, outside_target_proof_points=9,
                          reason='common publicly observed interval lies outside output target')
        else:
            window.update(shared_support='unmeasurable', reason='no converged common column interval')
    else:
        left, right = bounds
        window.update(x_m=[left, right], shared_support='measurable',
                      column_guard_m=guard,
                      trimmed_m=[left-window['nominal_probe_x_m'][0],
                                 window['nominal_probe_x_m'][1]-right])
    model.sampler.native.release()
    return window


def sources_at(model, coefficients, band, x, q):
    """Production sampling identity only; truth is traced separately by mesh_points."""
    x, q = np.broadcast_arrays(np.asarray(x, float), np.asarray(q, float))
    if x.ndim != 1 or len(x) > 2048:
        raise ValueError('one-dimensional source batch of at most 2048 points required')
    if np.any(coefficients):
        nominal_x, nominal_q, inverse_ok = inverse_points(model, coefficients, band, x, q)
    else:
        nominal_x, nominal_q, inverse_ok = x, q, np.ones(len(x), bool)
    _, valid, score, sources = native_points(model.sampler, band, nominal_x, nominal_q, model.radius)
    table = np.zeros(len(x), MATCH)
    table['band_a'] = band
    for key, value in sources.items():
        table['a_'+key] = value
    return table, valid & inverse_ok, score


def map_points(model, coefficients, x, q, rows, camera, truth, mesh, workers=1):
    """Actual source selected by the production central-column rule, then true hit."""
    x, q = np.asarray(x), np.asarray(q)
    chunks = [(x[start:start+512], q[start:start+512]) for start in range(0, len(x), 512)]
    parts = parallel_map(map_chunk, chunks, dict(model=model, coefficients=coefficients, rows=rows,
                                                 camera=camera, truth=truth, mesh=mesh), workers)
    if not parts:
        return np.full((0, 2), np.nan), np.full(0, -1, np.int16), np.full(0, -1, np.int16)
    return tuple(np.concatenate(values) for values in zip(*parts))


def map_chunk(chunk, model, coefficients, rows, camera, truth, mesh):
    x, q = chunk
    points = np.full((len(x), 2), np.nan)
    material = np.full(len(x), -1, np.int16)
    selected_band = np.full(len(x), -1, np.int16)
    best = np.full(len(x), -np.inf)
    selected = np.zeros(len(x), MATCH)
    for band in range(len(model.sampler.segments)):
        table, valid, score = sources_at(model, coefficients, band, x, q)
        use = valid & (score > best)
        selected[use] = table[use]
        best[use] = score[use]
    valid = np.isfinite(best)
    if valid.any():
        points[valid], material[valid] = mesh_points(selected[valid], 'a', rows, camera, truth, mesh)
        selected_band[valid] = selected['band_a'][valid]
    model.sampler.native.release()
    return points, material, selected_band


def endpoint_drift(points, valid, xs, qs):
    """Difference between same-q endpoints. Removes translation only, never scale/tilt."""
    complete = valid[:, 0] & valid[:, -1]
    if not complete.any():
        return dict(status='unmeasurable', reason='no same-q endpoint pair is supported')
    reference_length = float(xs[-1]-xs[0])
    delta = points[complete, -1]-points[complete, 0]-np.array([reference_length, 0.])
    return dict(status='measured', paired_angles=int(complete.sum()),
                endpoint_delta_mean_m=delta.mean(0).tolist(),
                endpoint_delta_p95_m=np.percentile(abs(delta), 95, axis=0).tolist(),
                axial_scale_error_percent=float(100*delta[:, 0].mean()/reference_length),
                axial_scale_error_p95_abs_percent=float(np.percentile(100*abs(delta[:, 0])/reference_length, 95)),
                output_endpoint_x_m=[float(xs[0]), float(xs[-1])],
                convention='same-q true end minus start, subtract only nominal span; no fitted similarity')


def boundary_coordinates(grid):
    """Every output pixel on all four boundaries, with no duplicate corner pixels."""
    angles, xs = grid_axes(grid)
    qs = angles*grid['radius_m']
    return dict(top=(xs, np.full(len(xs), qs[0])), bottom=(xs, np.full(len(xs), qs[-1])),
                left=(np.full(len(qs)-2, xs[0]), qs[1:-1]),
                right=(np.full(len(qs)-2, xs[-1]), qs[1:-1]))


def boundary_chunk(chunk, model, coefficients):
    x, q = chunk
    supported = np.zeros(len(x), bool)
    for band in range(len(model.sampler.segments)):
        # Skip bands whose declared correction bounds cannot reach this x.
        lo, hi = model.sampler.band_x[band]
        margin = .03+.04*(model.radius+model.height)
        if np.max(x) < lo-margin or np.min(x) > hi+margin:
            continue
        supported |= sources_at(model, coefficients, band, x, q)[1]
    model.sampler.native.release()
    return supported


def boundary_support(model, coefficients, grid, workers=1):
    output = {}
    for name, (x, q) in boundary_coordinates(grid).items():
        chunks = [(x[start:start+512], q[start:start+512]) for start in range(0, len(x), 512)]
        parts = parallel_map(boundary_chunk, chunks, dict(model=model, coefficients=coefficients), workers)
        supported = np.concatenate(parts) if parts else np.zeros(0, bool)
        missing = np.flatnonzero(~supported)
        output[name] = dict(pixels=len(x), missing_pixels=len(missing),
                            missing_fraction=float(len(missing)/len(x)) if len(x) else 0.,
                            first_missing_xq_m=[float(x[missing[0]]), float(q[missing[0]])] if len(missing) else None)
    return output


def verify_session(session, upstream):
    if session.summary.get('status') != 'complete':
        raise ValueError('complete capture session required for geometry evaluation')
    for name in ('config/observable_config.json', 'metadata/manifest.json', 'raw/index.json'):
        if upstream['source_observation_hashes'].get(name) != sha256_file(session.root/name):
            raise ValueError('evaluation session differs from public observation: '+name)
    for name in ('evaluation/truth.json', 'evaluation/manifest.json', 'evaluation/config_source.yaml',
                 'config/backend.json', 'config/provenance.json'):
        if session.summary['files'].get(name) != sha256_file(session.root/name):
            raise ValueError('evaluation archived identity mismatch: '+name)


def seam_plan(sampler, grid, spacing_m=.2):
    """Two fixed interleaved lattices, independent of matches, fit and truth.

    At the frozen 0.2 m spacing the phases are 0.05 and 0.15 m from the
    output's lower q boundary. All exclusions are retained, never selected
    using a score. Axial points cover a bounded nominal overlap at that q.
    """
    if not np.isfinite(spacing_m) or spacing_m < .09:
        raise ValueError('bounded fixed seam sampling plan required')
    lower, upper = np.asarray(grid['theta_rad'])*grid['radius_m']
    usable = sampler.output_offsets[sampler.geometry_valid]
    plan = []
    for band in range(len(sampler.segments)-1):
        if sampler.segments[band+1] != sampler.segments[band]+1:
            continue
        for phase in (.25, .75):
            for q in np.arange(lower+phase*spacing_m, upper, spacing_m):
                item = dict(id=len(plan), bands=[band, band+1], phase=phase,
                            q_center_m=float(q))
                ranges = []
                for k in item['bands']:
                    lo, hi, _, supported = sampler.row_sources(k, np.array([q/grid['radius_m']]))
                    if not supported.all():
                        break
                    axis = sampler.projection['x_axis_m'][np.r_[lo, hi]]
                    ranges.append((float(axis.max()+usable[0]), float(axis.min()+usable[-1])))
                if len(ranges) != 2:
                    item.update(status='unmeasurable', reason='unsupported nominal exposure angle')
                else:
                    item['nominal_unclipped_x_m'] = [max(r[0] for r in ranges)+2*grid['dx_m'],
                                                    min(r[1] for r in ranges)-2*grid['dx_m']]
                    left = max(grid['target_x_m'][0], *(r[0] for r in ranges))+2*grid['dx_m']
                    right = min(grid['target_x_m'][1], *(r[1] for r in ranges))-2*grid['dx_m']
                    if right <= left:
                        item.update(status='excluded', reason='no nominal overlap inside output target')
                    else:
                        half = min((right-left)/2, 1023*grid['dx_m']/2)
                        centre = (left+right)/2
                        item.update(status='planned', x_m=[centre-half, centre+half])
                plan.append(item)
                if len(plan) > 32768:
                    raise ValueError('evaluation exceeds 32768-location budget')
    return plan


def match_membership(x, q, bands, training_windows, grid):
    """Label fixed evaluation points by verified D3 training rectangles.

    Membership labels do not alter the sampling plan. Rejected D2 windows
    absent from the fitted D3 report provide no training coverage.
    """
    inside = np.zeros(len(x), bool)
    for item in training_windows:
        source = item['source_window']
        if source['bands'] != bands:
            continue
        height, width = source['shape']
        x0, q0 = source['x_first_m'], source['q_first_m']
        inside |= ((x >= x0) & (x <= x0+(width-1)*grid['dx_m']) &
                   (q >= q0) & (q <= q0+(height-1)*grid['dq_m']))
    return np.where(inside, 'within_match_window', 'between_match_windows')


def stratum_gate(strata, threshold):
    """Each stratum must be measurable and pass; pooled scores cannot hide gaps."""
    groups = strata['optimized']
    if any(v['status'] == 'unmeasurable' for v in groups.values()):
        status = 'unmeasurable'
    elif any(v['missing_samples'] or v['norm_px']['p95'] > threshold for v in groups.values()):
        status = 'fail'
    else:
        status = 'pass'
    return dict(threshold_p95_px=threshold, status=status,
                scope='both within-match and between-match strata must pass with no missing planned samples')


def score_window(window, model, coefficients, samples_across, training_windows, grid, pitch,
                 rows, camera, truth, mesh):
    """Original-probe diagnostic plus nominal/optimized truth scores of one window."""
    original_x = np.linspace(*window['nominal_probe_x_m'], samples_across)
    q = np.full(len(original_x), window['q_center_m'])
    original_valid = [sources_at(model, coefficients, band, original_x, q)[1]
                      for band in window['bands']]
    legacy = dict(window=window['id'], bands=window['bands'],
                  x_m=window['nominal_probe_x_m'], q_m=float(q[0]),
                  missing_samples=int((~(original_valid[0] & original_valid[1])).sum()),
                  side_valid=[v.tolist() for v in original_valid])
    if window['status'] != 'planned':
        model.sampler.native.release()
        return legacy, None, [], {}
    x = np.linspace(*window['x_m'], samples_across)
    labels = match_membership(x, q, window['bands'], training_windows, grid)
    strata = ('within_match_window', 'between_match_windows')
    record = dict(window=window['id'], bands=window['bands'], phase=window['phase'],
                  q_m=float(q[0]), x_m=[float(x[0]), float(x[-1])],
                  stratum_counts={s: int((labels == s).sum()) for s in strata})
    samples, missing = [], {}
    for name, c in (('nominal', np.zeros(model.size)), ('optimized', coefficients)):
        tables = [sources_at(model, c, band, x, q) for band in window['bands']]
        valid = tables[0][1] & tables[1][1]
        if window['shared_support'] == 'unmeasurable':
            valid[:] = False
        for stratum in strata:
            missing[name, stratum] = int(((labels == stratum) & ~valid).sum())
        if not valid.any():
            record[name] = dict(status='unmeasurable', missing_samples=len(x))
            continue
        a, ma = mesh_points(tables[0][0][valid], 'a', rows, camera, truth, mesh)
        b, mb = mesh_points(tables[1][0][valid], 'a', rows, camera, truth, mesh)
        delta = b-a
        record[name] = dict(status='measured', missing_samples=int((~valid).sum()),
                            summary=residual_summary(delta, pitch))
        samples.append((name, window['id'], window['bands'], x[valid], q[valid], delta, ma, mb, labels[valid]))
    model.sampler.native.release()
    return legacy, record, samples, missing


def run(session_root, unroll, trajectory, output, scene=None, spacing_m=.2, samples_across=9, raw_root=None,
        workers=1):
    started = time.monotonic()
    if type(workers) is not int or not 1 <= workers <= (os.cpu_count() or 1):
        raise ValueError('worker count must be between 1 and the CPU count')
    output = Path(output).resolve()
    if 'evaluation' not in output.parts:
        raise ValueError('independent truth-side results must be inside evaluation/')
    if (not np.isfinite(spacing_m) or spacing_m < .09 or not isinstance(samples_across, int)
            or not 3 <= samples_across <= 33):
        raise ValueError('bounded fixed seam sampling plan required')
    if any(output.is_relative_to(Path(p).resolve()) for p in (unroll, trajectory)):
        raise ValueError('evaluation output must be separate from production inputs')
    session = Session(session_root)
    sampler, upstream, inputs = verified_bands(unroll, raw_root)
    try:
        verify_session(session, upstream)
        model, coefficients, optimized, trajectory_inputs = load_global(trajectory, sampler, upstream, unroll)
        grid = upstream['grid']; pitch = [grid['dx_m'], grid['dq_m']]
        windows = shared_seam_plan(model, coefficients, grid, spacing_m, workers)
        planned = [w for w in windows if w['status'] == 'planned']
        if not planned:
            raise ValueError('no nominal adjacent overlap to evaluate')
        output.mkdir(parents=True, exist_ok=False)
        plan_path = output/'sampling_plan.json'
        plan_path.write_text(json.dumps(windows, indent=2)+'\n')
        plan_hash = sha256_file(plan_path)
        (output/'sampling_plan_provenance.json').write_text(json.dumps(stage_record(
            'public_common_overlap_plan', inputs+trajectory_inputs, [plan_path],
            dict(schema=SAMPLING_SCHEMA, truth_used=False, angular_gaps_not_trimmed=True)), indent=2)+'\n')
        training_windows = read_json(Path(trajectory)/'windows.json')
        truth, config = session.truth(), session.config()
        generation = yaml.safe_load((session.root/'evaluation/config_source.yaml').read_text())
        track = generation.get('truth', {}).get('track_irregularity', {})
        imposed_track = any(float(track.get(k, 0)) > 0 for k in ('chord10_max_m', 'cross_level_tier_m'))
        rows = session.evaluation('row_truth')
        mesh = OpticalMesh.from_session(session, scene)
        per_window, samples, legacy = [], [], []
        stratum_missing = {name: dict(within_match_window=0, between_match_windows=0)
                           for name in ('nominal', 'optimized')}
        scored = parallel_map(score_window, [w for w in windows if 'nominal_probe_x_m' in w],
            dict(model=model, coefficients=coefficients, samples_across=samples_across,
                 training_windows=training_windows, grid=grid, pitch=pitch, rows=rows,
                 camera=config['camera'], truth=truth, mesh=mesh), workers)
        for entry, record, window_samples, missing in scored:
            legacy.append(entry)
            if record is None:
                continue
            for (name, stratum), count in missing.items():
                stratum_missing[name][stratum] += count
            samples.extend(window_samples)
            per_window.append(record)
        statistics = {}
        for name in ('nominal', 'optimized'):
            values = [v[5] for v in samples if v[0] == name]
            if not values:
                raise ValueError('no independent seam samples: '+name)
            statistics[name] = residual_summary(np.concatenate(values), pitch)
            statistics[name]['missing_samples'] = sum(w[name]['missing_samples'] for w in per_window)
        strata = {}
        for name in ('nominal', 'optimized'):
            strata[name] = {}
            for stratum, missing in stratum_missing[name].items():
                values = [v[5][v[8] == stratum] for v in samples if v[0] == name]
                values = np.concatenate(values) if values else np.empty((0, 2))
                result = (dict(status='measured', **residual_summary(values, pitch)) if len(values)
                          else dict(status='unmeasurable', count=0))
                result['missing_samples'] = missing
                strata[name][stratum] = result
        # Deterministic interior grid, including both axial and angular endpoints.
        angles, all_x = grid_axes(grid)
        xs = all_x[np.unique(np.rint(np.linspace(0, len(all_x)-1, 33)).astype(int))]
        qs = angles[np.unique(np.rint(np.linspace(0, len(angles)-1, 49)).astype(int))]*model.radius
        xx, qq = np.meshgrid(xs, qs)
        maps = {}
        boundaries = {}
        arrays = {}
        for name, c in (('nominal', np.zeros(model.size)), ('optimized', coefficients)):
            points, materials, bands = map_points(model, c, xx.ravel(), qq.ravel(), rows, config['camera'], truth, mesh,
                                                  workers)
            valid = bands >= 0
            maps[name] = dict(supported_samples=int(valid.sum()), missing_samples=int((~valid).sum()),
                              absolute_error=residual_summary(points[valid]-np.column_stack((xx.ravel(), qq.ravel()))[valid], pitch),
                              drift=endpoint_drift(points.reshape(xx.shape+(2,)), valid.reshape(xx.shape), xs, qs))
            arrays[name+'_true_xq_m'] = points.reshape(xx.shape+(2,))
            arrays[name+'_band'] = bands.reshape(xx.shape)
            arrays[name+'_material'] = materials.reshape(xx.shape)
            boundaries[name] = boundary_support(model, c, grid, workers)
        seam_gate = stratum_gate(strata, SEAM_P95_PX)
        controlled_gate = stratum_gate(strata, CONTROLLED_SEAM_P95_PX)
        perimeter_missing = sum(v['missing_pixels'] for v in boundaries['optimized'].values())
        report = dict(schema='ssb.global_geometry_evaluation.v3', evaluation_only=True, grid=grid,
            seam=statistics, seam_strata=strata, windows=per_window, mapping=maps, boundary=boundaries,
            gates=dict(strict_seam=seam_gate, controlled_seam=controlled_gate,
                       perimeter=dict(status='fail' if perimeter_missing else 'pass',
                       missing_pixels=perimeter_missing, scope='exact perimeter pixel support; not full interior coverage')),
            capture_conditions=dict(imposed_track_irregularity=imposed_track, archived_track_parameters=track,
                interpretation='1 px is the project seam target; 3 px is the secondary controlled-error target. '
                               'A perturbed track must not be labelled a perfectly flat ideal baseline.'),
            sampling=dict(schema=SAMPLING_SCHEMA, plan_sha256=plan_hash,
                          spacing_q_m=spacing_m, samples_across=samples_across, planned_windows=len(planned),
                          phase_fractions=[.25, .75], anchor='output lower q boundary',
                          rule='fixed q lattices; common nominal/fitted public sensor-column domain; '
                               'exact points saved before truth; original boundary probes retained',
                          shared_unmeasurable_windows=sum(w.get('shared_support') == 'unmeasurable' for w in windows),
                          outside_target_windows=sum(w['status'] == 'outside_target' for w in windows),
                          unmeasurable_windows=sum(w['status'] == 'unmeasurable' for w in windows),
                          excluded_windows=sum(w['status'] == 'excluded' for w in windows),
                          interior_shape=list(xx.shape)),
            nominal_domain_probe_diagnostic=dict(missing_samples=sum(w['missing_samples'] for w in legacy),
                windows=legacy, interpretation='original nominal-domain probes; missing dual support is '
                'not an output-image hole and is not silently converted into a valid measurement'),
            reference='archived float64 optical mesh and true native pixel-centre rays',
            limitations=['fixed seam/interior samples, not exhaustive surface accuracy or defect-width acceptance',
                'native centre-ray barycentre reference; exposure/pixel-area integration is not a point measurement',
                'absolute error retains coordinate gauge; endpoint differences remove translation only',
                'no truth offset, similarity or affine fit is subtracted from the seam or scale scores'],
            performance=dict(wall_s=time.monotonic()-started, workers=workers, peak_rss_bytes=peak_rss_bytes()))
        if sha256_file(plan_path) != plan_hash:
            raise ValueError('public sampling plan changed during truth evaluation')
        arrays.update(output_x_m=xs, output_q_m=qs)
        np.savez(output/'mapping_samples.npz', **arrays)
        dtype = [('variant','U10'), ('window','<i4'), ('band_a','<i2'), ('band_b','<i2'),
                 ('x_m','<f8'), ('q_m','<f8'), ('dx_m','<f8'), ('dq_m','<f8'), ('material_a','<i2'), ('material_b','<i2'),
                 ('stratum', 'U24')]
        records = []
        for name, identifier, bands, x, q, delta, ma, mb, labels in samples:
            records.extend(zip([name]*len(x), [identifier]*len(x), [bands[0]]*len(x), [bands[1]]*len(x),
                               x, q, delta[:,0], delta[:,1], ma, mb, labels))
        np.save(output/'seam_samples.npy', np.array(records, dtype=dtype))
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        manifest = read_json(session.root/'evaluation/manifest.json')
        inputs += trajectory_inputs+[session.root/'session.json',session.root/'evaluation/truth.json',
                  session.root/'evaluation/manifest.json',session.root/'evaluation'/manifest['row_truth']['file'],
                  session.root/'evaluation/config_source.yaml', session.root/'config/backend.json', *mesh.sources]
        (output/'provenance.json').write_text(json.dumps(stage_record('evaluate_global_geometry', inputs,
            sorted(output.iterdir()), report['sampling']), indent=2)+'\n')
        return report
    finally:
        if hasattr(sampler.native, 'close'):
            sampler.native.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('session','unroll','trajectory','output'):
        parser.add_argument('--'+name, required=True)
    parser.add_argument('--scene'); parser.add_argument('--raw')
    parser.add_argument('--workers', type=int, default=8,
                        help='evaluation processes; every score is identical for any count')
    args = parser.parse_args()
    report = run(args.session,args.unroll,args.trajectory,args.output,args.scene,raw_root=args.raw,
                 workers=args.workers)
    print(json.dumps({k:report[k] for k in ('seam','seam_strata','mapping','boundary','gates','performance')}))


if __name__ == '__main__':
    main()
