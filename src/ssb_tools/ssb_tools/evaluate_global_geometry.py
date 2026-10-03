"""Independent optical-mesh scoring of D3 output sampling; evaluation only.

Unlike a correspondence residual, a seam compares the true native footprints of
two bands sampling the SAME output coordinate. Truth is never fitted, or imported
by the production optimizer. Boundary checks are exact on the perimeter, while
the interior and seams are sampled; this does not claim complete image coverage.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import yaml

from .band_matching import MatchSettings
from .evaluate_band_matches import mesh_points
from .global_resample import inverse_points, load_global, native_points
from .initial_unroll import grid_axes
from .match_bands import MATCH, plan_windows, verified_bands
from .optimize_bands import residual_summary
from .quality_targets import SEAM_P95_PX, CONTROLLED_SEAM_P95_PX
from .provenance import stage_record
from .ref_mesh import OpticalMesh
from .session import Session, read_json, sha256_file
from .stage_b_scene import peak_rss_bytes


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


def map_points(model, coefficients, x, q, rows, camera, truth, mesh):
    """Actual source selected by the production central-column rule, then true hit."""
    x, q = np.asarray(x), np.asarray(q)
    points = np.full((len(x), 2), np.nan)
    material = np.full(len(x), -1, np.int16)
    selected_band = np.full(len(x), -1, np.int16)
    for start in range(0, len(x), 512):
        sl = slice(start, start+512)
        best = np.full(len(x[sl]), -np.inf)
        selected = np.zeros(len(x[sl]), MATCH)
        for band in range(len(model.sampler.segments)):
            table, valid, score = sources_at(model, coefficients, band, x[sl], q[sl])
            use = valid & (score > best)
            selected[use] = table[use]
            best[use] = score[use]
        valid = np.isfinite(best)
        if valid.any():
            points[sl][valid], material[sl][valid] = mesh_points(
                selected[valid], 'a', rows, camera, truth, mesh)
            selected_band[sl][valid] = selected['band_a'][valid]
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


def boundary_support(model, coefficients, grid):
    output = {}
    for name, (x, q) in boundary_coordinates(grid).items():
        supported = np.zeros(len(x), bool)
        for start in range(0, len(x), 512):
            sl = slice(start, start+512)
            for band in range(len(model.sampler.segments)):
                # Skip bands whose declared correction bounds cannot reach this x.
                lo, hi = model.sampler.band_x[band]
                margin = .03+.04*(model.radius+model.height)
                if np.max(x[sl]) < lo-margin or np.min(x[sl]) > hi+margin:
                    continue
                supported[sl] |= sources_at(model, coefficients, band, x[sl], q[sl])[1]
            model.sampler.native.release()
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


def run(session_root, unroll, trajectory, output, scene=None, spacing_m=.2, samples_across=9, raw_root=None):
    started = time.monotonic()
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
        truth, config = session.truth(), session.config()
        generation = yaml.safe_load((session.root/'evaluation/config_source.yaml').read_text())
        track = generation.get('truth', {}).get('track_irregularity', {})
        imposed_track = any(float(track.get(k, 0)) > 0 for k in ('chord10_max_m', 'cross_level_tier_m'))
        rows = session.evaluation('row_truth')
        mesh = OpticalMesh.from_session(session, scene)
        grid = upstream['grid']; pitch = [grid['dx_m'], grid['dq_m']]
        windows = plan_windows(sampler, grid, spacing_m, 64, 1024, MatchSettings(max_shift_mm=1.))
        planned = [w for w in windows if w['status'] == 'planned']
        if not planned:
            raise ValueError('no supported adjacent overlap to evaluate')
        per_window, samples = [], []
        for window in planned:
            x = np.linspace(window['x_first_m'], window['x_first_m']+(window['shape'][1]-1)*grid['dx_m'], samples_across)
            q = np.full(len(x), window['q_center_m'])
            record = dict(window=window['id'], bands=window['bands'], q_m=float(q[0]), x_m=[float(x[0]), float(x[-1])])
            for name, c in (('nominal', np.zeros(model.size)), ('optimized', coefficients)):
                tables = [sources_at(model, c, band, x, q) for band in window['bands']]
                valid = tables[0][1] & tables[1][1]
                if not valid.any():
                    record[name] = dict(status='unmeasurable', missing_samples=len(x))
                    continue
                a, ma = mesh_points(tables[0][0][valid], 'a', rows, config['camera'], truth, mesh)
                b, mb = mesh_points(tables[1][0][valid], 'a', rows, config['camera'], truth, mesh)
                delta = b-a
                record[name] = dict(status='measured', missing_samples=int((~valid).sum()),
                                    summary=residual_summary(delta, pitch))
                samples.append((name, window['id'], window['bands'], x[valid], q[valid], delta, ma, mb))
            per_window.append(record)
            sampler.native.release()
        statistics = {}
        for name in ('nominal', 'optimized'):
            values = [v[5] for v in samples if v[0] == name]
            if not values:
                raise ValueError('no independent seam samples: '+name)
            statistics[name] = residual_summary(np.concatenate(values), pitch)
            statistics[name]['missing_samples'] = sum(w[name]['missing_samples'] for w in per_window)
        # Deterministic interior grid, including both axial and angular endpoints.
        angles, all_x = grid_axes(grid)
        xs = all_x[np.unique(np.rint(np.linspace(0, len(all_x)-1, 33)).astype(int))]
        qs = angles[np.unique(np.rint(np.linspace(0, len(angles)-1, 49)).astype(int))]*model.radius
        xx, qq = np.meshgrid(xs, qs)
        maps = {}
        boundaries = {}
        arrays = {}
        for name, c in (('nominal', np.zeros(model.size)), ('optimized', coefficients)):
            points, materials, bands = map_points(model, c, xx.ravel(), qq.ravel(), rows, config['camera'], truth, mesh)
            valid = bands >= 0
            maps[name] = dict(supported_samples=int(valid.sum()), missing_samples=int((~valid).sum()),
                              absolute_error=residual_summary(points[valid]-np.column_stack((xx.ravel(), qq.ravel()))[valid], pitch),
                              drift=endpoint_drift(points.reshape(xx.shape+(2,)), valid.reshape(xx.shape), xs, qs))
            arrays[name+'_true_xq_m'] = points.reshape(xx.shape+(2,))
            arrays[name+'_band'] = bands.reshape(xx.shape)
            arrays[name+'_material'] = materials.reshape(xx.shape)
            boundaries[name] = boundary_support(model, c, grid)
        seam_gate = dict(threshold_p95_px=SEAM_P95_PX, status='pass' if statistics['optimized']['norm_px']['p95'] <= SEAM_P95_PX else 'fail',
                         scope='true optical-mesh separation of adjacent samples at the same output coordinate')
        controlled_gate = dict(threshold_p95_px=CONTROLLED_SEAM_P95_PX, status='pass' if statistics['optimized']['norm_px']['p95'] <= CONTROLLED_SEAM_P95_PX else 'fail',
                               scope='DESIGN controlled-error seam target; does not waive coverage or weak-region checks')
        perimeter_missing = sum(v['missing_pixels'] for v in boundaries['optimized'].values())
        report = dict(schema='ssb.global_geometry_evaluation.v1', evaluation_only=True, grid=grid,
            seam=statistics, windows=per_window, mapping=maps, boundary=boundaries,
            gates=dict(strict_seam=seam_gate, controlled_seam=controlled_gate,
                       perimeter=dict(status='fail' if perimeter_missing else 'pass',
                       missing_pixels=perimeter_missing, scope='exact perimeter pixel support; not full interior coverage')),
            capture_conditions=dict(imposed_track_irregularity=imposed_track, archived_track_parameters=track,
                interpretation='1 px is the project seam target; 3 px is the secondary controlled-error target. '
                               'A perturbed track must not be labelled a perfectly flat ideal baseline.'),
            sampling=dict(spacing_q_m=spacing_m, samples_across=samples_across, planned_windows=len(planned),
                          unmeasurable_windows=len(windows)-len(planned), interior_shape=list(xx.shape)),
            reference='archived float64 optical mesh and true native pixel-centre rays',
            limitations=['fixed seam/interior samples, not exhaustive surface accuracy or defect-width acceptance',
                'native centre-ray barycentre reference; exposure/pixel-area integration is not a point measurement',
                'absolute error retains coordinate gauge; endpoint differences remove translation only',
                'no truth offset, similarity or affine fit is subtracted from the seam or scale scores'],
            performance=dict(wall_s=time.monotonic()-started, peak_rss_bytes=peak_rss_bytes()))
        output.mkdir(parents=True, exist_ok=False)
        arrays.update(output_x_m=xs, output_q_m=qs)
        np.savez(output/'mapping_samples.npz', **arrays)
        dtype = [('variant','U10'), ('window','<i4'), ('band_a','<i2'), ('band_b','<i2'),
                 ('x_m','<f8'), ('q_m','<f8'), ('dx_m','<f8'), ('dq_m','<f8'), ('material_a','<i2'), ('material_b','<i2')]
        records = []
        for name, identifier, bands, x, q, delta, ma, mb in samples:
            records.extend(zip([name]*len(x), [identifier]*len(x), [bands[0]]*len(x), [bands[1]]*len(x),
                               x, q, delta[:,0], delta[:,1], ma, mb))
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
    args = parser.parse_args()
    report = run(args.session,args.unroll,args.trajectory,args.output,args.scene,raw_root=args.raw)
    print(json.dumps({k:report[k] for k in ('seam','mapping','boundary','gates','performance')}))


if __name__ == '__main__':
    main()
