"""Independent truth-side scoring, never imported by reconstruction.

Each match side is traced back to the true rays of its native source pixels. Two
references are reported: the analytic cylinder, and the archived float64 optical mesh
(panels, chamfers, groove walls and recessed filler). Signed B-A displacements expose
common biases that the D2 affine translation absorbs. Neither is optimized seam accuracy.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from .session import Session, read_json, sha256_file
from .ref_geometry import head_pose, evaluation_pixel_tangents
from .provenance import stage_record
from .public_capture import confined_file
from .ref_mesh import OpticalMesh


def paired_hits(origin, optical, line, tangents, radius, axis_z):
    direction = optical+tangents[:, None]*line
    y, z = origin[:, 1], origin[:, 2]-axis_z
    dy, dz = direction[:, 1], direction[:, 2]
    a = dy*dy+dz*dz; b = 2*(y*dy+z*dz); c = y*y+z*z-radius*radius
    distance = (-b+np.sqrt(b*b-4*a*c))/(2*a)
    return np.column_stack([origin[:, 0]+distance*direction[:, 0],
                           radius*np.arctan2(y+distance*dy, z+distance*dz)])


def native_rays(table, side, rows, camera, truth):
    """Centre rays of the four native samples behind each match side, with their weights.

    Returns origins/directions (n, 4, 3) and weights (n, 4): lower/upper exposure row x
    left/right native column. The renderer integrates each pixel over its area and the
    exposure; these centre rays are the conventional point reference for that average.
    """
    n = len(table); origins = np.zeros((n, 4, 3)); directions = np.zeros((n, 4, 3)); weights = np.zeros((n, 4))
    angular = table[side+'_angular_weight']
    for r, (name, row_weight) in enumerate([('lower', 1-angular), ('upper', angular)]):
        ids = table[side+'_'+name+'_sequence']
        if np.any(ids < 0) or np.any(ids >= len(rows)) or not np.array_equal(rows['sequence'][ids], ids):
            raise ValueError('match source does not identify an evaluation exposure')
        selected = rows[ids]
        origin, optical, line = head_pose(selected['theta'], selected['x'], truth, selected)
        column = table[side+'_'+name+'_column']
        if not np.isfinite(column).all() or np.any(column < 0) or np.any(column > camera['width']-1):
            raise ValueError('invalid native evaluation column')
        left = np.floor(column).astype(int); alpha = column-left
        for c, (columns, column_weight) in enumerate([(left, 1-alpha), (np.minimum(left+1, camera['width']-1), alpha)]):
            tangents = evaluation_pixel_tangents(camera, truth, columns)
            k = 2*r+c
            origins[:, k] = origin; directions[:, k] = optical+tangents[:, None]*line
            weights[:, k] = row_weight*column_weight
    return origins, directions, weights


def world_points(table, side, rows, camera, truth):
    """Analytic-cylinder reference point (x, q) of one match side."""
    origins, directions, weights = native_rays(table, side, rows, camera, truth)
    radius, axis_z = truth['tunnel']['radius_m'], truth['tunnel']['axis_z_m']
    result = np.zeros((len(table), 2))
    for k in range(4):
        hit = paired_hits(origins[:, k], directions[:, k], np.zeros_like(directions[:, k]),
                          np.zeros(len(table)), radius, axis_z)
        result += weights[:, k, None]*hit
    return result


def mesh_points(table, side, rows, camera, truth, mesh):
    """Optical-mesh reference point (x, q) and dominant hit material of one match side."""
    origins, directions, weights = native_rays(table, side, rows, camera, truth)
    points, _, material = mesh.intersect(origins.reshape(-1, 3), directions.reshape(-1, 3))
    radius, axis_z = truth['tunnel']['radius_m'], truth['tunnel']['axis_z_m']
    xq = np.column_stack([points[:, 0], radius*np.arctan2(points[:, 1], points[:, 2]-axis_z)]).reshape(-1, 4, 2)
    dominant = material.reshape(-1, 4)[np.arange(len(table)), np.argmax(weights, axis=1)]
    return np.einsum('nk,nkd->nd', weights, xq), dominant


def signed_summary(delta, table, pitch):
    """Bias and spread of B-A reference displacements, in output pixels."""
    d = delta/pitch
    windows = np.unique(table['window'])
    means = np.array([d[table['window'] == w].mean(0) for w in windows])
    counts = np.array([np.count_nonzero(table['window'] == w) for w in windows])
    pairs = {int(b): d[table['band_a'] == b].mean(0).tolist() for b in np.unique(table['band_a'])}
    column = table['a_lower_column']; edges = np.linspace(0, column.max()+1e-9, 9)
    by_column = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        selected = (column >= lo) & (column < hi)
        if selected.any():
            by_column.append(dict(a_column=[float(lo), float(hi)], matches=int(selected.sum()),
                                  mean_dx_px=float(d[selected, 0].mean()), mean_dq_px=float(d[selected, 1].mean())))
    norm = np.hypot(*d.T)
    return dict(norm_px=dict(zip(('p50', 'p95', 'p99', 'max'), map(float, np.percentile(norm, [50, 95, 99, 100])))),
        mean_px=dict(dx=float(d[:, 0].mean()), dq=float(d[:, 1].mean())),
        std_px=dict(dx=float(d[:, 0].std()), dq=float(d[:, 1].std())),
        window_mean_px=dict(dx=dict(zip(('min', 'p50', 'max'), map(float, np.percentile(means[:, 0], [0, 50, 100])))),
                            dq=dict(zip(('min', 'p50', 'max'), map(float, np.percentile(means[:, 1], [0, 50, 100]))))),
        window_mean_standard_error_px=float(np.median(np.hypot(*d.std(0))/np.sqrt(counts))),
        adjacent_pair_mean_px={str(k): v for k, v in pairs.items()},
        by_a_native_column=by_column)


def diagnostic_indices(table, per_window):
    """Fixed native-column quantiles chosen before loading any truth."""
    if type(per_window) is not int or not 2 <= per_window <= 64:
        raise ValueError('diagnostic sample budget must be an integer in [2,64]')
    selected = []
    for window in np.unique(table['window']):
        ids = np.flatnonzero((table['window'] == window) & table['inlier'].astype(bool))
        if len(ids):
            ids = ids[np.argsort(table['a_lower_column'][ids], kind='stable')]
            ranks = np.linspace(0, len(ids)-1, min(per_window, len(ids))).round().astype(int)
            selected.extend(ids[ranks].tolist())
    if not selected:
        raise ValueError('no inlier windows for diagnostic sampling')
    return np.asarray(selected, np.int64)


def run(session_root, matches_root, output, mesh_reference=True, scene=None, diagnostic_per_window=None):
    started = time.monotonic(); session = Session(session_root)
    output = Path(output).resolve()
    if 'evaluation' not in output.parts: raise ValueError('truth-side outputs must be inside evaluation/')
    matches_root = Path(matches_root).resolve()
    table_path = confined_file(matches_root, 'matches.npy')
    provenance_path = confined_file(matches_root, 'provenance.json')
    upstream = read_json(provenance_path)
    if upstream.get('stage') != 'band_matching': raise ValueError('D2 provenance required')
    for name in ('matches.npy', 'report.json'):
        identity = [v for k, v in upstream['outputs'].items() if Path(k).name == name]
        if len(identity) != 1 or identity[0] != sha256_file(matches_root/name):
            raise ValueError('match identity mismatch: '+name)
    match_report = read_json(matches_root/'report.json')
    config = session.config()
    if match_report['optical_signature'] != config['camera']['optical_signature']:
        raise ValueError('evaluation optical identity mismatch')
    for name in ('config/observable_config.json', 'metadata/manifest.json', 'raw/index.json'):
        if match_report.get('source_observation_hashes', {}).get(name) != sha256_file(session.root/name):
            raise ValueError('evaluation session is not the source public observation: '+name)
    truth_path = session.root/'evaluation/truth.json'
    if session.summary['files'].get('evaluation/truth.json') != sha256_file(truth_path):
        raise ValueError('evaluation truth identity mismatch')
    table = np.load(table_path)
    diagnostic = None
    if diagnostic_per_window is not None:
        ids = diagnostic_indices(table, diagnostic_per_window)
        output.mkdir(parents=True, exist_ok=False)
        # Persist the public selection BEFORE opening row truth or tracing rays.
        np.save(output/'diagnostic_match_indices.npy', ids)
        diagnostic = dict(per_window=diagnostic_per_window, full_inliers=int(np.count_nonzero(table['inlier'])),
            selected=len(ids), rule='native-column quantiles of each inlier window; selected before truth',
            scope='bounded bias diagnosis, not full-match or independent acceptance')
        table = table[ids]
    else:
        table = table[table['inlier'].astype(bool)]
    if not len(table): raise ValueError('no image-derived inlier matches to evaluate')
    rows = session.evaluation('row_truth'); truth = session.truth()
    a = world_points(table, 'a', rows, config['camera'], truth)
    b = world_points(table, 'b', rows, config['camera'], truth)
    error = np.linalg.norm(a-b, axis=1)
    if not np.isfinite(error).all(): raise ValueError('nonfinite independent geometry score')
    pitch = match_report['upstream_grid']['requested_pitch_m']
    nominal = np.hypot(table['x_b_m']-table['x_a_m'], table['q_b_m']-table['q_a_m'])
    fields = [('window', '<i4'), ('band_a', '<i2'), ('error_m', '<f8'), ('nominal_displacement_m', '<f8'),
              ('cylinder_dx_m', '<f8'), ('cylinder_dq_m', '<f8')]
    if mesh_reference:
        fields += [('mesh_dx_m', '<f8'), ('mesh_dq_m', '<f8'), ('material_a', '<i2'), ('material_b', '<i2')]
    scores = np.zeros(len(table), dtype=fields)
    scores['window'], scores['band_a'] = table['window'], table['band_a']
    scores['error_m'], scores['nominal_displacement_m'] = error, nominal
    scores['cylinder_dx_m'], scores['cylinder_dq_m'] = (b-a).T
    references = dict(cylinder=signed_summary(b-a, table, pitch))
    mesh_info = None; mesh_sources = []
    if mesh_reference:
        mesh_start = time.monotonic()
        mesh = OpticalMesh.from_session(session, scene); mesh_sources = mesh.sources
        ma, material_a = mesh_points(table, 'a', rows, config['camera'], truth, mesh)
        mb, material_b = mesh_points(table, 'b', rows, config['camera'], truth, mesh)
        scores['mesh_dx_m'], scores['mesh_dq_m'] = (mb-ma).T
        scores['material_a'], scores['material_b'] = material_a, material_b
        references['optical_mesh'] = signed_summary(mb-ma, table, pitch)
        same = material_a == material_b
        references['optical_mesh']['by_material'] = {str(int(m)): dict(matches=int(np.count_nonzero(same & (material_a == m))),
            mean_dx_px=float(((mb-ma)[same & (material_a == m), 0]/pitch).mean()))
            for m in np.unique(material_a[same])}
        references['optical_mesh']['mixed_material_matches'] = int(np.count_nonzero(~same))
        mesh_info = dict(triangles=len(mesh.faces), rays=8*len(table), seconds=time.monotonic()-mesh_start,
                         index='0.25 degree angular strips with axial bounds; float64 source triangles')
    report = dict(schema='ssb.band_matches_evaluation.v2', evaluation_only=True, matches=len(table),
        cylindrical_reference_error_px=references['cylinder']['norm_px'],
        nominal_displacement_px=dict(zip(('p50', 'p95'), map(float, np.percentile(nominal/pitch, [50, 95])))),
        signed_b_minus_a=references, mesh_reference=mesh_info,
        sign_convention='B-A in nominal (x, q=R*theta); +dx means B sees the point further along +x',
        assumption='centre rays of native pixels at exposure-centre truth poses; the renderer integrates pixel area and exposure',
        limitation='reference correspondence error, not optimized seam accuracy; never fed back to matching',
        wall_s=time.monotonic()-started)
    if diagnostic is None:
        output.mkdir(parents=True, exist_ok=False)
    else:
        report['diagnostic_sampling'] = diagnostic
    np.save(output/'reference_scores.npy', scores)
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    manifest = read_json(session.root/'evaluation/manifest.json')
    inputs = [table_path, provenance_path, matches_root/'report.json', truth_path,
              session.root/'evaluation/manifest.json', session.root/'evaluation'/manifest['row_truth']['file'],
              session.root/'config/observable_config.json', session.root/'config/backend.json',
              session.root/'evaluation/config_source.yaml', *mesh_sources]
    record = stage_record('evaluate_band_matching', inputs, sorted(output.iterdir()), dict(assumption=report['assumption'], mesh_reference=mesh_reference))
    (output/'provenance.json').write_text(json.dumps(record, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('session', 'matches', 'output'): parser.add_argument('--'+name, required=True)
    parser.add_argument('--no-mesh', action='store_true', help='cylinder reference only')
    parser.add_argument('--scene', help='relocated optical scene; must hash-match the capture record')
    parser.add_argument('--diagnostic-per-window', type=int,
                        help='bounded bias diagnosis only; save public column-quantile plan before truth')
    args = parser.parse_args()
    report = run(args.session, args.matches, args.output, not args.no_mesh, args.scene,
                 args.diagnostic_per_window)
    print(json.dumps({k: report[k] for k in ('matches', 'cylindrical_reference_error_px', 'mesh_reference')}))
    for name, summary in report['signed_b_minus_a'].items():
        print(name, json.dumps(dict(mean=summary['mean_px'], std=summary['std_px'], window_mean=summary['window_mean_px'])))


if __name__ == '__main__': main()
