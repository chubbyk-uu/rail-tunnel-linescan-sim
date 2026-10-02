"""Independent truth-side scoring, never imported by reconstruction.

This uses analytic cylinder intersections, not the grooved optical mesh; its
scores are explicitly a cylindrical reference and not final seam accuracy.
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


def paired_hits(origin, optical, line, tangents, radius, axis_z):
    direction = optical+tangents[:, None]*line
    y, z = origin[:, 1], origin[:, 2]-axis_z
    dy, dz = direction[:, 1], direction[:, 2]
    a = dy*dy+dz*dz; b = 2*(y*dy+z*dz); c = y*y+z*z-radius*radius
    distance = (-b+np.sqrt(b*b-4*a*c))/(2*a)
    return np.column_stack([origin[:, 0]+distance*direction[:, 0],
                           radius*np.arctan2(y+distance*dy, z+distance*dz)])


def world_points(table, side, rows, camera, truth):
    result = np.zeros((len(table), 2))
    angular = table[side+'_angular_weight']
    for name, row_weight in [('lower', 1-angular), ('upper', angular)]:
        ids = table[side+'_'+name+'_sequence']
        if np.any(ids < 0) or np.any(ids >= len(rows)) or not np.array_equal(rows['sequence'][ids], ids):
            raise ValueError('match source does not identify an evaluation exposure')
        selected = rows[ids]
        origin, optical, line = head_pose(selected['theta'], selected['x'], truth, selected)
        column = table[side+'_'+name+'_column']
        if not np.isfinite(column).all() or np.any(column < 0) or np.any(column > camera['width']-1):
            raise ValueError('invalid native evaluation column')
        left = np.floor(column).astype(int); alpha = column-left
        for columns, column_weight in [(left, 1-alpha), (np.minimum(left+1, camera['width']-1), alpha)]:
            tangents = evaluation_pixel_tangents(camera, truth, columns)
            hit = paired_hits(origin, optical, line, tangents,
                              truth['tunnel']['radius_m'], truth['tunnel']['axis_z_m'])
            result += (row_weight*column_weight)[:, None]*hit
    return result


def run(session_root, matches_root, output):
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
    table = np.load(table_path); table = table[table['inlier'].astype(bool)]
    if not len(table): raise ValueError('no image-derived inlier matches to evaluate')
    rows = session.evaluation('row_truth'); truth = session.truth()
    a = world_points(table, 'a', rows, config['camera'], truth)
    b = world_points(table, 'b', rows, config['camera'], truth)
    error = np.linalg.norm(a-b, axis=1)
    if not np.isfinite(error).all(): raise ValueError('nonfinite independent geometry score')
    pitch = match_report['upstream_grid']['requested_pitch_m']
    nominal = np.hypot(table['x_b_m']-table['x_a_m'], table['q_b_m']-table['q_a_m'])
    scores = np.zeros(len(table), dtype=[('window', '<i4'), ('error_m', '<f8'), ('nominal_displacement_m', '<f8')])
    scores['window'], scores['error_m'], scores['nominal_displacement_m'] = table['window'], error, nominal
    report = dict(schema='ssb.band_matches_evaluation.v1', evaluation_only=True, matches=len(table),
        cylindrical_reference_error_px=dict(zip(('p50', 'p95', 'p99', 'max'), map(float, np.percentile(error/pitch, [50, 95, 99, 100])))),
        nominal_displacement_px=dict(zip(('p50', 'p95'), map(float, np.percentile(nominal/pitch, [50, 95])))),
        assumption='analytic cylinder at archived radius; filled-joint recess and optical mesh faceting are not included',
        limitation='not an exact optical-mesh ground truth and not optimized seam accuracy',
        wall_s=time.monotonic()-started)
    output.mkdir(parents=True, exist_ok=False)
    np.save(output/'cylindrical_scores.npy', scores)
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    manifest = read_json(session.root/'evaluation/manifest.json')
    inputs = [table_path, provenance_path, matches_root/'report.json', truth_path,
              session.root/'evaluation/manifest.json', session.root/'evaluation'/manifest['row_truth']['file'],
              session.root/'config/observable_config.json']
    record = stage_record('evaluate_band_matching', inputs, sorted(output.iterdir()), dict(assumption=report['assumption']))
    (output/'provenance.json').write_text(json.dumps(record, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('session', 'matches', 'output'): parser.add_argument('--'+name, required=True)
    args = parser.parse_args(); print(json.dumps(run(args.session, args.matches, args.output)))


if __name__ == '__main__': main()
