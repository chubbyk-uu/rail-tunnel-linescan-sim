"""Stage A acceptance for one session (DESIGN.md §13 stage A, §12.2).

Every check reports pass, fail or unmeasurable; the overall result is pass only when
all checks pass. Writes evaluation/reports/stage_a.json with stage provenance.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

from . import ref_geometry, ref_timing, unroll
from .provenance import stage_record
from .session import Session, read_json, sha256_file

PASS, FAIL, UNMEASURABLE = 'pass', 'fail', 'unmeasurable'
TIME_TOL = 1e-9  # s, row times (never near zero speed: rows need a period estimate)
POSITION_TOL = 1e-12  # rad, edge crossings


def check(name, state, **detail):
    return {'name': name, 'state': state, **detail}


def verify_hashes(s):
    bad, n = [], 0
    for area in ('metadata', 'evaluation'):
        for key, entry in read_json(s.root / area / 'manifest.json').items():
            n += 1
            if sha256_file(s.root / area / entry['file']) != entry['sha256']:
                bad.append(f'{area}/{entry["file"]}')
    for block in s.raw_index()['blocks']:
        n += 1
        if sha256_file(s.root / 'raw' / block['file']) != block['sha256']:
            bad.append('raw/' + block['file'])
    return n, bad


def compare_timing(s, cfg, truth, poses):
    ref = ref_timing.reference(poses, cfg, truth)
    out = []

    def edges(name, got, want, value, rate):
        # Crossings are defined in position; near zero speed a 1e-16 rad difference is
        # microseconds, so agreement is judged by the position at each implementation's time.
        t, c, d = want
        if len(got) != len(t) or not len(t):
            return check(name, FAIL, recorded=len(got), reference=len(t))
        same = np.array_equal(got['count'], c) and np.array_equal(got['dir'], d)
        p_got, _ = ref_timing.hermite_value(poses, got['t'], value, rate)
        p_ref, _ = ref_timing.hermite_value(poses, t, value, rate)
        dp = float(np.max(np.abs(p_got - p_ref)))
        dt = float(np.max(np.abs(got['t'] - t)))
        return check(name, PASS if same and dp <= POSITION_TOL else FAIL, count=len(t),
                     max_position_difference_rad=dp, threshold_rad=POSITION_TOL, max_time_difference_s=dt,
                     counts_and_directions_equal=bool(same))

    out.append(edges('scan_edges_match_reference', s.metadata('scan_edges'), ref['scan_edges'], 'theta', 'omega'))
    out.append(edges('odometer_edges_match_reference', s.metadata('odometer_edges'), ref['odometer_edges'],
                     'wheel', 'wheel_omega'))

    gates = s.metadata('gate_events')
    want = ref['gates']
    same = len(gates) == len(want) and all(
        g['revolution'] == w[1] and g['kind'] == w[2] and g['dir'] == w[3] and abs(g['t'] - w[0]) <= TIME_TOL
        for g, w in zip(gates, want))
    out.append(check('gate_events_match_reference', PASS if same and len(want) else FAIL,
                     recorded=len(gates), reference=len(want)))

    rows = s.metadata('rows')
    want = ref['rows']
    if len(rows) != len(want) or not len(want):
        out.append(check('rows_match_reference', FAIL, recorded=len(rows), reference=len(want)))
    else:
        w = np.array([(r[0], r[1]) for r in want], np.int64)
        wt = np.array([(r[2], r[3]) for r in want])
        same = np.array_equal(rows['row'], w[:, 0]) and np.array_equal(rows['segment'], w[:, 1]) \
            and np.array_equal(rows['sequence'], np.arange(len(rows)))
        dt = float(max(np.max(np.abs(rows['t_trigger'] - wt[:, 0])), np.max(np.abs(rows['t_center'] - wt[:, 1]))))
        out.append(check('rows_match_reference', PASS if same and dt <= TIME_TOL else FAIL, rows=len(rows),
                         max_time_difference_s=dt, indices_equal=bool(same)))

    dropped = s.metadata('dropped_rows')
    got = {int(d['row']): (int(d['reason']), int(d['gated'])) for d in dropped}
    same = got == ref['dropped'] and len(got) == len(dropped)
    reasons = {int(r): int((dropped['reason'] == r).sum()) for r in np.unique(dropped['reason'])}
    out.append(check('dropped_rows_match_reference', PASS if same else FAIL, recorded=len(dropped),
                     reference=len(ref['dropped']), by_reason=reasons))
    return out, rows, dropped


def accounting(rows, dropped):
    """Inside every segment, recorded rows plus gated drops are contiguous and unique."""
    ids = np.concatenate([rows['row'], dropped['row']])
    unique = len(np.unique(ids)) == len(ids)
    monotonic = bool(np.all(np.diff(rows['t_trigger']) > 0) and np.all(np.diff(rows['row']) > 0))
    gated = set(dropped['row'][dropped['gated'] == 1].tolist())
    lost = 0
    for k in np.unique(rows['segment']):
        r = rows['row'][rows['segment'] == k]
        have = set(r.tolist())
        lost += sum(1 for i in range(int(r[0]), int(r[-1]) + 1) if i not in have and i not in gated)
    gated_drops = int((dropped['gated'] == 1).sum())
    ok = unique and monotonic and lost == 0 and len(rows) > 0
    return check('event_accounting_complete', PASS if ok else FAIL, unique=bool(unique),
                 time_and_index_monotonic=monotonic, unaccounted_rows_in_segments=lost,
                 gated_drops_reported=gated_drops,
                 note='every lattice row inside a gate is recorded or listed as dropped; '
                      'says nothing about whether drops are acceptable')


def valid_region(source_config, poses, rows, row_truth, dropped):
    """No exposure may be missing where the head is inside acceptance.valid_x_m.

    Dropped rows are located by the true head x at both ends of their time bracket;
    a drop touching the region counts. Drops outside it are start/stop buffer and are
    reported, not judged.
    """
    region = (source_config.get('acceptance') or {}).get('valid_x_m')
    if not region:
        return check('no_missing_rows_in_valid_region', UNMEASURABLE, reason='acceptance.valid_x_m not configured')
    try:
        x0, x1 = (float(v) for v in region)
    except (TypeError, ValueError):
        return check('no_missing_rows_in_valid_region', FAIL, reason='valid_x_m must be two numbers', valid_x_m=region)
    if not (math.isfinite(x0) and math.isfinite(x1) and x0 < x1):
        return check('no_missing_rows_in_valid_region', FAIL, reason='valid_x_m must be finite and increasing',
                     valid_x_m=region)
    if not len(row_truth) or row_truth['x'].min() > x0 or row_truth['x'].max() < x1:
        return check('no_missing_rows_in_valid_region', FAIL, reason='recorded rows do not span the valid region',
                     valid_x_m=region, recorded_x_m=[float(row_truth['x'].min()), float(row_truth['x'].max())]
                     if len(row_truth) else None)
    gated = dropped[dropped['gated'] == 1]
    t_end = poses['t'][-1]
    lo, _ = ref_timing.hermite_value(poses, np.minimum(gated['t_lo'], t_end), 'x', 'v')
    hi, _ = ref_timing.hermite_value(poses, np.minimum(gated['t_hi'], t_end), 'x', 'v')
    inside = (np.maximum(lo, hi) >= x0) & (np.minimum(lo, hi) <= x1)
    reasons = {int(r): int((gated['reason'][inside] == r).sum()) for r in np.unique(gated['reason'][inside])}
    buffer = {int(r): int((gated['reason'][~inside] == r).sum()) for r in np.unique(gated['reason'][~inside])}
    in_region = int(((row_truth['x'] >= x0) & (row_truth['x'] <= x1)).sum())
    if in_region == 0 and not inside.any():
        # Nothing was exposed there (e.g. a band inside one bottom-arc advance): no
        # missing rows, but also no evidence; never a pass.
        return check('no_missing_rows_in_valid_region', UNMEASURABLE, reason='no exposures inside the valid region',
                     valid_x_m=region, rows_in_region=0)
    return check('no_missing_rows_in_valid_region', PASS if not inside.any() else FAIL, valid_x_m=region,
                 rows_in_region=in_region, missing_in_region=int(inside.sum()), missing_by_reason=reasons,
                 buffer_drops_outside_region=int((~inside).sum()), buffer_drops_by_reason=buffer)


def gate_geometry(cfg, truth, rows, row_truth):
    """Every recorded row's true exposure-centre angle lies in its segment's gate."""
    start = cfg['gate']['start_rad'] + truth['gate_start_offset_rad'] + 2 * math.pi * rows['segment']
    end = cfg['gate']['end_rad'] + truth['gate_end_offset_rad'] + 2 * math.pi * rows['segment']
    th = row_truth['theta']
    inside = (th >= start - 1e-12) & (th < end + 1e-12)
    return check('rows_inside_their_gate', PASS if inside.all() and len(th) else FAIL,
                 rows_outside=int((~inside).sum()))


def advance_per_rev(cfg, row_truth, poses=None):
    if cfg.get("contact",{}).get("enabled"):
        if poses is None or len(poses)<2: return check("encoder_advance_per_revolution",UNMEASURABLE)
        a,b=row_truth[0],row_truth[-1]
        l=np.interp([a["t_center"],b["t_center"]],poses["t"],poses["wheel"])
        r=np.interp([a["t_center"],b["t_center"]],poses["t"],poses["right_wheel"])
        cal=cfg["calibration"];distance=((l[1]-l[0])*cal["odo_left_diameter_m"]+(r[1]-r[0])*cal["odo_right_diameter_m"])/4
        turns=(b["theta"]-a["theta"])/(2*np.pi);advance=distance/turns
        relative=abs(advance/cfg["motion"]["advance_per_rev_m"]-1)
        return check("encoder_advance_per_revolution",PASS if relative<.001 else FAIL,estimated_m=advance,actual_m=(b["x"]-a["x"])/turns,relative_error=relative,threshold=.001,note="servo follows estimated distance; actual pitch may differ")
    dth = row_truth['theta'][-1] - row_truth['theta'][0]
    if dth < math.pi:
        return check('advance_per_revolution', UNMEASURABLE, reason='less than half a revolution recorded')
    adv = (row_truth['x'][-1] - row_truth['x'][0]) / dth * 2 * math.pi
    rel = abs(adv / cfg['motion']['advance_per_rev_m'] - 1)
    return check('advance_per_revolution', PASS if rel <= 1e-6 else FAIL, measured_m=adv,
                 target_m=cfg['motion']['advance_per_rev_m'], relative_error=rel, threshold=1e-6,
                 note='true car travel per true scan revolution over the recorded span')


def gpu_vs_cpu(s, cfg, truth, row_truth):
    hits = s.evaluation('debug_hits')
    columns = read_json(s.root / 'evaluation' / 'manifest.json')['debug_hits'].get('columns', [])
    if not columns:
        return [check('gpu_hits_match_cpu', UNMEASURABLE, reason='no debug columns archived')], None
    origin, optical, line = ref_geometry.head_pose(row_truth['theta'], row_truth['x'], truth)
    x, q = ref_geometry.wall_hits(origin, optical, line, ref_geometry.pixel_tangents(cfg['camera'], columns), truth)
    err = np.maximum(np.abs(hits['hits'][..., 0] - x), np.abs(hits['hits'][..., 1] - q))
    worst = float(np.nanmax(err))
    finite = bool(np.isfinite(hits['hits']).all())
    return [check('gpu_hits_match_cpu', PASS if finite and worst < 1e-5 else FAIL, max_error_m=worst,
                  threshold_m=1e-5, rows=len(hits), columns=len(columns))], (np.asarray(columns), x, q)


def pixel_spot_check(s, cfg, truth, row_truth, stride=97):
    seq = np.arange(0, len(row_truth), stride)
    origin, optical, line = ref_geometry.head_pose(row_truth['theta'][seq], row_truth['x'][seq], truth)
    x, q = ref_geometry.wall_hits(origin, optical, line, ref_geometry.pixel_tangents(cfg['camera']), truth)
    expect = ref_geometry.albedo_code(ref_geometry.wall_albedo(x, q))
    got = s.raw_rows(row_truth['sequence'][seq])
    frac = float((got != expect).mean())
    # Float ray directions on the GPU can move a hit across a 1 mm texture cell edge.
    return check('pixels_match_cpu_texture', PASS if frac <= 1e-3 else FAIL, rows_checked=len(seq),
                 differing_fraction=frac, threshold=1e-3)


def ideal_unroll(s, cfg, rows, truth_hits):
    if truth_hits is None:
        return check('ideal_unroll_matches_truth', UNMEASURABLE, reason='no debug columns archived')
    columns, xt, qt = truth_hits
    x, q, theta = unroll.row_coordinates(cfg, rows, s.metadata('scan_edges'), s.metadata('odometer_edges'),
                                         s.metadata('gate_events'), columns)
    gx = cfg['camera']['fov_at_nominal_m'] / cfg['camera']['width']
    R = cfg['calibration']['radius_m']
    dq = unroll.wrap_angle(q / R - qt / R) * R
    ex, eq = np.abs(x - xt) / gx, np.abs(dq) / gx
    worst = float(max(ex.max(), eq.max()))
    p95 = float(max(np.percentile(ex, 95), np.percentile(eq, 95)))
    ok = p95 <= 0.01 and worst <= 0.05
    return check('ideal_unroll_matches_truth', PASS if ok else FAIL, p95_px=p95, max_px=worst,
                 max_x_px=float(ex.max()), max_q_px=float(eq.max()), thresholds={'p95_px': 0.01, 'max_px': 0.05},
                 note='implementation check with nominal mount; output pixel = fov/width')


def planned_motion(summary):
    motion = summary.get('motion') or {}
    done = motion.get('complete')
    if done is None:
        return check('planned_motion_complete', UNMEASURABLE, reason='pose source has no planned motion', **motion)
    return check('planned_motion_complete', PASS if done else FAIL, **motion)


def observable_from_source(src):
    """Fields of config/observable_config.json re-derived from the source YAML."""
    rad = math.pi / 180
    cam = src['camera']
    return {
        ('camera', 'width'): cam['width'], ('camera', 'pixel_pitch_m'): cam['pixel_pitch_m'],
        ('camera', 'fov_at_nominal_m'): cam['fov_at_nominal_m'],
        ('camera', 'nominal_distance_m'): cam['nominal_distance_m'], ('camera', 'exposure_s'): cam['exposure_s'],
        ('camera', 'trigger_delay_s'): cam['trigger_delay_s'], ('camera', 'max_line_rate_hz'): cam['max_line_rate_hz'],
        ('rescaler', 'multiply'): src['rescaler']['multiply'], ('rescaler', 'divide'): src['rescaler']['divide'],
        ('rescaler', 'max_period_s'): src['rescaler']['max_period_s'],
        ('scan_encoder', 'ppr'): src['scan_encoder']['ppr'],
        ('scan_encoder', 'edges_per_cycle'): src['scan_encoder']['edges_per_cycle'],
        ('odometer', 'ppr'): src['odometer']['ppr'], ('odometer', 'edges_per_cycle'): src['odometer']['edges_per_cycle'],
        ('odometer', 'gear_ratio'): src['odometer']['gear_ratio'],
        ('gate', 'start_rad'): src['gate']['start_deg'] * rad, ('gate', 'end_rad'): src['gate']['end_deg'] * rad,
        **({('calibration','odo_left_diameter_m'):src['calibration'].get('odo_left_diameter_m',src['calibration']['wheel_diameter_m']),
            ('calibration','odo_right_diameter_m'):src['calibration'].get('odo_right_diameter_m',src['calibration']['wheel_diameter_m'])} if src.get('contact',{}).get('enabled') else {}),
        ('calibration', 'wheel_diameter_m'): src['calibration']['wheel_diameter_m'],
        ('calibration', 'radius_m'): src['calibration']['radius_m'],
        ('calibration', 'head_mount_x_m'): src['calibration']['head_mount_x_m'],
        ('motion', 'line_rate_hz'): src['motion']['line_rate_hz'],
        ('motion', 'advance_per_rev_m'): src['motion']['advance_per_rev_m'],
        ('motion', 'start_x_m'): src['motion']['start_x_m'],
        ('motion', 'sample_period_s'): src['motion']['sample_period_s'],
        ('tunnel', 'x_min_m'): src['tunnel']['x_min_m'], ('tunnel', 'x_max_m'): src['tunnel']['x_max_m'],
    }


def truth_from_source(src, source_sha):
    """evaluation/truth.json as the render stage must have written it from the source YAML."""
    rad = math.pi / 180
    t, m = src['truth'], src['truth']['mount']
    return {
        ('tunnel', 'radius_m'): src['tunnel']['radius_m'], ('tunnel', 'axis_z_m'): src['tunnel']['axis_z_m'],
        ('start_theta_rad',): src['motion']['start_theta_deg'] * rad,
        ('wheel_diameter_m',): t['wheel_diameter_m'],
        **({('odo_left_diameter_m',):t.get('odo_left_diameter_m',t['wheel_diameter_m']),
            ('odo_right_diameter_m',):t.get('odo_right_diameter_m',t['wheel_diameter_m'])} if src.get('contact',{}).get('enabled') else {}),
        ('scan_encoder_zero_rad',): t['scan_encoder_zero_deg'] * rad,
        ('gate_start_offset_rad',): t['gate_start_offset_deg'] * rad,
        ('gate_end_offset_rad',): t['gate_end_offset_deg'] * rad,
        ('head_mount_x_m',): t['head_mount_x_m'],
        **{('mount', k): m[k] for k in ('e_m', 'tangential_m', 'dy_m', 'dz_m', 'tilt_y_rad', 'tilt_z_rad', 'twist_rad')},
        ('config_sha256',): source_sha,
    }


def field_mismatches(record, expected):
    bad = []
    for path, want in expected.items():
        got = record
        for key in path:
            got = got.get(key) if isinstance(got, dict) else None
        if isinstance(want, str):
            ok = got == want
        else:
            ok = got is not None and math.isclose(float(got), float(want), rel_tol=1e-12, abs_tol=1e-15)
        if not ok:
            bad.append('.'.join(path))
    return bad


def provenance_chain(s, cfg, truth, prov, backend):
    """Each link must hold: config source -> truth and observable config -> run inputs ->
    backend -> archived pose stream. Table and block hashes are checked separately."""
    source_path = s.root / 'evaluation' / 'config_source.yaml'
    source_sha = sha256_file(source_path)
    src = yaml.safe_load(source_path.read_text())
    links = {
        'config_source_equals_truth_record': source_sha == truth.get('config_sha256'),
        'config_source_equals_run_config': source_sha == prov.get('config_sha256'),
        'run_config_input_hash_matches': (prov.get('inputs') or {}).get('config', {}).get('sha256') == source_sha,
        'backend_self_check_passed': bool(backend.get('self_check', {}).get('passed')),
        'backend_ptx_identified': bool(backend.get('describe', {}).get('ptx_sha256')),
        'provenance_names_pose_source': prov.get('pose_source') == s.summary.get('pose_source'),
    }
    mismatched = []
    for (area, key), value in observable_from_source(src).items():
        got = cfg.get(area, {}).get(key)
        if got is None or not math.isclose(float(got), float(value), rel_tol=1e-12, abs_tol=1e-15):
            mismatched.append(f'{area}.{key}')
    links['observable_config_derives_from_source'] = not mismatched
    truth_mismatched = field_mismatches(truth, truth_from_source(src, source_sha))
    links['truth_derives_from_source'] = not truth_mismatched
    # Content identity recorded when the session completed (session.json "files").
    recorded = s.summary.get('files') or {}
    changed = [name for name, digest in recorded.items() if sha256_file(s.root / name) != digest]
    links['completion_file_hashes_recorded'] = bool(recorded)
    links['completion_file_hashes_match'] = bool(recorded) and not changed
    pose_input = (prov.get('inputs') or {}).get('pose_stream')
    if pose_input:
        archived = read_json(s.root / 'evaluation' / 'manifest.json')['pose_stream']['sha256']
        links['archived_pose_stream_equals_input'] = pose_input['sha256'] == archived
    ok = all(links.values())
    return check('provenance_chain', PASS if ok else FAIL, links=links, observable_mismatches=mismatched,
                 truth_mismatches=truth_mismatched, files_changed_since_completion=changed)


def compare_sessions(a, b):
    diffs, n = [], 0
    for area in ('raw', 'metadata'):
        for p in sorted((a / area).iterdir()):
            n += 1
            other = b / area / p.name
            if not other.exists() or sha256_file(p) != sha256_file(other):
                diffs.append(f'{area}/{p.name}')
    for p in sorted((a / 'evaluation').glob('*.bin')):
        n += 1
        if sha256_file(p) != sha256_file(b / 'evaluation' / p.name):
            diffs.append('evaluation/' + p.name)
    return check('reimaging_byte_identical', PASS if not diffs and n else FAIL, files_compared=n,
                 differing=diffs, other_session=str(b))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('session')
    ap.add_argument('--compare', help='a re-imaged session that must be byte-identical')
    args = ap.parse_args(argv)
    s = Session(args.session)
    checks = []
    status = s.summary.get('status')
    checks.append(check('session_complete', PASS if status == 'complete' else FAIL, status=status))
    if status != 'complete':
        return finish(s, checks, args)
    n, bad = verify_hashes(s)
    checks.append(check('stored_hashes_verify', PASS if not bad and n else FAIL, files=n, mismatched=bad))
    prov = read_json(s.root / 'config' / 'provenance.json')
    backend = read_json(s.root / 'config' / 'backend.json')
    checks.append(check('backend_self_check', PASS if backend['self_check'].get('passed') else FAIL,
                        optix_library=backend['describe'].get('optix_library'),
                        ptx_sha256=backend['describe'].get('ptx_sha256')))
    checks.append(check('binary_matches_source', PASS if prov.get('binary_matches_source') else FAIL,
                        build=prov.get('build'), source_at_run=prov.get('source_at_run')))

    cfg, truth = s.config(), s.truth()
    checks.append(provenance_chain(s, cfg, truth, prov, backend))
    checks.append(planned_motion(s.summary))
    source_config = yaml.safe_load((s.root / 'evaluation' / 'config_source.yaml').read_text())
    poses, row_truth = s.evaluation('pose_stream'), s.evaluation('row_truth')
    timing, rows, dropped = compare_timing(s, cfg, truth, poses)
    checks += timing
    checks.append(accounting(rows, dropped))
    checks.append(valid_region(source_config, poses, rows, row_truth, dropped))
    checks.append(gate_geometry(cfg, truth, rows, row_truth))
    checks.append(advance_per_rev(cfg, row_truth))
    gpu, truth_hits = gpu_vs_cpu(s, cfg, truth, row_truth)
    checks += gpu
    checks.append(pixel_spot_check(s, cfg, truth, row_truth))
    checks.append(ideal_unroll(s, cfg, rows, truth_hits))
    if args.compare:
        checks.append(compare_sessions(s.root, Path(args.compare)))
    else:
        checks.append(check('reimaging_byte_identical', UNMEASURABLE, reason='no --compare session given'))
    return finish(s, checks, args)


def finish(s, checks, args):
    states = [c['state'] for c in checks]
    overall = FAIL if FAIL in states else (UNMEASURABLE if UNMEASURABLE in states else PASS)
    report_dir = s.root / 'evaluation' / 'reports'
    report_dir.mkdir(exist_ok=True)
    report = report_dir / 'stage_a.json'
    body = {'schema': 'ssb.stage_a_report.v1', 'overall': overall, 'checks': checks}
    report.write_text(json.dumps(body, indent=2, default=float) + '\n')
    # Everything the checks read. Raw blocks are bound through raw/index.json, whose
    # per-block hashes the stored_hashes_verify check re-computes.
    def session_files(root):
        root = Path(root)
        files = [root / 'session.json', root / 'raw' / 'index.json', root / 'logs' / 'performance.json']
        files += sorted((root / 'config').glob('*.json'))
        files += sorted((root / 'metadata').glob('*'))
        files += [p for p in sorted((root / 'evaluation').glob('*')) if p.is_file()]
        return [p for p in files if p.exists()]
    inputs = session_files(s.root)
    if args.compare:
        inputs += session_files(args.compare)
    record = stage_record('validate_stage_a', inputs, [report], {'compare': args.compare})
    (report_dir / 'stage_a.provenance.json').write_text(json.dumps(record, indent=2) + '\n')
    width = max(len(c['name']) for c in checks)
    for c in checks:
        print(f"{c['name']:<{width}}  {c['state']}")
    print(f'overall: {overall}  ({report})')
    return 0 if overall == PASS else 1


if __name__ == '__main__':
    sys.exit(main())
