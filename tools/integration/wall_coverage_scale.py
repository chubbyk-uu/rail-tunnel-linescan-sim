#!/usr/bin/env python3
"""20 m / 240 degree coverage resource regression on synthetic public sensor tables.

This is a geometry/metadata benchmark, not a real Gazebo or optical capture.
Only the reference's observable configuration and measured calibration are read.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import resource
import time

import numpy as np

from ssb_tools.session import Session, sha256_file
from ssb_tools.wall_coverage import calibrated_spans, inspect_session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    config = copy.deepcopy(Session(args.reference).config())
    calibration = json.loads(args.calibration.read_text())
    left, right = max(calibrated_spans(config, calibration), key=lambda pair: pair[1]-pair[0])
    pitch = config['motion']['advance_per_rev_m']; guard = .01
    encoder = config['scan_encoder']; counts = encoder['ppr']*encoder['edges_per_cycle']
    rescaler = config['rescaler']; lattice = counts*rescaler['multiply']/rescaler['divide']
    speed = pitch*config['motion']['line_rate_hz']/lattice
    mount = config['calibration']['head_mount_x_m']
    start = -pitch-left-mount-speed/2-guard
    end = 20+pitch-right-mount+speed/2+guard
    config['motion']['start_x_m'] = start
    gate_start = config['gate']['start_rad']
    arc = (config['gate']['end_rad']-gate_start) % (2*math.pi)
    assert math.isclose(arc, 4*math.pi/3), 'benchmark requires full upper 240 degrees'
    config['inspection'] = dict(schema='ssb.wall_target.v1', target_x_m=[0., 20.],
                                theta_rad=[gate_start, gate_start+arc], grid_pitch_m=.0002, guard_m=guard)
    # Explicit synthetic constant-speed trajectory; no simulator pose/scene files.
    phase = math.pi; duration = (end-start)/speed; omega = 2*math.pi*speed/pitch
    row_ids = np.arange(1, math.floor(duration*config['motion']['line_rate_hz'])+1, dtype=np.int64)
    angles = phase+row_ids*2*math.pi/lattice
    wrapped = (angles-gate_start) % (2*math.pi)
    selected = wrapped < arc
    row_ids, angles = row_ids[selected], angles[selected]
    times = row_ids/config['motion']['line_rate_hz']
    row_dtype = [('sequence', '<i8'), ('row', '<i8'), ('segment', '<i8'),
                 ('t_trigger', '<f8'), ('t_center', '<f8')]
    rows = np.zeros(len(times), row_dtype)
    rows['sequence'] = np.arange(len(rows)); rows['row'] = row_ids
    rows['segment'] = np.floor((angles-gate_start)/(2*math.pi)).astype(np.int64)
    rows['t_trigger'] = times; rows['t_center'] = times
    edge_dtype = [('t', '<f8'), ('count', '<i8'), ('dir', '<i8')]
    scan_ids = np.arange(math.floor(phase*counts/(2*math.pi))+1,
                         math.floor((phase+omega*duration)*counts/(2*math.pi))+1, dtype=np.int64)
    scan = np.zeros(len(scan_ids), edge_dtype); scan['count'] = scan_ids; scan['dir'] = 1
    scan['t'] = (scan_ids*2*math.pi/counts-phase)/omega
    revs = np.unique(rows['segment'])
    gates = np.zeros(len(revs), [('t', '<f8'), ('revolution', '<i8'), ('kind', '<i4'), ('dir', '<i4')])
    gates['revolution'] = revs; gates['dir'] = 1
    gates['t'] = (gate_start+revs*2*math.pi-phase)/omega
    root = output/'public_fixture'; (root/'config').mkdir(parents=True); (root/'metadata').mkdir()
    (root/'config/observable_config.json').write_text(json.dumps(config))
    manifest = {}
    def save(name, table):
        path = root/'metadata'/(name+'.bin'); table.tofile(path)
        manifest[name] = dict(file=path.name, dtype=table.dtype.descr, record_size=table.dtype.itemsize,
                              count=len(table), sha256=sha256_file(path))
    save('rows', rows); save('scan_edges', scan); save('gate_events', gates)
    odometer = config['odometer']; odo_counts = odometer['ppr']*odometer['edges_per_cycle']*odometer['gear_ratio']
    for name, diameter in [('odometer_edges', config['calibration']['odo_left_diameter_m']),
                           ('odometer_right_edges', config['calibration']['odo_right_diameter_m'])]:
        odo_ids = np.arange(1, math.floor((end-start)*odo_counts/(math.pi*diameter))+1, dtype=np.int64)
        odo = np.zeros(len(odo_ids), edge_dtype); odo['count'] = odo_ids; odo['dir'] = 1
        odo['t'] = odo_ids*math.pi*diameter/(odo_counts*speed); save(name, odo)
    row_count = len(rows)
    del row_ids, angles, wrapped, selected, times, rows, scan_ids, scan, gates, odo_ids, odo
    (root/'metadata/manifest.json').write_text(json.dumps(manifest))
    summary = dict(status='complete', motion=dict(complete=True), rows=row_count,
                   synthetic=True, files={name: sha256_file(root/name) for name in
                                          ('config/observable_config.json', 'metadata/manifest.json')})
    (root/'session.json').write_text(json.dumps(summary))
    prepared = time.monotonic()
    result = inspect_session(root, args.calibration, output/'coverage')
    finished = time.monotonic()
    report = dict(schema='ssb.coverage_scale_test.v1', synthetic=True, nominal_complete=result['nominal_complete'],
                  target_x_m=[0., 20.], target_degrees=240, rows=row_count, grid=result['grid'],
                  missing_pixels=result['missing_pixels'], preparation_s=prepared-started,
                  coverage_s=finished-prepared, total_s=finished-started,
                  peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
                  counts_bytes=(output/'coverage/counts.bin').stat().st_size,
                  reference=str(args.reference.resolve()), calibration_sha256=sha256_file(args.calibration))
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))
    assert result['nominal_complete'] and result['missing_pixels'] == 0, report
    assert report['peak_rss_bytes'] < 1024**3, report
    assert report['counts_bytes'] < 256*1024**2, report


if __name__ == '__main__': main()
