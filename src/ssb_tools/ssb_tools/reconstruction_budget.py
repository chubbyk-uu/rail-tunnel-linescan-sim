"""Bounded nominal resource planning before capture; no pixels or rig truth used."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import shutil
import time

import numpy as np
import yaml

from .band_matching import MatchSettings
from .global_geometry import Trajectory, reconstruction_settings
from .initial_unroll import BandSampler, PROJECTION, sensor_geometry
from .match_bands import plan_windows, WINDOW_BUDGET, DIAGNOSTIC_WINDOW_BUDGET
from .mission_plan import wall_plan
from .session import sha256_file
from .wall_coverage import calibrated_row_footprint, target_grid
from .reconstruction_support import matching_halo_m

MAXIMUM_PLANNING_LATTICE_ROWS = 4_000_000


class PlanningRows:
    def __init__(self, shape):
        self.shape = shape

    def gather(self, *args):
        raise AssertionError('resource planning must not access pixels')


def plan(config, calibration, start, length, settings, spacing_m=.1, height=256, max_q_shift_mm=10.):
    started = time.monotonic()
    settings.validate()
    # Generator templates also contain truth. It cannot influence this estimate.
    config = {k: v for k, v in config.items() if k != 'truth'}
    _, task = wall_plan(config, start, length, calibration,
                        relative_encoder_scale=settings.relative_encoder_scale)
    e, r = config['scan_encoder'], config['rescaler']
    per_rev = e['ppr']*e['edges_per_cycle']*r['multiply']/r['divide']
    pitch = config['motion']['advance_per_rev_m']
    if not np.isfinite([per_rev, pitch]).all() or min(per_rev, pitch) <= 0:
        raise ValueError('positive finite nominal encoder pitch and rescaling required')
    lattice_rows = math.ceil(task['distance_m']/pitch*per_rev)
    if lattice_rows > MAXIMUM_PLANNING_LATTICE_ROWS:
        raise ValueError('nominal resource planning requires bounded partitions above four million lattice rows')
    rows = np.arange(1, lattice_rows, dtype=np.int64)
    theta = math.radians(config['motion']['start_theta_deg'])+rows*(2*math.pi/per_rev)
    segment = np.floor((theta+math.pi)/(2*math.pi)).astype(np.int64)
    wrapped = theta-segment*2*math.pi
    keep = ((wrapped >= math.radians(config['gate']['start_deg'])) &
            (wrapped < math.radians(config['gate']['end_deg'])))
    p = np.zeros(np.count_nonzero(keep), PROJECTION)
    p['sequence'] = np.arange(len(p)); p['lattice_row'] = rows[keep]
    p['segment'] = segment[keep]; p['theta_rad'] = theta[keep]
    p['x_axis_m'] = task['start_m']+rows[keep]*pitch/per_rev+config['calibration']['head_mount_x_m']
    del rows, theta, segment, wrapped, keep
    offsets, corrected, valid = sensor_geometry(config, calibration)
    sampler = BandSampler(p, PlanningRows((len(p), len(offsets))), offsets, corrected, valid,
                          calibrated_row_footprint(config, calibration))
    model = Trajectory(sampler, config['calibration']['radius_m'], config['robot']['scan_axis_height_m'], settings)
    grid = target_grid([start, start+length], model.radius, [-2*math.pi/3, 2*math.pi/3], .0002)
    halo=matching_halo_m([float(p['x_axis_m'].min()),float(p['x_axis_m'].max())],settings.relative_encoder_scale)
    windows = plan_windows(sampler, grid, spacing_m, height, 1024, MatchSettings(max_q_shift_mm=max_q_shift_mm),halo)
    raw_bytes = len(p)*len(offsets)
    pixels = math.prod(grid['shape'])
    return dict(schema='ssb.nominal_reconstruction_budget.v1', status='pass',
        task=task, settings=asdict(settings),
        d2=dict(spacing_m=spacing_m, height=height, max_q_shift_mm=max_q_shift_mm),
        estimated_rows=len(p), bands=len(sampler.bounds), trajectory_coefficients=model.size,
        coefficient_limit=2048, available_refinement_coefficients=2048-model.size,
        matching=dict(descriptors=len(windows), image_windows=sum(w['status'] == 'planned' for w in windows),halo_m=halo,
                      image_window_limit=WINDOW_BUDGET, descriptor_limit=DIAGNOSTIC_WINDOW_BUDGET),
        scale_margin_m=model.scale_margin_m(), projection_bytes=p.nbytes, grid=grid,
        storage=dict(estimated_raw_bytes=raw_bytes, optimized_mosaic_bytes=3*pixels,
                     approximate_free_bytes_required=math.ceil(1.1*(2*raw_bytes+3*pixels+2*len(p)*512))),
        wall_s=time.monotonic()-started,
        limitations=['nominal spatial planning, not measured dynamics, matches or peak RSS',
                     'actual rows/windows/refinement and free disk must still be checked during the run',
                     'no image pixels, wheel truth, mount truth, track realization or evaluation score used'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', required=True); parser.add_argument('--output', required=True)
    parser.add_argument('--start', type=float, required=True); parser.add_argument('--length', type=float, required=True)
    parser.add_argument('--spacing-m', type=float, default=.1); parser.add_argument('--height', type=int, default=256)
    parser.add_argument('--max-q-shift-mm', type=float)
    parser.add_argument('--adaptive-attitude', action='store_true'); parser.add_argument('--slow-translation', action='store_true')
    parser.add_argument('--relative-encoder-scale', action='store_true')
    parser.add_argument('--geometry-backend', choices=('numpy', 'cpu', 'cuda'), default='cuda')
    parser.add_argument('--surface-relief', action='store_true', help='accepted for the shared pipeline arguments')
    args = parser.parse_args(); demo, output = Path(args.demo).resolve(), Path(args.output).resolve()
    if output.exists():
        raise ValueError('resource report must be fresh')
    settings = reconstruction_settings(.02, args.adaptive_attitude, args.slow_translation, args.relative_encoder_scale, args.geometry_backend)
    report = plan(yaml.safe_load((demo/'capture.yaml').read_text()), json.loads((demo/'calibration.json').read_text()),
                  args.start, args.length, settings, args.spacing_m, args.height, args.max_q_shift_mm)
    report['input_hashes'] = {str(demo/name): sha256_file(demo/name) for name in ('capture.yaml', 'calibration.json')}
    output.parent.mkdir(parents=True, exist_ok=True)
    available = shutil.disk_usage(output.parent).free
    report['storage']['available_bytes'] = available
    if available < report['storage']['approximate_free_bytes_required']:
        report['status'] = 'fail'
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('status', 'estimated_rows', 'trajectory_coefficients', 'matching', 'storage', 'wall_s')}))
    if report['status'] != 'pass':
        raise SystemExit('insufficient free storage for nominal capture/reimage/reconstruction budget')


if __name__ == '__main__':
    main()
