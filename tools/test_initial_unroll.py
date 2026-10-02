#!/usr/bin/env python3
"""Export only public ROI inputs; reproduce D1 with another backend/tile size."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from ssb_tools.initial_unroll import trace_pixel, load_bands, release_pages
from ssb_tools.session import sha256_file


def run(source, reference, calibration, output, backend='cuda'):
    source, reference, output = map(lambda p: Path(p).resolve(), (source, reference, output))
    output.mkdir(parents=True, exist_ok=False)
    public = output/'public_inputs'; public.mkdir()
    for name in ('config', 'metadata', 'raw'): (public/name).mkdir()
    original = json.loads((source/'session.json').read_text())
    allowed = ('config/observable_config.json', 'metadata/manifest.json', 'raw/index.json')
    for name in allowed: shutil.copyfile(source/name, public/name)
    manifest = json.loads((public/'metadata/manifest.json').read_text())
    config = json.loads((public/'config/observable_config.json').read_text())
    names = ['rows', 'scan_edges', 'odometer_edges', 'gate_events']
    if config.get('contact', {}).get('enabled'): names.append('odometer_right_edges')
    for name in names: os.link(source/'metadata'/manifest[name]['file'], public/'metadata'/manifest[name]['file'])
    baseline = json.loads((reference/'report.json').read_text())
    for block in baseline['sensor']['raw_blocks']: os.link(source/'raw'/block['file'], public/'raw'/block['file'])
    # Strip descriptive/evaluation fields: reconstruction sees only public completion facts.
    (public/'session.json').write_text(json.dumps(dict(status=original['status'], motion=original['motion'],
        rows=original['rows'], files={name: original['files'][name] for name in allowed})))
    shutil.copyfile(calibration, public/'calibration.json')
    result = output/'unroll'
    with (output/'unroll.log').open('w') as log:
        subprocess.run([sys.executable, '-m', 'ssb_tools.initial_unroll', '--session', str(public),
            '--calibration', str(public/'calibration.json'), '--output', str(result),
            '--target-x', *map(str, baseline['grid']['target_x_m']), '--angular-tile-rows', '17',
            '--backend', backend],
            stdout=log, stderr=subprocess.STDOUT, check=True)
    products = ['sensor_flat.npy', 'sensor_valid_bits.npy', 'projection.npy', 'mapping.npz',
                'mosaic.npy', 'coverage.npy', 'source_band.npy', 'bands.json', 'calibration.json']
    differing = [name for name in products if sha256_file(reference/name) != sha256_file(result/name)]
    # NPZ ZIP timestamps may differ despite identical arrays; compare contents explicitly.
    import numpy as np
    if 'mapping.npz' in differing:
        with np.load(reference/'mapping.npz') as a, np.load(result/'mapping.npz') as b:
            assert a.files == b.files and all(np.array_equal(a[k], b[k]) for k in a.files)
        differing.remove('mapping.npz')
    assert not differing, differing
    provenance = json.loads((result/'provenance.json').read_text())
    assert all(Path(p).resolve().is_relative_to(public) for p in provenance['inputs'])
    assert not (public/'evaluation').exists()
    counts = np.load(result/'coverage.npy', mmap_mode='r')
    holes = []
    for q in range(0, len(counts), 64):
        row, column = np.nonzero(counts[q:q+64] == 0)
        holes.extend(zip((row+q).tolist(), column.tolist()))
        release_pages(counts)
    # Check stored-pixel provenance and collect invalid-pixel locations, without
    # loading the dense 864-million-pixel grid or using defect ground truth.
    traces = []
    for q in np.linspace(100, len(counts)-101, 5, dtype=int):
        x = int(np.flatnonzero(counts[q] > 0)[len(np.flatnonzero(counts[q] > 0))//2])
        value = trace_pixel(result, int(q), x)
        assert value['valid'] and value['error_dn'] < 1e-5
        assert abs(sum(v['weight'] for v in value['contributions'])-1) < 1e-10
        traces.append(value)
    sampler = load_bands(result); grid = baseline['grid']
    native_mask_holes = 0
    for q, column in holes:
        angle = grid['theta_rad'][0]+(q+.5)*grid['dq_m']/grid['radius_m']
        x = grid['target_x_m'][0]+(column+.5)*grid['dx_m']
        blocked = False
        for band in range(len(sampler.segments)):
            lo, hi, _, supported = sampler.row_sources(band, np.array([angle]))
            if not supported[0]: continue
            geometric = True; invalid_native = False
            for row in (lo[0], hi[0]):
                delta = x-sampler.projection['x_axis_m'][row]
                if not (sampler.offsets[0] <= delta <= sampler.offsets[-1] and
                        sampler.output_offsets[0] <= delta <= sampler.output_offsets[-1]):
                    geometric = False; break
                cu = np.interp(delta, sampler.output_offsets, np.arange(len(sampler.offsets)))
                c0 = int(np.floor(cu)); c1 = min(c0+1, len(sampler.offsets)-1)
                if not (sampler.geometry_valid[c0] and sampler.geometry_valid[c1]):
                    geometric = False; break
                u = np.interp(delta, sampler.offsets, np.arange(len(sampler.offsets)))
                u0 = int(np.floor(u)); u1 = min(u0+1, len(sampler.offsets)-1)
                invalid_native |= not np.isfinite(sampler.image[row, [u0, u1]]).all()
            blocked |= geometric and invalid_native
        native_mask_holes += int(blocked)
    report = dict(public_only=True, no_evaluation_directory=True,
        identical_products=len(products), pixel_traces=traces, invalid_pixels=len(holes),
        invalid_sample_coordinates=holes[:16],
        holes_with_invalid_native_support=native_mask_holes,
        result=str(result), backend=backend,
        performance=json.loads((result/'report.json').read_text())['performance'],
        performance_including_hashes=provenance.get('performance'))
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('public_only','identical_products','invalid_pixels','performance')}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('session','reference','calibration','output'): parser.add_argument('--'+name, required=True)
    parser.add_argument('--backend', choices=['cpu', 'cuda'], default='cuda')
    args = parser.parse_args(); run(args.session, args.reference, args.calibration, args.output, args.backend)
