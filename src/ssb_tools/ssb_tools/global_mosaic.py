"""Full-resolution, unblended D3 mosaics and exact pixel coverage from public inputs."""
import argparse
import html
import json
import math
from pathlib import Path
import shutil
import time
import numpy as np
from PIL import Image
from .reconstruction_support import correction_reach_m
from .global_resample import load_global, corrected_tile
from .initial_unroll import (rasterize, preview_sample, release_pages, display,
                             MOSAIC_INVALID, MOSAIC_SCALE)
from .match_bands import verified_bands
from .provenance import stage_record
from .stage_b_scene import peak_rss_bytes


class CpuGlobalRaster:
    """Explicit reference path; never chosen as fallback for a CUDA error."""
    def __init__(self, model, coefficients):
        self.model, self.coefficients = model, coefficients
        self.margin = correction_reach_m(model.radius, model.height)

    def candidate_bands(self, xs):
        return [i for i, (lo, hi) in enumerate(self.model.sampler.band_x)
                if hi+self.margin >= xs[0] and lo-self.margin <= xs[-1]]

    def tile(self, angles, xs, bands=None):
        values, counts = corrected_tile(self.model, self.coefficients, np.asarray(angles)*self.model.radius, xs)
        return values, counts, None

    def close(self):
        pass

    def describe(self):
        return dict(name='cpu', geometry_precision='float64', role='explicit reference',
                    surface_relief=self.model.relief is not None)


def write_preview(output, cover_min, stride):
    stored = np.load(output/'mosaic_u16.npy', mmap_mode='r')
    try:
        code = preview_sample(stored, stride)
    finally:
        release_pages(stored)
        stored._mmap.close()
    values = code.astype(np.float32)/MOSAIC_SCALE
    values[code == MOSAIC_INVALID] = np.nan
    Image.fromarray(display(values)).save(output/'preview.png')
    rgb = np.zeros((*cover_min.shape, 3), np.uint8)
    rgb[cover_min == 0] = [190, 40, 40]
    rgb[cover_min == 1] = [60, 150, 80]
    rgb[cover_min > 1] = [45, 105, 210]
    Image.fromarray(rgb).save(output/'coverage.png')


def run(unroll, trajectory, output, raw_root=None, backend='cuda', comparison=True,
        angular_tile_rows=32, tile_columns=8192):
    started = time.monotonic()
    output = Path(output).resolve()
    if any(output.is_relative_to(Path(p).resolve()) for p in (unroll, trajectory)):
        raise ValueError('mosaic output must be separate from inputs')
    if backend not in ('cuda', 'cpu'):
        raise ValueError('explicit cpu or cuda backend required')
    if (not isinstance(angular_tile_rows, int) or angular_tile_rows < 1 or
            not isinstance(tile_columns, int) or tile_columns < 1):
        raise ValueError('positive mosaic tile dimensions required')
    sampler, upstream, inputs = verified_bands(unroll, raw_root)
    accelerator = nominal = None
    try:
        model, coefficients, _, trajectory_inputs = load_global(trajectory, sampler, upstream, unroll)
        grid = upstream['grid']; nq, nx = grid['shape']
        output.parent.mkdir(parents=True, exist_ok=True)
        required = nq*nx*3*(2 if comparison else 1)
        if shutil.disk_usage(output.parent).free < required*1.1:
            raise ValueError('insufficient disk for requested full-resolution mosaics and margin')
        # Fail before creating output if CUDA is unavailable or the trajectory is invalid.
        if backend == 'cuda':
            from .global_cuda import GlobalCudaRaster
            from .unroll_cuda import CudaRaster
            accelerator = GlobalCudaRaster(model, coefficients)
            if comparison:
                nominal = CudaRaster(sampler)
        else:
            accelerator = CpuGlobalRaster(model, coefficients)
        output.mkdir(exist_ok=False)
        stride = max(1, math.ceil(max(nq, nx)/1800))
        products = {}
        for name, engine in ([('nominal', nominal)] if comparison else [])+[('optimized', accelerator)]:
            phase = time.monotonic()
            folder = output/name; folder.mkdir()
            stats, cover = rasterize(sampler, grid, folder, angular_tile_rows, engine,
                                     (0, nx), tile_columns, preview_stride=stride)
            write_preview(folder, cover, stride)
            products[name] = dict(coverage=stats, wall_s=time.monotonic()-phase,
                                  backend=engine.describe() if engine else dict(name='cpu', geometry_precision='float64'))
        covered = products['optimized']['coverage']
        if covered['missing_pixels'] == covered['total_pixels']:
            raise ValueError('optimized mosaic has no supported pixel')
        report = dict(schema='ssb.global_mosaic.v1', stage='D3 full-resolution output',
            status='complete' if not covered['missing_pixels'] else 'coverage_incomplete',
            grid=grid, products=products,
            storage=dict(image='mosaic_u16.npy', scale=MOSAIC_SCALE, invalid=MOSAIC_INVALID,
                         count='mosaic_count.npy; valid independent bands, saturating at 255',
                         exact_counts='coverage_runs.bin', planned_image_bytes=required,
                         quantization_max_dn=1/(2*MOSAIC_SCALE)),
            preview=dict(stride=stride, values='stored DN at stride-th cell centres; fixed 0..255 display',
                         coverage='minimum count over every stride x stride cell; red includes any missing pixel'),
            coverage_gate=dict(status='pass' if not covered['missing_pixels'] else 'fail',
                               requirement='all declared grid pixels have at least one valid native observation'),
            inputs='public raw rows, encoders, measured optical calibration, image-fitted trajectory and declared shared depth only',
            surface_relief=model.relief is not None,
            limitations=['coverage does not certify geometric accuracy', 'nominal surface coordinates; no actual poses or optical mesh',
                         'no seam fusion, inpainting, sharpening or contrast adjustment', 'noise robustness requires independent capture evidence'],
            performance=dict(wall_s=time.monotonic()-started, peak_rss_bytes=peak_rss_bytes()))
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        page = ['<!doctype html><meta charset="utf-8"><title>Full-resolution D3 comparison</title>',
                '<style>body{background:#202124;color:#eee;font:16px sans-serif;margin:24px} img{width:100%;image-rendering:auto} table{width:100%;table-layout:fixed}td{vertical-align:top}</style>',
                '<h1>Full-resolution D3 output</h1>',
                '<p>Same 0.2 mm grid and fixed DN display; no blending. Images below are overview samples. Full pixel data and coverage are linked.</p>',
                '<table><tr>']
        for name, item in products.items():
            page.append(f'<td><h2>{html.escape(name)}</h2><p>Missing pixels: {item["coverage"]["missing_pixels"]}</p>'
                        f'<img src="{name}/preview.png"><img src="{name}/coverage.png">'
                        f'<p><a href="{name}/mosaic_u16.npy">Full image (uint16)</a> · '
                        f'<a href="{name}/mosaic_count.npy">Coverage count</a></p></td>')
        page.append('</tr></table><p>Coverage: red = missing; green = one band; blue = overlapping bands.</p>')
        (output/'review.html').write_text('\n'.join(page)+'\n')
        files = sorted(p for p in output.rglob('*') if p.is_file())
        record = stage_record('global_mosaic', inputs+trajectory_inputs, files,
                              dict(backend=backend, grid=grid, comparison=comparison,
                                   angular_tile_rows=angular_tile_rows, tile_columns=tile_columns,
                                   cuda=accelerator.describe()))
        record['performance'] = dict(wall_s=time.monotonic()-started, peak_rss_bytes=peak_rss_bytes())
        (output/'provenance.json').write_text(json.dumps(record, indent=2)+'\n')
        return report
    finally:
        if nominal is not None:
            nominal.close()
        if accelerator is not None:
            accelerator.close()
        if hasattr(sampler.native, 'close'):
            sampler.native.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--unroll', required=True); p.add_argument('--trajectory', required=True)
    p.add_argument('--output', required=True); p.add_argument('--raw')
    p.add_argument('--backend', choices=('cuda', 'cpu'), default='cuda')
    p.add_argument('--optimized-only', action='store_true')
    p.add_argument('--angular-tile-rows', type=int, default=32)
    p.add_argument('--tile-columns', type=int, default=8192)
    args = p.parse_args()
    report = run(args.unroll, args.trajectory, args.output, args.raw, args.backend,
                 not args.optimized_only, args.angular_tile_rows, args.tile_columns)
    print(json.dumps({k: report[k] for k in ('status', 'products', 'performance')}))
    raise SystemExit(0 if report['coverage_gate']['status'] == 'pass' else 1)


if __name__ == '__main__':
    main()
