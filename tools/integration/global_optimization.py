#!/usr/bin/env python3
"""Reproduce D3 with relocated public products/raw blocks and no truth assets."""
import argparse
import json
import os
from pathlib import Path
import shutil
import sys

import numpy as np
from ssb_tools.optimize_bands import run, GeometrySettings
from ssb_tools.session import read_json, sha256_file


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('unroll', 'matches', 'observable', 'reference', 'output'):
        parser.add_argument('--'+name, required=True)
    parser.add_argument('--raw')
    args = parser.parse_args()
    root = Path(args.output).resolve(); root.mkdir(parents=True, exist_ok=False)
    public = root/'public'; public.mkdir()
    d1, d2, config, raw = [public/p for p in ('d1', 'd2', 'config', 'raw')]
    for path in (d1, d2, config, raw): path.mkdir()
    for name in ('report.json', 'provenance.json', 'bands.json', 'projection.npy', 'mapping.npz', 'native_source.json'):
        shutil.copyfile(Path(args.unroll)/name, d1/name)
    for name in ('report.json', 'provenance.json', 'windows.json', 'matches.npy'):
        shutil.copyfile(Path(args.matches)/name, d2/name)
    shutil.copyfile(args.observable, config/'observable_config.json')
    source = read_json(d1/'native_source.json')
    source_raw = Path(args.raw) if args.raw else Path(source['session_root'])/'raw'
    for block in source['blocks']:
        path = source_raw/block['file']
        if not path.resolve().is_relative_to(source_raw.resolve()) or sha256_file(path) != block['sha256']:
            raise ValueError('invalid public raw block before relocation')
        # Read-only verification; hard links avoid a second 1.3 GB copy on this filesystem.
        try:
            os.link(path, raw/block['file'])
        except OSError:
            shutil.copyfile(path, raw/block['file'])
    settings = GeometrySettings(**read_json(Path(args.reference)/'report.json')['settings'])
    reads = set()

    def public_audit(event, arguments):
        if event != 'open' or not isinstance(arguments[0], (str, bytes, os.PathLike)):
            return
        path = Path(os.fsdecode(arguments[0])).absolute()
        if 'evaluation' in path.parts or path.name in ('truth.json', 'scene.json') or path.suffix == '.sdf':
            raise RuntimeError('private input opened during production optimization: '+str(path))
        if path.suffix == '.u8' and not path.resolve().is_relative_to(raw):
            raise RuntimeError('original raw locator was dereferenced instead of relocated public input')
        if path.is_relative_to(public): reads.add(str(path))

    sys.addaudithook(public_audit)
    result = run(d1, d2, config/'observable_config.json', root/'optimized', settings, raw)
    # Compare geometry/content, never timestamps, output paths or performance fields.
    for name in ('trajectory.json', 'windows.json'):
        if sha256_file(root/'optimized'/name) != sha256_file(Path(args.reference)/name):
            raise ValueError('public reproduction differs: '+name)
    with np.load(root/'optimized/match_residuals.npz') as a, np.load(Path(args.reference)/'match_residuals.npz') as b:
        if a.files != b.files or any(not np.array_equal(a[k], b[k]) for k in a.files):
            raise ValueError('public reproduction residuals differ')
    report = dict(status='pass', public_raw_blocks=len(source['blocks']), public_files_opened=sorted(reads),
        geometry_and_residuals_identical=True, evaluation_scene_world_reads=0,
        performance=result['performance'],
        limitation='audit applies to Python production input opens; hard links retain the original raw contents')
    (root/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('status', 'public_raw_blocks', 'geometry_and_residuals_identical', 'performance')}))


if __name__ == '__main__':
    main()
