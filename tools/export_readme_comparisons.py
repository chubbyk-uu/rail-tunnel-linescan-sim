#!/usr/bin/env python3
"""Present two verified feature crops at 100% pixels; titles never touch data."""
import argparse
import json
import os
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from ssb_tools.public_capture import confined_file
from ssb_tools.provenance import stage_record
from ssb_tools.session import read_json, sha256_file


def export(features, output):
    features = Path(features).resolve()
    output = Path(output).resolve()
    report = read_json(features/'report.json')
    record = read_json(features/'provenance.json')
    if report.get('schema') != 'ssb.feature_review.v2' or record.get('stage') != 'feature_review':
        raise ValueError('verified public feature review required')
    selected = {}
    for crop in report['crops']:
        selected.setdefault(crop['candidate']['kind'], crop)
    if not {'wide', 'thin'} <= selected.keys():
        raise ValueError('both wide and thin image structures required')

    inputs = [features/'report.json', features/'provenance.json']
    sources = {}
    for kind in ('wide', 'thin'):
        for name in ('raw', 'optimized'):
            file = f'feature_{selected[kind]["id"]}_{name}.png'
            sources[(kind, name)] = confined_file(features, file)
    sources[('raw', 'band')] = confined_file(features, 'raw_band.png')
    archived_root = Path(os.path.commonpath(list(record['outputs'])))
    identities = {str(Path(p).relative_to(archived_root)): v for p, v in record['outputs'].items()}
    for path in inputs[:1]+list(sources.values()):
        if identities.get(str(path.relative_to(features))) != sha256_file(path):
            raise ValueError('feature product identity mismatch: '+path.name)
    inputs.extend(sources.values())
    output.mkdir(parents=True, exist_ok=False)
    products = []
    for kind, label in (('wide', 'joint'), ('thin', 'crack')):
        pixels = [np.array(Image.open(sources[kind, name]).convert('RGB'))
                  for name in ('raw', 'optimized')]
        if pixels[0].shape != pixels[1].shape:
            raise ValueError('paired crop dimensions differ')
        h, w = pixels[0].shape[:2]
        width, height = 2*w+24, h+48
        fig = plt.figure(figsize=(width/100, height/100), dpi=100, facecolor='#202124')
        try:
            for x, image, title in zip((0, w+24), pixels,
                    ('Raw helix (no motion compensation)', 'Feature matching + global optimization')):
                ax = fig.add_axes([x/width, 0, w/width, h/height])
                ax.imshow(image, interpolation='nearest'); ax.set_axis_off()
                fig.text((x+8)/width, (h+24)/height, title, color='white', fontsize=12,
                         ha='left', va='center')
            path = output/f'{label}_comparison.png'
            fig.savefig(path, dpi=100, facecolor=fig.get_facecolor())
        finally:
            plt.close(fig)
        stored = np.array(Image.open(path).convert('RGB'))
        for x, source in zip((0, w+24), pixels):
            if not np.array_equal(stored[48:48+h, x:x+w], source):
                raise ValueError('presentation changed source pixels')
        products.append(dict(file=path.name, candidate=selected[kind],
                             source_pixels_identical=True, panel_shape=[h, w]))
    shutil.copyfile(sources['raw', 'band'], output/'raw_band.png')
    (output/'report.json').write_text(json.dumps(dict(schema='ssb.readme_comparisons.v1',
        method='first wide/thin public image candidates; 1:1 panels, titles outside image',
        source= str(features), products=products, raw_band=report['raw']['single_band']), indent=2)+'\n')
    files = sorted(output.iterdir())
    (output/'provenance.json').write_text(json.dumps(stage_record('readme_comparisons',
        inputs, files, dict(pixel_scale=1, fusion=False, sharpening=False)), indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--features', required=True)
    parser.add_argument('--output', required=True)
    export(**vars(parser.parse_args()))
