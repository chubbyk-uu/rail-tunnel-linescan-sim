#!/usr/bin/env python3
"""Compose hash-verified public review pixels without changing their values."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ssb_tools.public_capture import confined_file
from ssb_tools.provenance import stage_record
from ssb_tools.session import read_json, sha256_file


def export(features, output, overview_x=None):
    features, output = Path(features).resolve(), Path(output).resolve()
    report, record = read_json(features/'report.json'), read_json(features/'provenance.json')
    if report.get('schema') != 'ssb.feature_review.v2' or record.get('stage') != 'feature_review':
        raise ValueError('verified public feature review v2 required')
    identities = {str(Path(p).relative_to(features)): digest for p, digest in record['outputs'].items()}
    inputs = [features/'report.json', features/'provenance.json']
    if identities.get('report.json') != sha256_file(inputs[0]):
        raise ValueError('review report hash mismatch')
    def pixels(name):
        path = confined_file(features, name)
        if identities.get(name) != sha256_file(path):
            raise ValueError('review product hash mismatch: '+name)
        inputs.append(path)
        with Image.open(path) as im:
            return np.array(im.convert('RGB'))
    lo, hi = report['grid']['target_x_m']
    selected = {}
    for crop in report['crops']:
        if lo <= crop['candidate']['x_m'] <= hi:
            selected.setdefault(crop['candidate']['kind'], crop)
    if not {'wide', 'thin'} <= selected.keys():
        raise ValueError('wide and thin public image candidates inside target required')
    raw, optimized = pixels('raw_helix.png'), pixels('geometry/optimized.png')
    if raw.shape != optimized.shape:
        raise ValueError('overview grids differ')
    grid, stride = report['grid'], report['preview_stride']
    view_lo, view_hi = (lo, hi) if overview_x is None else map(float, overview_x)
    if not np.isfinite([view_lo, view_hi]).all() or not lo <= view_lo < view_hi <= hi:
        raise ValueError('overview interval must be finite, ordered and inside the original target')
    xs = lo+(np.arange(raw.shape[1])*stride+.5)*grid['dx_m']
    columns = np.flatnonzero((xs >= view_lo)&(xs < view_hi))
    if not len(columns) or len(columns) > 512:
        raise ValueError('select an overview interval containing 1..512 saved preview columns')
    first_x, end_x = int(columns[0]), int(columns[-1]+1)
    q = grid['radius_m']*grid['theta_rad'][0]+(np.arange(raw.shape[0])*stride+.5)*grid['dq_m']
    rows = np.flatnonzero(abs(q/grid['radius_m']) <= math.radians(30))
    if not len(rows):
        raise ValueError('top sector absent')
    panels = [dict(label=f'{view_lo:g}–{view_hi:g} m | top 60 deg | overview, stride {stride}',
                   images=[raw[rows[0]:rows[-1]+1, first_x:end_x], optimized[rows[0]:rows[-1]+1, first_x:end_x]],
                   source_files=['raw_helix.png', 'geometry/optimized.png'],
                   source_box=[first_x, int(rows[0]), end_x, int(rows[-1]+1)])]
    crops = []
    for kind, label in (('wide', 'Joint candidate'), ('thin', 'Crack candidate')):
        crop = selected[kind]
        names = [f'feature_{crop["id"]}_{state}.png' for state in ('raw', 'optimized')]
        pair = [pixels(n) for n in names]
        if pair[0].shape != pair[1].shape:
            raise ValueError('crop grids differ')
        centre = round((crop['candidate']['q_m']-crop['q_first_m'])/grid['dq_m'])
        first = max(0, min(pair[0].shape[0]-192, centre-96))
        panels.append(dict(label=label+' | 1:1 pixels, approx 0.2 mm/px',
            images=[p[first:first+192] for p in pair], source_files=names,
            source_box=[0, first, pair[0].shape[1], first+192]))
        crops.append(crop)
    width = 1048
    height = 58+sum(40+p['images'][0].shape[0]+16 for p in panels)+32
    canvas = Image.new('RGB', (width,height), '#202124')
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    except OSError:
        font = ImageFont.load_default()
    draw.text((12,14),'RAW HELIX',font=font,fill='white')
    draw.text((548,14),'CALIBRATED + OPTIMIZED',font=font,fill='white')
    positions, y = [], 58
    for panel in panels:
        draw.text((12,y),panel['label'],font=font,fill='#d4d4d4');y+=40
        for column,p in enumerate(panel['images']):
            x = column*536+(512-p.shape[1])//2
            canvas.paste(Image.fromarray(p),(x,y))
            positions.append(dict(source_file=panel['source_files'][column],
                source_box=panel['source_box'],destination_box=[x,y,x+p.shape[1],y+p.shape[0]]))
        y+=panel['images'][0].shape[0]+16
    draw.text((12,y),'Fixed DN 0–255 | no fusion, sharpening or automatic contrast',font=font,fill='#d4d4d4')
    output.mkdir(parents=True, exist_ok=False)
    image_path=output/'reconstruction_comparison.png';canvas.save(image_path)
    stored=np.array(Image.open(image_path).convert('RGB'))
    for pos in positions:
        # Verify the saved figure against original, hash-verified source rectangles.
        source=pixels(pos['source_file']);a,b,c,d=pos['source_box'];x,y,z,w=pos['destination_box']
        if not np.array_equal(stored[y:w,x:z],source[b:d,a:c]):
            raise ValueError('figure changed source pixel values')
    summary=dict(schema='ssb.readme_summary.v1',source=str(features),target_x_m=[lo,hi],
        overview_x_m=[view_lo,view_hi], overview_theta_deg=[float(np.degrees(q[rows[0]]/grid['radius_m'])),float(np.degrees(q[rows[-1]]/grid['radius_m']))],
        selection='first wide/thin public-image candidates with centre inside original target; no evaluation input',
        candidates=crops,panels=positions,source_pixels_identical=True,
        raw_correction=False,within_band_travel_compensation=False,fusion=False,sharpening=False)
    (output/'report.json').write_text(json.dumps(summary,indent=2)+'\n')
    files=sorted(output.iterdir())
    (output/'provenance.json').write_text(json.dumps(stage_record('readme_summary',
        list(dict.fromkeys(inputs+[Path(__file__).resolve()])),files,dict(overview_stride=stride,detail_pixel_scale=1)),indent=2)+'\n')
    print(json.dumps(dict(size=[width,height],candidate_ids=[c['id'] for c in crops],source_pixels_identical=True)))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--features',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--overview-x',type=float,nargs=2,help='crop saved overview only; never change the reconstruction or its acceptance target')
    export(**vars(parser.parse_args()))
