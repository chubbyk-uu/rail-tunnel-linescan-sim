"""Hash-verified streaming Mono8 quality statistics from public raw observations."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from .provenance import stage_record
from .public_capture import confined_file
from .session import Session, sha256_file
from .stage_b_scene import peak_rss_bytes


def inspect(root, output):
    started = time.monotonic()
    root = Path(root).resolve(); output = Path(output).resolve()
    if output.is_relative_to(root) and not output.is_relative_to(root/'reconstruction'):
        raise ValueError('quality output inside a capture must stay in reconstruction/')
    if output.exists():
        raise ValueError('refuse to overwrite raw quality results')
    session = Session(root)
    if session.summary.get('status') != 'complete':
        raise ValueError('completed capture required')
    index_path = confined_file(root/'raw', 'index.json')
    if session.summary.get('files', {}).get('raw/index.json') != sha256_file(index_path):
        raise ValueError('raw index identity mismatch')
    index = session.raw_index(); width = index['width']
    if not isinstance(width, int) or not 1 <= width <= 65536:
        raise ValueError('unsupported raw width')
    histogram = np.zeros(256, np.int64); rows = 0; saturated_rows = 0
    inputs = [root/'session.json', index_path]
    # A few large reads per block; no per-row open/hash/status operations.
    chunk_rows = min(1024, max(1, (1 << 22)//width))
    for block in index['blocks']:
        if block['first_sequence'] != rows or block['rows'] <= 0:
            raise ValueError('raw block gap, overlap or empty block')
        path = confined_file(root/'raw', block['file'])
        if path.stat().st_size != block['rows']*width:
            raise ValueError('raw block size mismatch')
        digest = hashlib.sha256()
        with path.open('rb') as file:
            for payload in iter(lambda: file.read(chunk_rows*width), b''):
                digest.update(payload)
                values = np.frombuffer(payload, np.uint8)
                histogram += np.bincount(values, minlength=256)
                saturated_rows += int(np.any(values.reshape(-1, width) == 255, axis=1).sum())
        if digest.hexdigest() != block['sha256']:
            raise ValueError('raw block hash mismatch: '+block['file'])
        rows += block['rows']; inputs.append(path)
    if not rows or rows != session.summary['rows']:
        raise ValueError('raw row count mismatch')
    total = int(histogram.sum()); cumulative = np.cumsum(histogram)
    percentiles = {name: int(np.searchsorted(cumulative, max(1, int(np.ceil(total*fraction)))))
                   for name, fraction in [('p50', .5), ('p99', .99), ('p999', .999)]}
    report = dict(schema='ssb.raw_quality.v1', status='fail' if histogram[255] else 'pass',
        scope='all recorded public Mono8 pixels; saturation sentinel 255', rows=rows, width=width,
        pixels=total, histogram=histogram.tolist(), percentiles_dn=percentiles,
        minimum_dn=int(np.flatnonzero(histogram)[0]), maximum_dn=int(np.flatnonzero(histogram)[-1]),
        saturated_pixels=int(histogram[255]), saturated_rows=saturated_rows,
        saturation_fraction=float(histogram[255]/total),
        performance=dict(wall_s=time.monotonic()-started, peak_rss_bytes=peak_rss_bytes()),
        limitations=['no claim of SNR, dynamic-range calibration or noise robustness',
                     'no exposure compensation or rewriting of raw pixels'])
    output.mkdir(parents=True)
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    # Block hashes were checked during the same streaming pass above. Reuse them
    # for provenance rather than reading GiB of raw again.
    provenance = stage_record('raw_quality', inputs[:2], [output/'report.json'], dict(chunk_rows=chunk_rows))
    provenance['inputs'].update({str(root/'raw'/b['file']): b['sha256'] for b in index['blocks']})
    (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True); parser.add_argument('--output', required=True)
    args = parser.parse_args(); report = inspect(args.session, args.output)
    print(json.dumps({k: report[k] for k in ('status','rows','pixels','maximum_dn','saturated_pixels','performance')}))
    if report['status'] != 'pass':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
