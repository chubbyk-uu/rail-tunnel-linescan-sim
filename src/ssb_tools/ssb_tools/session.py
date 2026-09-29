"""Read a session through its manifests; every table is hash-checked on load."""
import hashlib
import json
from pathlib import Path

import numpy as np


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 22), b''):
            h.update(chunk)
    return h.hexdigest()


def read_json(path):
    with open(path) as f:
        return json.load(f)


def dtype_from(entry):
    fields = []
    for field in entry['dtype']:
        name, kind = field[0], field[1]
        fields.append((name, kind, tuple(field[2])) if len(field) > 2 else (name, kind))
    dt = np.dtype(fields)
    if dt.itemsize != entry['record_size']:
        raise ValueError(f"{entry['file']}: dtype size {dt.itemsize} != record size {entry['record_size']}")
    return dt


class Session:
    def __init__(self, root):
        self.root = Path(root)
        self.summary = read_json(self.root / 'session.json')

    def config(self):
        """Observable configuration: the only settings reconstruction may use."""
        return read_json(self.root / 'config' / 'observable_config.json')

    def _table(self, area, name):
        entry = read_json(self.root / area / 'manifest.json')[name]
        path = self.root / area / entry['file']
        if sha256_file(path) != entry['sha256']:
            raise ValueError(f'hash mismatch: {path}')
        data = np.fromfile(path, dtype=dtype_from(entry))
        if len(data) != entry['count']:
            raise ValueError(f'record count mismatch: {path}')
        return data

    def metadata(self, name):
        return self._table('metadata', name)

    def evaluation(self, name):
        """Truth-side tables. Reconstruction code must never call this."""
        return self._table('evaluation', name)

    def truth(self):
        return read_json(self.root / 'evaluation' / 'truth.json')

    def raw_index(self):
        return read_json(self.root / 'raw' / 'index.json')

    def raw_rows(self, sequences):
        """Pixel rows for the given sequence numbers, verifying each block read."""
        index = self.raw_index()
        width = index['width']
        sequences = np.asarray(sequences)
        out = np.empty((len(sequences), width), np.uint8)
        verified = {}
        for block in index['blocks']:
            first, count = block['first_sequence'], block['rows']
            mask = (sequences >= first) & (sequences < first + count)
            if not mask.any():
                continue
            path = self.root / 'raw' / block['file']
            if block['file'] not in verified:
                if sha256_file(path) != block['sha256']:
                    raise ValueError(f'hash mismatch: {path}')
                verified[block['file']] = True
            data = np.fromfile(path, np.uint8).reshape(count, width)
            out[mask] = data[sequences[mask] - first]
        return out
