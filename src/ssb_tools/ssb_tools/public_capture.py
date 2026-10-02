"""Restricted reader for reconstruction: public manifests, pixels and measured calibration."""
from pathlib import Path
import numpy as np
from .session import Session, read_json, sha256_file
from .unroll import row_axis_coordinates
from .wall_coverage import calibrated_spans, calibrated_row_footprint


def confined_file(base, name, required=True):
    path = Path(base)/name
    if not path.resolve().is_relative_to(Path(base).absolute()):
        raise ValueError('dependency escapes the public directory: '+str(name))
    if required and not path.is_file(): raise ValueError('missing public input: '+str(path))
    return path


class PublicCapture:
    def __init__(self, root, calibration_path):
        self.root = Path(root).resolve()
        confined_file(self.root, 'session.json')
        session = Session(self.root)
        summary = session.summary
        if summary.get('status') != 'complete' or not summary.get('motion', {}).get('complete'):
            raise ValueError('initial unroll requires a completed planned capture')
        self.inputs = [self.root/'session.json']
        for name in ('config/observable_config.json', 'metadata/manifest.json', 'raw/index.json'):
            relative = Path(name)
            path = confined_file(self.root/relative.parent, relative.name)
            expected = summary.get('files', {}).get(name)
            if not expected or sha256_file(path) != expected:
                raise ValueError('public manifest identity mismatch: '+name)
            self.inputs.append(path)
        self.config = session.config()
        self.calibration_path = Path(calibration_path).resolve()
        self.calibration = read_json(self.calibration_path)
        if self.calibration.get('schema') != 'ssb.measured_optical_calibration.v1':
            raise ValueError('image-measured optical calibration required')
        if self.config['camera']['optical_signature'] != self.calibration['optical_signature']:
            raise ValueError('session optical signature mismatch')
        self.inputs.append(self.calibration_path)
        # Validate measured geometry without consulting rendering coefficients.
        calibrated_spans(self.config, self.calibration)
        self.footprint = calibrated_row_footprint(self.config, self.calibration)
        manifest = read_json(self.root/'metadata/manifest.json')
        names = ['rows', 'scan_edges', 'odometer_edges', 'gate_events']
        if self.config.get('contact', {}).get('enabled'): names.append('odometer_right_edges')
        tables = {}
        for name in names:
            path = confined_file(self.root/'metadata', manifest[name]['file'])
            self.inputs.append(path)
            tables[name] = session.metadata(name)
        self.rows = tables['rows']
        if not len(self.rows) or len(self.rows) != summary['rows']:
            raise ValueError('empty or incomplete exposure rows')
        if not np.array_equal(self.rows['sequence'], np.arange(len(self.rows))):
            raise ValueError('exposure sequences are incomplete or repeated')
        if not np.all(np.diff(self.rows['t_center']) > 0):
            raise ValueError('exposure times must increase')
        self.x_axis, self.theta = row_axis_coordinates(self.config, self.rows,
            tables['scan_edges'], tables['odometer_edges'], tables['gate_events'],
            tables.get('odometer_right_edges'))
        if not np.isfinite(self.x_axis).all() or not np.isfinite(self.theta).all():
            raise ValueError('nonfinite public projection')
        self.raw_index = session.raw_index()
        self.width = int(self.config['camera']['width'])
        if self.raw_index['width'] != self.width: raise ValueError('raw camera width mismatch')
        first = 0
        for block in self.raw_index['blocks']:
            confined_file(self.root/'raw', block['file'], required=False)
            if block['first_sequence'] != first or block['rows'] <= 0:
                raise ValueError('raw blocks have a gap, overlap or empty block')
            first += block['rows']
        if first != len(self.rows): raise ValueError('raw rows do not match exposure table')
