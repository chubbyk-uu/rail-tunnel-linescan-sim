"""Private key provisioning; public identities are computed by the C++ optics model."""
import re
import secrets
import subprocess
from pathlib import Path


def ensure_optical_key(config):
    truth = config['truth']
    if 'optical_key' not in truth:
        truth['optical_key'] = secrets.token_hex(32)
    if not isinstance(truth['optical_key'], str) or not re.fullmatch('[0-9a-f]{64}', truth['optical_key']):
        raise ValueError('truth.optical_key must be 64 lowercase hex characters')


def check_calibration(config, calibration):
    from .package_paths import core_executable
    command = core_executable('ssb_optical_identity')
    return subprocess.run([str(command), '--config', str(config), '--calibration', str(calibration)],
                          check=True, capture_output=True, text=True).stdout.strip()
