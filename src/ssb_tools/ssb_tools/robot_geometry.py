"""Nominal assembly dimensions shared by world generation and optical configuration."""
import math


def mount_geometry(config):
    robot = config.get('robot', {})
    base = float(robot.get('base_reference_z_m', .3)) # legacy v1 configuration
    height = float(robot.get('scan_axis_height_m', config['tunnel']['axis_z_m']-base))
    if not all(math.isfinite(x) and x > 0 for x in (base, height)):
        raise ValueError('invalid robot base reference or scan axis height')
    return base, height
