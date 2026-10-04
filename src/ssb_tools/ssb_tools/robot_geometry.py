"""Nominal assembly dimensions shared by world generation and optical configuration."""
import math


def mount_geometry(config):
    robot = config.get('robot', {})
    base = float(robot.get('base_reference_z_m', .3)) # legacy v1 configuration
    height = float(robot.get('scan_axis_height_m', config['tunnel']['axis_z_m']-base))
    if not all(math.isfinite(x) and x > 0 for x in (base, height)):
        raise ValueError('invalid robot base reference or scan axis height')
    return base, height


def assembly_pose(config):
    """Simulation-only head placement in the model frame, before shaft rotation."""
    base, height = mount_geometry(config)
    truth = config['truth']; mount = truth['mount']
    result = [truth['head_mount_x_m'], mount['dy_m'], base+height+mount['dz_m'],
              0., mount['tilt_y_rad'], mount['tilt_z_rad']]
    if not all(math.isfinite(v) for v in result):
        raise ValueError('nonfinite fixed scanner assembly')
    return result
