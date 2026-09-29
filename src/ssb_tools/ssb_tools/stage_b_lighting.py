"""Bounded Ogre2 strip-beam preview; radiometric acquisition stays in OptiX.

Ogre2 projectors are rectangular emissive/decal volumes, not shadow-casting
perspective lights. Match the nominal wall footprint and document this limit.
"""
import math
from pathlib import Path

import numpy as np
from PIL import Image

from .stage_b_scene import sub


def add_strip_projector(head, folder, config, spec):
    if not spec.get('preview', {}).get('strip_light_enabled', True):
        return
    robot = spec['robot']
    length, width = robot['lamp_wall_footprint_m']
    # 1.35 FWHM support; tails at the texture border are <0.0005 of peak.
    extent = 1.35
    nx, nq = 1280, 128
    if not math.isclose(length/width, nx/nq):
        raise ValueError('strip preview texture aspect must match the beam')
    x = ((np.arange(nx)+.5)/nx-.5)*2*extent
    q = ((np.arange(nq)+.5)/nq-.5)*2*extent
    beam = np.exp(-math.log(2)*(x[None, :]**8+q[:, None]**8))
    rgba = np.full((nq, nx, 4), 255, np.uint8)
    rgba[:, :, 3] = np.rint(beam*170).astype(np.uint8)
    texture = Path(folder)/'strip_beam.png'
    Image.fromarray(rgba).save(texture)
    # Limit the volume to the lining. The same attached head transform moves
    # both the projector and the physical COB assembly with car and rotation.
    radius = config['tunnel']['radius_m']
    near, far = radius-.10, radius+.10
    projector = sub(head, 'projector', name='cob_strip_preview')
    # Projector +x points along head +z; its image width lies along the track x.
    sub(projector, 'pose', f"{robot['lamp_offset_axial_m']} 0 0 {math.pi/2} {-math.pi/2} 0")
    sub(projector, 'texture', str(texture.resolve()))
    sub(projector, 'fov', 2*math.atan(length*extent/(2*far)))
    sub(projector, 'near_clip', near)
    sub(projector, 'far_clip', far)
    sub(projector, 'visibility_flags', 4294967295)
