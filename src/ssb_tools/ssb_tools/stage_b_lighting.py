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


def add_work_lights(base, spec):
    """Four shielded chassis spots; direct cones stay below the imaging arc.

    Intensities are Ogre preview settings, not measured photometric quantities.
    Base link origin is 0.3 m above rail top. No collision / texture allocation.
    """
    if not spec.get('preview', {}).get('work_lights_enabled', False):
        return
    from .stage_b_scene import box
    for sx in (-1, 1):
        for sy in (-1, 1):
            name=f'work_{sx}_{sy}'
            direction=np.array([sx*.40, sy*.35, -.847])
            direction/=np.linalg.norm(direction)
            x,y,z=sx*.31,sy*.59,.15  # world rail-relative height 0.45 m
            # A little hood behind the emission plane; local +z looks out/down.
            pitch=math.acos(direction[2]);yaw=math.atan2(direction[1],direction[0])
            for label,depth,size,color in [('hood',-.018,'.060 .046 .030','0.06 0.07 0.08 1'),
                                            ('glass',-.001,'.048 .034 .004','0.82 0.86 0.89 1')]:
                p=np.array([x,y,z])+direction*depth
                box(base,name+'_'+label,f'{p[0]} {p[1]} {p[2]} 0 {pitch} {yaw}',size,color)
            light=sub(base,'light',name=name,type='spot')
            sub(light,'pose',f'{x} {y} {z} 0 0 0')
            sub(light,'direction',' '.join(map(str,direction)))
            sub(light,'diffuse','1.0 .96 .90 1');sub(light,'specular','.2 .2 .2 1')
            sub(light,'cast_shadows','true');sub(light,'intensity',1.2);sub(light,'visualize','false')
            attenuation=sub(light,'attenuation')
            for k,v in [('range',4),('constant',1),('linear',.15),('quadratic',.15)]:sub(attenuation,k,v)
            spot=sub(light,'spot');sub(spot,'inner_angle',.35);sub(spot,'outer_angle',.65);sub(spot,'falloff',1)


def apply_work_light_environment(world, spec):
    """Bounded shadow cost: track and vehicle cast; enclosing wall receives."""
    if not spec.get('preview', {}).get('work_lights_enabled', False):
        return
    world.find('scene/ambient').text=' '.join(map(str,[*spec['preview']['ambient_rgb'],1]))
    world.find('scene/shadows').text='true'
    for visual in world.findall("model[@name='tunnel']/link/visual"):
        item=visual.find('cast_shadows')
        if item is None:item=sub(visual,'cast_shadows')
        item.text='false'
