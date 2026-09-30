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
    """Four wide chassis spots directed diagonally outward and slightly down.

    Intensities are Ogre preview settings, not measured photometric quantities.
    Base link origin is 0.3 m above rail top. No collision / texture allocation.
    """
    if not spec.get('preview', {}).get('work_lights_enabled', False):
        return
    from .stage_b_scene import box
    settings=spec['preview']
    down=math.radians(settings.get('work_light_down_deg',15.))
    sideways=math.radians(settings.get('work_light_side_deg',15.))
    inner=math.radians(settings.get('work_light_inner_deg',80.))
    outer=math.radians(settings.get('work_light_outer_deg',100.))
    reach=settings.get('work_light_range_m',8.)
    if not (0<down<math.pi/4 and 0<sideways<math.pi/2 and 0<inner<outer<math.pi and math.isfinite(reach) and reach>0):
        raise ValueError('invalid work light pitch, full cone angles or range')
    for sx in (-1, 1):
        for sy in (-1, 1):
            name=f'work_{sx}_{sy}'
            direction=np.array([sx*math.cos(down)*math.cos(sideways), sy*math.cos(down)*math.sin(sideways), -math.sin(down)])
            direction/=np.linalg.norm(direction)
            x,y,z=sx*.51,sy*.59,.15  # world rail-relative height 0.45 m
            # Put the emitter beyond the deck edge; otherwise the electronics lid
            # blocks the downward beam once real shadow maps are enabled.
            box(base,name+'_post',f'{sx*.36} {y} .075 0 0 0','.025 .025 .19','0.48 0.53 0.59 1')
            box(base,name+'_arm',f'{sx*.43} {y} .17 0 0 0','.16 .025 .022','0.48 0.53 0.59 1')
            # A little hood behind the emission plane; local +z looks out/down.
            pitch=math.acos(direction[2]);yaw=math.atan2(direction[1],direction[0])
            for label,depth,size,color in [('hood',-.027,'.060 .046 .030','0.06 0.07 0.08 1'),
                                            ('glass',-.010,'.048 .034 .004','0.82 0.86 0.89 1')]:
                p=np.array([x,y,z])+direction*depth
                box(base,name+'_'+label,f'{p[0]} {p[1]} {p[2]} 0 {pitch} {yaw}',size,color)
                if label=='glass':
                    face=base.find(f"visual[@name='{name}_glass']")
                    sub(face,'cast_shadows','false')
                    # A rendering light illuminates receivers; the visible lens needs
                    # its own emission to appear lit when viewed head-on.
                    sub(face.find('material'),'emissive','1.0 .96 .90 1')
            light=sub(base,'light',name=name,type='spot')
            sub(light,'pose',f'{x} {y} {z} 0 0 0')
            sub(light,'direction',' '.join(map(str,direction)))
            sub(light,'diffuse','1.0 .96 .90 1');sub(light,'specular','.2 .2 .2 1')
            sub(light,'cast_shadows','true');sub(light,'intensity',2.0);sub(light,'visualize','false')
            attenuation=sub(light,'attenuation')
            for k,v in [('range',reach),('constant',1),('linear',.10),('quadratic',.10)]:sub(attenuation,k,v)
            # Ogre2/SDF use the FULL cone angle; coverage tests use half this value.
            spot=sub(light,'spot');sub(spot,'inner_angle',inner);sub(spot,'outer_angle',outer);sub(spot,'falloff',1)


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
