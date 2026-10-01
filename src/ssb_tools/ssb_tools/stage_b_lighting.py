"""Bounded shadow-casting GUI lighting; radiometric acquisition stays in OptiX."""
import math

import numpy as np

from .stage_b_scene import sub


def add_strip_light(head, folder, config, spec):
    """Approximate the custom lens with a bounded fan of shadow-casting spots.

    All spots share one exit point. Angular overlap forms a strip, so near-field
    receivers and blockers are handled by Ogre shadow maps instead of decals.
    This is a GUI approximation, not the acquisition OptiX light distribution.
    """
    if not spec.get('preview', {}).get('strip_light_enabled', True):
        return
    robot=spec['robot'];length,width=robot['lamp_wall_footprint_m']
    # Keep the point emitter 6 mm ahead of the visible lens apex. Otherwise its
    # own opaque preview lens would block every beam. No extra emitting faces.
    exit_z=.014
    reach=config['tunnel']['radius_m']-exit_z
    if reach<=0 or not math.isclose(length/width,10.):
        raise ValueError('strip shadow fan requires positive range and 10:1 footprint')
    # 17 shadow maps plus the four chassis spots stay below Ogre2's 25-map cap.
    # Smooth cones have ~10% axial ripple; nominal FWHM is ~1.2 x 0.12 m.
    outer=2*math.atan(width*(.107/.12)/reach)
    for i,x in enumerate(np.linspace(-length*(.64/1.2),length*(.64/1.2),17)):
        direction=np.array([x,0,reach]);direction/=np.linalg.norm(direction)
        weight=math.exp(-math.log(2)*(x/(length*(.62/1.2)))**8)
        light=sub(head,'light',name=f'cob_strip_{i:02d}',type='spot')
        sub(light,'pose',f'{robot["lamp_offset_axial_m"]} 0 {exit_z} 0 0 0')
        sub(light,'direction',' '.join(map(str,direction)))
        sub(light,'diffuse','1.0 .96 .90 1');sub(light,'specular','.1 .1 .1 1')
        sub(light,'intensity',8*weight);sub(light,'cast_shadows','true');sub(light,'visualize','false')
        attenuation=sub(light,'attenuation')
        for k,v in [('range',config['tunnel']['radius_m']+.35),('constant',1),('linear',0),('quadratic',1)]:
            sub(attenuation,k,v)
        spot=sub(light,'spot');sub(spot,'inner_angle',0);sub(spot,'outer_angle',outer);sub(spot,'falloff',1)


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
    outer=math.radians(settings.get('work_light_outer_deg',90.))
    reach=settings.get('work_light_range_m',8.)
    if not (0<down<math.pi/4 and 0<sideways<math.pi/2 and 0<inner<outer<math.pi and math.isfinite(reach) and reach>0):
        raise ValueError('invalid work light pitch, full cone angles or range')
    for sx in (-1, 1):
        for sy in (-1, 1):
            name=f'work_{sx}_{sy}'
            direction=np.array([sx*math.cos(down)*math.cos(sideways), sy*math.cos(down)*math.sin(sideways), -math.sin(down)])
            direction/=np.linalg.norm(direction)
            base_z=float(base.findtext('pose').split()[2])
            x,y,z=sx*.51,sy*.59,.45-base_z # nominal rail-relative emitter height
            # Put the emitter beyond the deck edge; otherwise the electronics lid
            # blocks the downward beam once real shadow maps are enabled.
            box(base,name+'_post',f'{sx*.36} {y} {.375-base_z} 0 0 0','.025 .025 .19','0.48 0.53 0.59 1')
            box(base,name+'_arm',f'{sx*.43} {y} {.47-base_z} 0 0 0','.16 .025 .022','0.48 0.53 0.59 1')
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
