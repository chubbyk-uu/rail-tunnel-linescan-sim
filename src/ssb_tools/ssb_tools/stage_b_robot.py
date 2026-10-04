"""Updated machine-photo assembly; optional front-drive wheel/rail contact.

User photos (2026-09-29) supersede the patent head and post arrangement.
80 mm square heatsink and 20 mm square COB are confirmed; remaining dimensions
are provisional engineering estimates, not measurements from the photos.
The two rail-face camera booms are deliberately absent: this project has one line camera.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .stage_b_scene import sub, box, cylinder, inertial, make_joint
from .robot_geometry import mount_geometry, assembly_pose

WHITE = '0.91 0.93 0.95 1'
ORANGE = '1.0 0.235 0.025 1'
DARK = '0.045 0.055 0.07 1'
METAL = '0.48 0.53 0.59 1'
GLASS = '0.025 0.09 0.12 1'


def pose(x=0,y=0,z=0,roll=0,pitch=0,yaw=0):
    return ' '.join(f'{v:.10g}' for v in (x,y,z,roll,pitch,yaw))


def tube(link,name,a,b,radius,color):
    delta=[b[i]-a[i] for i in range(3)]
    length=math.sqrt(sum(d*d for d in delta))
    if length<=0: raise ValueError('zero length robot tube')
    centre=[(a[i]+b[i])/2 for i in range(3)]
    cylinder(link,name,pose(*centre,pitch=math.acos(delta[2]/length),
                           yaw=math.atan2(delta[1],delta[0])),radius,length,color)


def extruded_profile(path,profile,thickness):
    """Convex x/z outline extruded in y, with outward normals on every triangle."""
    vertices=[]; normals=[]
    def triangle(a,b,c):
        u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
        n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
        length=math.sqrt(sum(e*e for e in n))
        if length<1e-12:raise ValueError('degenerate robot mesh')
        vertices.extend((a,b,c));normals.extend([tuple(e/length for e in n)]*3)
    # Ensure counter-clockwise in the x/z plane (normal points toward -y).
    area=sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(profile,profile[1:]+profile[:1]))
    if area<0:profile=list(reversed(profile))
    cx=sum(p[0] for p in profile)/len(profile);cz=sum(p[1] for p in profile)/len(profile)
    for a,b in zip(profile,profile[1:]+profile[:1]):
        am=(a[0],-thickness/2,a[1]);bm=(b[0],-thickness/2,b[1])
        ap=(a[0],thickness/2,a[1]);bp=(b[0],thickness/2,b[1])
        triangle((cx,-thickness/2,cz),am,bm)
        triangle((cx,thickness/2,cz),bp,ap)
        triangle(am,ap,bp);triangle(am,bp,bm)
    with Path(path).open('w') as f:
        f.write('# Convex robot cover; dimensions in metres; visual only\n')
        for p in vertices:f.write('v %.9f %.9f %.9f\n'%p)
        for n in normals:f.write('vn %.9f %.9f %.9f\n'%n)
        for first in range(1,len(vertices)+1,3):
            f.write('f '+' '.join(f'{i}//{i}' for i in range(first,first+3))+'\n')


def mesh_visual(link,name,path,placement,color):
    visual=sub(link,'visual',name=name);sub(visual,'pose',placement)
    sub(sub(sub(visual,'geometry'),'mesh'),'uri',str(Path(path).resolve()))
    material=sub(visual,'material');sub(material,'diffuse',color);sub(material,'ambient',color)


WHEEL_MASS_KG=5.
AXLE_MASS_KG=.5


def wheel_compliance(config,robot):
    """Radial polyurethane-tread compliance as a sprung vertical joint per wheel (truth).

    DART ignores contact kp/kd, so the elasticity lives in a joint. Its stiffness/damping are
    calibrated (ssb_tools.wheel_stiffness) so the realized static deflection equals the assumed
    value; generated configurations record them and the plugin checks them against the SDF.
    """
    entry=(config.get('truth') or {}).get('wheel_compliance')
    if not entry or not config.get('contact',{}).get('enabled'):return None
    deflection=float(entry['static_deflection_m']);zeta=float(entry.get('damping_ratio',.2))
    if not (math.isfinite(deflection) and 1e-4<=deflection<=.002 and math.isfinite(zeta) and 0<zeta<=1):
        raise ValueError('wheel compliance needs static_deflection_m in [0.1, 2] mm and damping_ratio in (0, 1]')
    if 'stiffness_n_m' not in entry or 'damping_n_s_m' not in entry:
        raise ValueError('wheel compliance needs calibrated stiffness_n_m and damping_n_s_m (prepare_contact_demo)')
    k,c=float(entry['stiffness_n_m']),float(entry['damping_n_s_m'])
    if not (math.isfinite(k) and k>0 and math.isfinite(c) and c>0):raise ValueError('invalid wheel compliance values')
    return dict(static_deflection_m=deflection,damping_ratio=zeta,stiffness_n_m=k,damping_n_s_m=c,
                axle_mass_kg=AXLE_MASS_KG,travel_m=.005)


GRAVITY=9.81


def measuring_wheel(config,robot):
    """Validated contact-mode measuring wheel settings, or None outside contact mode.

    Two spring-loaded wheels on vertical guides behind the rear axle carry the odometry
    encoders, so running-wheel lift on a twisted track cannot stop or overspin the odometer.
    The configured odometer diameters (truth and calibration) are these wheels' diameters.
    """
    if not config.get('contact',{}).get('enabled'):return None
    m=dict(robot['measuring_wheel'])
    for section in ('truth','calibration'):
        for key in ('odo_left_diameter_m','odo_right_diameter_m'):
            value=config[section].get(key)
            if value is None or not abs(float(value)/m['diameter_m']-1)<=.05:
                raise ValueError(f'{section}.{key} must be the measuring-wheel diameter '
                                 f'({m["diameter_m"]} m nominal, within 5%); regenerate the contact configuration')
    values=[m[k] for k in ('diameter_m','width_m','preload_n','spring_rate_n_m','damping_n_s_m','travel_m',
                           'slider_mass_kg','wheel_mass_kg')]
    if not all(math.isfinite(v) and v>0 for v in values):raise ValueError('invalid measuring wheel settings')
    half=robot['wheelbase_m']/2
    if not m['x_m']<-half-robot['wheel_diameter_m']/2-m['diameter_m']/2-.01:
        raise ValueError('measuring wheel must sit behind the rear running wheels')
    if m['preload_n']/m['spring_rate_n_m']<=m['travel_m']:
        raise ValueError('measuring wheel spring must stay compressed over the whole slide travel')
    return m


def running_wheel_load_mass(spec):
    """Mass whose weight the four running-wheel springs carry, for stiffness calibration.

    Measuring wheels carry their own weight plus the spring preload, which unloads the body.
    """
    robot=spec['robot'];m=robot['measuring_wheel']
    return (robot['total_mass_kg']-4*(WHEEL_MASS_KG+AXLE_MASS_KG)
            -2*(m['slider_mass_kg']+m['wheel_mass_kg'])-2*m['preload_n']/GRAVITY)


def make_robot(out,config,spec):
    robot=spec['robot']
    base_z, axis_height = mount_geometry(config)
    zc=base_z+axis_height
    contact=config.get('contact',{}).get('enabled',False)
    half=robot['wheelbase_m']/2;diameter=robot['wheel_diameter_m']
    rail_y=(spec['track']['gauge_m']+spec['track']['head_width_m'])/2
    guide_radius=robot.get('guide_bearing_radius_m',.025)
    guide_width=robot.get('guide_bearing_width_m',.024)
    guide_z=-.019  # centre of the existing 38 mm rail head
    if not (.01<=guide_radius<=.04 and .01<=guide_width<=.035):
        raise ValueError('guide bearing must fit the rail-head side')
    cradle=robot['head_cradle_length_m'];floor_depth=robot['head_floor_depth_m'];floor_z=zc-floor_depth
    lamp_x=robot['lamp_offset_axial_m'];sink=robot['lamp_heatsink_width_m']
    if robot['lamp_offset_tangential_m'] or robot['lamp_offset_radial_m']:
        raise ValueError('photo-based head requires parallel coplanar optical exits')
    if abs(lamp_x)+config['camera']['fov_at_nominal_m']/2>robot['lamp_wall_footprint_m'][0]/2:
        raise ValueError('parallel lamp does not cover the camera field')
    if abs(lamp_x)<sink/2+.075/2+.01:
        raise ValueError('lamp heatsink and camera need assembly clearance')
    if not (.5<cradle<1.2 and zc>1 and diameter>0 and robot['total_mass_kg']>35):
        raise ValueError('unsupported robot envelope')
    folder=Path(out)/'robot';folder.mkdir(exist_ok=True)
    # Curved end caps follow the two wheel positions; wheels remain visible outside.
    side_profile=[]
    for centre,start in ((half,-math.pi/2),(-half,math.pi/2)):
        side_profile.extend((centre+.125*math.cos(start+i*math.pi/12),
                             .14+.125*math.sin(start+i*math.pi/12)) for i in range(13))
    extruded_profile(folder/'drive_cover.obj',side_profile,.075)
    # Two separate electronic bays span the transverse bars, one on each side of y=0.
    cover_half=half-.028
    cover_profile=[(-cover_half,.285),(cover_half,.285)]
    cover_profile.extend((cover_half*(1-2*i/20),
                          .335+.075*(1-(1-2*i/20)**2)) for i in range(21))
    bay_inner=.075;bay_outer=rail_y-.12;bay_width=bay_outer-bay_inner
    extruded_profile(folder/'bay_cover.obj',cover_profile,bay_width)
    extruded_profile(folder/'bay_end.obj',cover_profile,.012)
    for sign,label in ((-1,'rear'),(1,'front')):
        extruded_profile(folder/f'head_corner_{label}.obj',
            [(sign*x,z) for x,z in ((cradle/2,-.118),(cradle/2-.016,-.112),
                                     (cradle/2-.083,-floor_depth+.008),
                                     (cradle/2-.075,-floor_depth-.008))],.13)

    car=ET.Element('model',name='scan_car')
    sub(car,'pose',pose(config['motion']['start_x_m']))
    base=sub(car,'link',name='base');sub(base,'pose',pose(z=base_z))
    compliance=wheel_compliance(config,robot)
    axles=4*compliance['axle_mass_kg'] if compliance else 0
    measuring=measuring_wheel(config,robot)
    measuring_mass=2*(measuring['slider_mass_kg']+measuring['wheel_mass_kg']) if measuring else 0
    inertial(base,robot['total_mass_kg']-35-(.8 if contact else 0)-axles-measuring_mass,(4.2,8.1,10.5))
    # Helpers take coordinates in the car frame, converting into the base link.
    def base_box(name,x,y,z,size,color=WHITE,collision=False):
        box(base,name,pose(x,y,z-base_z),size,color,collision)
    def base_cylinder(name,x,y,z,radius,length,color=METAL,roll=0,pitch=0):
        cylinder(base,name,pose(x,y,z-base_z,roll,pitch),radius,length,color)
    def base_tube(name,a,b,radius,color):
        tube(base,name,(a[0],a[1],a[2]-base_z),(b[0],b[1],b[2]-base_z),radius,color)

    for i,x in enumerate((-half,half)):
        base_box(f'crossbar_{i}',x,0,.245,f'.065 {2*rail_y-.08} .065',WHITE,True)
        base_box(f'crossbar_trim_{i}',x,0,.282,f'.055 {2*rail_y-.15} .008',METAL)
    for side,sign in (('left',1),('right',-1)):
        y=sign*(rail_y-.070)
        mesh_visual(base,side+'_drive_box',folder/'drive_cover.obj',pose(y=y,z=-base_z),ORANGE)
        # A shallow inset face and hub rings give the drive box its patent silhouette.
        base_box(side+'_drive_inset',0,y+sign*.041,.157,f'{2*half-.12} .006 .10',WHITE)
        for x in (-half,half):
            base_box(f'{side}_frame_socket_{x}',x,sign*(rail_y-.13),.245,'.10 .09 .095',METAL)
            # Patent [0038], bearings 351/352: vertical axes at the inner rail head.
            # Nominal tangent contact in the ideal guide; visual only, not contact dynamics.
            gy=sign*(spec['track']['gauge_m']/2-guide_radius-(.0002 if contact else 0))
            tag=f'{side}_guide_{x}'
            base_cylinder(tag,x,gy,guide_z,guide_radius,guide_width,METAL)
            # Dark shields inset within the outer race, central sleeve and mounting spindle.
            for face in (-1,1):
                z=guide_z+face*(guide_width/2+.0005)
                base_cylinder(tag+f'_seal_{face}',x,gy,z,guide_radius*.78,.001,DARK)
                base_cylinder(tag+f'_inner_race_{face}',x,gy,z+face*.0006,.010,.0012,METAL)
            base_cylinder(tag+'_spindle',x,gy,.020,.006,.12,METAL)
            base_cylinder(tag+'_retainer',x,gy,guide_z-guide_width/2-.003,.010,.003,METAL)
            base_box(tag+'_mount',x,gy,.054,'.052 .048 .024',WHITE)
            base_cylinder(tag+'_top_nut',x,gy,.070,.010,.008,METAL)
        for i,x in enumerate((-.13,.13)):
            base_tube(f'{side}_handle_leg_{i}',(x,y,.263),(x,y,.335),.012,DARK)
        base_tube(side+'_lifting_handle',(-.13,y,.335),(.13,y,.335),.012,DARK)
        for i,x in enumerate((-.22,0,.22)):
            base_cylinder(f'{side}_case_fastener_{i}',x,y+sign*.042,.21,.008,.006,METAL,roll=math.pi/2)
        yc=sign*(bay_inner+bay_outer)/2
        mesh_visual(base,side+'_electronics_cover',folder/'bay_cover.obj',pose(y=yc,z=-base_z),WHITE)
        for end,ey in (('inner',sign*bay_inner),('outer',sign*bay_outer)):
            mesh_visual(base,f'{side}_{end}_endplate',folder/'bay_end.obj',pose(y=ey,z=-base_z),ORANGE)
        base_box(side+'_bay_latch',0,sign*(bay_outer+.010),.353,'.09 .017 .028',DARK)
        # Low profile orange centre strip, shared profile keeps it flush with the lid.
        # Small label plate has no texture allocation or runtime GUI text rendering.
        base_box(side+'_label_plate',0,yc,.411,'.14 .15 .003',DARK)

    # The remaining central tubes attach outside the compact head, nearly vertical.
    upper_half=robot['post_top_spacing_m']/2
    for label,sign in (('front',1),('rear',-1)):
        bottom=(sign*half,0,.304);top=(sign*upper_half,0,zc+.035)
        base_box(label+'_foot_plate',bottom[0],0,.2865,'.12 .14 .018',ORANGE)
        base_tube(label+'_upright_post',bottom,top,robot['post_diameter_m']/2,WHITE)
        dx=[top[i]-bottom[i] for i in range(3)];length=math.sqrt(sum(v*v for v in dx))
        for name,lo,hi in (('lower',.03,.085),('middle',length/2-.027,length/2+.027),
                           ('upper',length-.075,length-.020)):
            a=tuple(bottom[i]+dx[i]*lo/length for i in range(3))
            b=tuple(bottom[i]+dx[i]*hi/length for i in range(3))
            base_tube(label+'_'+name+'_clamp',a,b,robot['post_diameter_m']/2+.006,
                      ORANGE if name=='middle' else DARK)
        for j,y in enumerate((-.047,.047)):
            base_cylinder(f'{label}_foot_bolt_{j}',bottom[0],y,.299,.008,.008,METAL)
        # Vehicle +x is forward, +y is left. Each brace stays in its own
        # crossbar plane: front post -> left base, rear post -> right base.
        # Base anchors sit on structural crossbars, outside the electronics lids.
        midpoint=tuple((bottom[i]+top[i])/2 for i in range(3))
        foot=(sign*half,sign*(rail_y-.18),.326)
        shoulder=(midpoint[0],sign*.030,midpoint[2])
        base_box(label+'_brace_foot',foot[0],foot[1],.298,'.048 .10 .024',ORANGE)
        base_cylinder(label+'_brace_lower_pin',*foot,.020,.044,METAL,pitch=math.pi/2)
        base_cylinder(label+'_brace_upper_pin',midpoint[0],sign*.022,midpoint[2],
                      .023,.050,METAL,pitch=math.pi/2)
        base_tube(label+'_diagonal_brace',foot,shoulder,robot['brace_diameter_m']/2,WHITE)
    base_box('u_cradle_floor',0,0,floor_z,f'{cradle-.15} .13 .016',WHITE)
    for side,sign in (('front',1),('rear',-1)):
        mesh_visual(base,side+'_cradle_corner',folder/f'head_corner_{side}.obj',pose(z=zc-.3),WHITE)
        base_box(side+'_bearing_plate',sign*(cradle/2-.008),0,zc-.042,'.016 .13 .150',ORANGE)
        base_cylinder(side+'_bearing',sign*cradle/2,0,zc,.035,.022,WHITE,pitch=math.pi/2)
        base_cylinder(side+'_bearing_seal',sign*(cradle/2+.014),0,zc,.022,.008,DARK,pitch=math.pi/2)
        base_tube(side+'_head_attachment',(sign*cradle/2,0,zc),
                  (sign*upper_half,0,zc),.018,METAL)
    # Right-hand slip-ring stator stays fixed; its shaft/rotor belongs to head.
    slip_x=.183
    base_cylinder('slipring',slip_x,0,zc,.040,.125,WHITE,pitch=math.pi/2)
    for sign in (-1,1):
        base_cylinder(f'slipring_end_{sign}',slip_x+sign*.057,0,zc,.042,.010,METAL,pitch=math.pi/2)
    base_box('motor_mount',.16,0,floor_z-.015,'.15 .10 .014',ORANGE)
    base_cylinder('rotation_motor',.16,0,floor_z-.0595,.032,.075,DARK)
    # Covered drive is an envelope estimate, not a measured transmission design.
    guard_bottom=-floor_depth-.035
    base_box('gear_guard',.235,-.083,zc+(guard_bottom+.030)/2,
             f'.070 .014 {.030-guard_bottom}',ORANGE)
    base_cylinder('scan_encoder',cradle/2+.040,0,zc,.023,.025,DARK,pitch=math.pi/2)

    wheels=[]
    for i,(x,y) in enumerate(((-half,rail_y),(half,rail_y),(-half,-rail_y),(half,-rail_y))):
        sign=1 if y>0 else -1
        if contact:
            diameter=config['truth']['wheel_diameter_m']
        name='odometer_wheel' if i==0 else f'wheel_{i}'
        joint='odometer' if i==0 else f'wheel_joint_{i}';wheels.append(joint)
        wheel=sub(car,'link',name=name);sub(wheel,'pose',pose(x,y,robot['wheel_diameter_m']/2))
        inertial(wheel,WHEEL_MASS_KG,(.013,.025,.013))
        cylinder(wheel,'tread',pose(roll=math.pi/2),diameter/2,.060,DARK)
        # No unsupported rail flange: a flush side ring remains inside the tread radius.
        cylinder(wheel,'inner_side_ring',pose(y=-sign*.029,roll=math.pi/2),diameter/2-.005,.002,METAL)
        cylinder(wheel,'hub',pose(y=sign*.032,roll=math.pi/2),diameter*.36,.010,WHITE)
        cylinder(wheel,'hub_cap',pose(y=sign*.041,roll=math.pi/2),diameter*.14,.012,ORANGE)
        for j in range(6):
            a=2*math.pi*j/6
            cylinder(wheel,f'hub_bolt_{j}',pose(diameter*.25*math.cos(a),sign*.04,
                     diameter*.25*math.sin(a),roll=math.pi/2),.005,.007,DARK)
        if i in (1,3):
            base_cylinder(f'hub_motor_{i}',x,y-sign*.081,diameter/2,.063,.071,DARK,roll=math.pi/2)
        if i in (0,2) and not measuring:
            tag='odometer' if i==0 else 'right_odometer'
            base_cylinder(tag+'_drive_gear',x,y-sign*.110,diameter/2,.035,.009,METAL,roll=math.pi/2)
            base_cylinder(tag+'_follow_gear',x+.060,y-sign*.110,diameter/2+.040,.037,.009,METAL,roll=math.pi/2)
            base_cylinder(tag+'_encoder',x+.060,y-sign*.137,diameter/2+.040,.023,.040,DARK,roll=math.pi/2)
        if contact:
            from .stage_b_track import friction
            col=sub(wheel,'collision',name='tread_contact');sub(col,'pose',pose(roll=math.pi/2))
            g=sub(sub(col,'geometry'),'cylinder');sub(g,'radius',diameter/2);sub(g,'length',.060)
            friction(col,1.0)
        parent='base'
        if compliance:
            # base -(sprung vertical slide)- axle -(revolute, encoder/drive)- wheel
            parent=name+'_axle';axle=sub(car,'link',name=parent)
            sub(axle,'pose',pose(x,y,robot['wheel_diameter_m']/2))
            inertial(axle,compliance['axle_mass_kg'],(5e-4,5e-4,5e-4))
            spring=sub(car,'joint',name=joint+'_suspension',type='prismatic')
            sub(spring,'parent','base');sub(spring,'child',parent)
            axis=sub(spring,'axis');sub(axis,'xyz','0 0 1');limit=sub(axis,'limit')
            sub(limit,'lower',-compliance['travel_m']);sub(limit,'upper',compliance['travel_m'])
            dyn=sub(axis,'dynamics');sub(dyn,'spring_reference',0)
            sub(dyn,'spring_stiffness',f"{compliance['stiffness_n_m']:.12g}");sub(dyn,'damping',f"{compliance['damping_n_s_m']:.12g}")
        make_joint(car,joint,'revolute',parent,name,'0 1 0')

    if measuring:
        add_measuring_wheels(car,base,base_z,config,robot,rail_y,measuring)

    # Under-body battery/drive compartment between the crossbars and the drive boxes, closing the
    # open space below the electronics bays. Visual only: its contents are already part of the
    # base mass, and it collides with nothing (bottom ~30 mm above the rail top, 200 mm above the bed).
    # Envelope is an estimate, not a measured enclosure.
    bay_bottom=.285;belly_x=half-.0325;belly_y=rail_y-.07-.0375-.0265;belly_z=.04
    base_box('belly_case',0,0,(belly_z+bay_bottom)/2,f'{2*belly_x:.6g} {2*belly_y:.6g} {bay_bottom-belly_z:.6g}',WHITE)
    base_box('belly_skid_plate',0,0,belly_z-.006,f'{2*belly_x-.03:.6g} {2*belly_y-.03:.6g} .012',ORANGE)
    for label,sign in (('front',1),('rear',-1)):
        for k,y in enumerate((-.3,.3)):
            base_box(f'belly_{label}_vent_{k}',sign*(belly_x+.002),y,.15,'.004 .42 .09',DARK)
    for side,sign in (('left',1),('right',-1)):
        base_box(side+'_belly_access_lid',0,sign*(belly_y+.0015),.165,f'{2*belly_x-.16:.6g} .003 .17',METAL)
        for k,x in enumerate((-.16,.16)):
            base_box(f'{side}_belly_latch_{k}',x,sign*(belly_y+.005),.235,'.04 .006 .02',DARK)

    head=sub(car,'link',name='head');sub(head,'pose',pose(*assembly_pose(config)))
    inertial(head,15,(.25,.20,.20))
    # Ideal thin-lens/pinhole baseline: the projection centre, NOT the front
    # glass or sensor, lies on the shaft. Derive image distance from the same
    # calibrated projection constant as Config::FocalLength(). No glass refraction
    # is simulated; the barrel envelope is an independent visual estimate.
    camera=config['camera']
    image_distance=(camera['pixel_pitch_m']*camera['nominal_distance_m']*
                    camera['width']/camera['fov_at_nominal_m'])
    glass_front=robot['camera_front_glass_offset_m']
    if not (0<glass_front<.12 and 0<image_distance<.12):
        raise ValueError('camera optics exceed the supported head envelope')
    body_front=-image_distance+.018 # estimated flange-to-sensor setback
    body_centre=body_front-.037
    box(head,'camera',pose(z=body_centre),'.075 .070 .074',ORANGE)
    box(head,'camera_connector',pose(z=body_front-.083),'.034 .036 .018',DARK)
    cylinder(head,'lens_barrel',pose(z=(body_front+glass_front)/2),.026,glass_front-body_front,DARK)
    cylinder(head,'lens_focus_ring',pose(z=body_front+.022),.030,.015,METAL)
    cylinder(head,'lens_bezel',pose(z=glass_front-.005),.032,.010,DARK)
    cylinder(head,'lens_glass',pose(z=glass_front-.0005),.027,.001,GLASS)
    optical=sub(car,'frame',name='camera_optical',attached_to='head');sub(optical,'pose','0 0 0 0 0 0')
    sensor=sub(car,'frame',name='camera_sensor',attached_to='head')
    sub(sensor,'pose',pose(z=-image_distance))
    front=sub(car,'frame',name='camera_front_glass',attached_to='head')
    sub(front,'pose',pose(z=glass_front))
    for side,sign in (('left',1),('right',-1)):
        box(head,side+'_camera_clamp',pose(y=sign*.045,z=body_centre),'.083 .009 .078',WHITE)
        # Small casing witness marks identify the internal image-plane height.
        box(head,side+'_sensor_plane_mark',pose(y=sign*.0355,z=-image_distance),'.045 .001 .0015',WHITE)
    # One rotor carries both optical modules; no shaft through the camera aperture.
    left_arm=lamp_x-sink/2-.014;right_arm=.060
    box(head,'rotor_tray',pose(x=(left_arm+right_arm)/2,z=-.175),
        f'{right_arm-left_arm+.014} .108 .012',ORANGE)
    for label,x in (('rear',left_arm),('front',right_arm)):
        tube(head,label+'_rotor_arm',(x,0,-.169),(x,0,0),.008,WHITE)
    cylinder(head,'rear_shaft',pose(x=(-cradle/2+left_arm)/2,pitch=math.pi/2),.012,
             left_arm+cradle/2,METAL)
    cylinder(head,'front_shaft',pose(x=(right_arm+cradle/2)/2,pitch=math.pi/2),.012,
             cradle/2-right_arm,METAL)
    cylinder(head,'slipring_rotor',pose(x=.110,pitch=math.pi/2),.023,.020,METAL)
    # Confirmed 80 x 80 mm heatsink and 20 x 20 mm COB. Axial heights and lens
    # diameter are photo-based estimates. Fins/fan remain visuals on the same link.
    box(head,'lamp_heatsink_base',pose(lamp_x,0,-.034),f'{sink} {sink} .012',METAL)
    for i in range(11):
        x=lamp_x-sink/2+.002+i*(sink-.004)/10
        box(head,f'lamp_fin_{i}',pose(x,0,-.059),f'.002 {sink} .038',METAL)
    box(head,'lamp_fan_frame',pose(lamp_x,0,-.087),f'{sink} {sink} .018',DARK)
    cylinder(head,'lamp_fan_hub',pose(lamp_x,0,-.099),.011,.009,DARK)
    for i in range(5):
        a=2*math.pi*i/5
        box(head,f'lamp_fan_blade_{i}',pose(lamp_x+.020*math.cos(a),.020*math.sin(a),-.098,yaw=a+.4),
            '.030 .010 .003',METAL)
    box(head,'lamp_cob',pose(lamp_x,0,-.026),
        f'{robot["lamp_length_m"]} {robot["lamp_width_m"]} .002','0.95 0.82 0.25 1')
    cylinder(head,'lamp_lens_retainer',pose(lamp_x,0,-.016),.043,.018,DARK)
    # A scaled sphere is an inexpensive convex custom-lens visual, not raytraced glass.
    visual=sub(head,'visual',name='lamp_window');sub(visual,'pose',pose(lamp_x,0,-.004))
    geometry=sub(visual,'geometry');ellipsoid=sub(geometry,'ellipsoid');sub(ellipsoid,'radii','.038 .038 .012')
    material=sub(visual,'material');sub(material,'diffuse','0.58 0.77 0.79 1')
    sub(material,'ambient','0.58 0.77 0.79 1');sub(material,'emissive','1.0 0.96 0.90 1')
    # Expose the shared effective exit plane separately from the lens glass surface.
    lamp_frame=sub(car,'frame',name='lamp_optical',attached_to='head')
    sub(lamp_frame,'pose',pose(lamp_x))
    for x in (lamp_x-.029,lamp_x+.029):
        tube(head,f'lamp_mount_{x}',(x,0,-.169),(x,0,-.106),.006,ORANGE)
    from .stage_b_lighting import add_strip_light, add_work_lights
    add_strip_light(head,folder,config,spec)
    add_work_lights(base,spec)
    # The bearing/cradle/stator assembly shares the fixed mount transform.
    # Rotor parts and lights already inherit it through the head link.
    import numpy as np
    import warnings
    from scipy.spatial.transform import Rotation
    placement = assembly_pose(config)
    rotation = Rotation.from_euler('xyz', placement[3:])
    cradle_names = ('u_cradle_', 'front_cradle_', 'rear_cradle_', 'front_bearing',
                    'rear_bearing', 'front_head_attachment', 'rear_head_attachment',
                    'slipring', 'motor_mount', 'rotation_motor', 'gear_guard', 'scan_encoder')
    for visual in base.findall('visual'):
        if not visual.get('name').startswith(cradle_names):
            continue
        node = visual.find('pose'); old = np.array(list(map(float, node.text.split())))
        local = old[:3]-np.array([0., 0., axis_height])
        xyz = rotation.apply(local)+np.array(placement[:3])-[0., 0., base_z]
        # Some nominal bearing cylinders are exactly at Euler pitch pi/2.
        # The equivalent Euler triple is nonunique; the rotation is unchanged.
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', message='Gimbal lock detected')
            angles = (rotation*Rotation.from_euler('xyz', old[3:])).as_euler('xyz')
        node.text = pose(*xyz, *angles)
    for visual in car.findall('link/visual'):
        # Uniform rough finishes, small low-poly primitives; no extra texture maps.
        material=visual.find('material')
        if material is not None:
            metal=sub(sub(material,'pbr'),'metal');sub(metal,'metalness',.08);sub(metal,'roughness',.52)
    if contact:
        from .stage_b_track import friction
        # Move bearing races (not mounting spindles) from chassis visuals into free links.
        for side,sign in [('left',1),('right',-1)]:
            for x in (-half,half):
                tag=f'{side}_guide_{x}'
                gy=sign*(spec['track']['gauge_m']/2-guide_radius-.0002)
                bearing=sub(car,'link',name=tag);sub(bearing,'pose',pose(x,gy,guide_z))
                inertial(bearing,.2,(.00004,.00004,.000063))
                for visual in list(base.findall('visual')):
                    n=visual.get('name')
                    if n==tag or n.startswith(tag+'_seal') or n.startswith(tag+'_inner_race'):
                        base.remove(visual);coords=list(map(float,visual.findtext('pose').split()))
                        coords[0]-=x;coords[1]-=gy;coords[2]+=base_z-guide_z
                        visual.find('pose').text=' '.join(map(str,coords));bearing.append(visual)
                col=sub(bearing,'collision',name='guide_contact');g=sub(sub(col,'geometry'),'cylinder')
                sub(g,'radius',guide_radius);sub(g,'length',guide_width);friction(col,.5)
                make_joint(car,tag+'_joint','revolute','base',tag,'0 0 1')
    else:
        make_joint(car,'carriage','prismatic','world','base','1 0 0')
    make_joint(car,'scan','revolute','base','head','-1 0 0')
    plugin=sub(car,'plugin',filename='ssb_gazebo_scan',name='ssb_gazebo::ContactSystem' if contact else 'ssb_gazebo::ScanSystem')
    if contact:
        cal=config['calibration'];settings=config['contact']
        for tag,value in [('left_encoder','measure_left'),('right_encoder','measure_right'),
                          ('left_drive','wheel_joint_1'),('right_drive','wheel_joint_3'),
                          ('left_diameter',cal['odo_left_diameter_m']),('right_diameter',cal['odo_right_diameter_m']),
                          ('drive_diameter',cal['wheel_diameter_m']),('settle_s',settings.get('settle_s',2.0))]:sub(plugin,tag,value)
    else:
        for tag,name in (('carriage_joint','carriage'),('scan_joint','scan'),('wheel_joint','odometer')):sub(plugin,tag,name)
        for name in wheels[1:]:sub(plugin,'follower_wheel_joint',name)
    return car


def add_measuring_wheels(car,base,base_z,config,robot,rail_y,m):
    """Measuring wheels: base -(sprung vertical slide, preloaded)- slider -(revolute, encoder)- wheel.

    The bracket hangs from the rear crossbar: a horizontal arm, a vertical mount plate with a
    linear guide rail and a spring seat. The slider carries the guide carriage, a leg down to the
    axle, the encoder (inboard, coaxial) and the axle stub; the wheel runs on the rail-head top.
    """
    from .stage_b_track import friction
    half=robot['wheelbase_m']/2;x=m['x_m'];travel=m['travel_m']
    for side,sign in (('left',1),('right',-1)):
        diameter=config['truth'][f'odo_{side}_diameter_m'];r=diameter/2
        def fixed(name,dx,inboard,z,size,color):
            box(base,f'measure_{side}_{name}',pose(x+dx,sign*(rail_y-inboard),z-base_z),size,color)
        # Fixed bracket on the base (car-frame heights above the nominal rail top).
        arm_start=-half-.0325
        fixed('arm',(arm_start-(x-.03))/2+(x-.03)-x,.052,.245,f'{arm_start-(x-.03):.6g} .03 .04',ORANGE)
        fixed('mount_plate',0,.052,.165,'.06 .008 .17',ORANGE)
        fixed('guide_rail',0,.044,.15,'.015 .008 .13',METAL)
        fixed('spring_seat',0,.036,.232,'.035 .032 .012',ORANGE)
        cylinder(base,f'measure_{side}_spring',pose(x,sign*(rail_y-.030),.193-base_z),.007,.066,METAL)
        # Moving slider; its frame is the wheel centre at the nominal height.
        slider=sub(car,'link',name=f'measure_{side}_slider');sub(slider,'pose',pose(x,sign*rail_y,r))
        inertial(slider,m['slider_mass_kg'],(4e-4,4e-4,2e-4))
        box(slider,'carriage',pose(0,-sign*.030,.145-r),'.035 .02 .03',DARK)
        box(slider,'leg',pose(0,-sign*.026,.085-r),'.03 .012 .10',ORANGE)
        cylinder(slider,'axle',pose(y=-sign*.0115,roll=math.pi/2),.006,.017,METAL)
        cylinder(slider,'encoder',pose(y=-sign*.0445,roll=math.pi/2),.018,.025,DARK)
        cylinder(slider,'encoder_cap',pose(y=-sign*.0575,roll=math.pi/2),.012,.002,METAL)
        wheel=sub(car,'link',name=f'measure_{side}_wheel');sub(wheel,'pose',pose(x,sign*rail_y,r))
        mass=m['wheel_mass_kg'];inertial(wheel,mass,(mass*(3*r*r+m['width_m']**2)/12,mass*r*r/2,mass*(3*r*r+m['width_m']**2)/12))
        cylinder(wheel,'tread',pose(roll=math.pi/2),r,m['width_m'],DARK)
        cylinder(wheel,'hub',pose(roll=math.pi/2),r*.45,m['width_m']+.004,METAL)
        # Collision is a sphere of the tread radius: the crowned rail head makes a near-point contact,
        # and DART's (ODE) cylinder-heightfield test let a lightly loaded 80 mm cylinder sink up to
        # 5.6 mm at a grid line when nearly stopped; sphere-heightfield contact is robust.
        col=sub(wheel,'collision',name='tread_contact')
        sub(sub(sub(col,'geometry'),'sphere'),'radius',r);friction(col,1.0)
        # Spring force on the slider: -k (q - q_ref); q_ref = -preload/k gives the preload at q = 0.
        joint=sub(car,'joint',name=f'measure_{side}_slide',type='prismatic')
        sub(joint,'parent','base');sub(joint,'child',f'measure_{side}_slider')
        axis=sub(joint,'axis');sub(axis,'xyz','0 0 1');limit=sub(axis,'limit')
        sub(limit,'lower',-travel);sub(limit,'upper',travel)
        dyn=sub(axis,'dynamics');sub(dyn,'spring_reference',f"{-m['preload_n']/m['spring_rate_n_m']:.12g}")
        sub(dyn,'spring_stiffness',f"{m['spring_rate_n_m']:.12g}");sub(dyn,'damping',f"{m['damping_n_s_m']:.12g}")
        make_joint(car,f'measure_{side}','revolute',f'measure_{side}_slider',f'measure_{side}_wheel','0 1 0')
