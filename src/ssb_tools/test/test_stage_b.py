import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from ssb_tools.stage_b_scene import load_spec, make_meshes, make_world, resource_plan
from ssb_tools.stage_b_materials import apply_transform, colour_transform, image_info, srgb_to_linear, linear_to_srgb, sources_match_spec
from ssb_tools.stage_b_cracks import sample_widths, trace_skeleton, sample_types, sample_branches, calibrate_long, coverage_patch


ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def inputs():
    return (yaml.safe_load((ROOT/'src/ssb_core/config/stage_b.yaml').read_text()),
            load_spec(ROOT/'src/ssb_tools/config/stage_b_scene.yaml'))


def test_batch_texture_budget_is_bounded_and_not_full_scene(inputs):
    config,spec = inputs
    plan = resource_plan(config,spec)
    assert plan['active_batch_tiles']*plan['texture_tile_bytes'] <= spec['resources']['gpu_texture_budget_bytes']
    longitudinal_tiles = math.ceil((config['tunnel']['x_max_m']-config['tunnel']['x_min_m']) /
                                   (spec['materials']['tile_core_pixels']*spec['materials']['background_texel_m']))
    assert plan['circumference_tiles']*longitudinal_tiles > plan['texture_tiles_in_budget']
    assert plan['spare_prefetch_tiles'] >= 1
    tiny = copy.deepcopy(spec)
    tiny['resources']['gpu_texture_budget_bytes'] = 1024
    with pytest.raises(ValueError,match='one active batch'):
        resource_plan(config,tiny)
    assert math.isclose(plan['longitudinal_pixel_m'],.0002080711111111111)
    assert math.isclose(plan['nominal_speed_m_s'],.2)


def test_preview_world_has_no_wall_collision_and_camera_at_axis(tmp_path,inputs):
    config,spec = inputs
    make_world(tmp_path,config,spec)
    root = ET.parse(tmp_path/'world.sdf').getroot()
    tunnel = root.find("world/model[@name='tunnel']")
    assert not tunnel.findall('.//collision')
    car = root.find("world/model[@name='scan_car']")
    mass = sum(float(n.text) for n in car.findall('link/inertial/mass'))
    assert mass == 120
    base=car.find("link[@name='base']")
    assert base.find("visual[@name='mast']") is None
    for name in ('front_upright_post','rear_upright_post','left_drive_box','right_drive_box',
                 'crossbar_0','crossbar_1','u_cradle_floor','slipring','rotation_motor'):
        assert base.find(f"visual[@name='{name}']") is not None
    head = car.find("link[@name='head']")
    assert float(head.find('pose').text.split()[2]) == config['tunnel']['axis_z_m']
    optical=car.find("frame[@name='camera_optical']")
    assert optical.get('attached_to')=='head'
    assert all(float(v)==0 for v in optical.find('pose').text.split())
    sensor_z=float(car.find("frame[@name='camera_sensor']/pose").text.split()[2])
    front_z=float(car.find("frame[@name='camera_front_glass']/pose").text.split()[2])
    assert sensor_z<0<front_z
    # Same ideal 90 mm lens equation as the imaging baseline, without putting
    # either physical surface at the projection origin.
    assert 1/.09 == pytest.approx(1/config['camera']['nominal_distance_m']+1/(-sensor_z))
    body=head.find("visual[@name='camera']")
    body_z=float(body.find('pose').text.split()[2])
    body_height=float(body.find('geometry/box/size').text.split()[2])
    assert body_z-body_height/2 < sensor_z < body_z+body_height/2
    lens=head.find("visual[@name='lens_barrel']")
    lens_z=float(lens.find('pose').text.split()[2]);length=float(lens.find('geometry/cylinder/length').text)
    assert lens_z-length/2 < 0 < lens_z+length/2
    assert not root.findall('.//light') # no fixed omnidirectional lights
    projector=head.find("projector[@name='cob_strip_preview']")
    assert projector is not None
    from PIL import Image
    beam=np.asarray(Image.open(projector.find('texture').text))[:,:,3]
    fov=float(projector.find('fov').text);far=float(projector.find('far_clip').text)
    physical_width=2*far*math.tan(fov/2)
    halfmax=beam.max()/2
    assert (beam[beam.shape[0]//2]>=halfmax).sum()/beam.shape[1]*physical_width==pytest.approx(1.2,abs=.004)
    assert (beam[:,beam.shape[1]//2]>=halfmax).sum()/beam.shape[0]*physical_width/10==pytest.approx(.12,abs=.004)
    lp=list(map(float,car.find("frame[@name='lamp_optical']/pose").text.split()))
    assert lp[0]==pytest.approx(spec['robot']['lamp_offset_axial_m'])
    assert lp[1:]==[0]*5 # common axial row, parallel optical axes
    assert abs(lp[0])+config['camera']['fov_at_nominal_m']/2 < spec['robot']['lamp_wall_footprint_m'][0]/2
    for name in ('front_upright_post','rear_upright_post'):
        pp=list(map(float,base.find(f"visual[@name='{name}']/pose").text.split()))
        assert abs(pp[4]) < math.radians(2)
    sink=head.find("visual[@name='lamp_heatsink_base']/geometry/box/size")
    assert list(map(float,sink.text.split()))[:2]==[.08,.08]
    cob=head.find("visual[@name='lamp_cob']/geometry/box/size")
    assert list(map(float,cob.text.split()))[:2]==[.02,.02]
    # Full-turn clearance: all rotating primitives must stay above the U floor's
    # swept-radius limit and inside the two fixed end plates.
    floor=base.find("visual[@name='u_cradle_floor']")
    floor_z=float(base.find('pose').text.split()[2])+float(floor.find('pose').text.split()[2])
    floor_half=float(floor.find('geometry/box/size').text.split()[2])/2
    clearance=config['tunnel']['axis_z_m']-floor_z-floor_half
    assert clearance==pytest.approx(.207)
    for label,sign in (('front',1),('rear',-1)):
        brace=base.find(f"visual[@name='{label}_diagonal_brace']")
        x,y,z,_,pitch,yaw=map(float,brace.find('pose').text.split())
        half_length=float(brace.find('geometry/cylinder/length').text)/2
        delta=np.array([math.sin(pitch)*math.cos(yaw),math.sin(pitch)*math.sin(yaw),math.cos(pitch)])*half_length
        foot=np.array([x,y,z+.3])-delta;shoulder=np.array([x,y,z+.3])+delta
        assert sign*foot[0]==pytest.approx(spec['robot']['wheelbase_m']/2)
        assert sign*foot[1]>.5 and sign*shoulder[1]==pytest.approx(.03)
        assert foot[2]<.35 and 1.1<shoulder[2]<1.25
        assert shoulder[2]+spec['robot']['brace_diameter_m']/2 < config['tunnel']['axis_z_m']-clearance
    for v in head.findall('visual'):
        x,y,z,_,pitch,_=map(float,v.find('pose').text.split())
        geom=v.find('geometry')
        if geom.find('box') is not None:
            sx,sy,sz=map(float,geom.find('box/size').text.split())
            radius=math.hypot(abs(y)+sy/2,abs(z)+sz/2)
            assert abs(x)+sx/2 < spec['robot']['head_cradle_length_m']/2-.016
        elif geom.find('cylinder') is not None:
            r=float(geom.find('cylinder/radius').text);h=float(geom.find('cylinder/length').text)
            radius=math.hypot(y,z)+ (r if abs(pitch-math.pi/2)<1e-6 else math.hypot(r,h/2))
        else:
            radius=math.hypot(y,z)+max(map(float,geom.find('ellipsoid/radii').text.split()))
        assert radius < clearance, v.get('name')
    slipring=base.find("visual[@name='slipring']")
    sx=float(slipring.find('pose').text.split()[0]);length=float(slipring.find('geometry/cylinder/length').text)
    assert sx>0>lp[0]
    assert abs(sx)+length/2<spec['robot']['head_cradle_length_m']/2
    track = root.find("world/model[@name='track']/link")
    heads = [track.find(f"visual[@name='{side}_head']") for side in ('left','right')]
    positions = [float(n.find('pose').text.split()[1]) for n in heads]
    assert abs(positions[0]-positions[1])-spec['track']['head_width_m'] == pytest.approx(1.435)
    assert len(car.findall("plugin/follower_wheel_joint")) == 3
    assert not car.find("link[@name='odometer_wheel']").findall('collision')
    for i in range(1,4):
        assert not car.find(f"link[@name='wheel_{i}']").findall('collision')


def test_panel_face_points_inward_and_geometry_respects_budget(tmp_path,inputs):
    config,spec = inputs
    config['tunnel'].update(x_min_m=0,x_max_m=1.2)
    geometry = make_meshes(tmp_path,config,spec)
    assert geometry['triangles'] <= spec['resources']['max_mesh_triangles']
    assert geometry['max_chord_sag_m'] < .000027
    points = []
    for line in (tmp_path/'panels.obj').read_text().splitlines():
        if line.startswith('v '):
            points.append([float(v) for v in line.split()[1:]])
            if len(points)==3:
                break
    a,b,c = np.asarray(points)
    normal = np.cross(b-a,c-a)
    radial = (a+b+c)/3-np.array([0,0,config['tunnel']['axis_z_m']])
    radial[0] = 0
    assert normal@radial < 0
    # Ogre2 cannot build textured wall materials without OBJ normals.
    lines=(tmp_path/'panels.obj').read_text().splitlines()
    normals=np.array([[float(v) for v in line.split()[1:]] for line in lines if line.startswith('vn ')])
    np.testing.assert_allclose(np.linalg.norm(normals,axis=1),1,atol=1e-8)
    assert all(len(v.split('/'))==3 and v.split('/')[2]
               for line in lines if line.startswith('f ') for v in line.split()[1:])
    assert normals[0]@radial < 0
    tiny = copy.deepcopy(spec)
    tiny['resources']['max_mesh_triangles'] = 1
    with pytest.raises(ValueError,match='geometry resource budget'):
        make_meshes(tmp_path,config,tiny)


def test_three_source_colour_correction_preserves_local_structure():
    ramp = np.linspace(.12,.28,64).reshape(8,8,1)
    target = np.repeat(ramp,3,axis=2)
    warm = target*np.array([1.2,1,.85])+.015
    recipe = colour_transform(warm,target)
    corrected = apply_transform(warm,recipe,.8)
    assert np.max(np.abs(corrected-target*.8)) < 1e-6
    assert np.corrcoef(corrected[:,:,0].ravel(),warm[:,:,0].ravel())[0,1] > .99999
    # Encoding is a transfer function, not an exposure multiplier in sRGB space.
    x = np.linspace(0,1,100,dtype=np.float32)
    np.testing.assert_allclose(linear_to_srgb(srgb_to_linear(x)),x,atol=2e-7)


def test_source_decode_is_rejected_before_allocation(tmp_path):
    from PIL import Image
    path = tmp_path/'source.png'
    Image.new('RGB',(100,100)).save(path)
    with pytest.raises(ValueError,match='memory estimate'):
        image_info(path,1000)


def test_width_distribution_is_truncated_without_endpoint_piles(inputs):
    parameters = inputs[1]['cracks']
    samples = sample_widths(10000,parameters,20260929)
    assert samples.min()>.2 and samples.max()<.6
    assert abs(samples.mean()-.4)<.003
    assert .08 < samples.std() < .10
    np.testing.assert_array_equal(samples,sample_widths(10000,parameters,20260929))


def test_ai_atlas_has_all_types_and_unscaled_source_paths():
    data = json.loads((ROOT/'assets/cracks/generated/crack_candidates_v1.catalog.json').read_text())
    assert len(data['motifs'])==18
    assert {item['kind'] for item in data['motifs']}=={'longitudinal','transverse','diagonal','curved','branching','network'}
    for item in data['motifs']:
        points = np.concatenate(item['paths_xy_px'])
        assert np.isfinite(points).all() and len(points)>10
        assert item['physical_width_mm'] is None
    assert not data['procedural_paths']


def test_source_topology_retains_network_loop():
    import cv2
    mask = np.zeros((64,64),np.uint8)
    cv2.rectangle(mask,(12,12),(50,50),1,3)
    paths = trace_skeleton(mask)
    assert any(np.linalg.norm(path[0]-path[-1]) <= 1.5 for path in paths)


def test_thin_long_cracks_dominate_the_type_mix(inputs):
    parameters=inputs[1]['cracks']
    kinds=sample_types(10000,parameters,20260929)
    assert .79<(kinds=='long_slender').mean()<.81
    assert .14<(kinds=='short_slender').mean()<.16
    assert .04<(kinds=='network').mean()<.06
    branches=sample_branches(kinds,parameters,20260929)
    assert not branches[kinds=='network'].any()
    for kind in ('long_slender','short_slender'):
        assert .05<branches[kinds==kind].mean()<.11


def test_long_length_calibration_is_independent_of_width():
    motif=dict(id='trial',paths_xy_px=[[[0,0],[10,1],[20,0]],[[10,1],[12,4]]])
    thin=calibrate_long(motif,10,.2)
    thick=calibrate_long(motif,10,.6)
    for a,b in zip(thin['paths_xq_m'],thick['paths_xq_m']):
        np.testing.assert_array_equal(a,b)
    for candidate in (thin,thick):
        length=np.linalg.norm(np.diff(candidate['main_path_xq_m'],axis=0),axis=1).sum()
        assert length==pytest.approx(10,abs=1e-9)
    broken=dict(id='bad',paths_xy_px=[[[0,0],[1,0]],[[5,0],[6,0]]])
    with pytest.raises(ValueError,match='disconnected'):
        calibrate_long(broken,10,.4)


def test_local_coverage_preserves_measured_subpixel_width():
    # Independent analytic area of a straight strip whose end caps lie outside the crop.
    bounds=[0,.02,0,.004]
    paths=[np.array([[-.01,.002],[.03,.002]])]
    for width in (.2,.4,.6):
        coverage=coverage_patch(paths,width,bounds)
        measured_width=coverage.sum()*.0002**2/(bounds[1]-bounds[0])*1000
        assert measured_width==pytest.approx(width,abs=.001)
    with pytest.raises(ValueError,match='pixel budget'):
        coverage_patch(paths,.4,[0,10,0,1])


def test_crack_mix_update_does_not_invalidate_material_downloads(inputs):
    spec=inputs[1]
    sources=dict(materials=[dict(id=k,source_width_m=spec['materials']['source_width_m'][k]) for k in spec['materials']['sources']])
    changed=copy.deepcopy(spec)
    changed['cracks']['required_long_crack_length_m']=12
    assert sources_match_spec(sources,changed)
    changed['materials']['source_width_m']['Concrete030']=3
    assert not sources_match_spec(sources,changed)


def test_generated_long_candidates_have_ten_metre_main_paths():
    data=json.loads((ROOT/'assets/cracks/generated/long_crack_candidates_v1.catalog.json').read_text())
    assert len(data['metric_candidates'])==3
    for item in data['metric_candidates']:
        measured=np.linalg.norm(np.diff(item['main_path_xq_m'],axis=0),axis=1).sum()
        assert measured==pytest.approx(10,abs=1e-7)
        assert .2<=item['body_width_mm']<=.6


def test_optics_config_persists_accepted_texture_footprint_sampling(tmp_path):
    from ssb_tools.stage_b_optics import prepare as optics
    geometry=tmp_path/'geometry';geometry.mkdir()
    for name in ('panels.obj','joints.obj'):(geometry/name).write_text('v 0 0 0\n')
    (tmp_path/'surface.json').write_text('{}');(tmp_path/'defects.json').write_text('{}')
    (tmp_path/'config.yaml').write_text('render: {}\n')
    scene=optics(tmp_path/'config.yaml',geometry,tmp_path/'surface.json',tmp_path/'defects.json',tmp_path/'out',integrated=True)
    written=json.loads((tmp_path/'out/scene.json').read_text())
    assert scene['sampling']['texture_footprint_samples']==2 and written['sampling']['texture_footprint_samples']==2
    assert written['sampling']['texture_prefilter'] is False
    with pytest.raises(ValueError,match='sample limits'):
        optics(tmp_path/'config.yaml',geometry,tmp_path/'surface.json',tmp_path/'defects.json',tmp_path/'out2',texture_footprint_samples=9)


def test_joint_profile_states_and_filler_geometry(tmp_path,inputs):
    config,spec = inputs
    config['tunnel'].update(x_min_m=-1.5,x_max_m=21.5)
    geometry = make_meshes(tmp_path,config,spec)
    p=spec['panels'];r=config['tunnel']['radius_m'];zc=config['tunnel']['axis_z_m']
    c,g,D=p['joint_chamfer_m'],p['joint_groove_half_width_m'],p['joint_depth_m']
    joints=geometry['joints'];n=len(joints)
    frac={k:geometry['joint_state_counts'][k]/n for k in ('filled','unfilled','damaged')}
    assert n>200 and abs(frac['filled']-.8)<.08 and abs(frac['unfilled']-.15)<.07 and 0<frac['damaged']<.12
    assert geometry['triangles']<=spec['resources']['max_mesh_triangles']
    other=tmp_path/'b';other.mkdir();again=make_meshes(other,config,spec)
    assert again['joints']==joints                               # deterministic from the spec seed
    def radii(name):
        v=np.array([[float(x) for x in l.split()[1:]] for l in (tmp_path/name).read_text().splitlines() if l.startswith('v ')])
        return v,np.hypot(v[:,1],v[:,2]-zc)
    v,rad=radii('panels.obj')   # lining at r, chamfer bottoms at r+c
    assert np.all(np.isclose(rad,r,atol=1e-9)|np.isclose(rad,r+c,atol=1e-9)) and np.isclose(rad,r+c,atol=1e-9).any()
    _,rad=radii('gap.obj');assert np.allclose(rad,r+D+p['joint_gap_depth_m'],atol=1e-9)
    v,rad=radii('filler.obj')
    deepest=c+max(p['joint_filler_recess_m'][1],p['joint_damage']['loss_depth_m'][1])
    assert rad.min()>=r+c-1e-9 and rad.max()<=r+deepest+1e-9
    # Unfilled sections carry no filler: no filler vertex at a ring joint section marked unfilled.
    for j in joints:
        if j['kind']!='ring' or j['state']!='unfilled':continue
        ang=np.arctan2(v[:,1],v[:,2]-zc);u,w=j['angle_rad']
        inside=(np.abs(v[:,0]-j['x_m'])<g+1e-9)&(np.mod(ang-u,2*np.pi)>1e-6)&(np.mod(ang-u,2*np.pi)<(w-u)-1e-6)
        assert not inside.any()
    damaged=[j for j in joints if j['state']=='damaged']
    assert all(j['losses_m'] and all(l[2]>j['filler_recess_m'] for l in j['losses_m']) for j in damaged)
