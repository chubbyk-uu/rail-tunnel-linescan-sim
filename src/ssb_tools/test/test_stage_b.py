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
    assert not root.findall('world/light') # no fixed overhead lights
    assert len(base.findall('light'))==4
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
    assert written['sampling']['crack_area_samples']==64
    assert 'interior_ratio' in written['crack_optics']                     # no depths: flat opening
    (tmp_path/'defects.json').write_text('{"files": {"depths.bin": {}}}')
    cavity=optics(tmp_path/'config.yaml',geometry,tmp_path/'surface.json',tmp_path/'defects.json',tmp_path/'out3',integrated=True,crack_area_samples=32)
    assert cavity['crack_optics']['model']=='cavity_v2'
    assert cavity['sampling']['crack_area_samples']==32
    with pytest.raises(ValueError,match='sample limits'):
        optics(tmp_path/'config.yaml',geometry,tmp_path/'surface.json',tmp_path/'defects.json',tmp_path/'out2',texture_footprint_samples=9)


def test_production_joints_are_all_filled(tmp_path,inputs):
    config,spec = inputs
    config['tunnel'].update(x_min_m=0,x_max_m=3.6)
    geometry = make_meshes(tmp_path,config,spec)
    assert geometry['joint_state_counts']=={'filled':len(geometry['joints']),'unfilled':0,'damaged':0}


def test_joint_profile_states_and_filler_geometry(tmp_path,inputs):
    config,spec = inputs
    # Exercise the retained unfilled/damaged code paths with the former development ratios.
    spec['panels']['joint_states']={'filled':.8,'unfilled':.15,'damaged':.05}
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
    v,rad=radii('panels.obj')   # lining at r, rounded lips down to the wall top at r+c+t
    tan=p['joint_edge_radius_m']*math.tan(math.pi/8)
    # Crossing patches may slope from a lip down to the filler level (at most c + max recess).
    assert rad.min()>=r-1e-9 and rad.max()<=r+c+max(tan,p['joint_filler_recess_m'][1])+1e-9 and (rad>r+c).any()
    _,rad=radii('gap.obj');assert np.allclose(rad,r+D+p['joint_gap_depth_m'],atol=1e-9)
    v,rad=radii('filler.obj')
    deepest=c+max(p['joint_filler_recess_m'][1],p['joint_damage']['loss_depth_m'][1])+1e-9
    # Crossing patches: filler cells at the lip boundary may rise onto the inner fillet.
    assert rad.min()>=r+c-5e-4 and rad.max()<=r+deepest+1e-9
    # Unfilled sections carry no filler: no filler vertex at a ring joint section marked unfilled.
    for j in joints:
        if j['kind']!='ring' or j['state']!='unfilled':continue
        ang=np.arctan2(v[:,1],v[:,2]-zc);u,w=j['angle_rad']
        inside=(np.abs(v[:,0]-j['x_m'])<g+1e-9)&(np.mod(ang-u,2*np.pi)>1e-6)&(np.mod(ang-u,2*np.pi)<(w-u)-1e-6)
        assert not inside.any()
    damaged=[j for j in joints if j['state']=='damaged']
    assert all(j['losses_m'] and all(l[2]>j['filler_recess_m'] for l in j['losses_m']) for j in damaged)


def test_lip_profile_is_tangent_and_meets_filler_on_the_fillet():
    from ssb_tools.stage_b_scene import lip_profile,lip_half_width
    g,c,rho=.005,.003,.001
    pts,t=lip_profile(g,c,rho,8)
    s=np.array([q[0] for q in pts]);d=np.array([q[1] for q in pts])
    assert np.isclose(s[0],g+c+t) and np.isclose(d[0],0)          # starts on the lining surface
    assert np.isclose(s[-1],g) and np.isclose(d[-1],c+t)         # ends on the groove wall
    # First facet nearly horizontal (tangent to the surface), last nearly vertical (tangent to wall),
    # and the facets around the arc junction follow the 45 deg chamfer.
    seg=np.diff(np.column_stack([s,d]),axis=0);ang=np.degrees(np.arctan2(seg[:,1],-seg[:,0]))
    assert ang[0]<45/8+1e-6 and ang[-1]>90-45/8-1e-6 and np.isclose(ang[8],45,atol=1e-6)
    assert np.all(np.diff(ang)>=-1e-9)                             # convex lip: monotonic turning
    for depth in np.linspace(c,c+t,7):                             # filler edge lies on the inner fillet
        x=lip_half_width(g,c,rho,depth)
        assert np.isclose(np.hypot(x-(g+rho),depth-(c+t)),rho,atol=1e-12) or np.isclose(x,g)


def test_crack_refinement_keeps_topology_and_width_range(inputs):
    from ssb_tools.stage_b_defects import refine_path
    config,spec=inputs;cr=spec['cracks']
    rng=np.random.default_rng(3)
    # A 4.8 mm pixel staircase like the source skeleton, 0.3 m long, shared start (branch point).
    xs=np.arange(0,.3,.0048);ys=.0048*np.round(np.cumsum(rng.normal(0,.6,len(xs))))
    path=np.column_stack([xs,ys]);arc=np.r_[0,np.cumsum(np.hypot(*np.diff(path,axis=0).T))]
    radius=.0002*np.minimum(1,(arc[-1]-arc)/.005)          # free tip at the end
    q,r=refine_path(path,radius,(True,False),np.random.default_rng(5),spec,cr)
    np.testing.assert_allclose(q[0],path[0],atol=1e-12);np.testing.assert_allclose(q[-1],path[-1],atol=1e-12)
    seg=np.hypot(*np.diff(q,axis=0).T);assert seg.max()<.001        # wiggle lengthens, never jumps
    chord=np.hypot(*(path[-1]-path[0]));assert seg.sum()<1.3*chord    # tortuosity stays moderate
    lo,hi=cr['width_min_mm']*.0005,cr['width_max_mm']*.0005
    body=r[:len(r)//2];assert body.min()>=lo-1e-12 and body.max()<=hi+1e-12 and np.ptp(body)>.1*body.mean()
    assert r[-1]<1e-9                                        # free tip still tapers to zero
    # A straight crack drawn as a one-pixel staircase: refined path stays near the line, with
    # moderate tortuosity (synthetic wiggle, not the source staircase or a random walk).
    xs=np.arange(0,.3,.0048);stair=np.column_stack([xs,.0048*np.floor(xs/.05)])
    q2,_=refine_path(stair,np.full(len(xs),.0002),(False,False),np.random.default_rng(6),spec,cr)
    line=np.polyfit(stair[:,0],stair[:,1],1);dev=q2[:,1]-np.polyval(line,q2[:,0])
    src=np.abs(stair[:,1]-np.polyval(line,stair[:,0])).max()   # the staircase's own quantisation
    assert np.abs(dev).max()<src+.0015 and np.hypot(*np.diff(q2,axis=0).T).sum()<1.15*np.hypot(*(stair[-1]-stair[0]))


def test_crack_depth_profile_and_index_alignment(inputs):
    from ssb_tools.stage_b_defects import depth_profile, build_grid, DEPTH
    config,spec=inputs;dep=spec['cracks']['depth']
    s=np.linspace(0,1.,2001);r=np.full(len(s),.0002);r[-20:]=np.linspace(.0002,0,20)   # free tip at the end
    d=depth_profile(s,r,np.random.default_rng(1),dep)
    aspect=d[:-20]/(2*r[:-20])
    assert abs(np.log(np.median(aspect))-np.log(dep['aspect_median']*dep.get('scale',1)))<.3
    assert (aspect<1.5*dep['plug_aspect']).any()                      # shallow (debris) stretches exist
    la=np.log(aspect)                                                 # correlated over >= 5 mm: 0.5 mm
    assert np.median(np.abs(np.diff(la)))<.3*la.std()                 # steps are small against the spread
    assert d[-1]==0 and d.max()<=dep['max_depth_m']                   # tip closes; depth bounded
    path=np.column_stack([1+.1*s,.5+0*s])
    item=dict(paths_xq_m=[path.tolist()],vertex_radius_m=[r.tolist()],vertex_depth_m=[d.tolist()])
    seg,off,idx,grid,dd=build_grid([item],[0,2,0,1],.01,with_depths=True)
    assert dd.dtype==DEPTH and len(dd)==len(seg)
    np.testing.assert_allclose(dd['d0'][1:],dd['d1'][:-1],atol=1e-9)  # continuous, same order as segments
    np.testing.assert_allclose(dd['d0'][0],d[0],rtol=1e-6)


def test_refinement_bounds_variable_body_width_separately_from_tips(inputs):
    from ssb_tools.stage_b_defects import refine_path
    spec=inputs[1];s=np.linspace(0,.5,401);tip=np.minimum(1,np.minimum(s/.005,(s[-1]-s)/.005))
    path=np.column_stack([s,.0005*np.sin(s*40)])
    radius=(.00011+.000045*(1+np.sin(s*70)))*tip
    q,r=refine_path(path,radius,(False,False),np.random.default_rng(5),spec,spec['cracks'])
    body=(s[0]+.02<np.linspace(0,s[-1],len(r)))&(np.linspace(0,s[-1],len(r))<s[-1]-.02)
    assert r[body].min()>=.0001-1e-12 and r[body].max()<=.0003+1e-12
    assert r[0]==r[-1]==0


def test_refined_main_route_uses_rendered_edges_and_original_directions():
    from ssb_tools.stage_b_defects import refined_main_path
    original=[np.array([[0,0],[1,0]]),np.array([[2,0],[1,0]]),np.array([[1,0],[1,1]])]
    refined=[np.array([[0,0],[.5,.01],[1,0]]),np.array([[2,0],[1.5,-.02],[1,0]]),original[2]]
    spine=np.array([[0,0],[1,0],[2,0]])
    actual=refined_main_path(original,refined,spine)
    np.testing.assert_array_equal(actual,np.concatenate([refined[0],refined[1][::-1][1:]]))
    with pytest.raises(ValueError,match='main route'):
        refined_main_path(original,refined,[[0,0],[1,0],[3,0]])


def test_short_refined_graph_edges_keep_both_junctions(inputs):
    from ssb_tools.stage_b_defects import refine_path
    spec=inputs[1]
    path=np.array([[0,0],[.002,.001],[.004,.003],[.006,.004]])
    q,r=refine_path(path,np.full(len(path),.0002),(True,True),np.random.default_rng(17),spec,spec['cracks'])
    np.testing.assert_array_equal(q[0],path[0]);np.testing.assert_array_equal(q[-1],path[-1])
    assert np.linalg.norm(np.diff(q,axis=0),axis=1).max()<.001


def test_defect_derivatives_use_snapshot_and_reject_changed_inputs(tmp_path,inputs):
    from ssb_tools.stage_b_defects import snapshot_input, build_grid, regrid, refine, add_depth
    from ssb_tools.stage_b_scene import digest
    config=copy.deepcopy(inputs[0]);config['tunnel'].update(radius_m=.1,x_min_m=0.,x_max_m=1.)
    original=tmp_path/'config.yaml';original.write_text(yaml.safe_dump(config))
    source=tmp_path/'source';source.mkdir();entry=snapshot_input(source,original,'config.yaml')
    instance=dict(paths_xq_m=[[[.2,0],[.3,0]]],vertex_radius_m=[[.0002,.0002]])
    _,_,_,grid=build_grid([instance],[0,1,-math.pi*.1,math.pi*.1])
    layout=dict(inputs=dict(config=entry),instances=[instance],grid=grid)
    (source/'defects.json').write_text(json.dumps(layout))
    config['tunnel']['x_max_m']=2;original.write_text(yaml.safe_dump(config))
    result=regrid(source,tmp_path/'regrid',.02)
    assert result['grid']['bounds_xq_m']==[0,1,-math.pi*.1,math.pi*.1]
    assert result['inputs']['config']['sha256']==digest(entry['file'])
    # Mutation of the archived bytes must fail in every derivative, before any output.
    Path(entry['file']).write_bytes(original.read_bytes())
    spec=ROOT/'src/ssb_tools/config/stage_b_scene.yaml'
    for operation,args in ((regrid,(.02,)),(refine,(spec,)),(add_depth,(spec,))):
        with pytest.raises(ValueError,match='config hash mismatch'):
            operation(source,tmp_path/('rejected_'+operation.__name__),*args)
    # Even re-hashing a changed config cannot silently change the recorded domain.
    layout['inputs']['config']['sha256']=digest(entry['file']);(source/'defects.json').write_text(json.dumps(layout))
    with pytest.raises(ValueError,match='grid bounds changed'): regrid(source,tmp_path/'changed_domain',.01)


def test_shallower_depth_preserves_profile_exactly(inputs):
    from ssb_tools.stage_b_defects import depth_profile
    dep=copy.deepcopy(inputs[1]['cracks']['depth']);s=np.linspace(0,1,2001);r=np.full(len(s),.0002)
    dep['scale']=1;old=depth_profile(s,r,np.random.default_rng(37),dep)
    dep['scale']=.8;new=depth_profile(s,r,np.random.default_rng(37),dep)
    np.testing.assert_allclose(new,old*.8,rtol=1e-14,atol=0)


def test_patent_guides_touch_inner_rail_and_wheels_rest_on_top(tmp_path,inputs):
    from ssb_tools.stage_b_robot import make_robot
    config,spec=inputs
    car=make_robot(tmp_path,config,spec)
    base=car.find("link[@name='base']")
    base_z=float(base.findtext('pose').split()[2])
    half=spec['robot']['wheelbase_m']/2
    for side,sign in [('left',1),('right',-1)]:
        for x in (-half,half):
            visual=base.find(f"visual[@name='{side}_guide_{x}']")
            pose=list(map(float,visual.findtext('pose').split()))
            radius=float(visual.findtext('geometry/cylinder/radius'))
            width=float(visual.findtext('geometry/cylinder/length'))
            assert sign*pose[1]+radius==pytest.approx(spec['track']['gauge_m']/2)
            assert -.038 < pose[2]+base_z-width/2 < pose[2]+base_z+width/2 < 0
            assert pose[3:]==[0,0,0] # vertical bearing spindle, rolling along rail side
    wheels=[l for l in car.findall('link') if l.find("visual[@name='tread']") is not None]
    assert len(wheels)==4
    for wheel in wheels:
        position=list(map(float,wheel.findtext('pose').split()))
        tread=wheel.find("visual[@name='tread']")
        radius=float(tread.findtext('geometry/cylinder/radius'))
        assert position[2]-radius==pytest.approx(0)
        assert abs(position[1])==pytest.approx((spec['track']['gauge_m']+spec['track']['head_width_m'])/2)
        assert wheel.find("visual[@name='inner_flange']") is None
        assert not wheel.findall('collision')
    carriage=car.find("joint[@name='carriage']")
    assert carriage.get('type')=='prismatic' and carriage.findtext('axis/xyz')=='1 0 0'
    assert len(car.findall('.//collision'))==2
    assert sum(float(v.text) for v in car.findall('link/inertial/mass'))==120


def test_track_sleepers_and_contact_planes(tmp_path,inputs):
    from ssb_tools.stage_b_track import make_track
    config,spec=inputs;track=make_track(tmp_path,config,spec)
    rails=track.find("link[@name='rails']")
    for sign,side in [(1,'left'),(-1,'right')]:
        c=rails.find(f"collision[@name='{side}_head']")
        x,y,z,*_=map(float,c.findtext('pose').split())
        length,width,height=map(float,c.findtext('geometry/box/size').split())
        assert z+height/2==pytest.approx(0)
        assert sign*y-width/2==pytest.approx(spec['track']['gauge_m']/2)
    sleepers=track.find("link[@name='sleepers']")
    positions=[float(v.findtext('pose').split()[0]) for v in sleepers.findall('visual') if v.get('name').startswith('sleeper_')]
    assert len(positions)>30 and np.diff(positions)==pytest.approx(.6)
    # Rail render geometry has actual head, web and foot sections, no hidden solid beam.
    vertices=[]
    for file in (tmp_path/'track').glob('*.obj'):
        vertices.extend([list(map(float,l.split()[1:])) for l in file.read_text().splitlines() if l.startswith('v ')])
    assert min(v[2] for v in vertices)==pytest.approx(-.176)
    assert max(v[2] for v in vertices)==pytest.approx(0)


def test_contact_front_drive_rear_encoders_and_free_guides(tmp_path,inputs):
    from ssb_tools.stage_b_robot import make_robot
    config,spec=inputs;config['contact']={'enabled':True}
    config['truth'].update(odo_left_diameter_m=.198,odo_right_diameter_m=.202)
    car=make_robot(tmp_path,config,spec)
    assert car.find("joint[@name='carriage']") is None
    assert all(j.findtext('parent')!='world' for j in car.findall('joint'))
    plugin=car.find('plugin');assert plugin.get('name')=='ssb_gazebo::ContactSystem'
    for tag in ('left_drive','right_drive','left_encoder','right_encoder'):
        joint=car.find(f"joint[@name='{plugin.findtext(tag)}']")
        link=car.find(f"link[@name='{joint.findtext('child')}']")
        x=float(link.findtext('pose').split()[0])
        assert (x>0) == tag.endswith('drive')
        assert link.find('collision') is not None
    for joint,diameter in [('odometer',.198),('wheel_joint_2',.202)]:
        link=car.find(f"link[@name='{car.findtext('joint[@name="'+joint+'"]'+'/child')}']")
        assert float(link.findtext('collision/geometry/cylinder/radius'))==pytest.approx(diameter/2)
    assert len([j for j in car.findall('joint') if '_guide_' in j.get('name')])==4
    assert len(car.findall('link'))==10
    assert sum(float(m.text) for m in car.findall('link/inertial/mass'))==pytest.approx(120)


def test_work_light_cones_miss_imaging_arc_and_cast_shadows(tmp_path,inputs):
    config,spec=inputs
    from ssb_tools.stage_b_robot import make_robot
    car=make_robot(tmp_path,config,spec)
    lamps=car.findall("link[@name='base']/light")
    assert len(lamps)==4
    lowest_image_z=config['tunnel']['axis_z_m']+config['tunnel']['radius_m']*math.cos(math.radians(120))
    for lamp in lamps:
        d=np.array(list(map(float,lamp.find('direction').text.split())))
        a=float(lamp.find('spot/outer_angle').text)
        top_direction=d[2]*math.cos(a)+np.linalg.norm(d[:2])*math.sin(a)
        # Even the top rim of each cone goes downward. At nominal attitude the
        # complete beam lies below 0.45 m; imaged wall starts at 0.64 m.
        assert top_direction<-.3
        assert float(lamp.find('pose').text.split()[2])+.3 < lowest_image_z-.15
        assert lamp.find('cast_shadows').text=='true'
