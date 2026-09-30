import copy
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from ssb_tools.stage_b_surface import inverse_orientation, quilt_layout, QuiltSampler, TEXEL
from ssb_tools.stage_b_defects import assemble, build_grid, exact_composition
from ssb_tools.stage_b_scene import load_spec

ROOT=Path(__file__).resolve().parents[3]


def test_preview_reads_native_tiles_across_partial_period_and_lru_eviction(tmp_path):
    from ssb_tools.stage_b_preview import AlbedoTiles
    from ssb_tools.stage_b_scene import digest
    # q=5 ends inside the final four-pixel tile, not at a tile boundary.
    expected=(np.arange(5)[:,None]*100+np.arange(7)[None,:]).astype(np.uint16)
    entries=[]
    for iq in range(2):
        for ix in range(2):
            packed=np.zeros((6,6),TEXEL)
            for q in range(4):
                for x in range(4):
                    packed['albedo'][q+1,x+1]=expected[(iq*4+q)%5,min(ix*4+x,6)]
            p=tmp_path/f'{ix}_{iq}.bin';packed.tofile(p)
            entries.append(dict(ix=ix,iq=iq,file=p.name,sha256=digest(p)))
    manifest=dict(core_pixels=4,gutter_pixels=1,texel_xq_m=[1.,1.],
                  origin_xq_m=[0.,0.],pixels_xq=[7,5],tiles=entries)
    path=tmp_path/'surface.json';path.write_text(json.dumps(manifest))
    reader=AlbedoTiles(path,cache_bytes=72) # one albedo tile only
    actual=reader.crop(1,7,3,9,1024)
    np.testing.assert_array_equal(actual,expected[np.arange(3,9)%5,1:7])
    assert len(reader.cache)==1
    with pytest.raises(ValueError,match='working-set'):
        reader.crop(0,7,0,5,10)
    (tmp_path/'0_0.bin').write_bytes(b'broken')
    with pytest.raises(ValueError,match='identity'):
        AlbedoTiles(path).tile(0,0)


def test_preview_budget_fails_before_writing_textures(tmp_path):
    from ssb_tools.stage_b_preview import prepare_preview
    surface=tmp_path/'surface.json'
    tunnel=dict(radius_m=2.75,axis_z_m=2.015,x_min_m=0,x_max_m=1.2)
    surface.write_text(json.dumps(dict(tunnel=tunnel,core_pixels=4,gutter_pixels=1,
        texel_xq_m=[.001,.001],tiles=[])))
    spec=load_spec(ROOT/'src/ssb_tools/config/stage_b_scene.yaml')
    spec['resources']['gpu_preview_texture_budget_bytes']=1
    geometry=dict(panels=[dict(x_m=[0,1.2],angle_rad=[0,1])])
    with pytest.raises(ValueError,match='before allocation'):
        prepare_preview(surface,tmp_path,geometry,dict(tunnel=tunnel),spec)
    assert not (tmp_path/'preview').exists()


def test_orientation_maps_native_pixel_centres_and_normal_gradients():
    # Independent image rotation/mirror oracle, including all eight orientations.
    size=11
    yy,xx=np.mgrid[:size,:size]
    image=xx+100*yy
    for k in range(4):
        for flip in (False,True):
            actual=np.rot90(image,k)
            if flip: actual=np.fliplr(actual)
            matrix,offset=inverse_orientation(k,flip,size)
            destination=np.stack([xx+.5,yy+.5],axis=-1)
            source=destination@matrix.T+offset
            indices=np.floor(source).astype(int)
            np.testing.assert_array_equal(actual,image[indices[:,:,1],indices[:,:,0]])
            # A scalar height gradient transforms by the transpose of this mapping.
            normal=np.array([.2,-.3])@matrix
            for axis in range(2):
                delta=np.zeros(2);delta[axis]=1e-4
                difference=((matrix@delta)@np.array([.2,-.3]))/1e-4
                assert normal[axis]==pytest.approx(difference)


def test_quilt_replay_is_continuous_across_arbitrary_tile_partition(tmp_path):
    rng=np.random.default_rng(44)
    guide=rng.uniform(.1,.4,(100,100)).astype(np.float32)
    layout=quilt_layout([guide],[2.],[0,2.2,0,1.8],77,tmp_path/'quilt',min_repeat_x_m=5,near_crop_m=.05,candidates=8)
    source=np.zeros((100,100),TEXEL)
    source['albedo']=np.uint16(guide*65535)
    source['roughness']=128;source['nx']=3000;source['nq']=-1000
    sampler=QuiltSampler(layout,tmp_path/'quilt',[(source,2.)])
    xs=np.linspace(.015,2.185,113);qs=np.linspace(.015,1.785,89)
    whole=sampler.sample(xs,qs)
    pieces=np.concatenate([sampler.sample(xs[:41],qs),sampler.sample(xs[41:],qs)],axis=1)
    np.testing.assert_array_equal(whole,pieces)
    assert np.isfinite(whole).all() and whole[:,:,0].min()>.05
    for a_index,a in enumerate(layout['placements']):
        for b in layout['placements'][:a_index]:
            if abs(a['left']-b['left'])*layout['guide_texel_m']<5:
                assert np.linalg.norm(np.array(a['source_centre_m'])-b['source_centre_m'])>=.05
    again=quilt_layout([guide],[2.],[0,2.2,0,1.8],77,tmp_path/'again',min_repeat_x_m=5,near_crop_m=.05,candidates=8)
    assert layout==again


def test_scene_composition_ten_metre_spine_and_bounds():
    config=yaml.safe_load((ROOT/'src/ssb_core/config/stage_b.yaml').read_text())
    spec=load_spec(ROOT/'src/ssb_tools/config/stage_b_scene.yaml')
    long=json.loads((ROOT/'assets/cracks/generated/long_crack_candidates_v1.catalog.json').read_text())
    short=json.loads((ROOT/'assets/cracks/generated/crack_candidates_v1.catalog.json').read_text())
    instances=assemble(config,spec,long,short)
    assert [sum(i['composition']==k for i in instances) for k in ('long_slender','short_slender','network')]==[48,9,3]
    first=instances[0]
    assert first['main_length_m']==pytest.approx(10,abs=1e-9)
    assert first['source_motif']=='long_01' and len(first['paths_xq_m'])==1
    assert 9<first['longitudinal_span_m']<10
    for item in instances:
        points=np.concatenate(item['paths_xq_m'])
        assert points[:,0].min()>=0 and points[:,0].max()<=20
        assert abs(points[:,1]).max()<=2.75*2*math.pi/3
        assert .2<=item['body_width_mm']<=.6
        radii=np.concatenate(item['vertex_radius_m'])
        assert radii.min()>=0 and radii.max()<=.000300001
        if item['composition']!='network' and not item['has_minor_branches']:
            assert len(item['paths_xq_m'])==1
    assert instances==assemble(config,spec,long,short)


@pytest.mark.parametrize('cell_m',[.01,.05])
def test_spatial_index_never_loses_crack_coverage_at_grid_boundaries(cell_m):
    item=dict(paths_xq_m=[[[.1,.05],[.8,.3],[.9,.05]]],vertex_radius_m=[[.00015,.0003,.0001]])
    segments,offsets,indices,grid=build_grid([item],[0,1,0,.5],cell_m)
    assert grid['cell_m']==cell_m
    rng=np.random.default_rng(101)
    probes=[]
    for s in segments:
        for t in (0,.5,1):
            p=np.array([s['x0'],s['q0']])*(1-t)+np.array([s['x1'],s['q1']])*t
            probes.append(p+rng.uniform(-.0002,.0002,2))
    def covered(point,selected):
        s=segments[selected]
        a=np.column_stack([s['x0'],s['q0']]);b=np.column_stack([s['x1'],s['q1']]);v=b-a
        t=np.clip(((point-a)*v).sum(axis=1)/(v*v).sum(axis=1),0,1)
        r=s['r0']+(s['r1']-s['r0'])*t
        return bool((((point-a-t[:,None]*v)**2).sum(axis=1)<=r*r).any())
    for p in probes:
        ix,iq=np.floor(p/grid['cell_m']).astype(int);cell=iq*grid['cells_xq'][0]+ix
        selected=indices[offsets[cell]:offsets[cell+1]]
        assert covered(p,selected)==covered(p,np.arange(len(segments)))
    with pytest.raises(ValueError,match='budget'): build_grid([item],[0,100,0,100],max_index_bytes=1000)


def test_joint_crossings_remain_opaque_with_different_facet_partitions(tmp_path):
    from ssb_tools.stage_b_scene import make_meshes
    config=yaml.safe_load((ROOT/'src/ssb_core/config/stage_b.yaml').read_text())
    config['tunnel'].update(x_min_m=7.2,x_max_m=9.6)
    spec=load_spec(ROOT/'src/ssb_tools/config/stage_b_scene.yaml')
    geometry=make_meshes(tmp_path,config,spec)
    vertices=[];faces=[]
    for name in ('panels.obj','joints.obj'):
        base=len(vertices)
        for line in (tmp_path/name).read_text().splitlines():
            if line.startswith('v '): vertices.append(list(map(float,line.split()[1:])))
            if line.startswith('f '): faces.append([base+int(v.split('/')[0])-1 for v in line.split()[1:]])
    vertices=np.asarray(vertices);faces=np.asarray(faces)
    a=vertices[faces[:,0]];e1=vertices[faces[:,1]]-a;e2=vertices[faces[:,2]]-a
    for x,theta,column in ((7.9760832,-.2504628,4088),(7.991475,-.089278,3958)):
        for dt in (-2e-6,0,2e-6):
            for subpixel in (-.25,0,.25):
                angle=theta+3.68155389092554*dt
                focal=config['camera']['pixel_pitch_m']*2.75*4096/config['camera']['fov_at_nominal_m']
                d=np.array([(column+subpixel-2047.5)*7.04e-6/focal,math.sin(angle),math.cos(angle)])
                o=np.array([x+.3515625*dt,0,2.015]);p=np.cross(d,e2);det=(e1*p).sum(axis=1)
                with np.errstate(divide='ignore',invalid='ignore'):
                    rel=o-a;u=(rel*p).sum(axis=1)/det;cross=np.cross(rel,e1)
                    v=(d*cross).sum(axis=1)/det;t=(e2*cross).sum(axis=1)/det
                hits=t[(np.abs(det)>1e-12)&(u>=0)&(v>=0)&(u+v<=1)&(t>0)]
                assert hits.size
                r=hits.min()*math.hypot(d[1],d[2])
                assert 2.7499<=r<=geometry['opaque_backing_radius_m']+.00001


def test_post_bake_guard_does_not_change_pbr_values_or_claim_crack_opacity(tmp_path):
    from ssb_tools.stage_b_surface import tag_cracks
    from ssb_tools.stage_b_defects import SEGMENT
    from ssb_tools.stage_b_scene import digest
    source=tmp_path/'source';source.mkdir();(source/'sources').mkdir();(source/'quilt').mkdir();(source/'tiles').mkdir()
    packed=np.zeros((24,24),TEXEL);packed['albedo']=17000;packed['roughness']=128;packed['nx']=1500;packed['nq']=-1500
    packed.tofile(source/'tiles/a.bin')
    data=dict(core_pixels=16,gutter_pixels=4,texel_xq_m=[.001,.001],origin_xq_m=[0,0],pixels_xq=[16,16],
              tiles=[dict(ix=0,iq=0,file='tiles/a.bin',sha256=digest(source/'tiles/a.bin'))])
    (source/'surface.json').write_text(json.dumps(data))
    defects=tmp_path/'defects';defects.mkdir()
    segments=np.array([(.001,.008,.015,.008,.0001,.0001)],SEGMENT);segments.tofile(defects/'segments.bin')
    (defects/'defects.json').write_text(json.dumps(dict(files={'segments.bin':dict(file='segments.bin',sha256=digest(defects/'segments.bin'))})))
    manifest=tag_cracks(source/'surface.json',defects/'defects.json',tmp_path/'tagged')
    tagged=np.fromfile(tmp_path/'tagged/tiles/a.bin',TEXEL).reshape(24,24)
    for field in ('albedo','roughness','nx','nq'): np.testing.assert_array_equal(tagged[field],packed[field])
    assert tagged['reserved'].max()==1 and tagged['reserved'].min()==0
    # Every crack-covered metric point, including tips, has a guard in its bilinear neighbourhood.
    for x in np.linspace(.001,.015,31):
        xp=(x/.001-.5)+4;qp=(.008/.001-.5)+4
        assert tagged['reserved'][math.floor(qp):math.floor(qp)+2,math.floor(xp):math.floor(xp)+2].any()
    assert manifest['adaptive_defects_sha256']==digest(defects/'defects.json')
    assert digest(source/'tiles/a.bin')==data['tiles'][0]['sha256']


def test_aligned_offsets_make_generated_texels_exact_source_copies():
    from ssb_tools.stage_b_runtime_surface import ReferenceRecipe,align_offsets,native_grid
    rng=np.random.default_rng(5)
    side=64;period=2*math.pi*2.75;texel,pixel_q=native_grid(period,3.2/16384)
    assert abs(texel/(3.2/16384)-1)<1e-5 and pixel_q*texel==pytest.approx(period,abs=1e-12)
    source=rng.integers(0,65535,(side,side)).astype(np.uint16)
    x0,q0=3.6,-period/2;guide0=[3.6-.0013,q0-.0007];step=.02
    for k in range(4):
        for flip in (False,True):
            matrix,_=inverse_orientation(k,flip,1)
            placement=dict(left=-1,top=3,source_matrix=np.asarray(matrix).astype(int).tolist(),
                           source_offset_m=(rng.uniform(.004,.008,2)+np.maximum(0,-np.asarray(matrix)).sum(1)*.009).tolist())
            align_offsets([placement],[x0,q0],guide0,step,texel)
            shift=np.array(placement['source_offset_m'])
            # Generated texel centres far along q, where accumulated pitch error would show first.
            i=np.arange(10);j=pixel_q//2+np.arange(10)
            gx,gq=np.meshgrid((x0+(i+.5)*texel-guide0[0])/step-placement['left'],(q0+(j+.5)*texel-guide0[1])/step-placement['top'])
            m=np.asarray(placement['source_matrix'])
            u=(m[0,0]*gx*step+m[0,1]*gq*step+shift[0])/texel-.5
            v=(m[1,0]*gx*step+m[1,1]*gq*step+shift[1])/texel-.5
            assert np.abs(u-np.round(u)).max()<1e-6 and np.abs(v-np.round(v)).max()<1e-6
            u,v=u-u.min(),v-v.min() # exercise interpolation on the in-range test source
            np.testing.assert_array_equal(ReferenceRecipe.interp(source,u,v),source[np.round(v).astype(int),np.round(u).astype(int)])


def test_orientation_maps_non_square_sources():
    # Independent rot90/fliplr oracle on a 7x11 (rows x cols) image, metres = pixels here.
    h,w=7,11
    yy,xx=np.mgrid[:h,:w];image=xx+100*yy
    for k in range(4):
        for flip in (False,True):
            actual=np.rot90(image,k)
            if flip: actual=np.fliplr(actual)
            matrix,offset=inverse_orientation(k,flip,w,h)
            dy,dx=np.mgrid[:actual.shape[0],:actual.shape[1]]
            source=np.stack([dx+.5,dy+.5],axis=-1)@matrix.T+offset
            idx=np.floor(source).astype(int)
            np.testing.assert_array_equal(actual,image[idx[:,:,1],idx[:,:,0]])


def test_quilt_uses_only_fully_valid_crops_of_non_square_sources(tmp_path):
    rng=np.random.default_rng(3)
    wide=rng.uniform(.2,.6,(30,60)).astype(np.float32)   # 0.6 x 0.3 m at 10 mm
    square=rng.uniform(.2,.6,(50,50)).astype(np.float32)
    valid=np.ones(square.shape,bool);valid[20:30,:]=False  # an excluded band
    layout=quilt_layout([wide,square],[(.6,.3),(.5,.5)],[0,1.5,0,1.2],5,tmp_path/'q',patch_m=.2,overlap_m=.08,
                        guide_texel_m=.01,min_repeat_x_m=.3,near_crop_m=.02,candidates=16,
                        valid=[np.ones(wide.shape,bool),valid],weights=[.5,.5],repeat_metric='wall')
    assert {p['material'] for p in layout['placements']}=={0,1}
    for p in layout['placements']:
        m=np.asarray(p['source_matrix']);o=np.asarray(p['source_offset_m'])
        corners=np.array([[0,0],[.2,0],[0,.2],[.2,.2]])@m.T+o   # crop extent in source metres
        lo,hi=corners.min(0),corners.max(0)
        extent=(.6,.3) if p['material']==0 else (.5,.5)
        assert lo.min()>=-1e-9 and hi[0]<=extent[0]+1e-9 and hi[1]<=extent[1]+1e-9
        if p['material']==1:
            assert hi[1]<=.2+1e-9 or lo[1]>=.3-1e-9   # never overlaps rows 20..29 (0.2..0.3 m)


def test_seam_feather_never_reaches_the_overlap_edge(tmp_path):
    # Regression: overlap 8 guide px left partial alpha where the previous patch ends.
    rng=np.random.default_rng(9)
    guide=rng.uniform(.1,.6,(60,60)).astype(np.float32)
    layout=quilt_layout([guide],[.6],[0,1.2,0,1.2],4,tmp_path/'q',patch_m=.3,overlap_m=.08,guide_texel_m=.01,
                        min_repeat_x_m=.2,near_crop_m=.01,candidates=8,repeat_metric='wall')
    # Independent composite at off-centre positions (the generators' bilinear alpha).
    from ssb_tools.stage_b_runtime_surface import ReferenceRecipe
    side=layout['patch_pixels'];g=layout['guide_size']
    ys,xs=np.meshgrid(np.arange(1,g[0]-1,.37),np.arange(1,g[1]-1,.37),indexing='ij');filled=np.zeros(ys.shape)
    for p in layout['placements']:
        a=cv2.imread(str(tmp_path/'q'/p['alpha']),cv2.IMREAD_GRAYSCALE)
        px,py=xs-p['left'],ys-p['top'];inside=(px>=0)&(py>=0)&(px<side)&(py<side)
        v=ReferenceRecipe.interp(a,np.where(inside,px,0)-.5,np.where(inside,py,0)-.5)/255
        filled=np.where(inside,filled*(1-v)+v,filled)
    assert filled.min()>=.999


def test_same_orientation_duplicates_are_kept_apart(tmp_path):
    rng=np.random.default_rng(12)
    guide=rng.uniform(.2,.6,(60,110)).astype(np.float32)   # 1.1 x 0.6 m at 10 mm
    R,limit,patch=.6,.25,.3
    layout=quilt_layout([guide],[(1.1,.6)],[0,2.4,0,2.0],8,tmp_path/'q',patch_m=patch,overlap_m=.08,guide_texel_m=.01,
                        min_repeat_x_m=R,near_crop_m=.01,candidates=24,repeat_metric='wall',max_same_orientation_overlap=limit)
    P=layout['placements'];g=layout['guide_texel_m']
    def rect(p):
        m=np.asarray(p['source_matrix']);o=np.asarray(p['source_offset_m'])
        c=m@(np.array([[0,0],[1,0],[0,1],[1,1]]).T*patch)+o[:,None];return c.min(1),c.max(1)
    pairs=0
    for i,a in enumerate(P):
        for b in P[:i]:
            if a['orientation']!=b['orientation'] or np.hypot(a['left']-b['left'],a['top']-b['top'])*g>=R:continue
            (a0,a1),(b0,b1)=rect(a),rect(b);w,h=np.clip(np.minimum(a1,b1)-np.maximum(a0,b0),0,None)
            assert w*h/patch**2<=limit+1e-9;pairs+=1
    assert pairs>0   # the check exercised real same-orientation neighbours


def test_orientation_subset_is_respected(tmp_path):
    rng=np.random.default_rng(2)
    guide=rng.uniform(.2,.6,(50,50)).astype(np.float32)
    layout=quilt_layout([guide],[.5],[0,1.2,0,1.2],3,tmp_path/'q',patch_m=.2,overlap_m=.08,guide_texel_m=.01,
                        min_repeat_x_m=.2,near_crop_m=.01,candidates=8,repeat_metric='wall',orientations=[[0,1,4,5]])
    used={p['orientation'] for p in layout['placements']}
    assert used<={0,1,4,5} and len(used)>1
    for p in layout['placements']:   # image x stays along wall x: no quarter turns
        m=np.asarray(p['source_matrix']);assert m[0,1]==0 and m[1,0]==0


def test_repetition_measure_finds_a_planted_translated_copy():
    from ssb_tools.stage_b_repetition import off_peak,highpass
    rng=np.random.default_rng(4);pitch=.002
    img=cv2.GaussianBlur(rng.standard_normal((500,500)).astype(np.float32),(0,0),1.5)
    clean=off_peak(highpass(img,pitch,.01),pitch,count=60,seed=3)
    assert clean.max()<.5                               # unrepeated texture: no strong off-peak match
    near=img.copy();near[200:300,300:400]=near[200:300,200:300]  # 0.2 m copy inside the search
    sc=off_peak(highpass(near,pitch,.01),pitch,count=400,seed=3)
    assert sc.max()>.95
