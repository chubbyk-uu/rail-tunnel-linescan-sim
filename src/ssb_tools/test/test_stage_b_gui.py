import math
import numpy as np
import pytest

from ssb_tools.stage_b_gui import PreviewRecipe, mesh_plan, crack_preview, read_mesh
from ssb_tools.stage_b_surface import TEXEL


def test_filtered_recipe_preserves_metric_location_and_linear_ramp():
    # Single native patch: physical coordinates, transpose and macro-free filtering
    # must agree with a linear function even after native resolution is reduced.
    r=PreviewRecipe.__new__(PreviewRecipe)
    n=128; pitch=.0001
    source=np.zeros((n,n),TEXEL)
    yy,xx=np.mgrid[:n,:n]
    source['albedo']=np.rint(65535*(.2+.05*(xx+.5)/n+.03*(yy+.5)/n)).astype('u2')
    r.sources=[source];r.filtered={};r.macro=None
    r.alpha=np.full((1,n,n),255,'u1')
    r.recipe=dict(origin_xq_m=[0,0],guide_texel_m=pitch,patch_pixels=n,
        sources=[dict(source_width_m=n*pitch)],
        placements=[dict(left=0,top=0,material=0,source_matrix=[[0,1],[1,0]],source_offset_m=[0,0])])
    xs=np.linspace(.002,.01,21);qs=np.linspace(.003,.009,17)
    expected=.2+.05*qs[:,None]/(n*pitch)+.03*xs[None,:]/(n*pitch)
    np.testing.assert_allclose(r.albedo(xs,qs,.0008),expected,atol=3e-5)


def test_mesh_partition_retains_every_face_and_includes_all_vertices(tmp_path):
    mesh=tmp_path/'mesh.obj'
    mesh.write_text('v 0 0 1\nv 1 0 1\nv 0 1 0\nv 1 1 0\n'+
        'vn 0 0 -1\n'*4+'vt 0 0\nvt 1 0\nvt 0 1\nvt 1 1\n'+
        'f 1/1/1 2/2/2 3/3/3\nf 2/2/2 4/4/4 3/3/3\n')
    vertices,normals,uv,faces=read_mesh(mesh)
    tunnel=dict(x_min_m=0,x_max_m=1,radius_m=.1)
    xq,plan,estimate=mesh_plan(vertices,uv,faces,tunnel,1<<30)
    assert sorted(np.concatenate([p['ids'] for p in plan]))==list(range(len(faces)))
    for p in plan:
        assert np.all(xq[p['used']]>=p['lo']) and np.all(xq[p['used']]<=p['hi'])
    with pytest.raises(ValueError,match='exceeds'):
        mesh_plan(vertices,uv,faces,tunnel,estimate-1)


def test_gui_crack_metric_coverage_depth_and_union():
    wall=np.full((40,80),.2,np.float32)
    # On a 1 mm preview pixel, a 0.5 mm crack must not become a one-pixel ink line.
    seg=np.array([[.005,.020,.075,.020,.00025,.00025]],dtype='f4')
    depth=np.full((1,2),.0003,dtype='f4')
    out=crack_preview(wall,np.array([0.,0.]),.001,seg,depth)
    f=1/(1+2*.0003/.0005);ratio=f/(1-.2*(1-f))
    measured=(1-out[:,10:70]/.2).sum(axis=0).mean()
    assert measured==pytest.approx(.5*(1-ratio),abs=.015)
    duplicated=crack_preview(wall,np.array([0.,0.]),.001,np.repeat(seg,2,axis=0),np.repeat(depth,2,axis=0))
    np.testing.assert_array_equal(out,duplicated)
    shallow=crack_preview(wall,np.array([0.,0.]),.001,seg,depth*.8)
    assert np.all(shallow>=out) and np.any(shallow>out)
