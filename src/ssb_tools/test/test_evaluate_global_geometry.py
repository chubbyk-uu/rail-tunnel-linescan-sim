import numpy as np
import pytest

from ssb_tools.evaluate_band_matches import mesh_points
from ssb_tools.evaluate_global_geometry import sources_at, endpoint_drift, boundary_coordinates, boundary_support, verify_session, run
from ssb_tools.global_geometry import Trajectory, GeometrySettings
from ssb_tools.initial_unroll import BandSampler, PROJECTION
from ssb_tools.native_rows import MemoryRows
from ssb_tools.ref_mesh import OpticalMesh


def independent_fixture(depth=0.):
    phases = np.linspace(-.2, .2, 101)
    p = np.zeros(202, PROJECTION)
    for band in range(2):
        sl = slice(101*band, 101*(band+1))
        p['sequence'][sl] = np.arange(sl.start, sl.stop)
        p['lattice_row'][sl] = np.arange(101)+band*1000
        p['segment'][sl] = band
        p['theta_rad'][sl] = phases+band*2*np.pi
        p['x_axis_m'][sl] = .4+.2*band+.1*phases
    width=513
    raw = np.full((202,width),150,np.uint8)
    native = MemoryRows(raw,dict(offset=np.zeros(width),gain=np.ones(width),valid=np.ones(width,bool)))
    offsets=(np.arange(width)-256)*.001
    sampler=BandSampler(p,native,offsets,offsets,np.ones(width,bool),.00401)
    model=Trajectory(sampler,1.,.7,GeometrySettings(attitude_spacing_m=.1))
    coefficients=np.zeros(model.size)
    # Independent injected axial scale: true vehicle x = 1.01*nominal x - .005.
    # Greville coordinates reproduce the affine correction exactly in cubic splines.
    k=model.knots[0]
    greville=np.array([k[i+1:i+4].mean() for i in range(model.sizes[0])])
    coefficients[:model.sizes[0]]=.01*(greville-.5)/model.scale
    rows=np.zeros(len(p),dtype=[('sequence','<i8'),('theta','<f8'),('x','<f8')])
    rows['sequence']=p['sequence'];rows['theta']=p['theta_rad']
    rows['x']=1.01*p['x_axis_m']-.005
    camera=dict(width=width,pixel_pitch_m=.001,nominal_distance_m=1.,fov_at_nominal_m=.513)
    truth=dict(head_mount_x_m=0.,robot=dict(base_reference_z_m=.3,scan_axis_height_m=.7),
               tunnel=dict(radius_m=1.,axis_z_m=1.),mount=dict(
                   dy_m=0.,dz_m=0.,e_m=0.,tangential_m=0.,tilt_y_rad=0.,tilt_z_rad=0.,twist_rad=0.))
    v=np.array([[-1.,-.5,2.+depth],[-1.,.5,2.+depth],[3.,.5,2.+depth],[3.,-.5,2.+depth]])
    mesh=OpticalMesh(v,[[0,1,2],[0,2,3]],[2,2],1.)
    return model,coefficients,rows,camera,truth,mesh


@pytest.mark.parametrize('depth,expected_after', [(0.,0.),(.02,-.00404)])
def test_actual_seam_uses_inverse_native_footprints_and_recessed_optical_mesh(depth,expected_after):
    model,c,rows,camera,truth,mesh=independent_fixture(depth)
    def seam(coefficients):
        points=[]
        for band in (0,1):
            table,valid,_=sources_at(model,coefficients,band,np.array([.5]),np.array([0.]))
            assert valid.all()
            point,material=mesh_points(table,'a',rows,camera,truth,mesh)
            assert material[0]==2
            points.append(point[0])
        return points[1]-points[0]
    before=seam(np.zeros(model.size))
    after=seam(c)
    np.testing.assert_allclose(after,[expected_after,0.],atol=1e-12)
    assert abs(before[0])>.001
    # The 20 mm recess leaves a real parallax error that an ideal-cylinder
    # evaluator would hide even with the correct injected vehicle trajectory.
    if depth:assert abs(after[0])>.004


def test_drift_removes_translation_but_retains_scale_and_circumferential_shear():
    xs=np.linspace(3.,6.,33);qs=np.linspace(-1.,1.,49)
    xx,qq=np.meshgrid(xs,qs)
    points=np.stack((1.01*xx+.1,qq+.02+.002*(xx-3)),axis=-1)
    report=endpoint_drift(points,np.ones(xx.shape,bool),xs,qs)
    assert report['axial_scale_error_percent']==pytest.approx(1.)
    np.testing.assert_allclose(report['endpoint_delta_mean_m'],[.03,.006],atol=1e-12)
    valid=np.ones(xx.shape,bool);valid[:,0]=False
    assert endpoint_drift(points,valid,xs,qs)['status']=='unmeasurable'


def test_perimeter_enumerates_every_edge_pixel_without_duplicate_corners():
    grid=dict(shape=[7,5],target_x_m=[0.,2.],theta_rad=[-1.,1.],radius_m=1.,dx_m=.4,dq_m=2/7)
    boundary=boundary_coordinates(grid)
    values=np.concatenate([np.column_stack(v) for v in boundary.values()])
    assert len(values)==2*7+2*5-4 and len(np.unique(values,axis=0))==len(values)


def test_perimeter_requires_valid_original_pixels_and_does_not_fill_saturation():
    model,c,*_=independent_fixture()
    grid=dict(shape=[11,9],target_x_m=[.4,.6],theta_rad=[-.15,.15],radius_m=1.,
              dx_m=.2/9,dq_m=.3/11)
    baseline=boundary_support(model,np.zeros(model.size),grid)
    assert sum(v['missing_pixels'] for v in baseline.values())==0
    phase=model.sampler.projection['theta_rad']-2*np.pi*model.sampler.projection['segment']
    model.sampler.native.raw_rows[phase>=0]=255
    report=boundary_support(model,np.zeros(model.size),grid)
    assert report['bottom']['missing_pixels']==9
    assert report['top']['missing_pixels']==0
    assert report['left']['missing_pixels']>0


def test_evaluator_cannot_write_private_results_into_public_reconstruction(tmp_path):
    with pytest.raises(ValueError,match='evaluation/'):
        run(tmp_path/'nonexistent_session',tmp_path/'d1',tmp_path/'d3',tmp_path/'public')


@pytest.mark.parametrize('changed', ['evaluation/manifest.json','config/backend.json','evaluation/truth.json'])
def test_truth_reference_requires_the_original_archived_identity(tmp_path,changed):
    from types import SimpleNamespace
    from ssb_tools.session import sha256_file
    names=('config/observable_config.json','metadata/manifest.json','raw/index.json',
           'evaluation/truth.json','evaluation/manifest.json','evaluation/config_source.yaml',
           'config/backend.json','config/provenance.json')
    files={}
    for name in names:
        p=tmp_path/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('{}')
        files[name]=sha256_file(p)
    session=SimpleNamespace(root=tmp_path,summary=dict(status='complete',files=files))
    upstream=dict(source_observation_hashes={k:files[k] for k in names[:3]})
    verify_session(session,upstream)
    (tmp_path/changed).write_text('{"replaced":true}')
    with pytest.raises(ValueError,match='archived identity'):
        verify_session(session,upstream)
