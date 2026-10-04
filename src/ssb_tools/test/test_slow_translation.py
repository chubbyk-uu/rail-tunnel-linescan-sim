import numpy as np
import pytest

from ssb_tools.global_geometry import GeometrySettings, Trajectory, reconstruction_settings
from ssb_tools.optimize_bands import regularizer, bounded_normal_step
from test_global_optimization import synthetic_matches


@pytest.mark.parametrize('adaptive',[False,True])
def test_declared_slow_model_adds_only_coarse_translation_nodes(adaptive):
    base,_,_=synthetic_matches()
    settings=reconstruction_settings(.1,adaptive,True)
    settings.validate()
    model=Trajectory(base.sampler,1.,.7,settings)
    np.testing.assert_array_equal(model.knots[4],model.knots[0])
    np.testing.assert_array_equal(model.knots[5],model.knots[1])
    assert model.size==sum(model.sizes[:4])+2*model.sizes[0]
    prior=regularizer(model)
    # A linear low-frequency translation has zero curvature, finite nominal prior.
    coefficients=np.zeros(model.size)
    for field in (4,5):
        greville=np.array([model.knots[field][i+1:i+4].mean() for i in range(model.sizes[field])])
        coefficients[model.starts[field]:model.starts[field+1]]=2+3*greville
    penalty=prior@coefficients
    assert np.isfinite(penalty).all() and np.linalg.norm(penalty)>0
    assert np.count_nonzero(abs(penalty)>1e-10)==sum(model.sizes[4:])


@pytest.mark.parametrize('settings',[
    GeometrySettings(coarse_translation=True),
    GeometrySettings(fit_translation=True,coarse_translation=1),
    GeometrySettings(fit_heave=True,coarse_translation=True),
])
def test_malformed_or_undeclared_slow_translation_is_rejected(settings):
    with pytest.raises(ValueError,match='coarse translation'):
        settings.validate()


@pytest.mark.parametrize('current', [np.zeros(3),np.array([1.,-.7,.4])])
def test_coupled_box_step_matches_independent_exhaustive_kkt_faces(current):
    from itertools import product
    from scipy.sparse import csc_matrix
    # A strongly coupled quadratic whose unconstrained minimizer crosses bounds.
    a=np.array([[1.,3.,-2.],[2.,1.,1.],[.1,-.2,.4],[.01,0,0]])
    h=a.T@a; gradient=a.T@np.array([4.,-3.,2.,1.]); damping=.003
    matrix=h+np.diag(damping*np.diag(h)); lower=-1-current; upper=1-current
    candidates=[]
    for face in product((-1,0,1),repeat=3):
        face=np.array(face); x=np.where(face<0,lower,np.where(face>0,upper,0.))
        free=np.flatnonzero(face==0); bound=np.flatnonzero(face!=0)
        if len(free):x[free]=np.linalg.solve(matrix[np.ix_(free,free)],-gradient[free]-matrix[np.ix_(free,bound)]@x[bound])
        if np.all(x>=lower-1e-10) and np.all(x<=upper+1e-10):
            candidates.append((float(x@matrix@x/2+gradient@x),x.copy()))
    expected=min(candidates,key=lambda p:p[0])[1]
    actual=bounded_normal_step(csc_matrix(h),gradient,current,np.ones(3),damping)
    np.testing.assert_allclose(actual,expected,atol=1e-10)


def test_cylinder_roll_offset_gauge_is_relative_not_measured_body_attitude():
    from test_global_optimization import independent_hits
    rng=np.random.default_rng(760)
    axis=rng.uniform(2,5,40);theta=rng.uniform(-2,2,40);tangent=rng.uniform(-.1,.1,40)
    radius,height=2.75,1.715; psi=.006
    original=np.zeros((40,6));original[:,2]=rng.uniform(-.004,.004,40)
    original[:,4]=.02+.001*axis;original[:,5]=-.015+.002*axis
    shifted=original.copy();shifted[:,2]+=psi
    dy,dz=original[:,4],original[:,5]
    shifted[:,4]=dy*np.cos(psi)-dz*np.sin(psi)+height*np.sin(psi)
    shifted[:,5]=dy*np.sin(psi)+dz*np.cos(psi)+height*(1-np.cos(psi))
    before=independent_hits(axis,theta,tangent,original,radius,height)
    after=independent_hits(axis,theta,tangent,shifted,radius,height)
    np.testing.assert_allclose(after[:,0],before[:,0],atol=1e-12)
    np.testing.assert_allclose(after[:,1],before[:,1]-radius*psi,atol=1e-12)
