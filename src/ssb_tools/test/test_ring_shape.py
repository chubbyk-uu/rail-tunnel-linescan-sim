import importlib.util
import numpy as np
import pytest
from pathlib import Path
spec=importlib.util.spec_from_file_location('ring_eval',Path(__file__).resolve().parents[3]/'tools'/'evaluate_ring_shape.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_inverse_retraces_returned_positions_and_keeps_fixed_targets():
    target=np.array([[1.2,-1],[1.2,0],[1.2,1.]])
    initial=target.copy()
    def trace(p):
        actual=p.copy();actual[:,0]+=.003*p[:,1]**2+.2
        return actual,np.full(len(p),2),np.where(p[:,0]<1.1,0,1)
    location,valid,residual,material,band=module.locate(target,initial,trace)
    np.testing.assert_allclose(location[:,0],1.-.003*target[:,1]**2)
    assert valid.all() and residual.max()<1e-12
    np.testing.assert_array_equal(band,0)
    np.testing.assert_array_equal(target[:,0],1.2)


def test_ring_bow_is_retained_instead_of_fitting_a_line():
    q=np.linspace(-1,1,33)
    target=np.column_stack((np.full(33,1.2),q))
    p=target.copy();p[:,0]+=.1+.002*q
    result=module.ring_summary(target,p,np.ones(33,bool),1,33)
    assert result['rings'][0]['peak_to_peak_mm']==pytest.approx(4.)
    assert result['p95_abs_from_per_ring_median_mm']>1.8


def test_common_position_offset_does_not_create_ring_bow():
    target=np.column_stack((np.full(33,1.2),np.linspace(-1,1,33)))
    result=module.ring_summary(target,target+[.1,.02],np.ones(33,bool),1,33)
    assert result['p95_abs_from_per_ring_median_mm']==0


def test_missing_ring_point_is_unmeasurable_and_not_dropped():
    target=np.column_stack((np.full(33,1.2),np.linspace(-1,1,33)))
    valid=np.ones(33,bool);valid[3]=False
    result=module.ring_summary(target,target,valid,1,33)
    assert result['status']=='unmeasurable' and result['missing']==1
    assert result['p95_abs_from_per_ring_median_mm'] is None


def test_empty_ring_collection_cannot_create_evidence():
    with pytest.raises(ValueError):
        module.ring_summary(np.empty((0,2)),np.empty((0,2)),np.empty(0,bool),0,33)


@pytest.mark.parametrize('option',[dict(iterations=0),dict(iterations=True),dict(tolerance=0),dict(tolerance=np.nan)])
def test_inverse_rejects_zero_work_or_invalid_evidence_tolerance(option):
    from ssb_tools.evaluate_defect_preservation import locate
    target=np.array([[1.,0.]])
    with pytest.raises(ValueError):locate(target,target,lambda p:(p,np.zeros(1,int),np.zeros(1,int)),**option)
