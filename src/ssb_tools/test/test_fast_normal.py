import numpy as np
import pytest
from scipy.sparse import csr_matrix

from ssb_tools.fast_normal import accumulator


@pytest.mark.parametrize('index_dtype',[np.int32,np.int64])
@pytest.mark.parametrize('chunk',[1,7,100])
def test_numeric_csr_matches_independent_dense_sum_and_fixed_chunk_order(index_dtype,chunk):
    rng=np.random.default_rng(220)
    dense=rng.normal(size=(41,13));dense[rng.random(dense.shape)<.7]=0
    residual=rng.normal(size=41);matrix=csr_matrix(dense)
    matrix.indices=matrix.indices.astype(index_dtype);matrix.indptr=matrix.indptr.astype(index_dtype)
    part,cap=accumulator(matrix,residual)
    results=[part(first,min(first+chunk,len(residual))) for first in range(0,len(residual),chunk)]
    normal=sum((v[0].toarray() for v in results));gradient=sum(v[1] for v in results)
    np.testing.assert_allclose(normal,dense.T@dense,rtol=1e-13,atol=1e-13)
    np.testing.assert_allclose(gradient,dense.T@residual,rtol=1e-13,atol=1e-13)
    assert cap*(13*13*8+13*8)<=192<<20
    repeated=[part(first,min(first+chunk,len(residual))) for first in range(0,len(residual),chunk)]
    for (a,b),(c,d) in zip(results,repeated):
        np.testing.assert_array_equal(a.toarray(),c.toarray());np.testing.assert_array_equal(b,d)


@pytest.mark.parametrize('bad',['residual_shape','residual_nan','data_nan','overflow','column',
                               'negative_column','row_bounds','too_many_columns','row_offsets'])
def test_invalid_or_unsafe_normal_arrays_fail_before_using_results(bad):
    matrix=csr_matrix(np.eye(3));residual=np.ones(3)
    if bad=='residual_shape':residual=np.ones(2)
    elif bad=='residual_nan':residual[0]=np.nan
    elif bad=='data_nan':matrix.data[0]=np.nan
    elif bad=='overflow':matrix.data[0]=1e308
    elif bad=='column':matrix.indices[0]=3
    elif bad=='negative_column':matrix.indices[0]=-1
    elif bad=='too_many_columns':matrix=csr_matrix((3,2049))
    elif bad=='row_offsets':matrix.indptr[-1]=100
    with pytest.raises(ValueError):
        part,_=accumulator(matrix,residual)
        part(-1,3) if bad=='row_bounds' else part(0,3)


def test_empty_and_zero_rows_are_finite_not_uninitialized():
    part,_=accumulator(csr_matrix((3,5)),np.ones(3))
    normal,gradient=part(0,3)
    assert normal.nnz==0
    np.testing.assert_array_equal(gradient,np.zeros(5))
    normal,gradient=part(1,1)
    assert normal.nnz==0
    np.testing.assert_array_equal(gradient,np.zeros(5))


def test_native_fixed_row_blocks_are_identical_across_worker_counts(monkeypatch):
    import ssb_tools.global_geometry as geometry
    from ssb_tools.optimize_bands import normal_equations
    rng=np.random.default_rng(70)
    dense=rng.normal(size=((1<<17)+17,13));dense[rng.random(dense.shape)<.7]=0
    residual=rng.normal(size=len(dense));matrix=csr_matrix(dense)
    reference=normal_equations(matrix,residual)
    values=[]
    for workers in (1,4):
        monkeypatch.setattr(geometry,'THREADS',workers)
        normal,gradient=normal_equations(matrix,residual,native=True)
        values.append((normal.toarray(),gradient))
    np.testing.assert_allclose(values[0][0],reference[0].toarray(),rtol=1e-12,atol=1e-8)
    np.testing.assert_allclose(values[0][1],reference[1],rtol=1e-12,atol=1e-8)
    for a,b in zip(values[0],values[1]):np.testing.assert_array_equal(a,b)
