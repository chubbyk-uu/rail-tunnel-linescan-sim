"""Numeric normal equations on already-public CSR arrays; bounded CPU scratch."""
import ctypes as ct
from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy.sparse import csr_matrix,csc_matrix
from ament_index_python.packages import get_package_prefix
from .session import sha256_file


@lru_cache(maxsize=1)
def backend():
    path=Path(get_package_prefix('ssb_core'))/'lib/libssb_normal.so'
    lib=ct.CDLL(str(path))
    lib.ssb_normal_abi.restype=ct.c_int
    if lib.ssb_normal_abi()!=1:
        raise RuntimeError('unsupported CPU normal-equation ABI')
    lib.ssb_normal_accumulate.argtypes=[ct.c_int,*([ct.c_int64]*4),ct.c_int,*([ct.c_void_p]*6)]
    lib.ssb_normal_accumulate.restype=ct.c_int
    return lib,dict(name='cpu_numeric_csr',abi=1,library=str(path),library_sha256=sha256_file(path),
                    dense_scratch_budget_bytes=192<<20,
                    memory_scope='dense kernel scratch; sparse results remain part of measured optimizer RSS',
                    source='public Jacobian and residual only; no file access in kernel')


def accumulator(matrix,residual):
    matrix=csr_matrix(matrix,dtype=np.float64)
    residual=np.ascontiguousarray(residual,dtype=np.float64)
    if (not 0<matrix.shape[1]<=2048 or residual.shape!=(matrix.shape[0],) or
        matrix.indptr.shape!=(matrix.shape[0]+1,) or matrix.indices.shape!=matrix.data.shape or
        matrix.indptr[0]!=0 or matrix.indptr[-1]!=len(matrix.data) or
        not matrix.has_canonical_format or matrix.indices.dtype!=matrix.indptr.dtype or
        matrix.indices.dtype not in (np.dtype('int32'),np.dtype('int64')) or
        any(not a.flags.c_contiguous for a in (matrix.indptr,matrix.indices,matrix.data))):
        raise ValueError('finite canonical contiguous CSR with matching residual and <=2048 columns required')
    lib,_=backend();n=matrix.shape[1]
    def part(first,last):
        if not 0<=first<=last<=matrix.shape[0]:raise ValueError('normal row range outside matrix')
        normal=np.empty((n,n),np.float64);gradient=np.empty(n,np.float64)
        arrays=(matrix.indptr,matrix.indices,matrix.data,residual,normal,gradient)
        code=lib.ssb_normal_accumulate(n,first,last,matrix.shape[0],len(matrix.data),matrix.indices.dtype.itemsize*8,
                                       *(ct.c_void_p(a.ctypes.data) for a in arrays))
        if code:raise ValueError(f'invalid or nonfinite numeric normal-equation input/result ({code})')
        return csc_matrix(normal),gradient
    return part,max(1,(192<<20)//(n*n*8+n*8))
