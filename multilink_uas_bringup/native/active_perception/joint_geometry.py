"""Validate and propagate read-only ROVIO velocity/3D-landmark joint messages."""
import numpy as np


def decode(data):
    a=np.asarray(data,dtype=float)
    if a.ndim!=1 or len(a)<6 or not np.isfinite(a).all() or a[0]!=2:
        raise ValueError('Invalid joint geometry message')
    stamp,count=a[1:3]
    if stamp!=round(stamp) or not 0<=stamp<2**53 or count!=round(count) or not 0<=count<=25:
        raise ValueError('Invalid joint geometry stamp/count')
    n=int(count);size=3+3*n
    if len(a)!=6+4*n+size*size:raise ValueError('Invalid joint geometry length')
    tracks=a[6:6+4*n].reshape(n,4);ids=tracks[:,0]
    if np.any(ids<0) or not np.array_equal(ids,np.rint(ids)) or len(set(ids))!=n:
        raise ValueError('Invalid joint geometry identities')
    C=a[6+4*n:].reshape(size,size).copy()
    if not np.allclose(C,C.T,atol=1e-10,rtol=1e-10):raise ValueError('Asymmetric joint geometry covariance')
    return int(stamp),ids.astype(np.int64),tracks[:,1:].copy(),a[3:6].copy(),C


def subset(ids,C,selected):
    indices={int(i):j for j,i in enumerate(ids)}
    if len(indices)!=len(ids) or len(set(selected))!=len(selected):raise ValueError('Duplicate geometry identity')
    try:columns=[0,1,2]+[3+3*indices[int(i)]+j for i in selected for j in range(3)]
    except KeyError as e:raise ValueError('Missing geometry identity') from e
    return C[np.ix_(columns,columns)].copy()


def advance(points,velocity,C,rotation,age_s):
    """Constant world-velocity prediction into a new body frame, with covariance.

    rotation maps new-body vectors to the old body. Unknown future acceleration
    is not invented here; this is solely a short-horizon planning approximation.
    """
    R=np.asarray(rotation);n=len(points)
    if R.shape!=(3,3) or not np.isfinite(R).all() or not np.allclose(R.T@R,np.eye(3),atol=1e-10) or np.linalg.det(R)<0:
        raise ValueError('Invalid geometry frame rotation')
    if not np.isfinite(age_s) or age_s<0:raise ValueError('Invalid geometry age')
    if C.shape!=(3+3*n,3+3*n):raise ValueError('Invalid geometry covariance size')
    A=np.zeros_like(C);A[:3,:3]=R.T
    for i in range(n):
        A[3+3*i:6+3*i,:3]=-age_s*R.T
        A[3+3*i:6+3*i,3+3*i:6+3*i]=R.T
    return (points-velocity*age_s)@R,velocity@R,A@C@A.T
