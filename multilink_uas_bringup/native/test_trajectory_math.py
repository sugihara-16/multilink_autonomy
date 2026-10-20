"""Checks coordinate alignment and the actual trajectory sent to DRAGON."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from trajectory_math import aligned_error,interpolate_pose,segment_value

def test_rigid_alignment_and_six_axis_error():
    r0=Rotation.from_euler('xyz',[.2,-.1,.7]);p0=np.array([40,5,2.])
    p=np.array([2.,3.,1.]);r=Rotation.from_euler('xyz',[.1,.2,-.3])
    gt=r0.apply(p)+p0;qgt=r0*r
    _,_,e=aligned_error(p,r,gt,qgt,(r0,p0));assert np.allclose(e,0,atol=1e-10)
    qperturbed=r*Rotation.from_euler('xyz',[1,2,3],degrees=True)
    _,_,e=aligned_error(p+r0.inv().apply([.1,-.2,.3]),qperturbed,gt,qgt,(r0,p0))
    assert np.allclose(e,[.1,-.2,.3,1,2,3],atol=1e-10)

def test_same_timestamp_interpolation():
    a=[0,0,0,0,0,0,0,1];b=[2,2,4,6,*Rotation.from_euler('z',90,degrees=True).as_quat()]
    p,r=interpolate_pose(a,b,1);assert np.allclose(p,[1,2,3]);assert np.allclose(r.as_euler('xyz',degrees=True),[0,0,45])

def test_certified_segments_and_fc_cog_transform():
    segments=json.loads((Path(__file__).parent/'routes/cave_exploration/segments.json').read_text())['segments']
    for s in segments:
        for u in np.linspace(0,1,31):
            p,v,y,w=segment_value(s,s['t0']+u*s['duration'])
            assert np.linalg.norm(v)<=.8+1e-8 and abs(w)<=.3+1e-8
        assert np.allclose(segment_value(s,s['t0'])[0],s['start'])
        assert np.allclose(segment_value(s,s['t0']+s['duration'])[0],s['end'])
    p=np.array([40.,5.,2.]);r=Rotation.from_euler('z',1.3);offset=np.array([.1,.2,.03])
    cog=p+r.apply(offset);assert np.allclose(cog-r.apply(offset),p)

def test_straight_cruise_and_ramps():
    s=dict(kind='trapezoid',duration=102.,ramp_s=2.,speed_mps=1.,start=[0,0,0],end=[100,0,0],yaw0=0.,t0=0.)
    assert np.allclose(segment_value(s,0)[0],[0,0,0])
    assert np.allclose(segment_value(s,102)[0],[100,0,0])
    for t in [0,102]:assert np.allclose(segment_value(s,t)[1],0)
    for t in [2,30,100]:assert np.allclose(segment_value(s,t)[1],[1,0,0])
    for t in [1,2,50,100,101]:
        derivative=(segment_value(s,t+1e-4)[0]-segment_value(s,t-1e-4)[0])/2e-4
        assert np.allclose(derivative,segment_value(s,t)[1],atol=3e-5)

if __name__=='__main__':
    test_rigid_alignment_and_six_axis_error();test_same_timestamp_interpolation();test_certified_segments_and_fc_cog_transform();test_straight_cruise_and_ramps();print('4 checks passed')
