"""Limit gaze changes that discard other successfully tracked image bearings.

All geometry is angular. Compare the proposed slew with holding the measured
encoder angle under the same predicted body/feature motion. No truth, metric
velocity, or estimator weighting is used.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def protect_tracks(unit,rates,omega,R_home,current,command,dt,cfg,limits):
    unit=np.asarray(unit).reshape(-1,3);rates=np.asarray(rates).reshape(-1,3)
    current=np.asarray(current);command=np.asarray(command)
    if not len(unit):return command,dict(retention_scale=1.,retention_reference=0.)
    border=cfg.get('border_fraction',.2)
    fraction=cfg.get('retain_fraction',.9)
    if not 0<border<=1 or not 0<=fraction<=1:raise ValueError('Invalid retention setting')
    hfov,vfov=cfg['tan_half_hfov'],cfg['tan_half_vfov']
    def weights(rays,angles):
        optical=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        matrices=R_home@Rotation.from_euler('ZY',angles).as_matrix()@optical
        p=np.einsum('ni,cij->cnj',rays,matrices)
        edge=np.maximum(abs(p[:,:,0])/np.maximum(p[:,:,2],1e-12)/hfov,
                        abs(p[:,:,1])/np.maximum(p[:,:,2],1e-12)/vfov)
        return np.clip((1.-edge)/border,0.,1.)*(p[:,:,2]>0.)
    initial=weights(unit,current[None])[0]
    scales=np.linspace(0.,1.,33)
    allowed=np.ones(len(scales),dtype=bool)
    references=[];proposed=[]
    horizons=sorted(set([float(dt),max(float(dt),cfg.get('prediction_horizon_s',.8))]))
    for horizon in horizons:
        predicted=Rotation.from_rotvec(-np.asarray(omega)*horizon).apply(unit)+rates*horizon
        predicted/=np.maximum(np.linalg.norm(predicted,axis=1)[:,None],1e-12)
        # The actual next command is bounded already; extrapolate only for the
        # short lookahead, respecting joint limits. Holding is always feasible.
        angles=np.clip(current+(command-current)*scales[:,None]*(horizon/dt),*limits)
        score=np.minimum(weights(predicted,angles),initial[None]).sum(axis=1)
        allowed&=score>=fraction*score[0]-1e-12
        references.append(float(score[0]));proposed.append(float(score[-1]))
    chosen=np.flatnonzero(allowed)[-1]
    scale=float(scales[chosen])
    return current+scale*(command-current),dict(retention_scale=scale,
        retention_reference=float(initial.sum()),retention_hold_future=references,
        retention_full_slew_future=proposed,retention_horizons_s=horizons)
