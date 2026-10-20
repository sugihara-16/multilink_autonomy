"""Keep mature ROVIO landmarks in view, with evidence-based handovers.

LiDAR weak-direction information ranks replacement anchors. Once chosen, an
anchor is followed by its measured bearing, rather than switching views for
each small score change. No pose truth, trajectory commands, or extra sensor.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from core import Planner,camera_rotation,directional_uncertainty_gain


class Fixation:
    def __init__(self,fisher,cfg):
        self.fisher=fisher;self.cfg=cfg
        motion_cfg=cfg
        if cfg.get('fixation_yaw_only',False):
            motion_cfg=dict(cfg)
            motion_cfg['limits_min']=[cfg['limits_min'][0],0.]
            motion_cfg['limits_max']=[cfg['limits_max'][0],0.]
        self.motion=Planner(fisher,motion_cfg)
        limit_cfg=dict(motion_cfg);limit_cfg.pop('scan_speed',None)
        self.limiter=Planner(fisher,limit_cfg)
        self.anchors=[];self.missing_since=None
        self.quality_bad_since=None;self.quality_good_since=None;self.recovering=False
        self.gate_active=False;self.gate_probability=0.
        self.gate_quality_bad_since=None;self.rest_angles=np.zeros(2)
        if cfg.get('fixation_image_stabilization',False):
            from image_stabilization import ImageStabilization
            self.image_stabilization=ImageStabilization(fisher,cfg)

    def choose(self,record,H,U,weak,R_home,tbc,current,velocity,dt,omega,now):
        if self.cfg.get('fixation_image_stabilization',False):
            return self.image_stabilization.choose(record,R_home,tbc,current,dt,omega)
        if not self.cfg.get('fixation_degeneracy_gate',False):
            return self._choose(record,H,U,weak,R_home,tbc,current,velocity,dt,omega,now)
        if not 0<dt<=self.cfg['max_dt']:raise ValueError('Invalid fixation planning dt')
        probability=float(np.max(weak))
        if not np.isfinite(probability) or not 0<=probability<=1:raise ValueError('Invalid degeneracy probability')
        tau=self.cfg.get('fixation_gate_filter_s',1.)
        lower=self.cfg.get('fixation_gate_exit',.2);upper=self.cfg.get('fixation_gate_enter',.5)
        if not tau>0 or not 0<=lower<upper<=1:raise ValueError('Invalid degeneracy gate')
        self.gate_probability+=(-np.expm1(-dt/tau))*(probability-self.gate_probability)
        if self.gate_probability>=upper:self.gate_active=True
        elif self.gate_probability<=lower:self.gate_active=False
        gate=dict(degeneracy_gate_active=self.gate_active,degeneracy_gate_probability=self.gate_probability)
        if self.cfg.get('fixation_quality_recovery',False):
            ids,points,covs,ages,*unused=record
            rays=points-tbc;distance=np.linalg.norm(rays,axis=1)
            unit=rays/np.maximum(distance[:,None],1e-9)
            relative_sigma=np.sqrt(np.maximum(0.,np.einsum('ni,nij,nj->n',unit,covs,unit)))/np.maximum(distance,1e-9)
            mature=(ages>=self.cfg.get('fixation_min_age',1.))&(relative_sigma<=self.cfg.get('fixation_max_relative_sigma',.3))
            mature&=(distance>.3)&np.isfinite(covs).all(axis=(1,2))
            mature&=np.linalg.eigvalsh(covs).min(axis=1)>=-1e-9
            if int(mature.sum())>=self.cfg['min_features']:self.gate_quality_bad_since=None
            else:
                if self.gate_quality_bad_since is None:self.gate_quality_bad_since=now
                if now-self.gate_quality_bad_since>=self.cfg.get('recovery_delay_s',1.):self.recovering=True
            gate['mature_tracks']=int(mature.sum())
        enabled=self.gate_active or (self.cfg.get('fixation_quality_recovery',False) and self.recovering)
        gate.update(view_control_enabled=bool(enabled),rest_angles=self.rest_angles.tolist())
        if not enabled:
            self.anchors=[];self.missing_since=None
            self.quality_bad_since=None;self.quality_good_since=None;self.recovering=False
            command=self.motion.command(current,self.rest_angles,R_home,np.zeros(3),dt)
            return command,dict(reason='fixation_lidar_sufficient',target_angles=self.rest_angles.tolist(),**gate)
        was_recovering=self.recovering
        command,report=self._choose(record,H,U,weak,R_home,tbc,current,velocity,dt,omega,now)
        if self.cfg.get('fixation_keep_recovered_view',False) and was_recovering and not self.recovering:
            self.rest_angles=np.deg2rad(self.cfg.get('fixation_recovery_angles_deg',[0.,0.]))
            gate['rest_angles']=self.rest_angles.tolist()
            if not self.gate_active:
                command=self.motion.command(current,self.rest_angles,R_home,np.zeros(3),dt)
                report=dict(reason='fixation_recovery_completed',target_angles=self.rest_angles.tolist())
        report.update(gate)
        return command,report

    def _choose(self,record,H,U,weak,R_home,tbc,current,velocity,dt,omega,now):
        ids,points,covs,ages,*bearing_motion=record
        if not 0<dt<=self.cfg['max_dt']:raise ValueError('Invalid fixation planning dt')
        ray=points-tbc;distance=np.linalg.norm(ray,axis=1)
        unit=ray/np.maximum(distance[:,None],1e-9)
        relative_sigma=np.sqrt(np.maximum(0.,np.einsum('ni,nij,nj->n',unit,covs,unit)))/np.maximum(distance,1e-9)
        lo,hi=self.motion.limits();home=unit@R_home
        angles=np.c_[np.arctan2(home[:,1],home[:,0]),np.arctan2(-home[:,2],np.linalg.norm(home[:,:2],axis=1))]
        mature=(ages>=self.cfg.get('fixation_min_age',1.))&(relative_sigma<=self.cfg.get('fixation_max_relative_sigma',.3))
        mature&=(distance>.3)&np.isfinite(covs).all(axis=(1,2))
        mature&=np.linalg.eigvalsh(covs).min(axis=1)>=-1e-9
        if self.cfg.get('fixation_recover_quality',False):
            enough=int(mature.sum())>=self.cfg['min_features']
            delay=self.cfg.get('recovery_delay_s',1.)
            if enough:
                self.quality_bad_since=None
                if self.quality_good_since is None:self.quality_good_since=now
            else:
                self.quality_good_since=None
                if self.quality_bad_since is None:self.quality_bad_since=now
                if now-self.quality_bad_since>=delay:self.recovering=True
            if self.recovering:
                # Recovery changes the physical view, never the filter state.
                self.anchors=[];self.missing_since=None
                reference=np.deg2rad(self.cfg.get('fixation_recovery_angles_deg',[0.,0.]))
                if reference.shape!=(2,) or not np.isfinite(reference).all() or np.any(reference<lo) or np.any(reference>hi):
                    raise ValueError('Recovery view exceeds planning limits')
                settled=np.linalg.norm(current-reference)<=np.deg2rad(1.)
                if enough and settled and now-self.quality_good_since>=delay:
                    self.recovering=False
                else:
                    command=self.motion.command(current,reference,R_home,np.zeros(3),dt)
                    reason='fixation_recover_view' if np.any(reference) else 'fixation_recover_forward'
                    return command,dict(reason=reason,anchors=[],mature_tracks=int(mature.sum()),
                        target_angles=reference.tolist(),tracked_points=len(ids),recovery_settled=bool(settled))
            if not enough:
                return np.asarray(current),dict(reason='fixation_wait_for_quality',anchors=self.anchors,mature_tracks=int(mature.sum()),tracked_points=len(ids))
        reachable=np.all((angles>=lo)&(angles<=hi),axis=1)
        if self.cfg.get('fixation_yaw_only',False):
            # A landmark need not lie on the horizontal optical axis; it only
            # needs to remain vertically visible while yaw follows it.
            reachable=(angles[:,0]>=lo[0])&(angles[:,0]<=hi[0])
            reachable&=np.abs(home[:,2])<np.linalg.norm(home[:,:2],axis=1)*self.cfg['tan_half_vfov']
        if self.cfg.get('fixation_retain_visible',False):
            pc=ray@camera_rotation(R_home,*current)
            in_view=(pc[:,2]>.1)&(np.abs(pc[:,0])<pc[:,2]*self.cfg['tan_half_hfov'])&(np.abs(pc[:,1])<pc[:,2]*self.cfg['tan_half_vfov'])
            visible=set(int(i) for i in ids[in_view])
        else:visible=set(int(i) for i in ids[reachable])
        retained=[i for i in self.anchors if i in visible]
        reason='follow_mature_landmarks'
        if self.anchors and not retained:
            if self.missing_since is None:self.missing_since=now
            if now-self.missing_since<self.cfg.get('fixation_loss_timeout',.25):
                return np.asarray(current),dict(reason='fixation_wait_for_track',anchors=self.anchors,mature_tracks=int(mature.sum()))
        else:self.missing_since=None
        if not retained:
            eligible=mature&reachable
            if self.cfg.get('fixation_retain_visible',False):eligible&=in_view
            valid=np.flatnonzero(eligible)
            if not len(valid):
                self.anchors=[]
                return np.asarray(current),dict(reason='fixation_wait_for_depth',anchors=[],mature_tracks=int(mature.sum()))
            H0=(H+H.T)*.5+np.diag(1/np.square(self.cfg['prior_std']))
            L=np.linalg.cholesky(H0);s=np.linalg.solve(L,U);base=np.sum(s*s,axis=0)
            _,_,parts=self.fisher.information(points[valid],covs[valid],camera_rotation(R_home,*current),tbc,self.cfg['bearing_sigma'],per_point=True)
            total=H0[None]+parts;L=np.linalg.cholesky((total+total.transpose(0,2,1))*.5)
            s=np.linalg.solve(L,np.broadcast_to(U,L.shape))
            score=directional_uncertainty_gain(base,np.sum(s*s,axis=1),weak)
            # In nondegenerate geometry, preserve the most certain mature
            # bearing instead of assigning an artificial visual gain.
            order=np.lexsort((relative_sigma[valid],-score));best=valid[order[0]]
            radius=.5*np.arctan(self.cfg['tan_half_vfov'])
            cluster=valid[(unit[valid]@unit[best])>=np.cos(radius)]
            indices=[best]+[i for i in valid[order] if i!=best and i in cluster]
            retained=[int(ids[i]) for i in indices[:3]]
            reason='select_mature_landmarks';self.missing_since=None
        self.anchors=retained
        selected=np.isin(ids,self.anchors)
        aim=np.mean(unit[selected],axis=0)@R_home
        raw_target=np.array([np.arctan2(aim[1],aim[0]),np.arctan2(-aim[2],np.linalg.norm(aim[:2]))])
        target=np.clip(raw_target,lo,hi)
        deadband_report={}
        if self.cfg.get('fixation_deadband',False):
            # Minimal reorientation that retains the established landmarks.
            # Leaving them away from the centre preserves the rest of the view.
            if self.cfg.get('fixation_bearing_motion',False):
                if not bearing_motion:raise ValueError('Missing observed bearing motion')
                horizon=max(dt,self.cfg.get('fixation_deadband_horizon_s',dt))
                future=Rotation.from_rotvec(-omega*horizon).apply(unit[selected])+bearing_motion[0][selected]*horizon
            else:
                future=Rotation.from_rotvec(-omega*dt).apply(points[selected]-velocity*dt)-tbc
            yaw=np.linspace(lo[0],hi[0],int(np.ceil((hi[0]-lo[0])/np.deg2rad(2)))+1)
            pitch=np.linspace(lo[1],hi[1],int(np.ceil((hi[1]-lo[1])/np.deg2rad(2)))+1)
            yy,pp=np.meshgrid(yaw,pitch,indexing='ij')
            candidates=np.vstack([current,target,np.c_[yy.ravel(),pp.ravel()]])
            optical=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
            rotations=R_home@Rotation.from_euler('ZY',candidates).as_matrix()@optical
            pc=np.einsum('ni,cij->cnj',future,rotations)
            border=1.-2.*self.cfg.get('border_fraction',.2)
            if not 0<border<1:raise ValueError('Invalid fixation image margin')
            xy=np.abs(pc[:,:,:2])/np.maximum(pc[:,:,2:3],1e-9)
            normalized=xy/np.array([self.cfg['tan_half_hfov'],self.cfg['tan_half_vfov']])
            violation=np.maximum(normalized-border,0.).max(axis=(1,2))
            violation[np.any(pc[:,:,2]<=.1,axis=1)]=np.inf
            if not np.isfinite(violation).any():
                return np.asarray(current),dict(reason='fixation_invalid_motion_prediction',anchors=self.anchors,mature_tracks=int(mature.sum()))
            movement=np.linalg.norm(candidates-current,axis=1)
            best=np.lexsort((movement,violation))[0]
            target=candidates[best]
            command=self.limiter.command(current,target,R_home,np.zeros(3),dt)
            deadband_report=dict(deadband_violation=float(violation[best]),
                deadband_hold=bool(best==0),deadband_inner_fraction=float(border))
        elif self.cfg.get('fixation_motion_prediction',False):
            # Following a nearby point can require >5 deg/s solely due to
            # body translation. This is stabilization, not a new view search.
            if self.cfg.get('fixation_bearing_motion',False):
                if not bearing_motion:raise ValueError('Missing observed bearing motion')
                future=Rotation.from_rotvec(-omega*dt).apply(unit[selected])+bearing_motion[0][selected]*dt
            else:
                future=Rotation.from_rotvec(-omega*dt).apply(points[selected]-velocity*dt)-tbc
            future/=np.maximum(np.linalg.norm(future,axis=1)[:,None],1e-9)
            aim_future=np.mean(future,axis=0)@R_home
            future_angles=np.array([np.arctan2(aim_future[1],aim_future[0]),np.arctan2(-aim_future[2],np.linalg.norm(aim_future[:2]))])
            tracking_step=np.arctan2(np.sin(future_angles-raw_target),np.cos(future_angles-raw_target))
            if self.cfg.get('fixation_yaw_only',False):tracking_step[1]=0.
            centering=target-current
            centering*=min(1.,self.cfg.get('scan_speed',self.cfg['max_speed'])*dt/max(np.linalg.norm(centering),1e-9))
            command=self.limiter.command(current,current+tracking_step+centering,R_home,np.zeros(3),dt)
        else:
            command=self.motion.command(current,target,R_home,omega,dt)
        if self.cfg.get('fixation_protect_other_tracks',False):
            if not bearing_motion:raise ValueError('Track protection requires observed bearing motion')
            from retention import protect_tracks
            command,retention_report=protect_tracks(unit,bearing_motion[0],omega,R_home,current,command,dt,self.cfg,(lo,hi))
            deadband_report.update(retention_report)
        return command,dict(reason=reason,anchors=self.anchors,target_angles=target.tolist(),
            mature_tracks=int(mature.sum()),anchor_relative_sigma=relative_sigma[selected].tolist(),
            anchor_age_s=ages[selected].tolist(),**deadband_report)
