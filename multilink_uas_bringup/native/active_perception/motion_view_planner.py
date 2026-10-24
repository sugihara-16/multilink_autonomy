"""Motion-conditioned visual view selection; estimator and sensors are unchanged.

The planning proxy measures information in a pair of bearing observations about
relative motion, with common landmark uncertainty propagated through BOTH
observations. It is not the posterior covariance of the ROVIO filter.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from core import Planner as BasePlanner, camera_rotation, projected_uncertainty, directional_uncertainty_gain, uncertainty_gain, skew


def motion_information(points, covs, tbc, velocity, horizon, sigma):
    """Bearing-difference information in [rotation increment, displacement].

    Geometry is expressed in the current body frame. Gyro-compensated relative
    bearings share the same landmark covariance, so their cross-correlation
    cannot be replaced by two independent landmark errors.
    """
    p0=np.asarray(points)-tbc
    p1=p0-np.asarray(velocity)*horizon
    r0=np.linalg.norm(p0,axis=1);r1=np.linalg.norm(p1,axis=1)
    n0=p0/np.maximum(r0[:,None],1e-9);n1=p1/np.maximum(r1[:,None],1e-9)
    B0=(np.eye(3)[None]-n0[:,:,None]*n0[:,None,:])/np.maximum(r0[:,None,None],1e-9)
    B1=(np.eye(3)[None]-n1[:,:,None]*n1[:,None,:])/np.maximum(r1[:,None,None],1e-9)
    D=B1-B0
    S=2*sigma*sigma*np.eye(3)[None]+D@covs@D.transpose(0,2,1)
    J=np.concatenate([B1@np.array([skew(p+tbc) for p in p1]),-B1],axis=2)
    info=J.transpose(0,2,1)@np.linalg.solve(S,J)
    info[(r0<.3)|(r1<.3)]=0.
    return (info+info.transpose(0,2,1))*.5


def spatial_budget(points, tbc, rotations, visible, weights, cfg):
    """Expected information from spatially distributed features, not best-K.

    Average points within each image cell. One expected track per occupied cell
    avoids claiming that the detector will choose the geometrically best point.
    Duplicating a mapped corner cannot inflate a cell's information budget.
    """
    pc=np.einsum('ni,cij->cnj',points-tbc,rotations)
    uv=pc[:,:,:2]/np.maximum(pc[:,:,2,None],1e-9)
    limits=np.array([cfg['tan_half_hfov'],cfg['tan_half_vfov']])
    side=int(np.sqrt(cfg.get('feature_budget',25)))
    cell=np.clip(np.floor((uv/limits+1)*(.5*side)).astype(int),0,side-1)
    labels=cell[:,:,0]+side*cell[:,:,1]
    result=np.zeros_like(weights)
    for i in range(len(rotations)):
        ix=np.flatnonzero(visible[i])
        counts=np.bincount(labels[i,ix],minlength=side*side)
        result[i,ix]=weights[i,ix]/counts[labels[i,ix]]
    return result


class Planner(BasePlanner):
    def __init__(self,fisher,cfg):
        super().__init__(fisher,cfg)
        self.last_target=None
        self.target_age=0.

    def choose(self, points, covs, views, H, U, weakness, R_home, tbc,
               current, velocity_body, dt, obstacles=None, ages=None, angular_velocity=None, track_points=None):
        c = self.cfg
        horizon=c.get('prediction_horizon_s',0.)
        angular_velocity=np.zeros(3) if angular_velocity is None else np.asarray(angular_velocity)
        if not 0 < dt <= c['max_dt']: raise ValueError('Invalid planning dt')
        if not all(np.isfinite(a).all() for a in (H,U,weakness,current)): raise ValueError('Nonfinite state')
        # Explicit prior caps otherwise infinite unobservable variances. A scoring
        # prior only: it is never fed into MIMOSA or represented as a measurement.
        # The desired constraint is translation. Normalize relative LiDAR
        # weakness so a good geometry interval does not silently turn the
        # controller into a forward-facing heuristic.
        ev, axes=np.linalg.eigh((H[3:,3:]+H[3:,3:].T)*.5)
        weights_l=1/np.maximum(ev,1/float(c['prior_std'][3])**2)
        weights_l/=weights_l.sum()
        U=np.zeros((6,6));U[3:,3:]=axes
        U[:3,:3]=np.eye(3)
        weakness=np.r_[np.zeros(3),weights_l]
        prior = np.diag(1/np.square(c['prior_std']))
        H0 = (H+H.T)*.5 + prior
        baseline = projected_uncertainty(H0,U,weakness)
        tracking_points=points if track_points is None else np.asarray(track_points).reshape(-1,3)
        available=len(tracking_points)
        independent_map_available=c.get('allow_map_recovery',False) and len(points)>=c['min_features']
        if (baseline < 1e-12 and not horizon) or (available < c['min_features'] and not independent_map_available):
            if c.get('recovery_delay_s') is not None and available<c['min_features']:
                self.low_track_time+=dt
                if self.low_track_time>=c['recovery_delay_s']:
                    return self.command(current,np.zeros(2),R_home,angular_velocity,dt),dict(reason='recover_forward_no_map',candidates=[])
            return np.asarray(current), dict(reason='no_weak_direction_or_map', candidates=[])
        lower,upper=self.limits()
        self.target_age+=dt
        candidates = [np.asarray(current)]
        if self.last_target is not None:candidates.append(self.last_target.copy())
        if horizon:
            # Candidate keeping the current optical direction fixed in space
            # while the estimated body rotates; no flight trajectory or truth.
            forward=camera_rotation(R_home,*current)[:,2]
            forward=R_home.T@Rotation.from_rotvec(-angular_velocity*horizon).apply(forward)
            stabilization=np.array([np.arctan2(forward[1],forward[0]),np.arctan2(-forward[2],np.linalg.norm(forward[:2]))])
            candidates.append(np.clip(stabilization,lower,upper))
        # Global bounded search over previously observed geometry, then slew
        # towards the selected view. Scores refer to the TARGET, not to the
        # intermediate encoder angle. This avoids a purely local gradient trap.
        span = c['max_speed']*dt
        for yaw in np.linspace(lower[0],upper[0],17):
            for pitch in np.linspace(lower[1],upper[1],c.get('pitch_samples',5)):
                candidates.append(np.array([yaw,pitch]))
        dist=np.linalg.norm(points-tbc,axis=1)
        direction=(points-tbc)/np.maximum(dist[:,None],1e-9)
        base_ok=(dist>.3)&(dist<c['range_max'])&(np.sum(direction*views,axis=1)>c['min_view_cos'])
        if obstacles is not None and len(obstacles):
            rays=obstacles-tbc;ranges=np.linalg.norm(rays,axis=1)
            good=np.isfinite(ranges)&(ranges>.2);rays=rays[good]/ranges[good,None];ranges=ranges[good]
            if c.get('indexed_occlusion',False):
                from occlusion_queries import unoccluded
                base_ok=unoccluded(direction,dist,base_ok,rays,ranges,c['occlusion_cos'],c['occlusion_margin'])
            else:
                for i in np.flatnonzero(base_ok):
                    aligned=rays@direction[i]>c['occlusion_cos']
                    if np.any(ranges[aligned]+c['occlusion_margin']<dist[i]):base_ok[i]=False
        # With isotropic bearing noise and a common optical centre the full
        # body-pose information for each point is orientation invariant. Only
        # visibility changes. Cache per-point relative-motion planning information for this step.
        contributions=np.zeros((len(points),6,6))
        if base_ok.any():
            contributions[base_ok]=motion_information(points[base_ok],covs[base_ok],tbc,
                velocity_body,c.get('motion_information_horizon_s',2.),c['bearing_sigma'])
        reliable=c.get('tracking_aware',False)
        ages=np.zeros(len(points)) if ages is None else np.asarray(ages)
        fresh=ages<c.get('current_track_age',.15)
        if reliable:
            # Historical landmarks are predictions, not independent new observations.
            contributions*=np.exp(-ages/c.get('evidence_decay_s',2.))[:,None,None]
        future_points=Rotation.from_rotvec(-angular_velocity*horizon).apply(points-velocity_body*horizon) if horizon else points
        def visibility(angles,p,mask):
            pc=(p-tbc)@camera_rotation(R_home,*angles)
            uv=np.abs(pc[:,:2])/np.maximum(pc[:,2,None],1e-9)
            edge=np.maximum(uv[:,0]/c['tan_half_hfov'],uv[:,1]/c['tan_half_vfov'])
            ok=(pc[:,2]>.1)&mask&(edge<1.)
            # Keep established patches away from the image edge while rotating.
            w=np.clip((1-edge)/c.get('border_fraction',.2),0,1) if reliable else ok.astype(float)
            return ok,w*ok
        tracking_mask=base_ok if track_points is None else np.ones(available,dtype=bool)
        tracking_fresh=fresh if track_points is None else np.ones(available,dtype=bool)
        current_ok,current_w=visibility(current,tracking_points,tracking_mask)
        retained_base=float(np.sum(current_w*tracking_fresh))
        recovery_count=retained_base if track_points is None else int(np.sum(current_ok*tracking_fresh))
        if c.get('recovery_delay_s') is not None:
            self.low_track_time=self.low_track_time+dt if recovery_count<c['min_features'] and not independent_map_available else 0.
            if self.low_track_time>=c['recovery_delay_s']:
                return self.command(current,np.zeros(2),R_home,angular_velocity,dt),dict(reason='recover_forward_low_tracks',current_tracks=retained_base,candidates=[])
        # Evaluate the same candidates and 6x6 systems as a batch. Avoid hundreds
        # of Python/BLAS calls at every 5 Hz control update; no score is omitted.
        angles=np.asarray(candidates)
        optical=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        rotations=R_home@Rotation.from_euler('ZY',angles).as_matrix()@optical
        def batch_visibility(p,mask,view_rotations=None):
            if view_rotations is None:view_rotations=rotations
            pc=np.einsum('ni,cij->cnj',p-tbc,view_rotations)
            uv=np.abs(pc[:,:,:2])/np.maximum(pc[:,:,2,None],1e-9)
            edge=np.maximum(uv[:,:,0]/c['tan_half_hfov'],uv[:,:,1]/c['tan_half_vfov'])
            ok=(pc[:,:,2]>.1)&mask[None,:]&(edge<1.)
            w=np.clip((1-edge)/c.get('border_fraction',.2),0,1) if reliable else ok.astype(float)
            return ok,w*ok
        visible,weights=batch_visibility(points,base_ok)
        count=visible.sum(axis=1)
        used=weights.copy()
        if reliable:
            # Preserve the exact per-view sorting and feature budget, including
            # ties among zero-information points at the image boundary.
            # Select the finite feature budget for the weak translation
            # directions, rather than mixing rotation and metre units in trace.
            P0=np.linalg.solve(H0,np.eye(6));projected=P0@U
            importance=np.einsum('ik,nij,jk,k->n',projected,contributions,projected,weakness)
            if c.get('feature_budget_model')=='spatial_expected':
                used=spatial_budget(points,tbc,rotations,visible,weights,c)
            else:
                for i in range(len(angles)):
                    indices=np.flatnonzero(visible[i])
                    rejected=indices[np.argsort(importance[indices]*weights[i,indices])[:-c.get('feature_budget',25)]]
                    used[i,rejected]=0.
        used[count<c['min_features']]=0.
        Hv=np.einsum('cn,nij->cij',used,contributions)
        total=H0[None,:,:]+Hv
        L=np.linalg.cholesky((total+total.transpose(0,2,1))*.5)
        solved=np.linalg.solve(L,np.broadcast_to(U,L.shape))
        variances=np.sum(solved*solved,axis=1)
        uncertainty=np.sum(weakness[None,:]*variances,axis=1)
        if c.get('uncertainty_gain')=='directional_log':
            base_solved=np.linalg.solve(np.linalg.cholesky(H0),U)
            gain=directional_uncertainty_gain(np.sum(base_solved*base_solved,axis=0),variances,weakness)
        else:
            gain=uncertainty_gain(baseline,uncertainty,c.get('uncertainty_gain','fractional'))
        speed=np.linalg.norm(velocity_body)
        forward=0. if speed<.05 else 1-rotations[:,:,2]@velocity_body/speed
        motion=np.linalg.norm(angles-current,axis=1)/np.pi
        gain_scale=max(float(np.max(gain)),1e-12)
        cost=-gain/gain_scale+c['forward_weight']*forward+c['motion_weight']*motion*motion
        tracking_weights=weights if track_points is None else batch_visibility(tracking_points,tracking_mask)[1]
        retained_target=np.sum(np.minimum(tracking_weights,current_w[None,:])*tracking_fresh[None,:],axis=1)
        retained=retained_target.copy()
        commands=self.command_batch(current,angles,R_home,angular_velocity,dt)
        transition_aware=c.get('transition_aware',False)
        if transition_aware:
            # The immediate motion is small even when the desired target is far
            # away. Protect tracks during the actual next control interval, not
            # at an endpoint that may take several seconds to reach.
            next_points=Rotation.from_rotvec(-angular_velocity*dt).apply(tracking_points-velocity_body*dt)
            next_rotations=R_home@Rotation.from_euler('ZY',commands).as_matrix()@optical
            _,next_weights=batch_visibility(next_points,tracking_mask,next_rotations)
            retained=np.sum(np.minimum(next_weights,current_w[None,:])*tracking_fresh[None,:],axis=1)
        predicted_retained=retained.copy()
        if horizon and retained_base>0:
            predicted_points=future_points if track_points is None else Rotation.from_rotvec(-angular_velocity*horizon).apply(tracking_points-velocity_body*horizon)
            future_rotations=None
            if transition_aware:
                reachable=self.command_batch(current,angles,R_home,angular_velocity,horizon)
                future_rotations=R_home@Rotation.from_euler('ZY',reachable).as_matrix()@optical
            _,future_weights=batch_visibility(predicted_points,tracking_mask,future_rotations)
            predicted_retained=np.sum(np.minimum(future_weights,current_w[None,:])*tracking_fresh[None,:],axis=1)
            cost+=c.get('future_retention_weight',.5)*(1-predicted_retained/retained_base)
        allowed=np.ones(len(angles),dtype=bool) if not reliable else retained>=c.get('retain_fraction',.8)*retained_base-1e-9
        feasible=bool(allowed.any())
        if not feasible:
            # If body motion exceeds available gimbal authority, preserve as
            # many tracks as physically possible instead of silently using an
            # inadmissible first candidate. Record this exception explicitly.
            allowed=retained>=retained.max()-1e-9
        reports=[dict(angles=a.tolist(),command_angles=cmd.tolist(),features=int(n),gain=float(g),cost=float(k),uncertainty=float(u),retained=float(r),retained_target=float(rt),predicted_retained=float(pr),allowed=bool(ok)) for a,cmd,n,g,k,u,r,rt,pr,ok in zip(angles,commands,count,gain,cost,uncertainty,retained,retained_target,predicted_retained,allowed)]
        best_index=int(np.argmin(np.where(allowed,cost,np.inf)))
        # Keep a selected view long enough for repeated observations to build
        # depth. Loss of tracks or a large predicted benefit allows interruption.
        if self.last_target is not None and self.target_age<c.get('view_dwell_s',4.) and allowed[1]:
            if cost[1]-cost[best_index]<c.get('dwell_switch_improvement',.25):best_index=1
        if reliable and allowed[0] and cost[0]-cost[best_index]<c.get('min_cost_improvement',.03):best_index=0
        target=angles[best_index]
        if self.last_target is None or np.linalg.norm(target-self.last_target)>np.deg2rad(3.):
            self.last_target=target.copy();self.target_age=0.
        best=reports[best_index];command=commands[best_index]
        return command,dict(reason='evaluated',selected=best,
            selection_score_kind='motion_conditioned_translation_information',target_age_s=self.target_age,
            lidar_translation_weights=weights_l.tolist(),current_view=reports[0],baseline_uncertainty=baseline,current_tracks=recovery_count,retention_constraint_feasible=feasible,candidates=reports)
