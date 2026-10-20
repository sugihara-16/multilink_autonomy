"""Frame-explicit visual map and gimbal objective; no ROS or truth input.

All 6-vectors are body-local RIGHT pose perturbations [rotation, translation].
The objective is a conditional local information proxy, not ROVIO covariance.
"""
import ctypes
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def skew(p):
    x, y, z = p
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def adjoint(T):
    a = np.zeros((6, 6)); R = T[:3, :3]
    a[:3, :3] = a[3:, 3:] = R
    a[3:, :3] = skew(T[:3, 3]) @ R
    return a


def camera_rotation(R_home, yaw, pitch):
    # Joint yaw about home +Z; pitch about the yawed +Y; camera +X forward.
    optical = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    return R_home @ Rotation.from_euler('ZY', [yaw, pitch]).as_matrix() @ optical


class Fisher:
    def __init__(self, library):
        self.lib = ctypes.CDLL(str(library))
        p = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        self.lib.fif_information.argtypes = [ctypes.c_int, p, p, p, p, ctypes.c_double, p, p, p]
        self.lib.fif_information.restype = ctypes.c_int

    def information(self, points, covs, Rbc, tbc, sigma, per_point=False):
        arrays = [np.ascontiguousarray(x, dtype=np.float64) for x in (points, covs, Rbc, tbc)]
        n = len(arrays[0]); H = np.zeros((6, 6)); J = np.zeros((n, 3, 6))
        if arrays[0].shape != (n, 3) or arrays[1].shape != (n, 3, 3):
            raise ValueError('Invalid landmark array shapes')
        contributions=np.zeros((n,6,6))
        rc = self.lib.fif_information(n, *arrays, float(sigma), H, J, contributions)
        if rc: raise ValueError(f'FIF input rejected: {rc}')
        return (H,J,contributions) if per_point else (H,J)


@dataclass
class Landmark:
    position: np.ndarray
    covariance: np.ndarray
    view: np.ndarray
    stamp_ns: int
    observations: int


class VisualMap:
    """Latest estimate per persistent ROVIO ID, never sums correlated sightings."""
    def __init__(self, max_age=60., max_points=3000):
        self.points = {}; self.max_age_ns = int(max_age*1e9); self.max_points = max_points
        self.last_stamp = -1

    def update(self, stamp_ns, ids, positions, covariances, camera_position):
        if stamp_ns < self.last_stamp: raise ValueError('Map time moved backwards: reset required')
        self.last_stamp = stamp_ns
        for ident, p, C in zip(ids, positions, covariances):
            ident = int(ident); p = np.asarray(p); C = np.asarray(C)
            if ident < 0 or not np.isfinite(p).all() or not np.isfinite(C).all(): continue
            d = np.linalg.norm(p-camera_position)
            if not .3 < d < 40 or np.linalg.eigvalsh(C).min() < -1e-8: continue
            if np.trace(C) > .25*d*d: continue  # Reject uninitialized/very uncertain depth.
            old = self.points.get(ident)
            self.points[ident] = Landmark(p, C, (p-camera_position)/d, stamp_ns,
                                         1 if old is None else old.observations+1)
        self.points = {k:v for k,v in self.points.items() if stamp_ns-v.stamp_ns <= self.max_age_ns}
        if len(self.points) > self.max_points:
            self.points = dict(sorted(self.points.items(), key=lambda kv:kv[1].stamp_ns)[-self.max_points:])

    def body_points(self, T_W_B, metadata=False, min_observations=3):
        # A redetected feature may have a new ID. Keep one representative per
        # 20 cm cell so old and new tracks are not counted as independent rays.
        cells={}
        for v in self.points.values():
            if v.observations<min_observations:continue
            cell=tuple(np.floor(v.position/.2).astype(int))
            if cell not in cells or v.stamp_ns>cells[cell].stamp_ns:cells[cell]=v
        selected=list(cells.values())
        R = T_W_B[:3, :3]; t = T_W_B[:3, 3]
        p = np.array([R.T@(v.position-t) for v in selected]).reshape(-1,3)
        C = np.array([R.T@v.covariance@R for v in selected]).reshape(-1,3,3)
        views = np.array([R.T@v.view for v in selected]).reshape(-1,3)
        if metadata:
            return p,C,views,np.array([(self.last_stamp-v.stamp_ns)*1e-9 for v in selected])
        return p, C, views

    def save(self, path):
        # Store visual evidence, not a fused covariance tied to an old LiDAR scan.
        values = list(self.points.values())
        np.savez_compressed(path, ids=list(self.points),
            positions=np.array([v.position for v in values]).reshape(-1,3),
            covariances=np.array([v.covariance for v in values]).reshape(-1,3,3),
            views=np.array([v.view for v in values]).reshape(-1,3),
            stamps_ns=[v.stamp_ns for v in values], observations=[v.observations for v in values])

    def load(self, path):
        with np.load(path, allow_pickle=False) as a:
            self.points = {int(i):Landmark(p,C,v,int(t),int(n)) for i,p,C,v,t,n in zip(
                a['ids'],a['positions'],a['covariances'],a['views'],a['stamps_ns'],a['observations'])}
        self.last_stamp = max((v.stamp_ns for v in self.points.values()), default=-1)


def lidar_model(H_L, eigenvectors_L, probabilities, T_B_L):
    A = adjoint(np.linalg.inv(T_B_L))
    H_B = A.T @ H_L @ A
    # Weak scalar coordinates are u_L^T delta_L = u_L^T A delta_B.
    # These are covectors, not displacement vectors; the lever arm matters.
    U_B = A.T @ eigenvectors_L
    return H_B, U_B, np.clip(1-np.asarray(probabilities), 0., 1.)


def projected_uncertainty(H, U, weights):
    # Cholesky solve avoids explicitly inverting an ill-conditioned matrix.
    L = np.linalg.cholesky((H+H.T)*.5)
    x = np.linalg.solve(L, U)
    return float(np.sum(weights*np.sum(x*x, axis=0)))


def uncertainty_gain(baseline, posterior, mode='fractional'):
    """Reduction of a weighted projected variance; not mutual information.

    Log reduction distinguishes, for example, a further halving of an already
    small variance. Fractional reduction saturates near one in that situation.
    """
    posterior = np.asarray(posterior)
    if mode not in ('fractional', 'log'):
        raise ValueError('Unknown uncertainty gain: '+str(mode))
    if baseline < 1e-12:
        return np.zeros_like(posterior)
    if mode == 'fractional':
        return (baseline-posterior)/baseline
    return np.log(baseline/np.maximum(posterior, np.finfo(float).tiny))


def directional_uncertainty_gain(baseline, posterior, weakness):
    """Expected log reduction along each weak covector, a scoring proxy.

    Each variance ratio is dimensionless. Weight AFTER forming the ratio:
    scaling all degeneracy probabilities by k must scale the benefit by k,
    rather than cancelling it between numerator and denominator. Covariances
    still come from the full coupled pose system, not six separate inverses.
    """
    baseline=np.asarray(baseline);posterior=np.asarray(posterior);weakness=np.asarray(weakness)
    if np.any(baseline<=0) or np.any(posterior<=0) or np.any((weakness<0)|(weakness>1)):
        raise ValueError('Invalid projected variances or degeneracy probabilities')
    return np.sum(weakness*np.maximum(0.,np.log(baseline/posterior)),axis=-1)


class Planner:
    def __init__(self, fisher, cfg):
        self.fisher = fisher; self.cfg = cfg
        self.low_track_time=0.
        if cfg.get('axis_max_speed') is not None:
            rates=np.asarray(cfg['axis_max_speed'],dtype=float)
            if rates.shape!=(2,) or not np.isfinite(rates).all() or np.any(rates<=0):
                raise ValueError('Invalid physical joint velocity limits')

    def limits(self):
        lower=np.array(self.cfg['limits_min']);upper=np.array(self.cfg['limits_max'])
        margin=np.asarray(self.cfg.get('joint_stop_margin',[0.,0.]),dtype=float)
        if margin.shape!=(2,) or not np.isfinite(margin).all() or np.any(margin<0):
            raise ValueError('Invalid joint stop margin')
        moving=upper>lower
        lower[moving]+=margin[moving];upper[moving]-=margin[moving]
        if np.any(lower>upper):raise ValueError('Joint stop margin exceeds travel')
        if self.cfg.get('gaze_pitch_limit') is not None:
            limit=float(self.cfg['gaze_pitch_limit'])
            if not 0<limit<=np.pi/2:raise ValueError('Invalid gaze pitch trust region')
            lower[1]=max(lower[1],-limit);upper[1]=min(upper[1],limit)
        return lower,upper

    def command_batch(self,current,targets,R_home,angular_velocity,dt):
        """Joint positions satisfying scan, vector, and physical per-axis limits."""
        c=self.cfg;delta=np.atleast_2d(targets)-current
        if c.get('scan_speed') is not None:
            if c.get('rate_compensation',False):
                axes=np.column_stack([R_home[:,2],R_home@Rotation.from_euler('z',current[0]).apply([0.,1.,0.])])
                correction=-axes.T@angular_velocity*dt
            else:
                ray=R_home.T@Rotation.from_rotvec(-angular_velocity*dt).apply(camera_rotation(R_home,*current)[:,2])
                stable=np.array([np.arctan2(ray[1],ray[0]),np.arctan2(-ray[2],np.linalg.norm(ray[:2]))])
                correction=stable-current
            scan=delta-correction
            delta=correction+scan*np.minimum(1.,c['scan_speed']*dt/np.maximum(np.linalg.norm(scan,axis=1),1e-12))[:,None]
        delta*=np.minimum(1.,c['max_speed']*dt/np.maximum(np.linalg.norm(delta,axis=1),1e-12))[:,None]
        bounded=np.clip(current+delta,*self.limits())-current
        bounded*=np.minimum(1.,c['max_speed']*dt/np.maximum(np.linalg.norm(bounded,axis=1),1e-12))[:,None]
        if c.get('axis_max_speed') is not None:
            step=np.asarray(c['axis_max_speed'])*dt
            bounded=np.clip(bounded,-step,step)
        return np.clip(current+bounded,c['limits_min'],c['limits_max'])

    def command(self,current,target,R_home,angular_velocity,dt):
        return self.command_batch(current,np.asarray(target)[None,:],R_home,angular_velocity,dt)[0]

    def choose(self, points, covs, views, H, U, weakness, R_home, tbc,
               current, velocity_body, dt, obstacles=None, ages=None, angular_velocity=None, track_points=None):
        c = self.cfg
        horizon=c.get('prediction_horizon_s',0.)
        angular_velocity=np.zeros(3) if angular_velocity is None else np.asarray(angular_velocity)
        if not 0 < dt <= c['max_dt']: raise ValueError('Invalid planning dt')
        if not all(np.isfinite(a).all() for a in (H,U,weakness,current)): raise ValueError('Nonfinite state')
        # Explicit prior caps otherwise infinite unobservable variances. A scoring
        # prior only: it is never fed into MIMOSA or represented as a measurement.
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
        candidates = [np.asarray(current)]
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
        # visibility changes. Cache per-point official results for this step.
        contributions=np.zeros((len(points),6,6))
        if base_ok.any():
            _,_,contributions[base_ok]=self.fisher.information(points[base_ok],covs[base_ok],
                camera_rotation(R_home,*current),tbc,c['bearing_sigma'],per_point=True)
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
            importance=np.trace(contributions,axis1=1,axis2=2)
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
        cost=-gain+c['forward_weight']*forward+c['motion_weight']*motion*motion
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
        if reliable and allowed[0] and cost[0]-cost[best_index]<c.get('min_cost_improvement',.03):best_index=0
        best=reports[best_index];command=commands[best_index]
        return command,dict(reason='evaluated',selected=best,
            current_view=reports[0],baseline_uncertainty=baseline,current_tracks=recovery_count,retention_constraint_feasible=feasible,candidates=reports)
