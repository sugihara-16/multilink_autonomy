"""Conservative planning envelope for the ROS 2 DRAGON vectoring controller.

The allocator in gimbalrotor_controller.cpp solves A f = [m*a, I*alpha],
with A_i = [identity; skew(r_i) + direction_i * m_f_rate * identity].
For the two-axis, full-rank case, rotating the force coordinates and scaling
the six equations does not change the minimum-norm solution. This module uses
that equivalent physical-wrench form, including URDF mass and inertia.

This is a local hover envelope, NOT a proof of nonlinear closed-loop stability.
It deliberately excludes heavily tilted links, narrow footprints and gimbal
singularities, and reserves force for bounded corrective accelerations.
"""
import itertools
import numpy as np
from scipy.spatial.transform import Rotation

DEFAULTS = dict(max_link_tilt_deg=35., min_planar_spread_m=.20,
                max_rotor_height_span_m=.40, max_gimbal_angle_deg=60.,
                max_thrust_fraction=.85, min_thrust_fraction=.10,
                reserve_linear_acceleration_mps2=1.5,
                reserve_angular_acceleration_radps2=1.,
                max_body_speed_deg_s=4., max_body_acceleration_deg_s2=8.,
                stop_tilt_deg=8., resume_tilt_deg=4.,
                stop_rate_deg_s=15., resume_rate_deg_s=8.,
                feedback_timeout_s=.2, feedback_future_tolerance_s=.02,
                resume_dwell_s=.5)


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def gimbal_angles(local_force):
    f=np.asarray(local_force)
    roll=np.arctan2(-f[...,1],f[...,2])
    pitch=np.arctan2(f[...,0],np.hypot(f[...,1],f[...,2]))
    return np.stack((roll,pitch),axis=-1)


class ControlEnvelope:
    def __init__(self,kin,config=None):
        self.kin=kin;self.config=dict(DEFAULTS,**(config or {}))
        if any(not np.isfinite(v) or v<=0 for v in self.config.values()):
            raise ValueError('Control envelope parameters must be positive and finite')
        self.inertials=[]
        for link in kin.root.findall('link'):
            e=link.find('inertial')
            if e is None:continue
            o=e.find('origin');p=np.fromstring(o.get('xyz','0 0 0'),sep=' ') if o is not None else np.zeros(3)
            R=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix() if o is not None else np.eye(3)
            a=e.find('inertia').attrib
            I=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],
                        [float(a['ixy']),float(a['iyy']),float(a['iyz'])],
                        [float(a['ixz']),float(a['iyz']),float(a['izz'])]])
            self.inertials.append((link.get('name'),float(e.find('mass').get('value')),p,R@I@R.T))
        self.mass=sum(x[1] for x in self.inertials)
        self.thrust_limit=min(float(kin.root.find(f"joint[@name='rotor{i}']/limit").get('upper')) for i in range(1,5))
        self.mf=float(kin.root.find('m_f_rate').get('value'))
        self.directions=np.array([float(kin.root.find(f"joint[@name='rotor{i}']/axis").get('xyz').split()[2]) for i in range(1,5)])
        if self.mass<=0 or self.thrust_limit<=0:raise ValueError('Invalid controller model')
        # Vertices of the full six-axis acceleration box, not individual axes.
        self.offsets=np.array(list(itertools.product([-1.,1.],repeat=6)))
        self.offsets[:,:3]*=self.config['reserve_linear_acceleration_mps2']
        self.offsets[:,3:]*=self.config['reserve_angular_acceleration_radps2']
        self._cache={}

    def dynamics(self,frames):
        masses=np.array([x[1] for x in self.inertials])
        points=np.array([frames[n][:3,:3]@p+frames[n][:3,3] for n,m,p,I in self.inertials])
        cog=np.sum(masses[:,None]*points,axis=0)/self.mass
        inertia=np.zeros((3,3))
        for (n,m,p,I),point in zip(self.inertials,points):
            R=frames[n][:3,:3];r=point-cog
            inertia+=R@I@R.T+m*((r@r)*np.eye(3)-np.outer(r,r))
        centers=np.array([frames[f'thrust{i}'][:3,3] for i in range(1,5)])
        A=np.concatenate([np.vstack((np.eye(3),skew(r-cog)+d*self.mf*np.eye(3))) for r,d in zip(centers,self.directions)],axis=1)
        return cog,inertia,centers,A

    def evaluate(self,q):
        q=np.asarray(q,dtype=float)
        if q.shape!=(6,) or not np.isfinite(q).all():return dict(valid=False,reason='invalid_joints')
        key=q.tobytes()
        if key in self._cache:return self._cache[key]
        f=self.kin.frames(q)
        Rfc=f['fc'][:3,:3]
        links=np.array([Rfc.T@f[f'link{i}'][:3,:3] for i in range(1,5)])
        tilt=float(np.rad2deg(np.arccos(np.clip(links[:,2,2],-1,1))).max())
        centers=np.array([Rfc.T@f[f'thrust{i}'][:3,3] for i in range(1,5)])
        spread=float(np.linalg.svd(centers[:,:2]-centers[:,:2].mean(0),compute_uv=False)[-1])
        height=float(np.ptp(centers[:,2]))
        result=dict(valid=False,reason='link_tilt',link_tilt_deg=tilt,planar_spread_m=spread,rotor_height_span_m=height)
        c=self.config
        if tilt<=c['max_link_tilt_deg'] and spread>=c['min_planar_spread_m'] and height<=c['max_rotor_height_span_m']:
            # Reconstruct rotor-gimbal poses instead of treating their inertial
            # elements as rigidly attached to the moving body links. Two fixed
            # point iterations resolve the small gimbal/CoG coupling near hover.
            gravity=Rfc@np.array([0.,0.,9.798015])
            angles=gimbal_angles(np.einsum('nji,j->ni',np.array([f[f'link{i}'][:3,:3] for i in range(1,5)]),gravity))
            for _ in range(2):
                f=self.kin.frames(q,rotor_angles=angles)
                cog,I,centers,A=self.dynamics(f)
                inverse=np.linalg.pinv(A,rcond=1e-8)
                hover=(inverse@np.r_[self.mass*gravity,np.zeros(3)]).reshape(4,3)
                local=np.einsum('nji,nj->ni',np.array([f[f'link{i}'][:3,:3] for i in range(1,5)]),hover)
                angles=gimbal_angles(local)
            accelerations=self.offsets.copy()
            forces=self.mass*(accelerations[:,:3]@Rfc.T+gravity)
            torques=(accelerations[:,3:]@Rfc.T)@I.T
            allocated=(np.c_[forces,torques]@inverse.T).reshape(-1,4,3)
            local=np.einsum('nji,snj->sni',np.array([f[f'link{i}'][:3,:3] for i in range(1,5)]),allocated)
            norms=np.linalg.norm(allocated,axis=-1)
            # A circular cone is convex: checking every acceleration-box vertex
            # bounds every interpolation inside it. Positive local Z also
            # avoids the +/-pi roll branch and the +/-pi/2 pitch singularity.
            cone=float(np.rad2deg(np.arctan2(np.linalg.norm(local[:,:,:2],axis=-1),local[:,:,2])).max())
            upper=float(norms.max()/self.thrust_limit)
            lower=float(local[:,:,2].min()/self.thrust_limit)
            residual=float(np.max(abs(np.c_[forces,torques]-allocated.reshape(-1,12)@A.T)))
            result.update(max_thrust_fraction=upper,min_local_z_fraction=lower,max_gimbal_cone_deg=cone,allocation_residual=residual)
            result['reason']=('allocation_rank' if residual>1e-7 else 'thrust_reserve' if upper>c['max_thrust_fraction'] or lower<c['min_thrust_fraction'] else 'gimbal_cone' if cone>c['max_gimbal_angle_deg'] else 'ok')
            result['valid']=result['reason']=='ok'
        elif tilt<=c['max_link_tilt_deg']:
            result['reason']='support_geometry'
        if len(self._cache)>=4096:self._cache.clear()
        self._cache[key]=result
        return result

    def manifest(self):
        return dict(parameters=self.config,mass_kg=self.mass,per_rotor_limit_N=self.thrust_limit,
                    force_limit_source='Generated simulation URDF rotor limit; not a hardware rating',
                    allocation='Full two-axis minimum-norm vectoring allocation, URDF inertia, 64 acceleration-box vertices',
                    limitations='Sampled joint paths and nominal hover gimbal poses; no nonlinear stability guarantee. Runtime feedback independently inhibits motion.')
