"""Acceleration-limited coordinated segments and a fail-closed flight interlock."""
import numpy as np


class FlightInterlock:
    def __init__(self,config):
        self.cfg=config;self.odom=None;self.force=None;self.ready_since=None;self.blocked=True

    def reason(self,now):
        c=self.cfg
        if self.odom is None or self.force is None:
            self.ready_since=None;self.blocked=True;return 'feedback_missing'
        stamp,tilt,rate=self.odom;force_stamp,fraction=self.force
        # /clock and joint/odometry topics are delivered independently. Permit
        # at most 20 ms of inter-topic clock skew, never arbitrary future data.
        future=c['feedback_future_tolerance_s']
        if not np.isfinite([stamp,tilt,rate,force_stamp,fraction]).all() or not (-future<=now-stamp<=c['feedback_timeout_s'] and -future<=now-force_stamp<=c['feedback_timeout_s']):
            self.ready_since=None;self.blocked=True;return 'feedback_stale'
        if tilt>c['stop_tilt_deg'] or rate>c['stop_rate_deg_s'] or fraction>.90:
            self.ready_since=None;self.blocked=True;return 'flight_margin'
        if self.blocked:
            if tilt>c['resume_tilt_deg'] or rate>c['resume_rate_deg_s'] or fraction>c['max_thrust_fraction']:
                self.ready_since=None;return 'settling'
            if self.ready_since is None:self.ready_since=now
            if now-self.ready_since<c['resume_dwell_s']:return 'settling'
            self.blocked=False
        return 'ok'


class CoordinatedMotion:
    """Retarget only at rest; an abrupt new view must not reverse joint velocity.

    A disabled/stale/unsafe request brakes along the already checked segment.
    The caller keeps publishing the stopping setpoint instead of silently
    abandoning an earlier servo target. Hardware tracking is checked separately.
    """
    def __init__(self,kin,q):
        self.kin=kin;self.position=np.array(q,dtype=float);self.goal=self.position.copy()
        self.direction=np.zeros(6);self.rate=0.
        self.plan=None

    def step(self,target,dt,permit=True):
        if not np.isfinite(dt) or not 0<dt<=.02:raise ValueError('Invalid servo timestep')
        target=np.asarray(target)
        changed=np.max(abs(target-self.goal))>1e-6
        if not permit or changed:self.plan=None
        if self.rate<1e-10 and permit and self.plan is None:
            if not self.kin.path_valid(self.position,target):return self.position.copy(),'path_rejected'
            self.goal=target.copy();d=self.goal-self.position;length=np.max(abs(d))
            if length<1e-10:return self.position.copy(),'hold'
            self.direction=d/max(length,1e-12)
            peak=min(self.kin.speed,np.sqrt(length*self.kin.acceleration))
            ramp=peak/self.kin.acceleration;cruise=max(0.,length/peak-ramp)
            self.plan=[self.position.copy(),0.,length,peak,ramp,cruise]
        remaining=float(np.max(abs(self.goal-self.position)))
        a=self.kin.acceleration
        braking=self.plan is None
        if braking:
            duration=min(dt,self.rate/a);distance=self.rate*duration-.5*a*duration**2
            new_rate=max(0.,self.rate-a*dt);candidate=self.position+self.direction*distance
        else:
            origin,t,length,peak,ramp,cruise=self.plan;t+=dt
            if t<ramp:s=.5*a*t*t;new_rate=a*t
            elif t<ramp+cruise:s=.5*peak*ramp+peak*(t-ramp);new_rate=peak
            else:
                left=max(0.,2*ramp+cruise-t);s=length-.5*a*left*left;new_rate=a*left
            candidate=origin+self.direction*s;distance=float(np.max(abs(candidate-self.position)))
        # Whole stopping path is checked, including one cycle of latency.
        stop_distance=min(max(0.,remaining-distance),new_rate**2/(2*a)+new_rate*dt)
        stop=candidate+self.direction*stop_distance
        if permit and not self.kin.path_valid(self.position,stop):
            self.plan=None
            return self.step(target,dt,permit=False)
        self.position=candidate;self.rate=new_rate
        if self.plan is not None:
            self.plan[1]=t
            if remaining-distance<1e-10:self.position=self.goal.copy();self.rate=0.;self.plan=None
        return self.position.copy(),('braking' if braking and self.rate>0 else 'hold' if not permit else 'moving')
