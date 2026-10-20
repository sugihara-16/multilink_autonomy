"""Add image-stabilizing motion candidates to the existing predictive score."""
import numpy as np
from predictive_tracking_view import PredictiveTrackingView
from retinal_motion_candidates import candidate_steps


class RetinalTrackingView(PredictiveTrackingView):
    def candidate_commands(self,unit,rates,eligible,R_home,current,omega,dt):
        old=super().candidate_commands(unit,rates,eligible,R_home,current,omega,dt)
        axis=np.asarray(self.cfg['axis_max_speed']);lo,hi=self.motion.limits()
        lower=np.maximum(-axis*dt,lo-current);upper=np.minimum(axis*dt,hi-current)
        steps=candidate_steps(unit[eligible],rates[eligible],R_home,current,omega,dt,
            self.cfg.get('scan_speed',self.cfg['max_speed']),lower,upper)
        added=self.limiter.command_batch(current,current+steps,R_home,np.zeros(3),dt)
        self.candidate_groups=(old.copy(),added.copy())
        return np.vstack([old,added])

    def _choose(self,*args,**kwargs):
        self.candidate_groups=None
        target,report=super()._choose(*args,**kwargs)
        if self.candidate_groups is not None:
            old,added=self.candidate_groups
            report['retinal_candidates_added']=len(added)
            report['selected_new_retinal_candidate']=bool(
                report['reason']!='correlated_tracking_hold'
                and not np.any(np.all(old==target,axis=1))
                and np.any(np.all(added==target,axis=1)))
        return target,report
