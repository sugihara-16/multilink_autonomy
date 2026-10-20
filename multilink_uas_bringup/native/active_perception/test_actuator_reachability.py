import tempfile
import unittest
from pathlib import Path
import numpy as np
from core import Planner
from actuator_reachability import parameters


class ActuatorReachabilityTest(unittest.TestCase):
    def setUp(self):
        self.rate=np.deg2rad(20.)/np.sqrt(2.)
        self.cfg=dict(limits_min=np.deg2rad([-50.,-90.]),limits_max=np.deg2rad([50.,90.]),
                      max_speed=np.deg2rad(20.),axis_max_speed=[self.rate]*2,
                      joint_stop_margin=[2*self.rate*.001]*2)

    def test_read_controller_limit_instead_of_joint_urdf_limit(self):
        joints=''.join(f'<joint name="active_camera_{a}_joint"><axis><limit><velocity>{np.deg2rad(20.)}</velocity></limit></axis></joint>'
            f'<plugin name="gz::sim::systems::JointPositionController"><joint_name>active_camera_{a}_joint</joint_name>'
            f'<use_velocity_commands>true</use_velocity_commands><cmd_max>{self.rate}</cmd_max></plugin>' for a in ['yaw','pitch'])
        with tempfile.TemporaryDirectory() as directory:
            m=Path(directory)/'model.sdf';w=Path(directory)/'world.sdf'
            m.write_text('<sdf><model>'+joints+'</model></sdf>')
            w.write_text('<sdf><world><physics><max_step_size>0.001</max_step_size></physics></world></sdf>')
            p=parameters(m,w)
            np.testing.assert_allclose(p['axis_max_speed'],[self.rate]*2,rtol=0,atol=0)
            np.testing.assert_allclose(p['joint_stop_margin'],[2*self.rate*.001]*2,rtol=0,atol=0)
            m.write_text(m.read_text().replace('true','false'))
            with self.assertRaises(ValueError):parameters(m,w)

    def test_commands_reachable_in_gazebo_velocity_mode(self):
        # Independently integrate the actual controller's cmd_max clipping.
        rng=np.random.default_rng(918);planner=Planner(None,self.cfg)
        for _ in range(80):
            current=rng.uniform(*planner.limits());dt=.2
            target=rng.uniform(-4.,4.,2)
            command=planner.command(current,target,np.eye(3),np.zeros(3),dt)
            actual=current.copy()
            for step in range(200):actual+=np.clip(command-actual,-self.rate*.001,self.rate*.001)
            np.testing.assert_allclose(actual,command,atol=3e-14,rtol=0)
            self.assertLessEqual(np.linalg.norm(command-current),self.cfg['max_speed']*dt+1e-14)
            lo,hi=planner.limits();self.assertTrue(np.all(command>=lo)&np.all(command<=hi))

    def test_single_axis_does_not_claim_diagonal_speed(self):
        command=Planner(None,self.cfg).command(np.zeros(2),[1.,0.],np.eye(3),np.zeros(3),.2)
        np.testing.assert_allclose(command,[self.rate*.2,0.],rtol=0,atol=1e-15)
        old=dict(self.cfg);old.pop('axis_max_speed');old.pop('joint_stop_margin')
        np.testing.assert_allclose(Planner(None,old).command(np.zeros(2),[1.,0.],np.eye(3),np.zeros(3),.2),[np.deg2rad(4.),0.],atol=1e-15)

    def test_hard_stop_clearance_and_recovery_still_obey_rate(self):
        planner=Planner(None,self.cfg)
        for sign in [-1.,1.]:
            current=np.array([0.,sign*np.pi/2])
            target=current+[0.,sign]
            command=planner.command(current,target,np.eye(3),np.zeros(3),.2)
            self.assertLess(abs(command[1]),np.pi/2)
            self.assertLessEqual(np.max(abs(command-current)),self.rate*.2+1e-14)
        bad=dict(self.cfg,axis_max_speed=[self.rate,0.])
        with self.assertRaises(ValueError):Planner(None,bad)

    def test_clearance_covers_one_step_of_stale_position_feedback(self):
        limit=np.pi/2;step=self.rate*.001;target=limit-2*step
        positions=[limit-.1]*2
        for i in range(1000):
            # Explicitly adverse one-step-old feedback, unlike the ideal
            # instantaneous controller used in the reachability test above.
            positions.append(positions[-1]+np.clip(target-positions[-2],-step,step))
        self.assertLess(max(positions),limit)


if __name__=='__main__':unittest.main()
