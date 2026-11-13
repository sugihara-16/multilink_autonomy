"""Run with --urdf pointing to a generated DRAGON model (no ROS required)."""
import argparse
import sys
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_articulated import Kinematics,NAMES
from dragon_cameras import link_transforms
from dragon_control_guard import ControlEnvelope,gimbal_angles
from articulated_motion import CoordinatedMotion,FlightInterlock
from articulated_search import Roadmap


class Guard(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.k=Kinematics(URDF);cls.k.speed=np.deg2rad(4);cls.k.acceleration=np.deg2rad(8)

    def test_home_accepts_hover_with_correction_reserve(self):
        m=self.k.control.evaluate(self.k.home)
        self.assertTrue(self.k.valid(self.k.home),m)
        self.assertLess(m['max_thrust_fraction'],.85)

    def test_recorded_failure_shapes_rejected(self):
        # Actual measured joint angles 0.1 s before the two force landings.
        for q in ([20.07,65.51,87.91,-13.51,38.56,-21.45],
                  [6.79,65.82,43.57,7.09,53.76,-18.52]):
            q=np.deg2rad(q)
            self.assertFalse(self.k.valid(q))
            self.assertFalse(self.k.path_valid(self.k.home,q))

    def test_articulated_inertial_frames_match_independent_fk(self):
        q=np.deg2rad([15,80,-15,80,0,80]);angles=np.deg2rad([[5,10],[-3,12],[4,-8],[1,5]])
        values=dict(zip(NAMES,q))
        values.update({f'gimbal{i+1}_{axis}':angles[i,j] for i in range(4) for j,axis in enumerate(['roll','pitch'])})
        independent=link_transforms(self.k.root,values)
        for name,f in self.k.frames(q,rotor_angles=angles).items():
            np.testing.assert_allclose(f,independent[name],atol=1e-12)

    def test_physical_wrench_form_matches_controller_acceleration_form(self):
        q=np.deg2rad([15,80,-15,80,0,80]);f=self.k.frames(q)
        cog,I,centers,A=self.k.control.dynamics(f)
        rotations=[f[f'link{i}'][:3,:3] for i in range(1,5)]
        B=np.zeros((12,12))
        for i,R in enumerate(rotations):B[3*i:3*i+3,3*i:3*i+3]=R
        D=np.zeros((6,6));D[:3,:3]=np.eye(3)/self.k.control.mass;D[3:,3:]=np.linalg.inv(I)
        # Mirrors the actual C++ operations, separately from physical A solve.
        integrated=D@A@B
        acceleration=np.array([.4,-.3,9.8,.2,-.1,.3])
        controller=B@np.linalg.pinv(integrated)@acceleration
        physical=np.linalg.pinv(A)@np.linalg.solve(D,acceleration)
        np.testing.assert_allclose(controller,physical,atol=1e-10)

    def test_insufficient_thrust_or_gimbal_margin_rejected(self):
        thrust=ControlEnvelope(self.k,dict(max_thrust_fraction=.5))
        self.assertEqual(thrust.evaluate(self.k.home)['reason'],'thrust_reserve')
        cone=ControlEnvelope(self.k,dict(max_gimbal_angle_deg=10.))
        self.assertEqual(cone.evaluate(self.k.home)['reason'],'gimbal_cone')

    def test_invalid_data_and_stale_feedback_inhibit_motion(self):
        gate=FlightInterlock(self.k.control.config)
        self.assertNotEqual(gate.reason(0),'ok')
        for t in np.arange(0,1,.01):
            gate.odom=(t,0.,0.);gate.force=(t,.6);reason=gate.reason(t)
        self.assertEqual(reason,'ok')
        self.assertNotEqual(gate.reason(2),'ok')
        gate.odom=(2,9.,0.);gate.force=(2,.6)
        self.assertEqual(gate.reason(2),'flight_margin')
        gate.odom=(2,0.,0.);gate.force=(2,float('nan'))
        self.assertNotEqual(gate.reason(2),'ok')
        self.assertFalse(self.k.control.evaluate(np.full(6,np.nan))['valid'])

    def test_retarget_and_interlock_preserve_acceleration_and_stop(self):
        m=CoordinatedMotion(self.k,self.k.home)
        a=self.k.home.copy();a[1]-=.2
        b=self.k.home.copy();b[3]-=.2
        p=[m.position.copy()]
        for i in range(800):
            q,reason=m.step(a if i<100 else b,.01,not 200<=i<300)
            p.append(q)
            if i==260:self.assertEqual(m.rate,0.)
        speed=np.diff(p,axis=0)/.01;acceleration=np.diff(speed,axis=0)/.01
        self.assertLessEqual(np.max(abs(speed)),self.k.speed+1e-10)
        self.assertLessEqual(np.max(abs(acceleration)),self.k.acceleration+1e-9)
        np.testing.assert_allclose(q,b,atol=1e-8)

    def test_small_clock_skew_allowed_but_large_future_stamp_rejected(self):
        gate=FlightInterlock(self.k.control.config)
        for t in np.arange(0,1,.01):
            gate.odom=(t+.003,0.,0.);gate.force=(t,.6);reason=gate.reason(t)
        self.assertEqual(reason,'ok')
        gate.odom=(1.1,0.,0.);gate.force=(1.,.6)
        self.assertEqual(gate.reason(1.),'feedback_stale')

    def test_guarded_roadmap_retains_connected_active_views(self):
        road=Roadmap(self.k);audit=road.audit()
        az=np.array(audit['azimuth_deg'])
        for low in [-180,-90,0,90]:self.assertTrue(np.any((az>=low)&(az<low+90)))
        self.assertGreater(max(audit['elevation_deg']),20)
        self.assertLess(min(audit['elevation_deg']),-20)
        for _,path in road.home_paths.values():
            for a,b in zip(path,path[1:]):self.assertTrue(self.k.path_valid(a,b))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--urdf',type=Path,required=True)
    args,remaining=parser.parse_known_args();URDF=args.urdf
    unittest.main(argv=[sys.argv[0]]+remaining)
