import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from bearing_motion import BearingMotion,advance_bearings,unit_rows


class BearingMotionTest(unittest.TestCase):
    def test_pure_body_rotation_has_no_translation_rate(self):
        m=BearingMotion();ray=unit_rows([[6.,2.,1.]])
        rotation=lambda a,b:Rotation.from_euler('z',.3*(b-a)).as_matrix()
        for t in np.arange(0,.81,.05):
            current=ray@rotation(0,t)
            rates=m.update(t,[7],current,rotation)
        np.testing.assert_allclose(rates,0.,atol=1e-13)

    def test_translation_prediction_ignores_metric_scale(self):
        outputs=[]
        for scale in [.1,1.,100.]:
            m=BearingMotion()
            for t in np.arange(0,1.01,.05):
                point=scale*np.array([[6.-.5*t,2.,-.4]])
                rates=m.update(t,[7],point,lambda a,b:np.eye(3))
            predicted,_=advance_bearings(unit_rows(point),rates,np.eye(3),.2)
            outputs.append(predicted)
        for output in outputs[1:]:np.testing.assert_allclose(output,outputs[0],atol=1e-13)
        exact=unit_rows([[6.-.5*1.2,2.,-.4]])
        self.assertLess(np.linalg.norm(outputs[0]-exact),.0005)

    def test_constant_gyro_bias_cancels_with_visual_bearing_change(self):
        m=BearingMotion();ray=unit_rows([[6.,2.,1.]])
        rotation=lambda a,b:Rotation.from_euler('z',.01*(b-a)).as_matrix()
        for t in np.arange(0,.81,.05):rates=m.update(t,[7],ray,rotation)
        predicted,_=advance_bearings(ray,rates,rotation(.8,1.),.2)
        self.assertLess(np.linalg.norm(predicted-ray),1e-5)

    def test_missing_identity_and_gap_do_not_reuse_motion(self):
        m=BearingMotion();r=lambda a,b:np.eye(3)
        m.update(0,[1],[[1,0,0]],r)
        m.update(.2,[1],[[1,.1,0]],r)
        np.testing.assert_array_equal(m.update(.3,[2],[[1,.2,0]],r),[[0,0,0]])
        np.testing.assert_array_equal(m.update(.9,[1],[[1,.3,0]],r),[[0,0,0]])

    def test_rates_are_rotated_with_body_frame(self):
        ray=unit_rows([[4.,1.,0.]]);rate=np.array([[-.01,.04,0.]])
        R=Rotation.from_euler('z',.3).as_matrix()
        p,v=advance_bearings(ray,rate,R,.2)
        expected=unit_rows((ray+.2*rate)@R)
        np.testing.assert_allclose(p,expected)
        np.testing.assert_allclose(np.sum(p*v,axis=1),0.,atol=1e-14)

    def test_empty_tracks_stay_empty(self):
        m=BearingMotion();a=m.update(1,[],np.empty((0,3)),lambda a,b:np.eye(3))
        self.assertEqual(a.shape,(0,3))
        p,v=advance_bearings(a,a,np.eye(3),.2)
        self.assertEqual(p.shape,(0,3));self.assertEqual(v.shape,(0,3))

if __name__=='__main__':unittest.main()
