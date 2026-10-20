import unittest
from types import SimpleNamespace as N
from paired_initialization import Selector,coincident

def pose(yaw=0.):
    import math
    return N(pose=N(position=N(x=.18,y=0.,z=-.025),orientation=N(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))))

class Tests(unittest.TestCase):
    def test_actual_frames_are_shared_then_irreversibly_separate(self):
        selector=Selector()
        for stamp,yaw,separate in [(50,0.,False),(100,5e-8,False),(150,.001,True),(200,0.,True)]:
            active=object();reference=object();values=[active,pose(yaw),reference,pose()];result=[]
            for k in [2,0,3,1]:result+=selector.add(k,stamp,values[k])
            self.assertEqual(len(result),1);s,im,ex,state=result[0]
            self.assertEqual(s,stamp);self.assertEqual(state,separate)
            self.assertIs(im,active if separate else reference);self.assertIs(ex,values[1] if separate else values[3])
        self.assertEqual(selector.add(0,100,object()),[])

    def test_quaternion_sign_is_not_a_camera_motion(self):
        a,b=pose(),pose();b.pose.orientation.w=-1.
        self.assertTrue(coincident(a,b))
        b.pose.position.x+=.01;self.assertFalse(coincident(a,b))

    def test_missing_stream_is_a_failure_not_silent_interpolation(self):
        s=Selector()
        with self.assertRaises(RuntimeError):
            for stamp in range(101):s.add(0,stamp,object())

if __name__=='__main__':unittest.main()
