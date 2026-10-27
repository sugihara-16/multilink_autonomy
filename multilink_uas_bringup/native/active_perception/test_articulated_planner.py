"""Regressions for delayed pose messages and zero tracked features, no ROS master."""
from collections import deque
import unittest
import numpy as np
from articulated_planner import Planner
from core import VisualMap

class Timing(unittest.TestCase):
    def node(self):
        p=Planner.__new__(Planner);p.map=VisualMap();p.map_stamp=-1
        p.poses=deque([(1.,np.eye(4)),(1.05,np.eye(4))]);p.omega=np.zeros(3)
        p.geo=(1050000000,np.array([],int),np.empty((0,3)),np.array([.5,0,0]),np.eye(3))
        p.camera_at=lambda t:np.eye(4)
        return p
    def test_empty_feature_packet_does_not_stop_control(self):
        p=self.node();p.update_map(p.geo,np.zeros(6))
        self.assertEqual(p.map_stamp,1050000000);self.assertEqual(len(p.map.points),0)
    def test_measured_processing_delay_is_propagated(self):
        p=self.node();T=p.pose_at(1.2)
        np.testing.assert_allclose(T[:3,3],[.075,0,0],atol=1e-12)
        self.assertIsNone(p.pose_at(2.))

if __name__=='__main__':unittest.main()
