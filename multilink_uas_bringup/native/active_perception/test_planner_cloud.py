"""Regression for ROS1 packed feature layout and actual tracking-status gate."""
import unittest
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2,PointField,Image,Imu
from geometry_msgs.msg import PoseStamped
from sensor_msgs.point_cloud2 import create_cloud
from std_msgs.msg import Header,Float64MultiArray
from nav_msgs.msg import Odometry
from planner_node import Active
from core import VisualMap
from image_geometry_map import ImageGeometryMap
import cv2
import threading
import io
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
from collections import deque
from view_motion import RotationHistory

class CloudTest(unittest.TestCase):
    def test_live_fixation_receives_successive_frames_while_planning_lock_is_held(self):
        from bearing_motion import BearingMotion
        n=Active.__new__(Active)
        n.cfg={'direct_track_input':True,'fixation':True,'fixation_bearing_motion':True,'fixation_live_only':True,'max_sensor_age':.5}
        n.geometry=object();n.tbc=np.zeros(3);n.tracking_lock=threading.Lock();n.tracking_input=None
        n.track_history={};n.lock=threading.RLock();n.clouds={};n.join=Mock()
        n.bearing_motion=BearingMotion();n.motion=SimpleNamespace(rotation=lambda a,b:np.eye(3))
        names=['id','status','x','y','z','c_00','c_01','c_02','c_11','c_12','c_22']
        fields=[PointField(k,4*i,PointField.UINT32 if i<2 else PointField.FLOAT32,1) for i,k in enumerate(names)]
        def receive():
            for t,x in [(10.,6.),(10.2,5.9)]:
                n.features(create_cloud(Header(stamp=rospy.Time.from_sec(t)),fields,[[7,4,x,2.,0.,.01,0.,0.,.01,0.,.01]]))
        with n.lock:
            worker=threading.Thread(target=receive);worker.start();worker.join(.5)
            self.assertFalse(worker.is_alive(),'Second frame waited for the unused map receiver lock')
            self.assertAlmostEqual(n.tracking_input[0],10.2)
            self.assertEqual(n.clouds,{})
            n.join.assert_not_called()

    def test_cloud_updates_observed_motion_using_tracked_ids_only(self):
        from bearing_motion import BearingMotion,unit_rows
        n=Active.__new__(Active);n.cfg={'direct_track_input':True,'fixation':True,'fixation_bearing_motion':True,'max_sensor_age':.5}
        n.geometry=object();n.tbc=np.zeros(3);n.tracking_lock=threading.Lock();n.tracking_input=None
        n.track_history={};n.lock=threading.RLock();n.clouds={};n.join=lambda:None
        n.bearing_motion=BearingMotion();n.motion=SimpleNamespace(rotation=lambda a,b:np.eye(3))
        names=['id','status','x','y','z','c_00','c_01','c_02','c_11','c_12','c_22']
        fields=[PointField(k,4*i,PointField.UINT32 if i<2 else PointField.FLOAT32,1) for i,k in enumerate(names)]
        for t,x in [(10.,6.),(10.2,5.9)]:
            msg=create_cloud(Header(stamp=rospy.Time.from_sec(t)),fields,[[7,4,x,2.,0.,.01,0.,0.,.01,0.,.01],[8,3,6.,0.,0.,.01,0.,0.,.01,0.,.01]])
            n.features(msg)
        aligned,_=n.direct_tracks(10.3,lambda stamp:np.eye(4))
        ids,points,covs,ages,rates=n.fixation_record
        np.testing.assert_array_equal(ids,[7]);self.assertGreater(rates[0,1],0.)
        self.assertGreater(unit_rows(aligned)[0,1],unit_rows([[5.9,2.,0.]])[0,1])

    def test_observed_motion_alignment_does_not_use_fused_translation(self):
        n=Active.__new__(Active);n.cfg={'max_sensor_age':.5};n.tbc=np.array([.18,0,-.025]);n.tracking_lock=threading.Lock()
        point=np.array([[6.,2.,0.]])+n.tbc
        n.tracking_input=(10.,point,(np.array([7]),np.array([np.eye(3)*.01]),np.array([2.]),np.array([[-.01,.03,0.]])))
        T=np.eye(4);a,_=n.direct_tracks(10.2,lambda stamp:T)
        T[:3,3]=[100.,-20.,5.];b,_=n.direct_tracks(10.2,lambda stamp:T)
        np.testing.assert_array_equal(a,b)
        self.assertEqual(len(n.fixation_record),5)

    def test_fixation_preserves_metric_depth_covariance_identity_and_age(self):
        n=Active.__new__(Active);n.cfg={'direct_track_input':True,'fixation':True,'max_sensor_age':.5}
        n.geometry=object();n.tbc=np.zeros(3);n.tracking_lock=threading.Lock();n.tracking_input=None
        n.track_history={};n.lock=threading.RLock();n.clouds={};n.join=lambda:None
        names=['id','status','x','y','z','c_00','c_01','c_02','c_11','c_12','c_22']
        fields=[PointField(k,4*i,PointField.UINT32 if i<2 else PointField.FLOAT32,1) for i,k in enumerate(names)]
        for t in [10.,10.2]:
            msg=create_cloud(Header(stamp=rospy.Time.from_sec(t)),fields,[[7,4,3.,2.,0.,.01,0.,0.,.04,0.,.09],[8,3,6.,0.,0.,.01,0.,0.,.01,0.,.01]])
            n.features(msg)
        T=np.eye(4);T[0,3]=.1
        aligned,r=n.direct_tracks(10.3,lambda stamp:T)
        ids,points,covs,ages=n.fixation_record
        np.testing.assert_array_equal(ids,[7]);np.testing.assert_allclose(points,[[2.9,2.,0.]])
        np.testing.assert_allclose(covs,[np.diag([.01,.04,.09])]);np.testing.assert_allclose(ages,[.3])
        self.assertEqual(r['count'],1)

    def test_retention_reads_latest_tracks_before_slow_geometry(self):
        n=Active.__new__(Active);n.cfg={'direct_track_input':True,'max_sensor_age':.5}
        n.geometry=object();n.tbc=np.array([.18,0.,-.025]);n.tracking_lock=threading.Lock();n.tracking_input=None
        n.lock=threading.RLock();n.clouds={};n.current_tracks=(1,np.ones((1,3)))
        n.join=lambda:None
        fields=[PointField('id',0,PointField.UINT32,1),PointField('status',4,PointField.UINT32,1)]
        fields.extend(PointField(c,8+4*i,PointField.FLOAT32,1) for i,c in enumerate('xyz'))
        msg=create_cloud(Header(stamp=rospy.Time(10)),fields,[[1,4,4.18,0.,-.025],[2,3,1.,2.,3.]])
        with n.lock:
            worker=threading.Thread(target=n.features,args=(msg,));worker.start()
            deadline=time.monotonic()+1.
            while n.tracking_input is None and time.monotonic()<deadline:time.sleep(.001)
            self.assertIsNotNone(n.tracking_input,'Current tracks waited for the geometry/receiver lock')
            transform=np.eye(4);transform[0,3]=.1
            aligned,report=n.direct_tracks(10.2,lambda stamp:transform)
            np.testing.assert_allclose(aligned,[[8.08,0.,-.025]],atol=1e-6)
            self.assertEqual(report['count'],1);self.assertAlmostEqual(report['age_s'],.2)
            self.assertEqual(report['source'],'latest_rovio_bearings')
            self.assertIsNone(n.direct_tracks(9.9,lambda stamp:transform))
            self.assertIsNone(n.direct_tracks(10.6,lambda stamp:transform))
        worker.join(1.);self.assertFalse(worker.is_alive())

    def test_empty_current_tracks_are_distinct_from_missing_data(self):
        n=Active.__new__(Active);n.cfg={'max_sensor_age':.5};n.tracking_lock=threading.Lock()
        n.tracking_input=None;self.assertIsNone(n.direct_tracks(10.,lambda stamp:np.eye(4)))
        n.tracking_input=(10.,np.empty((0,3)))
        points,report=n.direct_tracks(10.1,lambda stamp:np.eye(4))
        self.assertEqual(points.shape,(0,3));self.assertEqual(report['count'],0)

    def test_paused_clock_retries_without_overproducing_decisions(self):
        n=Active.__new__(Active);n.cfg={'planning_rate':5};n.last_plan=9.9
        n.planning_stop=threading.Event();attempts=[];done=threading.Event()
        def tick(event):
            attempts.append(10.2)
            if len(attempts)==3:n.last_plan=10.2;done.set()
        n.tick=tick
        with patch('planner_node.rospy.Time.now',return_value=rospy.Time.from_sec(10.2)),patch('planner_node.rospy.is_shutdown',return_value=False):
            worker=threading.Thread(target=n.planning_worker);worker.start()
            try:
                self.assertTrue(done.wait(1.),'Pending work could not finish at paused simulated time')
                time.sleep(.04);self.assertEqual(len(attempts),3)
            finally:n.planning_stop.set();worker.join(1.)
        self.assertFalse(worker.is_alive())

    def test_missing_motion_does_not_commit_planning_timestamp(self):
        n=Active.__new__(Active);n.lock=threading.RLock();n.geometry=object();n.geometry_failure=None
        n.fixed_angles=None;n.cfg={'planning_rate':5,'max_dt':.5,'max_sensor_age':.5}
        n.last_plan=None;n.health={'stale':0,'planned':0};n.last_odom=Odometry()
        n.last_odom.header.stamp=rospy.Time(10);n.last_odom.pose.pose.orientation.w=1.
        n.encoder=(10.2,np.zeros(2));n.encoder_history=[n.encoder];n.encoder_lock=threading.Lock()
        n.lidar=(10.1,np.eye(6),np.eye(6),np.zeros(6));n.scan=None;n.scene_health={}
        n.motion=SimpleNamespace(latest=Mock(return_value=None),rotation=lambda a,b:np.eye(3),average_rate=lambda a,b:np.zeros(3))
        n.map=VisualMap();n.status=Mock();n.pub=[Mock(),Mock()];n.log=io.StringIO()
        with tempfile.TemporaryDirectory() as out,patch('planner_node.rospy.Time.now',return_value=rospy.Time.from_sec(10.2)):
            n.args=SimpleNamespace(output=Path(out),mode='fixed')
            n.tick(None);self.assertIsNone(n.last_plan);self.assertEqual(n.health['planned'],0)
            n.motion.latest.return_value=10.2;n.tick(None)
            self.assertAlmostEqual(n.last_plan,10.2);self.assertEqual(n.health['planned'],1)
            n.tick(None);self.assertEqual(n.health['planned'],1)
            self.assertEqual(n.status.publish.call_count,1)

    def test_map_worker_does_not_hold_receiver_lock(self):
        n=Active.__new__(Active);n.lock=threading.RLock();n.geometry_stop=threading.Event();n.geometry_event=threading.Event()
        n.geometry_pending='frame';n.geometry_failure=None;entered=threading.Event();release=threading.Event();accepted=[]
        def compute(frame):entered.set();release.wait(2.);return frame
        n.process_geometry=compute;n.accept_geometry=accepted.append;n.geometry_event.set()
        worker=threading.Thread(target=n.geometry_worker);worker.start()
        try:
            self.assertTrue(entered.wait(2.))
            self.assertTrue(n.lock.acquire(timeout=.2),'Image computation blocked incoming pose/point messages')
            n.lock.release()
        finally:
            n.geometry_stop.set();release.set();n.geometry_event.set();worker.join(timeout=2.)
        self.assertFalse(worker.is_alive());self.assertEqual(accepted,['frame']);self.assertIsNone(n.geometry_failure)

    def test_sensor_receivers_continue_during_heavy_planning(self):
        n=Active.__new__(Active);n.geometry=object();n.motion=RotationHistory()
        n.lock=threading.RLock();n.encoder_lock=threading.Lock();n.encoder_history=deque(maxlen=1000)
        def receive():
            for k in range(1,101):
                m=Imu();m.header.stamp=rospy.Time.from_sec(k*.001);n.imu(m)
                e=Float64MultiArray(data=[k*.001,0.,0.]);n.enc(e)
        with n.lock:
            worker=threading.Thread(target=receive);worker.start();worker.join(timeout=2.)
            self.assertFalse(worker.is_alive(),'Sensor reception waited for the image/planning lock')
        self.assertEqual(len(n.encoder_history),100);self.assertAlmostEqual(n.motion.latest(),.1)
        np.testing.assert_allclose(n.motion.rotation(.02,.08),np.eye(3))

    def test_real_offset_order_and_status(self):
        names=['id','camId','rgb','status','x','y','z','b_x','b_y','b_z','d','c_00','c_01','c_02','c_11','c_12','c_22','c_d']
        fields=[PointField(n,4*i,PointField.UINT32 if i<4 else PointField.FLOAT32,1) for i,n in enumerate(names)]
        node=Active.__new__(Active);node.cfg={'tracking_aware':True};node.map=VisualMap();node.tbc=np.zeros(3)
        for k in range(1,4):
            od=Odometry();od.header.stamp=rospy.Time(k);od.pose.pose.orientation.w=1
            rows=[[i,0,0,status,5.,1.,2.,0.,0.,1.,5.,.01,0.,0.,.02,0.,.03,0.] for i,status in [(7,4),(8,3),(9,0)]]
            msg=create_cloud(Header(stamp=od.header.stamp),fields,rows)
            node.odoms={k*10**9:od};node.clouds={k*10**9:msg};node.join()
        self.assertEqual(list(node.map.points),[7])
        p,C,_=node.map.body_points(np.eye(4));np.testing.assert_allclose(p,[[5,1,2]])
        np.testing.assert_allclose(C,[np.diag([.01,.02,.03])],rtol=1e-6)

    def test_geometry_map_uses_matched_images_and_fused_frame(self):
        n=Active.__new__(Active);n.K=np.array([[250.,0,240],[0,250,135],[0,0,1.]])
        n.geometry=ImageGeometryMap(n.K);n.cfg={'range_max':30};n.map=VisualMap()
        u,v=np.meshgrid(np.arange(0,481,5),np.arange(0,271,5))
        cloud=np.c_[(u.ravel()-240)/250*8,(v.ravel()-135)/250*8,np.full(u.size,8.)]
        gray=np.zeros((270,480),np.uint8)
        for x in range(20,450,30):
            for y in range(20,240,30):cv2.rectangle(gray,(x,y),(x+5,y+5),255,-1)
        fields=[PointField('id',0,PointField.UINT32,1),PointField('status',4,PointField.UINT32,1)]
        fields.extend(PointField(c,8+4*i,PointField.FLOAT32,1) for i,c in enumerate('xyz'))
        for k in range(1,4):
            od=Odometry();od.header.stamp=rospy.Time(k);od.pose.pose.orientation.w=1.;od.pose.pose.position.x=100.
            ex=PoseStamped();ex.header=od.header;ex.pose.orientation.w=1.
            im=Image();im.header=od.header;im.height,im.width=gray.shape;im.step=im.width;im.encoding='mono8';im.data=gray.tobytes()
            msg=create_cloud(od.header,fields,[[1,4,0.,0.,8.],[2,3,1.,0.,8.]])
            stamp=k*10**9;n.odoms={stamp:od};n.clouds={stamp:msg};n.images={stamp:im};n.extrinsics={stamp:ex};n.scans={stamp:cloud};n.join()
        self.assertGreater(len(n.map.points),50)
        self.assertEqual(n.scene_health['matched_rovio_depth'],1)
        np.testing.assert_allclose(n.current_tracks[1],[[100.,0,8]],atol=1e-8)
        self.assertTrue(all(90<p.position[0]<110 for p in n.map.points.values()))
if __name__=='__main__':unittest.main()
