#!/usr/bin/env python3
"""Pause simulation wall time when estimators lag; never change sensor stamps/dt."""
import argparse,json,threading,time
from pathlib import Path
import rospy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mimosa_msgs.msg import LidarManagerDebug,LidarGeometricDebug
from gz.transport13 import Node
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
from pacing_limits import planner_lag_limit

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--paired',action='store_true')
p.add_argument('--planner',action='store_true',help='Pace the simulation against completed gimbal decisions too')
p.add_argument('--planner-period',type=float,default=.2)
p.add_argument('--extra-lidar-topic',action='append',default=[],help='Also wait for these diagnostic LiDAR-completion topics')
p.add_argument('--lidar-stop-lag',type=float,default=.3);p.add_argument('--lidar-resume-lag',type=float,default=.15);a=p.parse_args()
if not 0<a.lidar_resume_lag<a.lidar_stop_lag<.5:raise ValueError('LiDAR pacing limits must satisfy 0 < resume < stop < planner freshness limit 0.5 s')
rospy.init_node('simulation_completion_pacer');lock=threading.RLock();now=0.;last={};n={};last_wall={};paused=False;pause_count=0;pause_total=0.;pause_start=0.
log=(a.output/'processing_times.jsonl').open('w',buffering=1)
def clock(m):
    global now
    with lock:now=m.clock.to_sec()
def record(name,m):
    with lock:
        last[name]=m.header.stamp.to_sec();last_wall[name]=time.monotonic();n[name]=n.get(name,0)+1
        if isinstance(m,LidarManagerDebug):log.write(json.dumps(dict(name=name,stamp=last[name],clock=now,full_ms=m.t_full,map_ms=m.t_update_map,update_ms=m.t_update,declare_ms=m.t_declare,points=m.n_points_in))+'\n')
subs=[rospy.Subscriber('/clock',Clock,clock,queue_size=10)]
def planned(m):
    try:stamp=json.loads(m.data)['stamp_ns']*1e-9
    except (ValueError,KeyError,TypeError):return
    with lock:
        last['planner']=stamp;last_wall['planner']=time.monotonic();n['planner']=n.get('planner',0)+1
if a.planner:subs.append(rospy.Subscriber('/active/status',String,planned,queue_size=10))
for name,topic,kind in [('lidar','/mimosa/lidar/manager/debug',LidarManagerDebug),('vio','/rovio/odometry',Odometry)]+([('fixed_lidar','/fixed_mimosa/lidar/manager/debug',LidarManagerDebug),('fixed_vio','/fixed/rovio/odometry',Odometry)] if a.paired else []):
    subs.append(rospy.Subscriber(topic,kind,lambda m,key=name:record(key,m),queue_size=100,tcp_nodelay=True))
extra_names=[]
for i,topic in enumerate(a.extra_lidar_topic):
    if not topic.startswith('/'):raise ValueError('Diagnostic completion topics must be absolute')
    name='extra_lidar_'+str(i);extra_names.append(name)
    subs.append(rospy.Subscriber(topic,LidarManagerDebug,lambda m,key=name:record(key,m),queue_size=100,tcp_nodelay=True))
def geometric(name,m):
    with lock:log.write(json.dumps(dict(name=name,stamp=m.header.stamp.to_sec(),**{k:getattr(m,k) for k in m.__slots__ if k.startswith('t_') or k.startswith('n_')}))+'\n')
for prefix in ['', '/fixed'] if a.paired else ['']:
    topic='/fixed_mimosa' if prefix else '/mimosa'
    subs.append(rospy.Subscriber(topic+'/lidar/geometric/debug',LidarGeometricDebug,lambda m,key=prefix+'geometry':geometric(key,m),queue_size=100))
node=Node();last_save=0.
try:
    while not rospy.is_shutdown():
        wall=time.monotonic()
        with lock:
            ages={k:now-v for k,v in last.items()}
            if now>35. and any(k not in last for k in extra_names):
                raise RuntimeError('A required diagnostic estimator has not published its first LiDAR completion')
            # 250 ms of simulated time allows transport/overlap but stays below
            # both the planner freshness limit and MIMOSA's 500 ms late gate.
            def threshold(k):
                if k=='planner':return planner_lag_limit(paused,a.planner_period)
                if 'vio' in k:return .23 if paused else .4
                return a.lidar_resume_lag if paused else a.lidar_stop_lag
            want=any(v>threshold(k) for k,v in ages.items())
            if paused and wall-pause_start>15:raise RuntimeError('Estimator failed to catch up while simulation paused: '+str(ages))
        if want!=paused:
            ok,response=node.request('/world/zab/control',WorldControl(pause=want),WorldControl,Boolean,1000)
            if not ok or not response.data:raise RuntimeError('Gazebo pause service failed')
            paused=want
            if paused:pause_start=wall;pause_count+=1
            else:pause_total+=wall-pause_start
        if wall-last_save>1:
            with lock:report=dict(sim_time=now,ages=ages,counts=n.copy(),paused=paused,pause_count=pause_count,pause_wall_s=pause_total+(wall-pause_start if paused else 0),extra_lidar_topics=a.extra_lidar_topic)
            tmp=a.output/'pacing.tmp';tmp.write_text(json.dumps(report));tmp.replace(a.output/'pacing.json');last_save=wall
        time.sleep(.005)
finally:
    log.close()
    if paused:node.request('/world/zab/control',WorldControl(pause=False),WorldControl,Boolean,1000)
