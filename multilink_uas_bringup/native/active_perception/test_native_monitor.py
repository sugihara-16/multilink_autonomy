#!/usr/bin/env python3
"""Compare recorder CSV timestamps/values and initialization gate over ROS1."""
import csv,json,os,signal,subprocess,time,tempfile
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3];children=[]
def wait(p):
 start=time.monotonic()
 while not p():
  if time.monotonic()-start>15:raise RuntimeError('Timeout')
  time.sleep(.01)
def stop(proc):
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
with tempfile.TemporaryDirectory() as tmp:
 out=Path(tmp);old=out/'old';new=out/'new';old.mkdir();new.mkdir()
 try:
  children.append(subprocess.Popen(['roscore','-p','11490'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True));time.sleep(2)
  rospy.init_node('recorder_comparison')
  for cmd,directory,name in [(['python3',str(HERE/'monitor.py')],old,'old_monitor'),([str(WS/'build/native-adapters/native_monitor')],new,'new_monitor')]:
   children.append(subprocess.Popen(cmd+['--output',str(directory),'__name:='+name],stdout=subprocess.DEVNULL,start_new_session=True))
  pubs=[rospy.Publisher(t,Odometry,queue_size=100) for t in ['/rovio/odometry','/slam/odometry','/dragon/ground_truth']]
  enc=rospy.Publisher('/active/encoder',Float64MultiArray,queue_size=100)
  wait(lambda:all(p.get_num_connections()==2 for p in pubs+[enc]))
  for i in range(41):
   m=Odometry();m.header.stamp=rospy.Time(12+i//10,(i%10)*100000000+12345);m.pose.pose.position.x=.001*i;m.pose.pose.position.y=-.002*i;m.pose.pose.orientation.w=1.
   for p in pubs:p.publish(m)
   enc.publish(Float64MultiArray(data=[m.header.stamp.to_sec(),.01*i,-.02*i]));time.sleep(.01)
  def complete():
   try:return all(json.loads((d/'evaluation_status.json').read_text())['counts']==dict(rovio=41,mimosa=41,truth=41) for d in [old,new])
   except (FileNotFoundError,json.JSONDecodeError):return False
  wait(complete)
  status=[json.loads((d/'evaluation_status.json').read_text()) for d in [old,new]]
  assert status[0]==status[1],status
  for proc in children[1:]:stop(proc)
  for name in ['rovio','mimosa','truth','encoders']:
   rows=[list(csv.reader((d/(name+'.csv')).open())) for d in [old,new]]
   assert rows[0][0]==rows[1][0]
   assert len(rows[0])==len(rows[1])==42
   for a,b in zip(rows[0][1:],rows[1][1:]):
    if name!='encoders':assert int(a[0])==int(b[0])
    assert list(map(float,a))==list(map(float,b)),(a,b)
  result=dict(rows_per_stream=41,streams=4,timestamps_and_values_identical=True,startup_gate_identical=True)
  (WS/'build/native-adapters/monitor_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(result)
 finally:
  for proc in reversed(children):stop(proc)
